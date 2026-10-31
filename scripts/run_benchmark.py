#!/usr/bin/env python3
"""Batch benchmark runner: N trials x methods x scenarios, unattended.

Drives the method-agnostic ``interception_metrics`` logger over a campaign
described in a YAML file (see ``benchmarks/campaigns/*.yaml``):

* for each scenario it (re)starts the sim with the scenario's launch args
  (``target_trajectory``, ``target_evade``, ...), unless ``--no-manage-sim``;
* trials are interleaved (trial 0 of every method, then trial 1, ...) so slow
  drift in the sim cannot bias one method;
* before each trial the interceptor is brought home and disarmed (AUTO.RTL,
  then AUTO.LAND), the target is checked to be armed, in OFFBOARD and
  airborne, and the sim is restarted if either check fails;
* PX4's own preflight check on the interceptor (``px4-commander check`` on
  instance 0) must pass; an estimator fault left by a collision (seen: an EKF
  magnetometer fault after a capture at 0.7 m) otherwise blocks arming for the
  rest of the campaign. A trial whose interceptor never lifts off is recorded
  as ``no_liftoff`` (an infrastructure fault, not a method result), the sim is
  restarted and the trial is run again (up to ``max_retries``);
* a seeded random delay of up to one target period randomizes the target's
  phase on its path when the engagement starts;
* ``px4_params`` from the campaign (e.g. a low ``RTL_RETURN_ALT``, so the
  trip home between trials takes seconds instead of a 30 m climb) are set on
  the interceptor once the sim is up;
* each method's launch (plus a metrics launch where the method has none) runs
  in its own process group and is stopped once its summary row appears or the
  trial times out.

All rows land in ``<out>/results.csv`` (the metrics summary format plus
``scenario`` and ``trial``); per-trial 10 Hz traces in ``<out>/traces/`` and
launch logs in ``<out>/logs/``. Re-running with the same ``--out`` skips
(method, scenario, trial) cells that already have a row, so a campaign can be
resumed after an interruption.

    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=21
    python3 scripts/run_benchmark.py benchmarks/campaigns/pipeline_vs_pn.yaml \\
        --out benchmarks/runs/pipeline_vs_pn
"""
import argparse
import csv
import math
import os
import random
import signal
import subprocess
import sys
import time

import yaml

SCRIPTS = os.path.dirname(os.path.abspath(__file__))
#: outcomes that are infrastructure faults: excluded from results, re-run
RETRY_OUTCOMES = ('no_liftoff', 'error', 'scene_error', 'superseded')


def log(msg):
    print(time.strftime('[%H:%M:%S] ') + msg, flush=True)


class Watch:
    """rclpy node caching both vehicles' mavros state + odometry."""

    def __init__(self):
        import rclpy
        from mavros_msgs.msg import State, StatusText
        from mavros_msgs.srv import CommandBool, SetMode
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        self._rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('benchmark_runner')
        self.state = {}
        self.odom = {}
        self.texts = {}   # last PX4 status texts per vehicle (arming-denied reasons)
        for ns in ('interceptor', 'target'):
            self.node.create_subscription(
                StatusText, f'/{ns}/mavros/statustext/recv',
                lambda m, ns=ns: self.texts.setdefault(ns, []).append(m.text),
                qos_profile_sensor_data)   # mavros publishes it best-effort
            self.node.create_subscription(
                State, f'/{ns}/mavros/state',
                lambda m, ns=ns: self.state.__setitem__(ns, m), 10)
            self.node.create_subscription(
                Odometry, f'/{ns}/mavros/local_position/odom',
                lambda m, ns=ns: self.odom.__setitem__(ns, m),
                qos_profile_sensor_data)
        self.set_mode = self.node.create_client(
            SetMode, '/interceptor/mavros/set_mode')
        self.arming = self.node.create_client(
            CommandBool, '/interceptor/mavros/cmd/arming')
        self._SetMode, self._CommandBool = SetMode, CommandBool

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)

    def reset(self):
        self.state.clear()
        self.odom.clear()
        self.texts.clear()

    def connected(self, ns):
        st = self.state.get(ns)
        return st is not None and st.connected and ns in self.odom

    def alt(self, ns):
        od = self.odom.get(ns)
        return od.pose.pose.position.z if od is not None else float('nan')

    def xy(self, ns):
        od = self.odom.get(ns)
        if od is None:
            return float('nan')
        p = od.pose.pose.position
        return math.hypot(p.x, p.y)

    def target_ok(self):
        st = self.state.get('target')
        return (self.connected('target') and st.armed and st.mode == 'OFFBOARD'
                and self.alt('target') > 2.0)

    def describe(self, ns):
        st = self.state.get(ns)
        if st is None or not st.connected:
            return f'{ns}: FCU not connected'
        if ns not in self.odom:
            return f'{ns}: no odometry'
        last = ' | '.join(dict.fromkeys(self.texts.get(ns, [])[-6:]))
        return (f'{ns}: armed={st.armed} mode={st.mode} '
                f'alt={self.alt(ns):.1f} m; PX4: {last or "-"}')

    def interceptor_home(self):
        st = self.state.get('interceptor')
        return (self.connected('interceptor') and not st.armed
                and self.alt('interceptor') < 0.6 and self.xy('interceptor') < 3.0)

    def request_mode(self, mode):
        if self.set_mode.service_is_ready():
            self.set_mode.call_async(self._SetMode.Request(custom_mode=mode))

    def disarm(self):
        if self.arming.service_is_ready():
            self.arming.call_async(self._CommandBool.Request(value=False))

    def close(self):
        self.node.destroy_node()
        self._rclpy.shutdown()


class Runner:
    def __init__(self, cfg, out, manage_sim, dry_run):
        self.cfg = cfg
        self.out = out
        self.manage_sim = manage_sim
        self.dry_run = dry_run
        self.partition = cfg.get('partition', 'ihunter')
        self.domain = str(cfg.get('domain', os.environ.get('ROS_DOMAIN_ID', '21')))
        self.timeout = float(cfg.get('timeout', 120.0))
        self.period = float(cfg.get('phase_period', 2 * math.pi / 0.4))
        self.rng = random.Random(int(cfg.get('seed', 1)))
        self.results = os.path.join(out, 'results.csv')
        os.makedirs(os.path.join(out, 'traces'), exist_ok=True)
        os.makedirs(os.path.join(out, 'logs'), exist_ok=True)
        self.sim = None
        self.sim_scenario = None
        self.px4_dir = os.path.expanduser(cfg.get(
            'px4_dir', os.environ.get('PX4_DIR', '~/drone_interception_ws/PX4-Autopilot')))
        self.max_retries = int(cfg.get('max_retries', 2))
        self.watch = None if dry_run else Watch()

    # --- process helpers -----------------------------------------------------
    def _spawn(self, argv, logname):
        env = dict(os.environ, ROS_DOMAIN_ID=self.domain,
                   GZ_PARTITION=self.partition,
                   RMW_IMPLEMENTATION=os.environ.get('RMW_IMPLEMENTATION',
                                                     'rmw_fastrtps_cpp'),
                   RRV_SHIM_BYPASS='1')
        f = open(os.path.join(self.out, 'logs', logname), 'w')
        return subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, env=env,
                                start_new_session=True)

    @staticmethod
    def _stop(proc, grace=10.0):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=grace)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass

    # --- sim lifecycle --------------------------------------------------------
    def _sim_args(self, scen):
        args = dict(self.cfg.get('sim_args') or {})
        args.update({'gz_partition': self.partition, 'ros_domain_id': self.domain})
        args.update(self.cfg['scenarios'][scen] or {})
        return [f'{k}:={v}' for k, v in args.items()]

    def stop_sim(self):
        self._stop(self.sim, grace=15.0)
        subprocess.run([os.path.join(SCRIPTS, 'stop_sim.sh'), self.partition],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.sim = None
        self.sim_scenario = None

    def start_sim(self, scen):
        self.stop_sim()
        argv = ['ros2', 'launch', 'drone_interception_sim', 'interception.launch.py'
                ] + self._sim_args(scen)
        log(f'sim start [{scen}]: {" ".join(argv[4:])}')
        self.sim = self._spawn(argv, f'sim_{scen}_{int(time.time())}.log')
        self.sim_scenario = scen
        self.watch.reset()
        # good starts are ready in ~20 s even under load; a start that is not
        # ready by then has a stuck target and only wastes the wait
        wait = float(self.cfg.get('sim_ready_timeout', 60.0))
        t_end = time.monotonic() + wait
        while time.monotonic() < t_end:
            self.watch.spin(1.0)
            if self.watch.target_ok() and self.watch.connected('interceptor'):
                log('sim ready (target airborne in OFFBOARD)')
                self.watch.spin(5.0)
                self.set_px4_params()
                return True
        log(f'sim did not become ready in {wait:g} s '
            f'({self.watch.describe("target")})')
        return False

    def set_px4_params(self):
        for name, value in (self.cfg.get('px4_params') or {}).items():
            for _ in range(5):   # the param node can lag discovery after a start
                r = subprocess.run(
                    ['ros2', 'param', 'set', '/interceptor/mavros/param', name,
                     str(value)], capture_output=True, text=True, timeout=30,
                    env=dict(os.environ, ROS_DOMAIN_ID=self.domain,
                             RRV_SHIM_BYPASS='1'))
                msg = (r.stdout or r.stderr).strip()
                if 'successful' in msg:
                    break
                self.watch.spin(3.0)
            log(f'px4 param {name}={value}: {msg}')

    def ensure_scene(self, scen):
        """Interceptor home + disarmed, target flying; restart the sim if not."""
        for attempt in range(3):
            if self.manage_sim and (self.sim_scenario != scen or self.sim is None
                                    or self.sim.poll() is not None):
                if not self.start_sim(scen):
                    continue
            home = self._bring_home()
            if home and self.watch.target_ok() and self.preflight_ok():
                return True
            log(f'scene unhealthy (attempt {attempt + 1}): '
                f'interceptor_home={self.watch.interceptor_home()} '
                f'target_ok={self.watch.target_ok()}')
            if not self.manage_sim:
                self.watch.spin(10.0)
                continue
            self.sim_scenario = None   # force a restart
        return False

    def preflight_ok(self, instance=0, tries=8):
        """PX4's preflight check on the interceptor instance (SITL client)."""
        exe = os.path.join(self.px4_dir, 'build', 'px4_sitl_default', 'bin',
                           'px4-commander')
        if not os.path.exists(exe):
            return True
        out = ''
        for i in range(tries):
            r = subprocess.run([exe, '--instance', str(instance), 'check'],
                               capture_output=True, text=True, timeout=20,
                               cwd=os.path.dirname(os.path.dirname(exe)))
            out = (r.stdout + r.stderr).strip()
            if 'Preflight check: OK' in out or 'FAILED' not in out:
                return True
            self.watch.spin(3.0)
        log(f'interceptor preflight check failed: {out.splitlines()[-1] if out else "?"}')
        return False

    def _bring_home(self):
        w = self.watch
        w.spin(1.0)
        if w.interceptor_home():
            return True
        for mode, wait in (('AUTO.RTL', 90.0), ('AUTO.LAND', 45.0)):
            t_end = time.monotonic() + wait
            while time.monotonic() < t_end:
                st = w.state.get('interceptor')
                if st is not None and not st.armed and w.alt('interceptor') < 0.6:
                    break
                if st is not None and st.mode != mode:
                    w.request_mode(mode)
                w.spin(1.0)
            if w.interceptor_home():
                w.spin(2.0)
                return True
        st = w.state.get('interceptor')
        if st is not None and st.armed and w.alt('interceptor') < 0.6:
            w.disarm()
            w.spin(3.0)
        return w.interceptor_home()

    # --- trials ----------------------------------------------------------------
    def _done_cells(self):
        done = set()
        if os.path.exists(self.results):
            with open(self.results) as f:
                for row in csv.DictReader(f):
                    if row['outcome'] not in RETRY_OUTCOMES:
                        done.add((row['method'], row['scenario'], row['trial']))
        return done

    def _mark_last(self, outcome):
        """Rewrite the outcome of the last results row (infrastructure fault)."""
        with open(self.results) as f:
            rows = list(csv.reader(f))
        rows[-1][1] = outcome
        with open(self.results, 'w', newline='') as f:
            csv.writer(f).writerows(rows)

    def _rows(self):
        if not os.path.exists(self.results):
            return 0
        with open(self.results) as f:
            return max(sum(1 for _ in f) - 1, 0)

    def _append_error(self, method, scen, trial, why):
        new = not os.path.exists(self.results)
        with open(self.results, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['method', 'outcome', 'time_to_intercept_s', 'min_range_m',
                            'capture_radius_m', 'trace_csv', 'time_engaged_s',
                            'scenario', 'trial'])
            w.writerow([method, why, '', '', '', '', '', scen, trial])

    def run_trial(self, m, scen, trial):
        name = m['name']
        slug = name.replace('/', '_').replace('+', '-')
        trace = os.path.join(self.out, 'traces', f'{scen}__{slug}__t{trial:02d}.csv')
        label = {'method': name, 'scenario': scen, 'trial': str(trial),
                 'csv_path': trace, 'summary_csv_path': self.results}
        pkg, launch_file, args = m['launch']
        # '~' and $VARS in launch args expand (e.g. a policy path under $HOME)
        args = {k: os.path.expanduser(os.path.expandvars(str(v)))
                for k, v in (args or {}).items()}
        cmds = []
        if m.get('metrics_in_launch', False):
            args.update({'metrics': 'true'}, **label)
        else:
            cmds.append(['ros2', 'launch', 'drone_interception_sim', 'metrics.launch.py']
                        + [f'{k}:={v}' for k, v in label.items()])
        cmds.insert(0, ['ros2', 'launch', pkg, launch_file]
                    + [f'{k}:={v}' for k, v in args.items()])
        if self.dry_run:
            for c in cmds:
                print('   ', ' '.join(c))
            return
        delay = self.rng.uniform(0.0, self.period)
        log(f'{scen} t{trial:02d} {name}: phase delay {delay:.1f} s')
        self.watch.spin(delay)
        before = self._rows()
        procs = [self._spawn(c, f'{scen}__{slug}__t{trial:02d}_{i}.log')
                 for i, c in enumerate(cmds)]
        try:
            outcome = self._await_row(procs, before, m)
        finally:   # also on Ctrl-C: orphaned stage nodes would pollute later trials
            for p in procs:
                self._stop(p)
        if outcome is None:
            self._append_error(name, scen, trial, 'error')
            outcome = 'error (no metrics row)'
        log(f'{scen} t{trial:02d} {name}: {outcome}')
        return outcome.split()[0] not in RETRY_OUTCOMES

    def _await_row(self, procs, before, m):
        t_end = time.monotonic() + self.timeout + float(m.get('extra_time', 60.0))
        outcome = None
        while time.monotonic() < t_end:
            self.watch.spin(1.0)
            if self._rows() > before:
                with open(self.results) as f:
                    last = list(csv.DictReader(f))[-1]
                if not last.get('time_engaged_s') and last['outcome'] != 'capture':
                    self._mark_last('no_liftoff')
                    last['outcome'] = 'no_liftoff'
                outcome = (f"{last['outcome']} t={last['time_to_intercept_s']}s "
                           f"engaged={last.get('time_engaged_s', '')}s "
                           f"min={last['min_range_m']}m")
                break
            if any(p.poll() is not None for p in procs[:1]):
                break
        return outcome

    def run(self, only_scenarios=None):
        trials = int(self.cfg.get('trials', 10))
        methods = self.cfg['methods']
        done = self._done_cells()
        if not self.manage_sim and not self.dry_run:
            self.set_px4_params()
        for scen in self.cfg['scenarios']:
            if only_scenarios and scen not in only_scenarios:
                continue
            for trial in range(trials):
                for m in methods:
                    if (m['name'], scen, str(trial)) in done:
                        continue
                    if self.dry_run:
                        print(f'[{scen} t{trial}] {m["name"]}')
                        self.run_trial(m, scen, trial)
                        continue
                    for attempt in range(1 + self.max_retries):
                        if not self.ensure_scene(scen):
                            log('scene could not be made healthy; skipping trial')
                            self._append_error(m['name'], scen, trial, 'scene_error')
                            break
                        if self.run_trial(m, scen, trial):
                            break
                        log('infrastructure fault: restarting the sim and retrying')
                        self.sim_scenario = None   # force a restart
        if not self.dry_run:
            self._bring_home()
            if self.manage_sim:
                self.stop_sim()
            self.watch.close()


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('campaign', help='campaign YAML')
    p.add_argument('--out', required=True, help='output directory')
    p.add_argument('--no-manage-sim', action='store_true',
                   help='use an already-running sim (single scenario)')
    p.add_argument('--scenario', action='append',
                   help='run only these scenarios (repeatable)')
    p.add_argument('--trials', type=int, help='override the campaign trial count')
    p.add_argument('--dry-run', action='store_true', help='print the launches only')
    a = p.parse_args()
    with open(a.campaign) as f:
        cfg = yaml.safe_load(f)
    if a.trials is not None:
        cfg['trials'] = a.trials
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'campaign.yaml'), 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    r = Runner(cfg, a.out, manage_sim=not a.no_manage_sim, dry_run=a.dry_run)
    try:
        r.run(only_scenarios=a.scenario)
    except KeyboardInterrupt:
        log('interrupted')
        if r.manage_sim and not a.dry_run:
            r.stop_sim()
        sys.exit(130)


if __name__ == '__main__':
    main()
