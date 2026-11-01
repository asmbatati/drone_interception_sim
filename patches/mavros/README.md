# mavros Jazzy build fixes: where they live

The workspace builds MAVROS from `https://github.com/asmbatati/mavros`, branch
`jazzy-build-fixes` (see `interception.repos`). That repo is a fork of Mohamed
Abdelkader's [`mzahana/mavros`](https://github.com/mzahana/mavros), itself a fork
of [`mavlink/mavros`](https://github.com/mavlink/mavros).

The branch is `mzahana/mavros` `ros2` plus two of Mohamed's commits from
`ros2_humble` (`a54cb996` "Define parameters for base_link, odom, map frames",
`3b3804f6` "Define _frd frames in odom plugin") plus two build fixes:

| Commit | What it fixes |
|---|---|
| `08d09b14` Fix tf2_eigen include for Jazzy (.h -> .hpp) | Jazzy ships only `tf2_eigen/tf2_eigen.hpp` |
| `723346c8` mavros_extras: link libyaml-cpp explicitly | `yaml_cpp_vendor` does not propagate the link library |

These two were kept here as patch files until they were pushed to the fork. The
patch files are gone from this folder because the branch replaces them; they
remain in this repo's history.
