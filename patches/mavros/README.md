# Local mavros patches (Jazzy build fixes)

The workspace builds MAVROS from the fork `https://github.com/mzahana/mavros`,
checked out as the local branch `feature/jazzy-tf2-eigen-fix` (also named
`asmbatati/jazzy-tf2-eigen-fix`). The last two commits on that branch exist
only on this machine. They are kept here so a fresh checkout can be patched;
they were not pushed to the fork, which belongs to someone else.

| Patch | What it fixes |
|---|---|
| `0001-Fix-tf2_eigen-include-for-Jazzy-.h-.hpp.patch` | Jazzy ships only `tf2_eigen/tf2_eigen.hpp`; five files still included the removed `.h` header (uas_tf, local/global/home_position, imu) |
| `0002-mavros_extras-link-libyaml-cpp-explicitly.patch` | `yaml_cpp_vendor` does not propagate the link library on Jazzy, so every mavros_extras plugin failed to load (`YAML::BadConversion` undefined at dlopen) |

The local branch is `origin/ros2` plus two fork commits that live on
`origin/ros2_humble` (`a54cb996` "Define parameters for base_link, odom, map
frames", `3b3804f6` "Define _frd frames in odom plugin") plus these two patches.
To rebuild it on a fresh clone of the fork:

```bash
cd ros2_ws/src/mavros
git checkout -b asmbatati/jazzy-tf2-eigen-fix origin/ros2
git cherry-pick a54cb996 3b3804f6
git am ../drone_interception_sim/patches/mavros/*.patch
```
