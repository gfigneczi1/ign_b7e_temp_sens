# `ros2_cpp_template` package
ROS 2 C++ package.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/gfigneczi1/ign_b7e_temp_sens
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select temp_sens --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` r
ros2 launch temp_sens temp_sens.launch.py
```

### Check
Echo the relevant topics:
``` r
ros2 topic echo /warning
```
While the emulator and warning nodes are running, warnings are thrown occasionally.