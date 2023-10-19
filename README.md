# ft_tools_ros2
Wrench estimation and calibration of F/T sensor for ROS2 applications.


***The current devs are based on the humble ROS 2 distribution (Ubuntu 22.04 LTS)***

[![CI](https://github.com/tpoignonec/ft_tools_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/tpoignonec/ft_tools_ros2/actions/workflows/ci.yml)


# Stack content

## F/T sensor calibration utils

### FT parameters object

The common interface to perform sensor calibration and wrench estimation is the [ft_tools::FtParameters](ft_tools/include/ft_tools/ft_parameters.hpp) object that is used by both [ft_tools::FtCalibration](ft_tools/include/ft_tools/ft_calibration.hpp) and [ft_tools::FtEstimation](ft_tools/include/ft_tools/ft_estimation.hpp).
The calibration sensor-specific parameters are:
```yaml
mass: 0.0
sensor_frame_to_com: [0.0, 0.0, 0.0]
force_offset: [0.0, 0.0, 0.0]
torque_offset: [0.0, 0.0, 0.0]
```

The parameters can be loaded:

- from a yaml file:

```cpp
ft_tools::FtParameters ft_parameters
// Load parameters from YAML file (absolut path)
const std::string filename("...")
bool ok = ft_parameters.from_yaml(filename);
// or from a config folder
bool ok = ft_parameters.from_yaml(config_filename, config_package);
```

- from a msg:

```cpp
// Get calibration msg (typically from a service call)
// i.e., ft_msgs::srv::FtCalibration ft_parameters_msg;

ft_tools::FtParameters ft_parameters
bool ok = ft_parameters.from_msg(ft_parameters_msg);
```

The parameters can likewise be written to dump to a yaml file or sent as a msg.

### FT calibration

The [ft_tools::FtCalibration](ft_tools/include/ft_tools/ft_calibration.hpp) implements a simple sensor calibration method [1] using a least square regression (i.e. eigen SVD solver).

We are given a N sets of measurements $\left( g, f, \tau \right)$ expressed in the sensor frame of reference where $g \in \mathbb{R}^3$ is the (signed) gravity and $f, \tau \in \mathbb{R}^3$ are the raw force and torque, respectively.

We want to retrieve the F/T sensor calibration that consists in
- the mass $m$ in Kg
- the center of mass $c \in \mathbb{R}^3$ in m
- the force offset $f_0 \in \mathbb{R}^3$ in N
- the torque offset $\tau_0 \in \mathbb{R}^3$ in N.m

If enough measurements were provided (i.e., about 6-10 well chosen robot poses), the different parameters are identified sing a least square regression (i.e. eigen SVD solver) such that

$$ f_\text{meas} = -mg + f_0 \text{ and } \tau_\text{meas} = -mc \times g + \tau_0$$

This process return a `ft_tools::FtParameters` object.

**Credits:** the code is inspired by the ROS1 package [force_torque_tools](https://github.com/kth-ros-pkg/force_torque_tools).

### FT estimation

The [ft_tools::FtEstimation](ft_tools/include/ft_tools/ft_estimation.hpp) implements a generic wrench estimator with the following features:

- Gravity and offsets compensation such that
  - ${}^{s}f_\text{est} = {}^{s}f_\text{meas} + m{}^{s}g - {}^{s}f_0$
  - ${}^{s}\tau_\text{est} = {}^{s}\tau_\text{meas} + {}^sp_{com} \times m{}^{s}g - {}^{s}\tau_0$

- Display force at interaction point (typically, the end-effector) such that
  -  ${}^{ee}f_\text{est} = {}^{ee}R_s {}^{s}f_\text{est}$
  -  ${}^{ee}\tau_\text{est} = {}^{ee}R_s \left( {}^{s}\tau_\text{est} - {}^s p_{ee} \times {}^{s}f_\text{est} \right) $

- Apply wrench deadband on ${}^{ee}f_\text{est}$ and ${}^{ee}\tau_\text{est}$ (optional)

## Basic GUI


## Examples

## References

References:
- [1] D. Kubus, T. Kroger and F. M. Wahl, "On-line rigid object recognition and pose estimation based on inertial parameters," 2007 IEEE/RSJ International Conference on Intelligent Robots and Systems, 2007, pp. 1402-1408, doi: 10.1109/IROS.2007.4399184.