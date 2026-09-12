# Project Audit

This document records the main problems found in the first version of the warehouse robot and the improvements made during development.

It is kept as a project history so that the changes can be understood clearly.

## Initial Problems

| Area                | Initial problem                                                              | Status      |
| ------------------- | ---------------------------------------------------------------------------- | ----------- |
| Safety controller   | Old LiDAR data could continue being used after the sensor stopped publishing | Fixed       |
| E-stop              | A normal deactivate command could clear the E-stop                           | Fixed       |
| IMU                 | IMU was configured but was not actually present in Gazebo                    | Fixed       |
| Localization        | Localization was based on wheel odometry but was not clearly documented      | Clarified   |
| Sensor health       | Sensor status information was limited                                        | Improved    |
| Robot health        | Health states were too simple                                                | Improved    |
| Dashboard           | There was no actual web interface                                            | Added       |
| Gazebo world        | The simulation used an empty environment                                     | Improved    |
| Fault injection     | Sensor failures had to be tested manually                                    | Added       |
| Testing             | There were no functional safety tests                                        | Added       |
| Obstacle handling   | Some invalid LiDAR data cases were not handled explicitly                    | Improved    |
| Battery model       | Battery information was limited                                              | Improved    |
| Diagnostics         | Multiple `/diagnostics` publishers were not clearly documented               | Documented  |
| Multi-robot support | Topics were global instead of namespaced                                     | Future work |

## Main Improvements

The project was upgraded with:

* LiDAR freshness checking
* Odometry freshness checking
* Battery communication monitoring
* Controlled E-stop reset
* IMU simulation and monitoring
* Sensor fault injection
* Robot health state handling
* Safety-focused obstacle handling
* Battery speed limiting
* A real-time web dashboard
* A warehouse Gazebo environment
* Functional safety tests
* Dashboard connection monitoring

## Validation

The upgraded system was tested using:

```text
ROS 2 Humble
Gazebo Classic 11.10.2
```

The workspace was successfully built with all five packages.

Application tests:

```text
Safety logic tests      34 passed
Dashboard tests          7 passed
Combined tests           41 passed
```

The following safety scenarios were also tested:

* Obstacle slowdown
* Obstacle stop
* LiDAR failure and recovery
* Odometry failure and recovery
* Battery communication failure and recovery
* IMU fault detection
* E-stop activation and reset
* Dashboard disconnect and reconnect

## Current Status

The major issues identified in the initial version have been addressed.

The project is now a working **ROS 2 warehouse robot safety and monitoring simulation**.

Some features are intentionally left for future development, including:

* Full Nav2 navigation
* Fused localization
* Real hardware integration
* Multi-robot coordination
* Hardware-in-the-loop testing
* CI for the complete workspace

## Note

This audit describes the project **before the current upgrade**. It should be read as a development history rather than a list of current failures.
