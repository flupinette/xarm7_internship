# Programs to Control the xArm7 Robot

This repository contains the final work developed during an internship in 2025. It provides tools and configurations to control the xArm7 robotic arm with a linear track, gripper, and camera integration of an Orbbec Astra Pro using ROS2.

---

## Requirements

### Hardware
- **[uFactory xArm7](https://www.ufactory.us/product/ufactory-xarm-7)**
- **[uFactory AC Control Box](https://www.ufactory.us/product/ufactory-xarm-control-box)**
- **[uFactory Linear Track](https://www.ufactory.us/product/direct-drive-linear-motor)**
- **[uFactory xArm Gripper](https://www.ufactory.us/product/ufactory-xarm-gripper)**
- **[uFactory xArm Camera Stand](https://www.ufactory.us/product/ufactory-xarm-camera-stand)**
- **[Orbbec Astra Pro Camera](https://www.orbbec.com/products/structured-light-camera/astra-series/)**

### Software
This project uses **ROS2 Humble**, as it was the most stable and feature-rich version of ROS2 available during the internship.

- **[xArm-Ros2](https://github.com/xArm-Developer/xarm_ros2)**
  The official ROS2 repository for xArm robots.

- **[pymoveit2](https://github.com/AndrejOrsula/pymoveit2)**
  A Python library to control the robot with MoveIt2. While the official MoveIt2 Python library is available, this repository was found to be more intuitive and easier to use.

- **[ros2_astra_camera](https://github.com/orbbec/ros2_astra_camera)**
  A repository to launch the Orbbec Astra Pro camera. To calibrate the camera, you need to adjust the `ir_info_url` and `color_info_url` parameters in the `astra_pro.launch.py` file.

- **[moveit2_calibration](https://github.com/AndrejOrsula/moveit2_calibration)**
  A repository for performing hand-eye calibration with the camera.

---

## Configuration

To use the rail and sensors, follow these steps:

1. Add a file named `xarm_user_params.yaml` in the `src/xarm_ros2/xarm_api/config/` directory.
2. Copy the contents from the `xarm_params.yaml` file (located in the same directory) into `xarm_user_params.yaml`.
3. Set the services corresponding to the rails and sensors to `True`.

**Note:**
During the internship, the services for the rails were referred to as *linear track*. However, toward the end of the internship, the terminology was updated to *linear motor*. This discrepancy may cause issues when running the code. If you encounter errors, update the service names accordingly.

