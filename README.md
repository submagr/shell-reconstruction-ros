# ROS2 package for shell-reconstruction

### Overview
- [shell_recon_interfaces](src/shell_recon_interfaces) defines `DoShellRecon` ros service
- [shell_recon_service](src/shell_recon_service) contains the shell reconstruction service and client
- We use [ros2_numpy](src/ros2_numpy) for conversion between numpy and `ros2 sensor_msgs.msg`. [ros2_numpy](src/ros2_numpy) is further dependent upon [ament_cmake](src/ament_cmake).

### Setup
- Download submodules:
    ```bash
    git submodule update --init
    ```
- Make sure your ros2-foxy-fitzroy environment and `colcon` is [correctly configured](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html) is correct.
- Setup python:
    ```bash
    python -m venv env  # Python should be >= 3.6 for open3d to work
    source env/bin/activate
    pip install -r build_requirements.txt
    pip install -r src/shell_recon_service/shell_recon_service/shell-reconstruction/requirements.txt 
    ```
- Build all the packages (make sure you are in the project root)
    ```bash
    colcon build
    ```

### Running service:
From project root, run the following commands:
- Run service:
    ```bash
    . install/setup.bash  # ROS2 Path setup
    export PYTHONPATH=$PWD/env/lib/python3.8/site-packages:$PYTHONPATH  # Help ROS2 find python environment packages
    export PYTHONPATH=$PWD/src/shell_recon_service/shell_recon_service/shell-reconstruction/:$PYTHONPATH  # Add shell module path
    ros2 run shell_recon_service service
    ```
- In seperate terminal, run client:
    ```bash
    . install/setup.bash
    export PYTHONPATH=$PWD/env/lib/python3.8/site-packages:$PYTHONPATH  # Help ROS2 find python environment packages
    export PYTHONPATH=$PWD/src/shell_recon_service/shell_recon_service/shell-reconstruction/:$PYTHONPATH  # Add shell module path
    ros2 run shell_recon_service client --pkl_data_path $PWD/src/shell_recon_service/shell_recon_service/shell-reconstruction/demo_data/real_data.pkl
    ```
- The shell reconstruction results will be generated at the following location:
    ```bash
    src/shell_recon_service/shell_recon_service/shell-reconstruction/demo_data/reconstruction_ros_service
    ```