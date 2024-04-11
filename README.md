> **NOTE:** should be used with your ros2 project running on linux `Ubuntu 22.04` [`ros-humble`] (e.g Raspberry Pi, PC, etc.)

## How to Use the Package
- ensure you've already set up your microcomputer or PC system with `ros-humble` with `colcon` and your `ros workspace` also setup

- install the `libserial-dev` package on your linux machine
  > ```sudo apt-get update```
  >
  > ```sudo apt install libserial-dev```

- install `rosdep` so you can install necessary ros related dependencies for the package.
  > ```sudo apt-get update```
  >
  > ```sudo apt install python3-rosdep2```
  >
  > ```rosdep update```

- In the src/ folder of your ros workspace, clone the repo (or you can download and add it manually to the src/ folder)
  > ```git clone -b humble https://github.com/samuko-things-company/sic_mpu9250_ros.git```

- cd into the package folder (i.e `sic_mpu9250_ros`) and run rosdep to install any necessary ros dependencies
  > ```cd sic_mpu9250_ros```
  >
  > ```rosdep install --from-paths src --ignore-src -r -y```

- ensure it is the madgwick code that is running in the driver module (i.e you should see the green LED turn on). if not, upload it.

- connect the already calibrated MPU9250 interfaced with the sic_mpu9250_driver module to the Computer (PC or microcomputer) and check it serial port:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  >
  > ```ls /dev/serial/by-path```
  >
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  > ```ls /dev/ttyU*```
  >
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on
  
- change the serial-port value in the sic_mpu9250_params.yaml file found in the config folder to the detected serial-port.
- you may not need to change the frame name and the publish_frequency.

- build the packages with colcon (in your ros workspace root folder):
  > ```colcon build --packages-select sic_mpu9250_ros --symlink-install``` or ```colcon build --packages-select sic_mpu9250_ros```

- to vizualize in rviz, run:
  > ```ros2 launch sic_mpu9250_ros test_sic_mpu9250.launch.py``` in a terminal and ```rviz2``` in another terminal.
  > add TF and move the IMU about to see the transform from the imu frame to the map frame for test.

- to use in your project (e.g with a URDF file), run:
  > ```ros2 launch sic_mpu9250_ros start_sic_mpu9250.launch.py```.
  > the imu frame with the imu_data topic should now be available.