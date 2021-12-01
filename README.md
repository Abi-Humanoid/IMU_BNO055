# IMU_BNO055

This is a ROS melodic package that publishes data from a BNO055 IMU connected to a computer over an I2C connection. It uses the https://github.com/adafruit/Adafruit_CircuitPython_BNO055 Python library to interact with the sensor.

## Installation

Make sure ROS melodic and Python3 are installed. 

Install the Python library for the BNO055:
```
sudo pip3 install adafruit-circuitpython-bno055
```

Clone the repo into the source folder of the catkin workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/Abi-Humanoid/IMU_BNO055
```

Build the package:
```
cd ~/catkin_ws
catkin_make
```

Launch the publisher node:
```
roslaunch IMU_BNO055 publish_imu_data.launch
```

You can now view the published data e.g. imu/data
```
rostopic echo imu/data
```

## ROS parameters
* frame_id: the frame id
* use_temperature: publish the temperature data
* use_magnetometer: publish the magnetic field
* frequency: set the publishing frequency

## Published topics
* imu/data (sensor_msgs/Imu)
    - Includes linear accelerations and angular velocities and orientations.
* imu/magnetometer (sensor_msgs/MagneticField)
    - The magnetic orientation vector
* imu/temperature (sensor_msgs/Temperature)
    - Temperature ºC
