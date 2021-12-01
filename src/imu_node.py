#!/usr/bin/env python3
 
import rospy
import board
import busio
import adafruit_bno055

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField
 
class SensorIMU:
 
   def __init__(self):
 
       # Init node
       rospy.init_node('imu_bno055', anonymous=False)
 
       # Get node name
       self.node_name = rospy.get_name()
 
       self.get_ros_params()
 
       # Create topics
       self.pub_imu_data = rospy.Publisher('imu/data', Imu, queue_size=1)
 
       if self.use_magnetometer == True:
           self.pub_imu_magnetometer = rospy.Publisher('imu/magnetometer', MagneticField, queue_size=1)
      
       if self.use_temperature == True:
           self.pub_imu_temperature = rospy.Publisher('imu/temperature', Temperature, queue_size=1)
 
       # Connect to IMU via I2C connetion
       i2c = busio.I2C(board.SCL, board.SDA)
       self.sensor = adafruit_bno055.BNO055_I2C(i2c)
 
       # Internal variables
       self.imu_data_seq_counter = 0
       self.imu_magnetometer_seq_counter = 0
       self.imu_temperature_seq_counter = 0
 
   def get_ros_params(self):
 
       self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'imu_link')
       self.frequency = rospy.get_param(self.node_name + '/frequency', 5)
       self.use_magnetometer = rospy.get_param(self.node_name + '/use_magnetometer', False)
       self.use_temperature = rospy.get_param(self.node_name + '/use_temperature', False)
 
 
   def get_quat(self):
       return self.sensor.quaternion
 
   def get_gyro(self):
       return self.sensor.gyro
 
   def get_accel(self):
       return self.sensor.acceleration
 
   def get_mag(self):
       return self.sensor.magnetic
 
   def get_temp(self):
       return self.sensor.temperature
 
   def publish_imu_data(self):
       imu_data = Imu() 
       imu_data.header.stamp = rospy.Time.now()
       imu_data.header.frame_id = self.frame_id
       imu_data.header.seq = self.imu_data_seq_counter
 
       quaternion = self.get_quat()
       imu_data.orientation.w = quaternion[0]
       imu_data.orientation.x = quaternion[1]
       imu_data.orientation.y = quaternion[2]
       imu_data.orientation.z = quaternion[3]
 
       linear_acceleration = self.get_accel()
       imu_data.linear_acceleration.x = linear_acceleration[0]
       imu_data.linear_acceleration.y = linear_acceleration[1]
       imu_data.linear_acceleration.z = linear_acceleration[2]
 
       gyroscope = self.get_gyro()
       imu_data.angular_velocity.x = gyroscope[0]
       imu_data.angular_velocity.y = gyroscope[1]
       imu_data.angular_velocity.z = gyroscope[2]
 
       imu_data.orientation_covariance[0] = -1
       imu_data.linear_acceleration_covariance[0] = -1
       imu_data.angular_velocity_covariance[0] = -1
 
       self.imu_data_seq_counter=+1
 
       self.pub_imu_data.publish(imu_data)
      
 
   def publish_imu_magnetometer(self):
       imu_magnetometer = MagneticField()
 
       imu_magnetometer.header.stamp = rospy.Time.now()
       imu_magnetometer.header.frame_id = self.frame_id
       imu_magnetometer.header.seq = self.imu_magnetometer_seq_counter
 
       magnetometer = self.get_mag()
       imu_magnetometer.magnetic_field.x = magnetometer[0]
       imu_magnetometer.magnetic_field.y = magnetometer[1]
       imu_magnetometer.magnetic_field.z = magnetometer[2]
 
       self.imu_magnetometer_seq_counter=+1
 
       self.pub_imu_magnetometer.publish(imu_magnetometer)
 
   def publish_imu_temperature(self):
 
       imu_temperature = Temperature()
 
       imu_temperature.header.stamp = rospy.Time.now()
       imu_temperature.header.frame_id = self.frame_id
       imu_temperature.header.seq = self.imu_temperature_seq_counter
 
       imu_temperature.temperature = self.get_temp()
 
       self.imu_temperature_seq_counter=+1
 
       self.pub_imu_temperature.publish(imu_temperature)
 
 
   def run(self):
       # Set frequency
       rate = rospy.Rate(self.frequency)
 
       while not rospy.is_shutdown():
           self.publish_imu_data()
 
           # Publish magnetometer data
           if self.use_magnetometer == True:
               self.publish_imu_magnetometer()
 
           # Publish temperature data               
           if self.use_temperature == True:
               self.publish_imu_temperature()
 
           rate.sleep()
 
 
if __name__ == '__main__':
 
   imu = SensorIMU()
 
   try:
       imu.run()
 
   except rospy.ROSInterruptException:
       pass

