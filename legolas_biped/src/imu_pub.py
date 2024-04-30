#!/usr/bin/env python3
import time
import board
import adafruit_mpu6050
from legolas_biped.msg import imu_readings
import rospy

class IMU_Biped:
    def __init__(self) -> None:
        rospy.init_node('imu')

        i2c = board.I2C()  # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        self.mpu = adafruit_mpu6050.MPU6050(i2c)

        pub_topic = 'imu_readings'
        self.imu_pub = rospy.Publisher(pub_topic, imu_readings, queue_size=1)

        self.imu_msg = imu_readings()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.read_imu()
            rate.sleep()

    def read_imu(self):
        ax = self.mpu.acceleration[0]
        ay = self.mpu.acceleration[1]
        az = self.mpu.acceleration[2]

        gx = self.mpu.gyro[0]
        gy = self.mpu.gyro[1]
        gz = self.mpu.gyro[2]

        self.imu_msg.Acceleration_X = ax
        self.imu_msg.Acceleration_Y = ay
        self.imu_msg.Acceleration_Z = az
        self.imu_msg.Gyro_X = gx
        self.imu_msg.Gyro_X = gy
        self.imu_msg.Gyro_X = gz

        self.imu_pub.publish(self.imu_msg)



if __name__ == '__main__':    
    IMU_Biped()
    # print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
    # print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % (mpu.gyro))
    # print("Temperature: %.2f C" % mpu.temperature)