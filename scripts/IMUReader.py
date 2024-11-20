# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055


class IMUReaderPi:
    def __init__(self):
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.last_val = 0xFFFF
        localTime = time.strftime('%Y-%m-%d, %H-%M-%S')

        self.file = open('./IMUData/IMU_Data ' + localTime + '.txt', 'w')
        self.file.write("Temp (c), Acc\n")
        rospy.loginfo("This is a log message from my_node")
    
    def temperature(self):
        result = self.sensor.temperature
        if abs(result - self.last_val) == 128:
            result = self.sensor.temperature
            if abs(result - self.last_val) == 128:
                return 0b00111111 & result
        self.last_val = result
        return result
    
    def run(self):
        sys, gyro, accel, mag = self.sensor.calibration_status
        print("calibration status: {}".format(self.sensor.calibration_status))
        self.file.write("(" + str(self.temperature()) +")\t" + \
                        str(self.sensor.acceleration) + "\t" + \
                        str(self.sensor.magnetic) +"\t" + \
                        str(self.sensor.gyro)+ "\t" + \
                        str(self.sensor.euler)+"\t" + \
                        str(self.sensor.quaternion)+"\t" + \
                        str(self.sensor.linear_acceleration)+"\t" + \
                        str(self.sensor.gravity)+ "\t("+ \
                        str(sys)+",\t"+str(gyro)+",\t"+str(accel)+",\t"+str(mag)+ ")\t(" +  \
                        str(time.time()) +")\t(\n"
                        )


if __name__ == '__main__':
    try:
        rospy.init_node('imu_reader', anonymous=True)
        rate_param = rospy.get_param('~rate', 100.0)
        rate = rospy.Rate(rate_param)
        
        imu_reader = IMUReaderPi()

        while not rospy.is_shutdown():
            imu_reader.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass