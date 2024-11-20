#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
from rplidar import RPLidar
import time
import rospy


PORT_NAME = '/dev/ttyUSB1'

class LidarReaderPi:
    def __init__(self, folder, storage_prefix):
        self.localtime = time.strftime('%Y-%m-%d, %H-%M-%S')
        
        self.lidar = RPLidar(PORT_NAME)
        #self.outfile = open("./LidarData/Lidar_Data " + self.localtime + '.txt', 'w')
        self.outfile = open(f"{folder}/{storage_prefix} {self.localtime}.txt", "w")
    
    def run(self):
    
        try:
            rospy.loginfo("Lidar Measuring")
            for measurment in self.lidar.iter_measures():
                line = '\t'.join(str(v) for v in measurment)
                self.outfile.write(line + "\t" + str(time.time()) + '\n')
                if rospy.is_shutdown():
                    break
        except KeyboardInterrupt:
            rospy.loginfo('lidar stop')
        self.cleanup()

    def cleanup(self):
        self.lidar.stop()
        self.lidar.disconnect()
        self.outfile.close()
            
        
if __name__ == '__main__':
    try:
        rospy.init_node('lidar_reader', anonymous=True)
        folder_param = rospy.get_param("~folder", "")
        storage_prefix_param = rospy.get_param("~storage_prefix", "Lidar_Data")
        
        lidar_reader = LidarReaderPi(folder_param, storage_prefix_param)
        lidar_reader.run() #From INSERR repo, I assume iter_measures measures repeatedly and does not end
    except rospy.ROSInterruptException:
        lidar_reader.cleanup()