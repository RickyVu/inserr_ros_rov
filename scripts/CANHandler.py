#!/usr/bin/env python3

'''
CANHandler Module

Subscribe Topics:

can.send
    "address": <hexadecimal>
    "data" <bytearrray>

Publish Topics:

can.receive.<arbitration_id>:
    "data" <bytearray>
    "extra" <dictionary>
	"timestamp" <float>

'''

import can
import rospy
import pubsub.json_message

class CANHandler:
    def __init__(self, baudrate):
        #self.pub = pubsub.json_message.Publisher('ethernet/send', queue_size=10)
        pubsub.json_message.Subscriber('inserr/can/thruster', self.message_listener)
        
        
        self.bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=baudrate)

    def message_listener(self, message):
        msg = can.Message(arbitration_id=message["address"], data=message["data"], is_extended_id=False)
        
        try:
            self.bus.send(msg, timeout=0.01)
        except Exception as e:
            print("Message not sent:", [e, msg])

    def run(pub):
        pass
        #msg = can.Bus().recv(0)
        
        """
        if msg is not None:
            data = msg.data
            data_array = [int.from_bytes(byte, byteorder='big') for byte in data]
            
            message={"type": "CAN", "address": msg.arbitration_id, "data": data_array}
            pub.publish(message)
        """

if __name__ == '__main__':
    try:
        rospy.init_node('can_handler', anonymous=True)
        rate_param = rospy.get_param('~rate', 10.0)
        baudrate_param = rospy.get_param('~baudrate', 2500000)
        rate = rospy.Rate(rate_param)
        
        can_handler = CANHandler(baudrate_param)

        while not rospy.is_shutdown():
            can_handler.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass