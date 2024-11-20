#!/usr/bin/env python3

from typing import Any, Union
import rospy
from std_msgs.msg import String
import json


class Publisher(rospy.Publisher):
    def __init__(
            self, 
            name: str,
            subscriber_listener: Union[Any , None] = None,
            tcp_nodelay: bool = False,
            latch: bool = False,
            headers: Union[Any , None] = None,
            queue_size: Union[Any , None] = None):
        super().__init__(name=name, data_class=String, subscriber_listener=subscriber_listener, tcp_nodelay=tcp_nodelay, latch=latch, headers=headers, queue_size=queue_size)

    def publish(self, message: dict):
        super().publish(json.dumps(message))


class Subscriber(rospy.Subscriber):
    def __init__(
            self, 
            name: str,
            callback: Union[Any , None] = None,
            callback_args: Union[Any , None] = None,
            queue_size: Union[Any , None] = None,
            tcp_nodelay: bool = False):
        

        def json_callback(message):
            callback(json.loads(message.data))


        super().__init__(name=name, data_class=String, callback=json_callback, callback_args=callback_args, queue_size=queue_size, tcp_nodelay=tcp_nodelay)


if __name__ == '__main__':
    
    def talker():
        pub = Publisher('chatter', queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate_param = rospy.get_param('~rate', 10.0)
        rate = rospy.Rate(rate_param)
        #rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            data = {"hello_str": hello_str, "random_int": 3242, "random_bool": False, "random_list": [1, 2, 3, "string", True]}
            json_data = json.dumps(data)
            rospy.loginfo(json_data)
            pub.publish(data)
            rate.sleep()

    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    """
    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

    def listener():

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        JsonSubscriber('chatter', callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    listener()
    
    """
    