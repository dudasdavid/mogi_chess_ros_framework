#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

def chess_clock_callback(msg):
    data_to_send.data = "w"
    dataPub.publish(data_to_send)

initial_value = "w"

rospy.init_node('mogi_chess_clock')
dataPub  = rospy.Publisher('/mogi_chess_clock/side', String, queue_size=1)
data_to_send = String()
data_to_send.data = initial_value
rospy.Subscriber("/mogi_chess_clock/gazebo_trigger", String, chess_clock_callback)

while not rospy.is_shutdown():
    input("Press enter to send user clock press")
    if data_to_send.data == "w":
        data_to_send.data = "b"
    else:
        data_to_send.data = "w"

    print(f"Sending {data_to_send.data}!")
    dataPub.publish(data_to_send)
