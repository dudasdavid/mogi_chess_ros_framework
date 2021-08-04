#! /usr/bin/env python3.8

import rospy
from std_msgs.msg import String

def chess_clock_callback(msg):
    if msg.data.split(";")[1] == "w":
        data_to_send.data = "b"
    elif msg.data.split(";")[1] == "b":
        data_to_send.data = "w"
    else:
        print(f"ERROR - msg.data: {msg.data} contains invalid side")
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
