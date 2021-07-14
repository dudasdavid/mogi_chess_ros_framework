#! /usr/bin/env python3

import time
import serial

from thread_wrapper import periodic

import rospy
from std_msgs.msg import String

class ChessClockNode:

    def __init__(self):
        self.data_to_send = String()
        self.side = "x"
        self.commandQueue = []
                    
    def main(self):                      
                           
        rospy.init_node('mogi_chess_clock')
        self.dataPub  = rospy.Publisher('/mogi_chess_clock/side', String, queue_size=1)
        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        self.serialPort = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.publishRate = float(rospy.get_param('~publish_rate', 10.0)) # in Hz
        self.serialRate = float(rospy.get_param('~serial_rate', 10.0))   # in Hz

        self.ser = serial.Serial(
            port=self.serialPort,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

        self.ser.close()
        self.ser.open()
        self.ser.isOpen()

        # Add an initial reset command
        self.commandQueue.append("RST")

        # there is a 0.01 wait time in USB comm thread
        self.commThread = periodic(self.chessClockCommThread, (1.0/self.serialRate)-0.01, "Comm")
        self.commThread.start()

        rate = rospy.Rate(self.publishRate)
        while not rospy.is_shutdown():
            self.publish()
            try:
                rate.sleep()
            except:
                print("Ctrl+C occured")

        print("Stopped")

        self.commThread.exit()
        self.ser.close()

    def publish(self):

        self.data_to_send.data = self.side
        self.dataPub.publish(self.data_to_send)


    def chessClockCommThread(self):
     
        try:
            if len(self.commandQueue) == 0:
                self.commandQueue.append("RDS")
            
            command = self.commandQueue.pop(0)

            #print(command)

            self.ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
            
            out = b''
            time.sleep(0.008)
            while self.ser.inWaiting() > 0:
                out += self.ser.read(1)

            if out != '':
                #print(">> %s" % out)
                out_splitted = str(out).split(";")
                if out_splitted[0] == "b'OK":
                    #print(out_splitted)
                    message = out_splitted[1].split('\\r')[0]
                    if message == "RST":
                        print(">>SUCCESSFUL RESET")
                    elif message == "1" or message == "0":
                        if message == "1":
                            self.side = "w"
                        else:
                            self.side = "b"

                    else:
                        print(">>INVALID COMMAND: %s" % out)
                else:
                    pass
                    print(">>NOT OK: %s" % out)
        except:
            print("exception")

if __name__ == '__main__':
    try:
        node = ChessClockNode()
        node.main()
    except rospy.ROSInterruptException:
        pass