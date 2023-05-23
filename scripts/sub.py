#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rs_yolo.msg import bbox

def doMsg(msg):
    print("recive data")
    rospy.loginfo("point: %f, %f, %f", msg.bbox[0], msg.bbox[1], msg.bbox[2])

if __name__ == "__main__":
    rospy.init_node("SUB")
    sub = rospy.Subscriber("/detect_result_out", bbox, doMsg, queue_size = 10)
    rospy.spin()