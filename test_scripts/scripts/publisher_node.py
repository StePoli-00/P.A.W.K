#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String 


def talk_to_me():
    #1:name of topic #2: type of message #3: queue size of messages
    #pub=rospy.Publisher("talking_topic",String , queue_size=10) 
    pub=rospy.Publisher("talking_topic",position , queue_size=10)
    #initialize node
    #1 name of node, create unique numer for the node
    rospy.init_node("publisher_node",anonymous=True)
    #rate in Hertz=1/sec
    rate=rospy.Rate(1)
    rospy.loginfo("Publisher node started, now publishing messages")
    #until the node is running
    while not rospy.is_shutdown():
        #msg="Hello Stefano %s" % rospy.get_time()
        msg=position()
        msg.message="My position is: "
        msg.x=2.0
        msg.y=1.5
        #publish the message
        pub.publish(msg)
        rate.sleep()


if __name__=="__main__":
    try: 
        talk_to_me()
    except rospy.ROSInterruptException:
        pass 