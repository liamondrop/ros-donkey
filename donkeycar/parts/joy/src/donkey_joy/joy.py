import rospy

import sensor_msgs.msg
import geometry_msgs.msg

class Joy:
    def __init__(self):
        self.axis_linear = rospy.get_param('~axis_linear')
        self.axis_angular = rospy.get_param('~axis_angular')
        self.scale_linear = rospy.get_param('~scale_linear', 1.0)
        self.scale_angular = rospy.get_param('~scale_angular', 1.0)

        # initialize joy subscriber and drive publisher
        rospy.Subscriber('joy', sensor_msgs.msg.Joy, self.joy_cb_)
        self.drive_pub = rospy.Publisher('donkey/drive',
                                         geometry_msgs.msg.Twist,
                                         queue_size=10)

    def joy_cb_(self, msg):
        twist = geometry_msgs.msg.Twist()
        twist.angular.z = self.scale_angular * msg.axes[self.axis_angular]
        twist.linear.x = self.scale_linear * msg.axes[self.axis_linear]
        self.drive_pub.publish(twist)
