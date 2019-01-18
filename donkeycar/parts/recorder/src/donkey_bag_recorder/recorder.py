import datetime
import os
import rosbag
import rospy
import sensor_msgs.msg
import std_msgs.msg
import geometry_msgs.msg


class BagRecorderException(Exception):
    pass


class BagRecorder:
    drive_msg = geometry_msgs.msg.Twist()
    out_bag = None

    def __init__(self):
        self.bag_dir = os.path.expanduser(rospy.get_param('~bag_dir'))

        # initiate subscribers
        rospy.Subscriber('/donkey/image', sensor_msgs.msg.CompressedImage,
                         self.image_msg_cb_)
        rospy.Subscriber('/donkey/drive', geometry_msgs.msg.Twist,
                         self.drive_msg_cb_)
        rospy.Subscriber('/donkey/record', std_msgs.msg.Bool,
                         self.record_toggle_cb_)


    def drive_msg_cb_(self, msg):
        self.drive_msg = msg

    def image_msg_cb_(self, image_msg):
        """
        Write the image message and the latest drive message to the
        bagfile only when new messages are received, and use the image
        message's stamp for both to ensure they are synchronized. This
        will make extracting labeled data for training vastly simpler.
        """
        if self.out_bag is not None:
            stamp = image_msg.header.stamp
            self.out_bag.write('/donkey/image', image_msg, stamp)
            self.out_bag.write('/donkey/drive', self.drive_msg, stamp)

    def record_toggle_cb_(self, msg):
        if msg.data: 
            if self.out_bag is None:
                ts = datetime.datetime.fromtimestamp(rospy.Time.now().to_sec())
                bag_name = os.path.join(self.bag_dir,
                                        ts.strftime('%Y-%m-%d-%H-%M-%S.bag'))
                rospy.loginfo('Opening bagfile for writing at %s' % bag_name)
                self.out_bag = rosbag.Bag(bag_name, mode='w')
        else:
            rospy.loginfo('Closing bagfile')
            self.out_bag.flush()
            self.out_bag.close()
            self.out_bag = None

    def __del__(self):
        if self.out_bag is not None:
            self.out_bag.flush()
            self.out_bag.close()
            self.out_bag = None
