import numpy as np
import os
import rospy
import tensorflow as tf
import geometry_msgs.msg
import sensor_msgs.msg

from tensorflow.python.keras.models import load_model
from donkey_keras.utils import image_deserialize



class KerasPilotException(Exception):
    pass


class KerasPilot:
    def __init__(self):
        if not rospy.has_param('~model_path'):
            raise KerasPilotException('Path to the model not given')

        model_path = os.path.expanduser(rospy.get_param('~model_path'))
        self.model = load_model(model_path)
        self.graph = tf.get_default_graph()
        self.drive_pub = rospy.Publisher('donkey/drive',
                                         geometry_msgs.msg.Twist,
                                         queue_size=10)
        self.image_sub = rospy.Subscriber('/donkey/image',
                                          sensor_msgs.msg.CompressedImage,
                                          self.image_cb_)

    def image_cb_(self, msg):
        img_arr = image_deserialize(msg.data)
        with self.graph.as_default():
            outputs = self.model.predict(img_arr.reshape((1,) + img_arr.shape))
        steering, throttle = outputs
        drive_msg = geometry_msgs.msg.Twist()
        drive_msg.angular.z = steering[0,0]
        drive_msg.linear.x = throttle[0,0]
        self.drive_pub.publish(drive_msg)
