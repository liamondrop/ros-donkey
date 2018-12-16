import Adafruit_PCA9685
import rospy
import time

from donkey_actuator.msg import Servo
from geometry_msgs.msg import Twist


def map_value_to_pulse_(servo, value):
    if value < -1.0001 or value > 1.0001:
        rospy.logerr('({}) value must be between -1.0 and 1.0'.format(value))
        return 0
    pulse = servo['direction'] * 0.5 * servo['range'] * value + servo['center']
    return int(pulse)


class Actuator:
    def __init__(self):
        self.servos = rospy.get_param('servos')
        self.controller = Adafruit_PCA9685.PCA9685()
        self.controller.set_pwm_freq(self.servos.get('pwm_frequency'))

        # send center pulse to throttle servo to calibrate ESC
        self.set_servo_center_(self.servos['throttle'])
        time.sleep(1)

        # initiate subscribers
        rospy.Subscriber('servo_absolute', Servo, self.servo_absolute_cb_)
        rospy.Subscriber('servos_drive', Twist, self.servos_drive_cb_,
                         queue_size=10)

    def servo_absolute_cb_(self, msg):
        """
        Callback function for the servo_absolute topic. Intended as a utility
        to help properly configure a servo's range of pulse values.

        The following are an example of finding the steering servo's center, i.e. 333:
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 300}"
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 350}"
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 330}"
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 335}"
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 333}"
        """
        servo = self.servos[msg.name]
        self.set_servo_pulse_(servo, int(msg.value))
        rospy.loginfo('servo: {}, channel: {}, value: {}'.format(
            msg.name, servo['channel'], value))

    def servos_drive_cb_(self, msg):
        """
        Callback function for the servos_drive topic

        Sets the values for the steering and throttle servos using s standard
        geometry_msgs/Twist message. The linear.x component controls the
        throttle, and the angular.z component controls the steering.

        The following is an example of a command to drive straight forward
        at 75% throttle:
        $ rostopic pub servos_drive geometry_msgs/Twist "{linear: {x: 0.75}, angular: {z: 0.0}}"
        """
        self.set_servo_proportional_(self.servos['steering'], msg.angular.z)
        self.set_servo_proportional_(self.servos['throttle'], msg.linear.x)

    def set_servo_center_(self, servo):
        self.set_servo_pulse_(servo, servo['center'])

    def set_servo_pulse_(self, servo, value):
        channel = servo['channel']
        self.controller.set_pwm(channel, 0, value)

    def set_servo_proportional_(self, servo, value):
        pulse = map_value_to_pwm_(servo, value)
        self.set_servo_pulse_(servo, pulse)

    def __del__(self):
        """
        Turn off servos when shutting down
        """
        self.set_servo_pulse_(self.servos['steering'], 0)
        self.set_servo_pulse_(self.servos['throttle'], 0)
