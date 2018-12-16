# import Adafruit_PCA9685
import rospy
from geometry_msgs.msg import Twist
from donkey_actuator.msg import Servo


def map_value_to_pwm_(servo_config, value):
    if value < -1.0001 or value > 1.0001:
        rospy.logerr('({}) value must be between -1.0 and 1.0'.format(value))
        return 0
    pwm = servo_config.get('direction') * \
        ((servo_config.get('range') / 2.) * value) + servo_config.get('center')
    return int(pwm)


class Actuator:
    def __init__(self):
        self.servos = rospy.get_param('servos')
        # self.pwm = Adafruit_PCA9685.PCA9685()
        # self.pwm.set_pwm_freq(self.servos.get('pwm_frequency'))

        # initiate subscribers
        self.abs_sub = rospy.Subscriber('servo_absolute', Servo,
                                        self.servo_absolute_cb_)
        self.drive_sub = rospy.Subscriber('servos_drive', Twist,
                                          self.servos_drive_cb_,
                                          queue_size=10)

    def servo_absolute_cb_(self, msg):
        """
        Callback function for the servo_absolute topic. Intended as a utility
        to help properly configure a servo's range of pulse values

        the following messages are an example of finding the steering servo's center, i.e. 333
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 300}"
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 350}"
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 330}"
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 335}"
        $ rostopic pub servo_absolute donkey_actuator/Servo "{name: steering, value: 333}"
        """
        config = self.servos.get(msg.name)
        # self.pwm.set_pwm(config.get('channel'), 0, int(msg.value))
        rospy.loginfo('servo: {}, channel: {}, value: {}'.format(
                      msg.name, config.get('channel'), int(msg.value)))

    def servos_drive_cb_(self, msg):
        self.set_servo_proportional_('steering', msg.angular.z)
        self.set_servo_proportional_('throttle', msg.linear.x)

    def set_servo_proportional_(self, servo_name, value):
        config = self.servos.get(servo_name)
        pulse = map_value_to_pwm_(config, value)
        # self.pwm.set_pwm(config.get('channel'), 0, pulse)
        rospy.loginfo(
            'servo: {}, channel: {}, value: {}, pulse: {}'.format(
            servo_name, config.get('channel'), value, pulse))
