import rospy


def main(node_name):
    rospy.init_node(node_name)

    rospy.spin()

    return 0
