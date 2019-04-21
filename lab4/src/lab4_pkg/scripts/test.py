from lab4_pkg.msg import SoftGripperState, SoftGripperCmd
# from geometry_msgs.msg import Twist
import rospy

rospy.init_node('main', anonymous=True)
pub = rospy.Publisher('soft_gripper_cmd', SoftGripperCmd, queue_size=10)
msg = SoftGripperCmd(120,120)
# msg = Twist()
rospy.sleep(1)
print msg
pub.publish(msg)