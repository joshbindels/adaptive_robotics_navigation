import rospy
from geometry_msgs.msg import Twist

class TurtleHandler:
    def __init__(self):
        rospy.init_node("TurtleHandler", anonymous=False)
        rospy.loginfo("To stop TurtleHandler Ctrl + C")
        rospy.on_shutdown(self._shutdown)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.rate = rospy.Rate(10)

        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.2
        self.move_cmd.angular.z = 0

        self._main()

    def _main(self):
        while not rospy.is_shutdown():
            self.cmd_vel.publish(self.move_cmd)
            self.rate.sleep()

    def _shutdown(self):
        rospy.loginfo("Stopping TurtleHandler")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        TurtleHandler()
    except:
        rospy.loginfo("Failed to run turtlehandler")
        
