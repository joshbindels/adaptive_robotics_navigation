import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations
import math

class GoToPose():
    def __init__(self):

        self.goal_sent = False
	rospy.on_shutdown(self.shutdown)
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, angle):
        quat = transformations.quaternion_from_euler(0,0,math.radians(angle))
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat[0], quat[1], quat[2], quat[3]))

        self.move_base.send_goal(goal)

	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

class MarkerPosition:
    def __init__(self, position, angle):
        self.position = position
        self.angle = angle

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()

        markers = []
        markers.append(MarkerPosition({'x': -4.14,  'y' : -4.63}, 0))
        markers.append(MarkerPosition({'x': -6.67,  'y' : -4.33}, 90))
        markers.append(MarkerPosition({'x': -4.14,  'y' : -4.63}, 180))
        markers.append(MarkerPosition({'x': -6.67,  'y' : -4.33}, 270))

        """
        # orientation north
        markers.append(MarkerPosition({'x': 1.75,  'y' : -0.43}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))
        markers.append(MarkerPosition({'x': 7.35,  'y' : 6.90 }, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))
        markers.append(MarkerPosition({'x': 10.56, 'y' : 5.42 }, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))
        markers.append(MarkerPosition({'x': 8.92,  'y' : 0.04 }, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))
        markers.append(MarkerPosition({'x': 15.93, 'y' : -3.11}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))
        
        # orientation west
        markers.append(MarkerPosition({'x': 8.91, 'y' : 2.12  }, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))
        markers.append(MarkerPosition({'x': 12.30, 'y' : 12.58}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))

        # orientation east
        markers.append(MarkerPosition({'x': 12.37, 'y' : -3.29}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))

        #orientation south
        markers.append(MarkerPosition({'x': 1.78, 'y' : 8.84}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}))
        """


        for marker in markers:
            rospy.loginfo("Go to (%s, %s) pose", marker.position['x'], marker.position['y'])
            success = navigator.goto(marker.position, marker.angle)
            print("Reached goal, staring into the void for 5 seconds..")
            rospy.sleep(5)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

