import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pyzbar import pyzbar

NORTH = 0
SOUTH = 180
EAST  = 270
WEST  = 90

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

	success = self.move_base.wait_for_result(rospy.Duration(60 * 5)) 

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

QR_WIDTH = 800
QR_HEIGHT = QR_WIDTH
class QRReader:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "camera/rgb/image_raw",
            Image,
            self.camera_callback
        )
        cv2.namedWindow("QrViewer", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("QrViewer", QR_WIDTH, QR_HEIGHT)
        cv2.waitKey(2)

    def camera_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        except Exception as e:
            print(e)

    def read(self):
        img = cv2.resize(self.image, (QR_WIDTH, QR_HEIGHT))
        codes = pyzbar.decode(img)
        if len(codes) > 0:  
            return codes[0].data.decode("utf-8")
        else:
            return -1


    def QR_show(self):
        print("Showing QR code")
        img = cv2.resize(self.image, (QR_WIDTH, QR_HEIGHT))
        codes = pyzbar.decode(img)
        for code in codes:
            (x, y, w, h) = code.rect
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
            qrdata = code.data.decode("utf-8")
            qrtype = code.type
            text = "{} ({})".format(qrdata, qrtype)
            cv2.putText(
                img,
                text,
                (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2
            )
            cv2.imshow("QrViewer", img)
            print("Found type: %s data: %s" % (qrtype, qrdata))


if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        qrviewer = QRReader()

        markers = []

        markers.append(MarkerPosition({'x': -4.14,  'y' : -4.63}, NORTH))
        markers.append(MarkerPosition({'x': 1.35, 'y' : 2.61}, NORTH))
        markers.append(MarkerPosition({'x': 4.84, 'y' : 0.79}, NORTH))
        markers.append(MarkerPosition({'x': 3.15, 'y' : -4.80}, NORTH))
        markers.append(MarkerPosition({'x': 10.15, 'y' : -8.30}, NORTH))

        markers.append(MarkerPosition({'x': 3.18, 'y' : -2.74}, WEST))
        markers.append(MarkerPosition({'x': 6.94, 'y' : 8.30}, WEST))

        markers.append(MarkerPosition({'x': 4.66, 'y' : 4.90}, SOUTH))
        markers.append(MarkerPosition({'x': -4.43, 'y' : 4.57}, SOUTH))

        markers.append(MarkerPosition({'x': 6.28, 'y' : -8.25}, EAST))

        markers.append(MarkerPosition({'x': -6.67,  'y' : -4.33}, NORTH)) # origin

        rospy.sleep(5)
        rospy.loginfo("Starting navigator...")

        qr_order = []

        #qrviewer.QR_show()

        for marker in markers:
            rospy.loginfo("Go to (%s, %s) pose", marker.position['x'], marker.position['y'])
            success = navigator.goto(marker.position, marker.angle)
            rospy.loginfo("Reached goal, reading QR code..")
            qrviewer.QR_show()
            #qr_val = qrviewer.read()
            #print("qr_val: %s" % qr_val)
            #qr_order.append(qr_val)
            rospy.sleep(5)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        print(", ".join(str(x) for x in qr_order))
        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

