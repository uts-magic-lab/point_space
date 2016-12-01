#!/usr/bin/env python

import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import sys
from math import radians, degrees
from moveit_msgs.msg import MoveItErrorCodes

# Build a useful mapping from MoveIt error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

"""
Give a pose stamped pose
and it will get the corresponding IK.


Author: Sammy Pfeiffer
"""


class GetIK(object):
    def __init__(self):
        rospy.loginfo("Initalizing GetIK...")
        self.ik_srv = rospy.ServiceProxy('/compute_ik',
                                         GetPositionIK)
        rospy.loginfo("Waiting for /compute_ik service...")
        self.ik_srv.wait_for_service()
        rospy.loginfo("Connected!")

    def get_ik(self, pose_stamped, group='right_arm_and_torso', duration=3.0,
               attempts=0):
        req = GetPositionIKRequest()
        req.ik_request.group_name = group
        # ik_link_name will be [r/l]_wrist_roll_link
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(duration)
        req.ik_request.attempts = attempts

        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))


if __name__ == '__main__':
    argv = sys.argv
    if len(sys.argv) not in [8, 9, 10]:
        print "Usage:"
        print argv[0] + " x y z roll pitch yaw frame_id (-rad)"  #  8, 9
        print argv[0] + " x y z qx qy qz qw frame_id (-rad)"  # 9, 10
        exit(0)

    print "Connecting to ROS node..."
    rospy.init_node('get_iker')
    gik = GetIK()

    use_radians = False
    if sys.argv[-1] == "-rad":
        use_radians = True
        argv = argv[:-1]

    # Asking for a pose transformation
    if len(argv) == 8 or len(argv) == 9:
        ps = PoseStamped()
        ps.pose.position.x = float(argv[1])
        ps.pose.position.y = float(argv[2])
        ps.pose.position.z = float(argv[3])

        if len(argv) == 8:
            roll = float(argv[4])
            pitch = float(argv[5])
            yaw = float(argv[6])
            if not use_radians:
                roll = radians(roll)
                pitch = radians(pitch)
                yaw = radians(yaw)
            q = quaternion_from_euler(roll, pitch, yaw)
            quat = Quaternion(*q)
            ps.pose.orientation = quat
            from_frame = argv[7]
        else:
            ps.pose.orientation.x = float(argv[4])
            ps.pose.orientation.y = float(argv[5])
            ps.pose.orientation.z = float(argv[6])
            ps.pose.orientation.w = float(argv[7])
            from_frame = argv[8]

        ps.header.frame_id = from_frame
        resp = gik.get_ik(ps)
        rospy.loginfo(str(resp))
        err_code = resp.error_code.val
        rospy.loginfo("Error is: " + moveit_error_dict[err_code])
