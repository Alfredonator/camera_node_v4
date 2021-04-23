#!/usr/bin/env python

import roslib
import rospy
import cv2
import message_filters
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from communication_handler import CommunicationHandlerCamera
from utils import Utils
from aruco_detector import ArucoDetector

roslib.load_manifest('camera_node_v4')


class ImageConverter:
    def __init__(self):
        print("[INFO]: Initializing MoveIt and ObjectDetector")
        # moveit_commander.roscpp_initialize(sys.argv)
        # self._scene = moveit_commander.PlanningSceneInterface()
        self._bridge = CvBridge()
        self._communication = CommunicationHandlerCamera()
        self._box_detector = ArucoDetector()
        print("[INFO]: Initializing MoveIt and ObjectDetector - completed")

        image_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=5)
        depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, queue_size=5)
        info_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size=5)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub, info_sub], 10, 0.5)
        ts.registerCallback(self.callback)

    def callback(self, rgb_data, depth_data, camera_info):

        try:
            color_image = cv2.resize(self._bridge.imgmsg_to_cv2(rgb_data, "bgr8"), (640, 480))
            depth_image = cv2.resize(self._bridge.imgmsg_to_cv2(depth_data, "32FC1"), (640, 480))
        except CvBridgeError as e:
            print(e)
            return

        # TODO: There is some problem with the depth frame. Most of pixels are showing depth 0.
        #  Potentially because camera is too close to the table. Needs to be investigated.
        print(depth_image)

        if not self._are_frames_valid(color_image, depth_image):
            return

        # get object and box detections
        object_detections = self._communication.get_formatted_detections(color_image)
        box_detections = self._box_detector.get_detected_boxes(color_image)

        # get 3D objects and 3D boxes list
        objects_3d_list = Utils.get_objects_3d_coordinates(object_detections, depth_image, camera_info)
        box_3d_list = Utils.get_boxes_3d_coordinates(box_detections, depth_image, camera_info)

        self._print_detections(objects_3d_list, box_3d_list)

        # add all 3D objects and all 3D boxes to moveit planning scene
        # self._update_planning_scene_with_detected_objects(objects_3d_list)
        # self._update_planning_scene_with_detected_boxes(box_3d_list)

    @staticmethod
    def _are_frames_valid(color_frame, depth_frame):
        if not color_frame.any() or not depth_frame.any():
            return False

        # row_color, col_color = color_frame.shape
        # row_depth, col_depth = depth_frame.shape
        #
        # if row_color != 480 or col_color != 640:
        #     return False
        #
        # if row_depth != 480 or col_depth != 640:
        #     return False

        return True

    @staticmethod
    def _print_detections(objects, boxes):
        for obj in objects:
            print(obj.obj_class, obj.obj_score, obj.x_middle, obj.y_middle, obj.z_middle)

        for box in boxes:
            print(box.container_name, box.x_middle, box.y_middle, box.z_middle)

    def _update_planning_scene_with_detected_objects(self, objects_list):
        object_pose = self._create_entity_pose_object()

        for identified_object in objects_list:
            object_pose.pose.position.x = identified_object.x_middle
            object_pose.pose.position.y = identified_object.y_middle
            object_pose.pose.position.z = identified_object.z_middle
            object_name = str(identified_object.obj_class)
            self._scene.add_box(object_name, object_pose, size=(0.05, 0.05, 0.05))

    def _update_planning_scene_with_detected_boxes(self, box_list):
        box_pose = self._create_entity_pose_object()

        for identified_box in box_list:
            print("Updating planing scene with box name, x, y, z", identified_box.container_name, identified_box.x, identified_box.y, identified_box.z)
            box_pose.pose.position.x = identified_box.x
            box_pose.pose.position.y = identified_box.y
            box_pose.pose.position.z = identified_box.z
            box_name = str(identified_box.container_name)
            self._scene.add_box(box_name, box_pose, size=(0.25, 0.1, 0.165))

    @staticmethod
    def _create_entity_pose_object():
        entity_pose = geometry_msgs.msg.PoseStamped()
        # entity_pose.header.frame_id = "panda_hand"
        entity_pose.header.frame_id = "camera_link"

        return entity_pose


if __name__ == '__main__':
    rospy.loginfo("Start")
    rospy.init_node('image_converter', anonymous=True)
    ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
