#!/usr/bin/env python

# to run camera: roslaunch realsense2_camera rs_camera.launch enable_infra:=true align_depth:=true
import os
import sys
import roslib
import rospy
import cv2
import message_filters
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from communication_handler import CommunicationHandlerCamera
from data_models.detected_box import Container3D
from utils import Utils
from aruco_detector import ArucoDetector

roslib.load_manifest('camera_node_v4')


class ImageConverter:
    _MESH_PATH = '/home/szymon/catkin_ws/src/camera_node_v4/src/box_mesh_v3.stl' if os.environ.get(
        'USER') == 'szymon' else '/home/ros/alfredo/edo_ws/src/camera_node_v4/src/box_mesh_v3.stl'
    _UPDATE_POSE_THRESHOLD = 0.05  # (in meters) / pose in the planning scene is updated, when value is exceeded

    previous_objects_poses = {}
    previous_boxes_poses = {}

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self._scene = moveit_commander.PlanningSceneInterface()

        self._communication = CommunicationHandlerCamera()
        self._box_detector = ArucoDetector()

        rospy.loginfo("Initializing MoveIt and detectors - completed")

        self._bridge = CvBridge()
        self.camera_base_name = "camera_top"
        image_sub = message_filters.Subscriber("/" + self.camera_base_name + "/color/image_raw", Image, queue_size=5)
        depth_sub = message_filters.Subscriber("/" + self.camera_base_name + "/aligned_depth_to_color/image_raw", Image,
                                               queue_size=5)
        info_sub = message_filters.Subscriber("/" + self.camera_base_name + "/aligned_depth_to_color/camera_info",
                                              CameraInfo, queue_size=5)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub, info_sub], 10, 0.5)
        ts.registerCallback(self.callback)

    def callback(self, rgb_data, depth_data, camera_info):
        try:
            color_image = self._bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_image = self._bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

        if not self._are_frames_valid(color_image, depth_image):
            return

        # get object and box detections
        object_detections = self._communication.get_formatted_detections(color_image)
        box_detections = self._box_detector.get_detected_boxes(color_image)
        self._visualize_object_detections(object_detections, color_image, camera_info)

        # get 3D objects and 3D boxes list
        objects_3d_list = Utils.get_objects_3d_coordinates(object_detections, depth_image, camera_info)
        box_3d_list = Utils.get_boxes_3d_coordinates(box_detections, depth_image, camera_info)
        # self._print_detections(objects_3d_list, box_3d_list)

        # add all 3D objects and all 3D boxes to moveit planning scene
        self._update_planning_scene_with_detected_objects(objects_3d_list)
        self._update_planning_scene_with_detected_boxes_mesh(box_3d_list)

    def _update_planning_scene_with_detected_objects(self, objects_list):
        object_pose = self._create_entity_pose_object()

        for identified_object in objects_list:
            if self._is_object_to_be_updated(identified_object):
                object_pose.pose.position.x = identified_object.x_middle - 0.05
                object_pose.pose.position.y = identified_object.y_middle
                object_pose.pose.position.z = identified_object.z_middle
                object_name = str(identified_object.obj_class)
                self._scene.add_box(object_name, object_pose, size=(0.05, 0.05, 0.05))

    def _update_planning_scene_with_detected_boxes_mesh(self, box_list):
        box_pose = self._create_entity_pose_object()

        for identified_box in box_list:
            if self._is_box_to_be_updated(identified_box):
                print("Updating planing scene with box name, x, y, z", identified_box.container_name, identified_box.x,
                      identified_box.y, identified_box.z)
                box_pose.pose.orientation.y = -0.707
                box_pose.pose.orientation.w = 0.707
                box_pose.pose.position.x = identified_box.x
                box_pose.pose.position.y = identified_box.y
                box_pose.pose.position.z = identified_box.z
                box_name = str(identified_box.container_name)
                self._scene.add_mesh(box_name, box_pose, self._MESH_PATH)

    def _create_entity_pose_object(self):
        entity_pose = geometry_msgs.msg.PoseStamped()
        entity_pose.header.frame_id = self.camera_base_name + "_link"

        return entity_pose

    def _is_object_to_be_updated(self, _object):
        if not self.previous_objects_poses.keys().__contains__(_object.obj_class):
            self.previous_objects_poses[_object.obj_class] = {'x': _object.x_middle,
                                                              'y': _object.y_middle}
            return True
        elif abs(self.previous_objects_poses[_object.obj_class]['x'] - _object.x_middle) > self._UPDATE_POSE_THRESHOLD \
                or abs(self.previous_objects_poses[_object.obj_class]['y'] - _object.y_middle) > self._UPDATE_POSE_THRESHOLD:

            self.previous_objects_poses[_object.obj_class]['x'] = _object.x_middle
            self.previous_objects_poses[_object.obj_class]['y'] = _object.y_middle
            return True
        else:
            return False

    def _is_box_to_be_updated(self, box):
        if not self.previous_boxes_poses.keys().__contains__(box.container_name):
            self.previous_boxes_poses[box.container_name] = {'x': box.x,
                                                             'y': box.y}
            return True
        elif abs(self.previous_boxes_poses[box.container_name]['x'] - box.x) > self._UPDATE_POSE_THRESHOLD or abs(
                self.previous_boxes_poses[box.container_name]['y'] - box.y) > self._UPDATE_POSE_THRESHOLD:

            self.previous_boxes_poses[box.container_name]['x'] = box.x
            self.previous_boxes_poses[box.container_name]['y'] = box.y
            return True
        else:
            return False

    @staticmethod
    def _are_frames_valid(color_frame, depth_frame):
        if not color_frame.any() or not depth_frame.any():
            return False

        return True

    @staticmethod
    def _visualize_object_detections(object_detections, color_image, camera_info):
        for detection in object_detections:
            x, y = Utils._get_object_center_pixel_coordinates(object_detections[detection]['detection_box'],
                                                              camera_info)
            color_image = cv2.circle(color_image, (int(x), int(y)), 1, (0, 0, 255), 5)

        cv2.imshow("Detections Visualisation", color_image)
        cv2.waitKey(1)

    @staticmethod
    def _print_detections(objects, boxes):
        for obj in objects:
            print(obj.obj_class, obj.obj_score, obj.x_middle, obj.y_middle, obj.z_middle)

        for box in boxes:
            print(box.container_name, box.x, box.y, box.z)


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    rospy.loginfo("Starting Camera Node v4")
    ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
