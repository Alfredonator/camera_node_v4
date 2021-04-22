import cv2
import cv2.aruco as aruco
from data_models.detected_box import Container2D


class ArucoDetector:
    def __init__(self):
        self.frame_count = 0
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        self.aruco_params = aruco.DetectorParameters_create()

    def get_detected_boxes(self, frame):
        container_kinds = {
            'Potato': {
                'aruco_left': 1,
                'aruco_right': 41
            },
            'Radish': {
                'aruco_left': 2,
                'aruco_right': 42
            },
            'Lemon': {
                'aruco_left': 3,
                'aruco_right': 43
            }
        }

        detected_containers = []

        (corners, detected_ids, rejected) = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if detected_ids is not None:
            detected_ids = list(detected_ids.flatten())

            for kind in container_kinds:
                if detected_ids.__contains__(container_kinds[kind]['aruco_left']) and detected_ids.__contains__(
                        container_kinds[kind]['aruco_right']):
                    detected_containers.append(Container2D(
                        self._get_container_center(corners[detected_ids.index(container_kinds[kind]['aruco_left'])],
                                                   corners[detected_ids.index(container_kinds[kind]['aruco_right'])],
                                                   frame), kind))

        return detected_containers

    def _get_container_center(self, aruco_left_square, aruco_right_square, frame):
        (aruco_left_top_left, aruco_left_top_right, aruco_left_bottom_right,
         aruco_left_bottom_left) = aruco_left_square.reshape((4, 2))
        (aruco_right_top_left, aruco_right_top_right, aruco_right_bottom_right,
         aruco_right_bottom_left) = aruco_right_square.reshape((4, 2))
        return self._calculate_box_center(aruco_left_bottom_right, aruco_right_top_left, frame)

    def _calculate_box_center(self, left_upper_corner, right_bottom_corner, frame):
        x, y = int((left_upper_corner[0] + right_bottom_corner[0]) / 2), int(
            (left_upper_corner[1] + right_bottom_corner[1]) / 2)
        cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)

        return x, y
