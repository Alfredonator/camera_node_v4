from data_models.detected_object import DetectedObject
from data_models.intrinsics import Intrinsics
from data_models.detected_box import Container3D


class Utils:

    @staticmethod
    def _project_pixels_to_phys_coord(x_pixel, y_pixel, depth, camera_info):
        # converting to right unit (0.001 m)
        depth = depth / 1000
        intrinsics = Intrinsics(camera_info.K[2], camera_info.K[5], camera_info.K[0], camera_info.K[4])

        x_coordinate = depth * ((x_pixel - intrinsics.ppx) / intrinsics.fx)
        y_coordinate = depth * ((y_pixel - intrinsics.ppy) / intrinsics.fy)

        return round(x_coordinate, 3), round(y_coordinate, 3), round(depth, 3)

    @staticmethod
    def _get_object_center_pixel_coordinates(detection_box, camera_info):
        x_min = float(detection_box['x_min'])
        x_max = float(detection_box['x_max'])
        y_min = float(detection_box['y_min'])
        y_max = float(detection_box['y_max'])

        print("boxes: ", x_min, y_min, x_max, y_max)
        # camera_info shows info about RGB stream which has higher resolution than depth,
        # thus resolution has to be explicitly set to 640 x 480
        # camera_resolution_height = camera_info.height
        # camera_resolution_width = camera_info.width

        camera_resolution_height = 640
        camera_resolution_width = 480

        mid_x = (x_max - x_min) * camera_resolution_width
        mid_y = (y_max - y_min) * camera_resolution_height

        return mid_x-1, mid_y-1

    @staticmethod
    def get_objects_3d_coordinates(detections, depth_image, camera_info):
        detected_objects = []

        for detection in detections:
            (mid_x_pixel, mid_y_pixel) = Utils._get_object_center_pixel_coordinates(detections[detection]['detection_box'],
                                                                                    camera_info)
            mid_x_pixel = int(mid_x_pixel)
            mid_y_pixel = int(mid_y_pixel)
            obj_class = detection
            obj_score = detections[detection]['detection_score']

            (x, y, z) = Utils._project_pixels_to_phys_coord(x_pixel=mid_x_pixel,
                                                            y_pixel=mid_y_pixel,
                                                            depth=depth_image[mid_x_pixel, mid_y_pixel],
                                                            camera_info=camera_info)

            detected_object = DetectedObject(x_middle=x,
                                             y_middle=y,
                                             z_middle=z,
                                             obj_class=obj_class,
                                             obj_score=obj_score)

            detected_objects.append(detected_object)

        return detected_objects

    @staticmethod
    def get_boxes_3d_coordinates(box_2d_detections, depth_image, camera_info):
        detected_boxes_3d = []

        for box_2d in box_2d_detections:
            x, y, z = Utils._project_pixels_to_phys_coord(x_pixel=box_2d.center[0],
                                                          y_pixel=box_2d.center[1],
                                                          depth=depth_image[box_2d.center[0], box_2d.center[1]],
                                                          camera_info=camera_info)
            detected_box_3d = Container3D(x, y, z, box_2d.container_name)
            detected_boxes_3d.append(detected_box_3d)

        return detected_boxes_3d
