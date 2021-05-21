import json
import cv2


class CommunicationUtils:
    @staticmethod
    def encode(color_image):
        _, color = cv2.imencode('.JPG', color_image)

        return color

    @staticmethod
    def decode(bytes_):
        json_ = bytes_.decode()

        return json.loads(json_)

    @staticmethod
    def get_object_length_in_bytes(bytes_):
        return str(len(bytes_)).encode()
