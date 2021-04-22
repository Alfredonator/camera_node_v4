#!/usr/bin/env python

import json
import socket
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


class CommunicationHandlerCamera:
    BUFFER_SIZE = 32768
    is_free = True
    host = "127.0.0.1"
    port = 5002

    def __init__(self):
        self.s = socket.socket()
        self.s.connect((self.host, self.port))
        print("Connected to ", self.host, ": ", self.port)

    def __del__(self):
        self.s.close()

    def _request_detections(self, bytes_):
        self.s.sendall(CommunicationUtils.get_object_length_in_bytes(bytes_))
        print("Bytes_ length is ", {len(bytes_)})
        print("Size of size encoded ", {len(CommunicationUtils.get_object_length_in_bytes(bytes_))})

        _counter = 0

        while True:
            if len(bytes_) - _counter < 0:
                # print("breaking")
                break

            if len(bytes_) - (_counter + self.BUFFER_SIZE) < 0:
                to_be_send = bytes_[_counter:]
                self.s.sendall(to_be_send)
                # print("sending bytes from ", {_counter}, "to ", {_counter + (len(bytes_) - _counter)})
            else:
                to_be_send = bytes_[_counter:(_counter + self.BUFFER_SIZE)]
                self.s.sendall(to_be_send)
                # print("sending bytes from, ", {_counter}, "to ", {_counter + self.BUFFER_SIZE})

            _counter += self.BUFFER_SIZE

        detections_bytes = self.s.recv(self.BUFFER_SIZE)
        print("received detections: ", {detections_bytes})

        return detections_bytes
    
    def get_formatted_detections(self, color_image):
        to_send_bytes = CommunicationUtils.encode(color_image)
        received_bytes = self._request_detections(to_send_bytes)

        objects_list = CommunicationUtils.decode(received_bytes)

        return objects_list

