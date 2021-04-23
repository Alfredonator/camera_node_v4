#!/usr/bin/env python

import socket
from communication_utils import CommunicationUtils


class CommunicationHandlerCamera:
    BUFFER_SIZE = 32768
    is_free = True
    host = "127.0.0.1"
    port = 5001

    def __init__(self):
        self.s = socket.socket()
        self.s.connect((self.host, self.port))
        print("Connected to ", self.host, ": ", self.port)

    def __del__(self):
        self.s.close()

    def get_formatted_detections(self, color_image):
        to_send_bytes = CommunicationUtils.encode(color_image)
        received_bytes = self._request_detections(to_send_bytes)

        objects_list = CommunicationUtils.decode(received_bytes)

        return objects_list

    def _request_detections(self, bytes_):
        self.s.sendall(CommunicationUtils.get_object_length_in_bytes(bytes_))
        print("Bytes_ length is ", {len(bytes_)})
        # print("Size of size encoded ", {len(CommunicationUtils.get_object_length_in_bytes(bytes_))})

        self._send_color_frame_bytes(bytes_)

        detections_bytes = self.s.recv(self.BUFFER_SIZE)

        return detections_bytes

    def _send_color_frame_bytes(self, bytes_):
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


