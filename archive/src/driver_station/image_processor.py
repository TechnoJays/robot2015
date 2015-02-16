"""This module is the tcp client that gets targets and sends them to a robot.

NOTE: THIS RUNS ON THE DRIVER STATION, NOT ON THE ROBOT.

DO NOT UPLOAD TO THE ROBOT!!

"""

import json_helper
import logging
import socket
import sys
import target
import targeting
import time


class ImageProcessor(object):
    """Gets targets and sends them to a tcp server."""

    _logger = None
    _targeting = None

    def __init__(self, port=1180, log_handler=None):
        """Initialize the image processor."""
        self._logger = logging.getLogger(__name__)
        handler = None
        if log_handler:
            handler = log_handler
        else:
            formatter = logging.Formatter('%(asctime)s - %(levelname)s:'
                                          '%(name)s:%(message)s')
            handler = logging.StreamHandler(stream=sys.stdout)
            handler.setLevel(logging.DEBUG)
            handler.setFormatter(formatter)
        self._logger.addHandler(handler)
        self._logger.setLevel(logging.DEBUG)

        self.port = port
        self._sock = None
        self._targeting = targeting.Targeting()

    def process(self):
        """Gets images and sends them to the server."""
        robot_connected = False
        camera_connected = False
        # Loop continuously
        while True:
            # Try to connect to the robot
            if not robot_connected:
                try:
                    self._logger.info("Attempting to connect to robot...")
                    if self._sock == None:
                        # Create a socket (SOCK_STREAM means a TCP socket)
                        self._sock = socket.socket(socket.AF_INET,
                                                   socket.SOCK_STREAM)
                    # Connect to server and send data
                    address = ("10.0.94.2", self.port)
                    self._sock.connect(address)
                    robot_connected = True
                except KeyboardInterrupt:
                    raise
                except Exception as excep:
                    self._logger.warn("Robot connection failed: " + str(excep))
                    self._sock = None
                    robot_connected = False
            # Try to connect to the camera
            if not camera_connected:
                self._logger.info("Attempting to connect to the camera...")
                if not self._targeting.open():
                    self._logger.warn("Camera connection failed.")
                    camera_connected = False
                else:
                    camera_connected = True
            # If one of the two isn't connected, we can't continue yet
            if not camera_connected or not robot_connected:
                self._logger.info("Retrying in 5 seconds.")
                time.sleep(5)
                continue
            # Both connections are active; time to get to work
            self._logger.info("Connected to both! Processing targets...")
            # Loop as long as we're connected
            while True:
                try:
                    data = None
                    # Get an image from the webcam and process it into a List of
                    # Targets
                    targets = self._targeting.get_targets()
                    if not targets:
                        self._logger.debug("Got None from get_targets()")
                        targets = []
                    # If there weren't any targets, create a 'no targets' object
                    if len(targets) <= 0:
                        self._logger.debug("No targets found.")
                        no_target = target.Target()
                        no_target.no_targets = True
                        targets.append(no_target)
                    # Convert Target list to JSON and send it to the robot
                    data = json_helper.to_json(targets)
                    self._logger.debug("Sending: " + str(data))
                    if data:
                        # Python3
                        #self._sock.send(bytes(data + '\n', "utf-8"))
                        # Python2
                        self._sock.send(bytes(data + '\n'))
                # If anything fails, bail out and try to reconnect
                except KeyboardInterrupt:
                    raise
                except Exception as excep:
                    self._logger.error("Connection error, disconnected: " +
                                       str(excep))
                    if self._sock:
                        self._sock.close()
                    #self._targeting.close()
                    self._sock = None
                    break
                # Wait before getting new targets
                #time.sleep(0.2)
            # Wait before trying to reconnect
            time.sleep(1)

# This lets us run this as a script
if __name__ == '__main__':
    # Create the Image Processor and start it
    iproc = ImageProcessor()
    iproc.process()

