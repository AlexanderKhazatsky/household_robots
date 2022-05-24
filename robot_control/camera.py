import threading
import numpy as np
import cv2

"""
potential idea: pass in an acceptable delay
if its greater than this delay, we wait for the new image

potential idea: sync cameras to request feed at the same time
"""


class CameraThread:
	def __init__(self, camera):
		self.camera = camera
		self.latest_img = None

        camera_thread = threading.Thread(target=self.update_camera_feed)
        camera_thread.daemon = True
        camera_thread.start()

    def get_image(self):
    	return self.latest_img.copy()

	def update_camera_feed(self):
		while True:
			img = self.camera.get_image()
			self.latest_img = img

class Zed2Camera:
	def __init__(self, camera_id):
		# Configure Camera #
		self.cam = cv2.VideoCapture(camera_id)
		if self.cam.isOpened() == 0: return exit(-1)


	def get_image(self):
		return self.latest_img.copy()


	def read_camera(self):
		while True:


class RealSenseCamera:
	def __init__(self, camera_id):
		# Configure Camera #
		self.cam = cv2.VideoCapture(camera_id)
		if self.cam.isOpened() == 0: return exit(-1)




        # Camera Feed Thread #
        self.latest_img = None
        camera_thread = threading.Thread(target=self.read_camera)
        camera_thread.daemon = True
        camera_thread.start()


	def get_image(self):
		return self.latest_img.copy()


	def read_camera(self):
		while True: