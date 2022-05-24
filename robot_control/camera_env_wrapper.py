import cv2

from camera import CameraThread

def get_cameras():
    max_tested = 100
    for i in range(max_tested):
        temp_camera = cv2.VideoCapture(i)
        if temp_camera.isOpened():
            temp_camera.release()
            continue
        return i


class CameraEnv:
	def __init__(self, env):
		self.env = env
		self.camera_list = [CameraThread() for i in range(10)] #example

	def get_observation(self):
		camera_obs = [c.get_image() for c in self.camera_list]

		obs = self.env.get_observation() #{ee_pos, ee_vel, etc}




