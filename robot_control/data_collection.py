from controller import VRPolicy

from datetime import date
import time
import os


class DataCollecter:

	def __init__(self):
		self.env = None
		self.num_cameras = self.env.get_num_cameras()
		self.action_noise = None
		self.controller = VRPolicy()
		self.traj_num = 0
		self.logdir = '/Users/sasha/Desktop/irisnet/{0}/'.format(date.today())

		# Data Variables #
		self.traj_name = None
		self.traj_info = None
		self.traj_data = None
		self.traj_saved = None

		return None


	def reset_robot(self):
		self.env.reset()

	def get_user_feedback(self):
		info = self.controller.get_info()
		return deepcopy(info)

    def save_trajectory(self):
    	print('Saving Trajectory #{0}'.format(self.traj_num))

        filepath = os.path.join(self.log_dir + self.traj_name + '.npy')
        os.makedirs(self.log_dir, exist_ok=True)
        np.save(filepath, self.traj_data)

        self.traj_saved = True
        self.traj_num += 1

	def delete_trajectory(self):
		filepath = os.path.join(self.log_dir + self.traj_name + '.npy')
		os.remove(filepath)

		self.traj_saved = False
		self.traj_num -= 1

	def collect_trajectory(self, info={}, practice=False):
		"""
		Collect trajectory until we end

		Notes: Save last trajectory, and whether or not we kept it
		"""
		self.reset_robot()
		self.traj_name = time.asctime().replace(" ", "_")
		self.traj_data = dict(observations=[], actions=[], info=info)
		self.traj_saved = False

		while True:

			# Determine If User Ended Episode #
			feedback = self.get_user_feedback()
			end_traj = feedback['A'] or feedback['B']
			delete = practice or feedback['B']

			# End Episode Appropriately #
			if end_traj and delete: return
			if end_traj and not delete: self.save_trajectory()

			# Get Latest Observation And Action #
			act = self.controller.get_action(noise=self.action_noise)
			obs = self.env.get_observation()

			# Save Data #
			self.traj_data['observations'].append(obs)
			self.traj_data['actions'].append(act)

			# Step Environment #
			obs = self.env.step(act)

	def get_observation(self):
		return None


	def set_action_noise(self, noise):
		self.action_noise = noise