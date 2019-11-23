# disabling contracts for speed
import contracts
import yaml
import numpy as np
import geometry as geo
import numpy as np
from duckietown_world.world_duckietown.tile_template import load_tile_types
from duckietown_world.geo.measurements_utils import iterate_by_class
from duckietown_world.world_duckietown.tile import get_lane_poses
import duckietown_world as dw
from copy import deepcopy
import geometry as g
import pandas as pd
import sys
import traceback

contracts.disable_all()

## LOAD MAPS ##
m = dw.load_map('ethz_amod_lab_k31')

## FOLDER NAMES ##
folderNames = ['autobot04']


#####################################

def relative_pose(q0, q1):
	return g.SE2.multiply(g.SE2.inverse(q0), q1)

def interpolate(q0, q1, alpha):
	q1_from_q0 = relative_pose(q0, q1)
	vel = g.SE2.algebra_from_group(q1_from_q0)
	rel = g.SE2.group_from_algebra(vel * alpha)
	q = g.SE2.multiply(q0, rel)
	return q


def calculatePose(qPose):
	timestamps = 1 # [0, 1, 2, ...]
	transforms = dw.SE2Transform.from_SE2(qPose)
	width = 0.188
	counter = 0
	center_points2 = []

	notWantedTiles = ['3way_left', '4way', 'asphalt']

	timestamp = 1
	pose_object = transforms
	hoi = list(get_lane_poses(m, pose_object.as_SE2()))

	try:
		tile = list(hoi[0].tile.children.keys())[0]
		if tile in notWantedTiles:
			return -100, -100, str(tile)

	except:
		print('error: ' + str(hoi))


	if len(hoi) == 0:
		return width, 0, 'asphalt'

	else:
		distance_from_left = hoi[0].lane_pose.distance_from_left
		distance_from_right = hoi[0].lane_pose.distance_from_right
		distance_from_center = hoi[0].lane_pose.distance_from_center
		rel_heading = hoi[0].lane_pose.relative_heading
		correct_direction = hoi[0].lane_pose.correct_direction
		alongInside = hoi[0].lane_pose.along_inside
		outsideLeft = hoi[0].lane_pose.outside_left
		inside = hoi[0].lane_pose.inside
		lateral = hoi[0].lane_pose.lateral
		lateral_left = hoi[0].lane_pose.lateral_left
		width = lateral_left
		tile = list(hoi[0].tile.children.keys())[0]

		correctDir = hoi[0].lane_pose.correct_direction
		return distance_from_center, rel_heading, str(tile)


finalArrayWithoutPose = []
finalArrayPoses = []
realTimestamps = []
seqs2 = []
final_trajectory = []
timeStart = []

for folderNamesSingle in folderNames:

	for trajectoryFileNumber in range(1,100):
		trajectoryFile = folderNamesSingle + '_' + str(trajectoryFileNumber) + '.yaml'
		print('Munging ' + str(trajectoryFile))
		pathToYaml =  str(folderNamesSingle) + '/' + str(trajectoryFile)

		try:
			with open(pathToYaml, 'r') as stream:

				try:

					data = yaml.safe_load(stream)

				except yaml.YAMLError as exc:

					print(exc)

		except:
			break
		timestart = data['begin_time_stamp']
		data_points = len(data['trajectory_data'])


		x = np.zeros((data_points,))

		y= np.zeros((data_points,))

		R = np.zeros((3,3, data_points))
		phi = np.zeros((3, data_points))



		dx = 999.999*np.ones((data_points, ))

		dy = 999.999*np.ones((data_points, ))

		dr = 999.999*np.ones((data_points, ))
		dphi = 999.999*np.ones((data_points, ))


		for idx, [time, traj] in enumerate(data['trajectory_data'].items()):

			x[idx] = np.array(traj[0])

			y[idx] = np.array(traj[1])

			R[:,:,idx] = np.reshape(np.array(traj[3:]), (3,3))


			phi[:,idx] = np.array([np.arctan2(-R[1,2,idx],R[2,2,idx]),
								   np.arctan2(R[0,2,idx],np.sqrt(R[0,0,idx]**2 + R[0,1,idx]**2)),
								   np.arctan2(-R[0,1,idx], R[0,0,idx])])

			realTimestamps.append(float(time) + float(timestart))
			timeStart.append(timestart)
			#print(phi[:,idx])
			z = phi[2,idx]
			points = np.array([x[idx], y[idx]])
			final_trajectory.append([points, z])

	print(len(realTimestamps), len(timeStart))
	for entry in range(0, len(final_trajectory)):

		x =  (final_trajectory[entry][0][0] )  # -2.2
		y = final_trajectory[entry][0][1] # + 0.8
		alpha = final_trajectory[entry][1]
		q5 = geo.SE2_from_translation_angle([x,y],alpha)

		seqs2.append(q5)

	path = str(folderNamesSingle) + '/image_timestamps.csv'
	imagesDFcolumns = ['ImageName', 'notImportant','Seconds', 'Nanoseconds']
	imagesDF = pd.read_csv(path)
	timeStampImagesSecondsArray = imagesDF.iloc[:,2]
	timeStampImagesNanoSecArray = imagesDF.iloc[:,3]
	imageNumber = imagesDF.iloc[:,0]
	entryNumberTrajectory = 0


	for entry in range(0, len(timeStampImagesSecondsArray)-1):
		print(entry, entryNumberTrajectory)
		if entryNumberTrajectory + 2 >= len(realTimestamps):
			break
		#timestart = np.array(timeStart)[entryNumberTrajectory]
		timeStampImagesSeconds = imagesDF.iloc[entry, 2]
		timeStampImagesNanoSec = imagesDF.iloc[entry,3] * (10**(-9))
		imageNumber = imagesDF.iloc[entry,0]
		# Check if exact time is available in both dataframes
		timeStampImages = float(timeStampImagesSeconds) + float(timeStampImagesNanoSec)
		#print(timeStampImagesSeconds, timeStampImagesNanoSec, timeStampImages)
		timeStampTrajectory = float(realTimestamps[entryNumberTrajectory])
		timeStampTrajectoryAfter = float(realTimestamps[entryNumberTrajectory + 1])
		print(timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)
		#print(timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)

		if timeStampImages == timeStampTrajectory:
			q2 = seqs2[entryNumberTrajectory]
			finalArrayPoses.append(q2)

			centerDistance, relativeHeading, tile = calculatePose(q2)
			print(imageNumber, timeStampImages, centerDistance, relativeHeading, tile)
			finalArrayWithoutPose.append([imageNumber, timeStampImages, centerDistance, relativeHeading, tile])
			#entryNumberTrajectory += 1



		elif timeStampImages < timeStampTrajectory:
			if entryNumberTrajectory == 0:
				continue

			while timeStampImages < timeStampTrajectory:

				if timeStampImages < timeStampTrajectory and timeStampImages > (float(realTimestamps[entryNumberTrajectory - 1])):
					timeStampTrajectory = float(realTimestamps[entryNumberTrajectory-1])
					timeStampTrajectoryAfter = float(realTimestamps[entryNumberTrajectory])
					print(entryNumberTrajectory, timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)
					break

				elif timeStampImages < timeStampTrajectory and timeStampImages > (float(realTimestamps[entryNumberTrajectory - 2])):
					timeStampTrajectory = float(realTimestamps[entryNumberTrajectory - 2])
					timeStampTrajectoryAfter = float(realTimestamps[entryNumberTrajectory])
					print(entryNumberTrajectory, timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)
					break
				else:
					entryNumberTrajectory -= 1
					timeStampTrajectory = float(realTimestamps[entryNumberTrajectory])
					timeStampTrajectoryAfter = float(realTimestamps[entryNumberTrajectory + 1])
					print(entryNumberTrajectory, timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)
					if  500 > entryNumberTrajectory or entryNumberTrajectory > 750 :
						print(entryNumberTrajectory, timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)
						break
			print(entryNumberTrajectory, timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)

			if timeStampTrajectory < timeStampImages < timeStampTrajectoryAfter:
				timeStampTrajectory = float(realTimestamps[entryNumberTrajectory])
				timeStampTrajectoryAfter =  float(realTimestamps[entryNumberTrajectory + 1])

				timeStamp1 = timeStampTrajectory
				timeStampWanted = timeStampImages
				timeStamp2 = timeStampTrajectoryAfter
				param = (timeStampWanted- timeStamp1) / (timeStamp2 - timeStamp1)
				q2 = seqs2[entryNumberTrajectory+1]
				q1 = seqs2[entryNumberTrajectory]

				qInter = interpolate(q1, q2, param)
				centerDistance, relativeHeading, tile = calculatePose(qInter)
				if centerDistance == -100 and relativeHeading == -100:
					continue

				finalArrayPoses.append(qInter)
				finalArrayWithoutPose.append([imageNumber, timeStampImages, centerDistance, relativeHeading, tile])

				continue


		elif timeStampImages > timeStampTrajectory:

			if timeStampImages < timeStampTrajectoryAfter:
				timeStampTrajectory = float(realTimestamps[entryNumberTrajectory])
				timeStampTrajectoryAfter = float(realTimestamps[entryNumberTrajectory + 1])
				timeStamp1 = timeStampTrajectory
				timeStampWanted = timeStampImages
				timeStamp2 = timeStampTrajectoryAfter

				param = (timeStampWanted- timeStamp1)/ (timeStamp2 - timeStamp1)

				q2 = seqs2[entryNumberTrajectory+1]
				q1 = seqs2[entryNumberTrajectory]

				qInter = interpolate(q1, q2, param)
				centerDistance, relativeHeading, tile = calculatePose(qInter)
				if centerDistance == -100 and relativeHeading == -100:
					continue
				finalArrayPoses.append(qInter)
				finalArrayWithoutPose.append([imageNumber, timeStampImages, centerDistance, relativeHeading, tile])


			else:
				while timeStampImages > timeStampTrajectoryAfter:
					if entryNumberTrajectory + 2 >= len(realTimestamps):
						break
					if timeStampImages > timeStampTrajectory and timeStampImages > timeStampTrajectoryAfter:
						print(entryNumberTrajectory, timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)
						timeStampTrajectoryAfter = float(realTimestamps[entryNumberTrajectory + 2])
						if timeStampTrajectory < timeStampImages < timeStampTrajectoryAfter:
							break
						else:
							entryNumberTrajectory += 1
							if timeStampTrajectoryAfter < timeStampImages:
								timeStampTrajectory =  float(realTimestamps[entryNumberTrajectory])
							timeStampTrajectoryAfter = float(realTimestamps[entryNumberTrajectory + 1])
							print(entryNumberTrajectory, timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)

					print(entryNumberTrajectory, timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)
					#timeStampTrajectoryAfter = int(timestart) + float(realTimestamps[entryNumberTrajectory + 1])
				if entryNumberTrajectory > 500:
					timeStampTrajectory = float(realTimestamps[entryNumberTrajectory])
					print(timeStampTrajectory, timeStampImages, timeStampTrajectoryAfter)

				timeStampTrajectory = float(realTimestamps[entryNumberTrajectory])
				timeStamp1 = timeStampTrajectory
				timeStampWanted = timeStampImages
				timeStamp2 = timeStampTrajectoryAfter
				#print(str(timeStamp1) + ' ' + str(timeStampWanted) + ' ' + str(timeStamp2))

				param = (timeStampWanted- timeStamp1) / (timeStamp2 - timeStamp1)

				q2 = seqs2[entryNumberTrajectory+1]
				q1 = seqs2[entryNumberTrajectory]
				qInter = interpolate(q1, q2, param)
				centerDistance, relativeHeading, tile = calculatePose(qInter)

				if centerDistance == -100 and relativeHeading == -100:
					continue

				finalArrayPoses.append(qInter)
				finalArrayWithoutPose.append([imageNumber, timeStampImages, centerDistance, relativeHeading, tile])
		else:
			print('unhandled')

	finalArrayWithoutPose = np.array(finalArrayWithoutPose)

	ArrayCol = ['ImageNumber',' timeStamp','centerDistance', 'relativeHeading', 'Tile']
	finalArrayWithPose = pd.DataFrame(finalArrayWithoutPose, columns=ArrayCol)
	fileName = str(folderNamesSingle) + '/output_oli.csv'
	finalArrayWithPose.to_csv(fileName, index=False)
	print('saved as: ' + str(fileName))
