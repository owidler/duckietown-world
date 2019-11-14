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

### SET TRAJECTORY FILES ######
trajectoryFiles = ['autobot29.yaml']
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
    # SE2Transform is the wrapper for SE2 used by Duckietown World
    transforms = dw.SE2Transform.from_SE2(qPose)

    #transforms = [dw.SE2Transform.from_SE2(_) for _ in qPose]
    #seq_me = dw.SampledSequence(timestamps, transforms)
    counter = 0
    center_points2 = []

    timestamp = 1
    pose_object = transforms
    hoi = list(get_lane_poses(m, pose_object.as_SE2()))

    if len(hoi) == 0:
        return 10, 0
        #center_points2.append(dw.SE2Transform.from_SE2(geo.SE2_from_translation_angle([0,0],0)))

    else:
        #print('outside left: ' + str(hoi[0].lane_pose.distance_from_left) + ' outside right: ' + str(hoi[0].lane_pose.distance_from_right))
        distance_from_left = hoi[0].lane_pose.distance_from_left
        distance_from_right = hoi[0].lane_pose.distance_from_right
        distance_from_center = hoi[0].lane_pose.distance_from_center
        rel_heading = hoi[0].lane_pose.relative_heading

        #absTime = float(relTimestamps[counter]) + int(timestart)
        return distance_from_center, rel_heading
        #center_points2.append(hoi[0].lane_pose.center_point)
        #print(hoi[0].lane_pose.distance_from_center, hoi[0].lane_pose.relative_heading)
    #lane_pose = lanesegment.lane_pose_from_SE2Transform(pose_object)
    #center_points.append(lane_pose.center_point)
    #print(relTimestamps[counter])

    counter += 1



for trajectoryFile in trajectoryFiles:
    print('Munging ' + str(trajectoryFile))
    realTimestamps = []
    pathToYaml = trajectoryFile.split('.')[0] + '/' + trajectoryFile
    with open(pathToYaml, 'r') as stream:

        try:

            data = yaml.safe_load(stream)

        except yaml.YAMLError as exc:

            print(exc)



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

        final_trajectory = []

        for idx, [time, traj] in enumerate(data['trajectory_data'].items()):

            x[idx] = np.array(traj[0])

            y[idx] = np.array(traj[1])

            R[:,:,idx] = np.reshape(np.array(traj[3:]), (3,3))


            phi[:,idx] = np.array([np.arctan2(-R[1,2,idx],R[2,2,idx]),
                                   np.arctan2(R[0,2,idx],np.sqrt(R[0,0,idx]**2 + R[0,1,idx]**2)),
                                   np.arctan2(-R[0,1,idx], R[0,0,idx])])

            realTimestamps.append(time)
            #print(phi[:,idx])
            z = phi[2,idx]
            points = np.array([x[idx], y[idx]])
            final_trajectory.append([points, z])
        final_array = final_trajectory

    seqs2 = []

    for entry in range(0, len(final_array)):
        x =  (final_array[entry][0][0] )  # -2.2
        y = final_array[entry][0][1] # + 0.8
        alpha = final_array[entry][1]
        q5 = geo.SE2_from_translation_angle([x,y],alpha)

        seqs2.append(q5)

    path = str(trajectoryFile.split('.')[0]) + '/image_timestamps.csv'
    imagesDFcolumns = ['ImageName', 'Seconds', 'Nanoseconds']
    imagesDF = pd.read_csv(path)
    timeStampImagesSecondsArray = imagesDF.iloc[:,1]
    timeStampImagesNanoSecArray = imagesDF.iloc[:,2]
    imageNumber = imagesDF.iloc[:,0]

    entryNumberTrajectory = 0
    finalArrayWithoutPose = []
    finalArrayPoses = []

    for entry in range(0, len(timeStampImagesSecondsArray)-1):
        timeStampImagesSeconds = imagesDF.iloc[entry, 1]
        timeStampImagesNanoSec = imagesDF.iloc[entry,2] * (10**(-9))
        imageNumber = int(imagesDF.iloc[entry,0])
        # Check if exact time is available in both dataframes
        timeStampImages = float(timeStampImagesSeconds) + float(timeStampImagesNanoSec)
        timeStampTrajectory = int(timestart) + float(realTimestamps[entryNumberTrajectory])
        timeStampTrajectoryAfter = int(timestart) + float(realTimestamps[entryNumberTrajectory + 1])

        if timeStampImages == timeStampTrajectory:
            q2 = seqs2[entryNumberTrajectory]
            finalArrayPoses.append(q2)
            centerDistance, relativeHeading = calculatePose(q2)
            finalArrayWithoutPose.append([int(imageNumber), timeStampImages, centerDistance, relativeHeading])
            #entryNumberTrajectory += 1

        elif timeStampImages < timeStampTrajectory:
            if entryNumberTrajectory == 0:
                continue
            while timeStampImages < timeStampTrajectory:
                entryNumberTrajectory -= 1
                timeStampTrajectory = int(timestart) + float(realTimestamps[entryNumberTrajectory])
                timeStampTrajectoryAfter = int(timestart) + float(realTimestamps[entryNumberTrajectory + 1])

            if timeStampImages > timeStampTrajectory and timeStampImages < timeStampTrajectoryAfter:
                timeStampTrajectory = int(timestart) + float(realTimestamps[entryNumberTrajectory])
                timeStampTrajectoryAfter = int(timestart) + float(realTimestamps[entryNumberTrajectory + 1])

                timeStamp1 = timeStampTrajectory
                timeStampWanted = timeStampImages
                timeStamp2 = timeStampTrajectoryAfter
                param = (timeStampWanted- timeStamp1) / (timeStamp2 - timeStamp1)
                q2 = seqs2[entryNumberTrajectory+1]
                q1 = seqs2[entryNumberTrajectory]

                qInter = interpolate(q1, q2, param)
                finalArrayPoses.append(qInter)
                centerDistance, relativeHeading = calculatePose(qInter)
                finalArrayWithoutPose.append([int(imageNumber), timeStampImages, centerDistance, relativeHeading])
                continue


        elif timeStampImages > timeStampTrajectory:

            if timeStampImages < timeStampTrajectoryAfter:
                timeStampTrajectory = int(timestart) + float(realTimestamps[entryNumberTrajectory])
                timeStampTrajectoryAfter = int(timestart) + float(realTimestamps[entryNumberTrajectory + 1])
                timeStamp1 = timeStampTrajectory
                timeStampWanted = timeStampImages
                timeStamp2 = timeStampTrajectoryAfter

                param = (timeStampWanted- timeStamp1)/ (timeStamp2 - timeStamp1)

                q2 = seqs2[entryNumberTrajectory+1]
                q1 = seqs2[entryNumberTrajectory]

                qInter = interpolate(q1, q2, param)
                finalArrayPoses.append(qInter)
                centerDistance, relativeHeading = calculatePose(qInter)
                finalArrayWithoutPose.append([int(imageNumber), timeStampImages, centerDistance, relativeHeading])

            else:
                while timeStampImages > timeStampTrajectoryAfter:
                    entryNumberTrajectory += 1
                    #print(entryNumberTrajectory, timeStampImages, timeStampTrajectory, timeStampTrajectoryAfter)
                    timeStampTrajectoryAfter = int(timestart) + float(realTimestamps[entryNumberTrajectory + 1])
                timeStampTrajectory = int(timestart) + float(realTimestamps[entryNumberTrajectory])
                timeStampTrajectoryAfter = int(timestart) + float(realTimestamps[entryNumberTrajectory + 1])
                timeStamp1 = timeStampTrajectory
                timeStampWanted = timeStampImages
                timeStamp2 = timeStampTrajectoryAfter
                #print(str(timeStamp1) + ' ' + str(timeStampWanted) + ' ' + str(timeStamp2))

                param = (timeStampWanted- timeStamp1) / (timeStamp2 - timeStamp1)

                q2 = seqs2[entryNumberTrajectory+1]
                q1 = seqs2[entryNumberTrajectory]
                qInter = interpolate(q1, q2, param)
                finalArrayPoses.append(qInter)
                centerDistance, relativeHeading = calculatePose(qInter)
                finalArrayWithoutPose.append([int(imageNumber), timeStampImages, centerDistance, relativeHeading])

        else:
            print('unhandled')

    finalArrayWithoutPose = np.array(finalArrayWithoutPose)
    ArrayCol = ['ImageNumber',' timeStamp','centerDistance', 'relativeHeading']
    finalArrayWithPose = pd.DataFrame(finalArrayWithoutPose, columns=ArrayCol)
    fileName = str(trajectoryFile.split('.')[0]) + '/' + 'matchedDataFrame.csv'
    finalArrayWithPose.to_csv(fileName, index=False)
    print('saved as: ' + str(fileName))
