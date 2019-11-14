
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
import duckietown_world as dw
from duckietown_world.svg_drawing.ipython_utils import ipython_draw_svg, ipython_draw_html
import pandas as pd
import sys
import traceback


contracts.disable_all()

#### SET MAP NAME #######


### SET TRAJECTORY FILES ######
trajectoryFiles = ['autobot29_v3.yaml']
#####################################

convertFile = False
readImages = True

for trajectoryFile in trajectoryFiles:

    if convertFile is True:
        m = dw.load_map('ethz_amod_lab_k31')

        class Person(dw.PlacedObject):
            def __init__(self, radius, *args, **kwargs):
                self.radius = radius
                dw.PlacedObject.__init__(self, *args, **kwargs)

            def draw_svg(self, drawing, g):
                # drawing is done using the library svgwrite
                c = drawing.circle(center=(0, 0), r=self.radius, fill='pink')
                g.add(c)
                # draws x,y axes
                dw.draw_axes(drawing, g)

            def extent_points(self):
                # set of points describing the boundary
                L = self.radius
                return [(-L, -L), (+L, +L)]

        root = dw.PlacedObject()



        relTimestamps = []
        with open(trajectoryFile, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

            timestart = data['begin_time_stamp']
            print(timestart)
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
                        #print(phi[:,idx])
                relTimestamps.append(time)

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

        timestamps = range(len(seqs2)) # [0, 1, 2, ...]
        # SE2Transform is the wrapper for SE2 used by Duckietown World
        transforms = [dw.SE2Transform.from_SE2(_) for _ in seqs2]
        seq_me = dw.SampledSequence(timestamps, transforms)

        counter = 0
        center_points2 = []
        essentialInfo = []

        for timestamp, pose_object in seq_me:

            counter += 1

            hoi = list(get_lane_poses(m, pose_object.as_SE2()))
            if len(hoi) == 0:
                #print(dw.SE2Transform.from_SE2(geo.SE2_from_translation_angle([0,0],0)))
                #center_points2.append(dw.SE2Transform.from_SE2(geo.SE2_from_translation_angle([0,0],0)))
                continue
            else:
                #print('outside left: ' + str(hoi[0].lane_pose.distance_from_left) + ' outside right: ' + str(hoi[0].lane_pose.distance_from_right))
                distance_from_left = hoi[0].lane_pose.distance_from_left
                distance_from_right = hoi[0].lane_pose.distance_from_right
                distance_from_center = hoi[0].lane_pose.distance_from_center
                correct_direction = hoi[0].lane_pose.correct_direction
                rel_heading = hoi[0].lane_pose.relative_heading
                absTime = float(relTimestamps[counter]) + int(timestart)
                absTimeSeconds = str(absTime).split('.')[0]
                absTimeNanoSeconds = str(absTime).split('.')[1]
                print(absTimeSeconds, absTimeNanoSeconds, distance_from_center, rel_heading)

                center_points2.append(hoi[0].lane_pose.center_point)
                essentialInfo.append([absTimeSeconds, absTimeNanoSeconds, distance_from_center, rel_heading])
                #print(hoi[0].lane_pose.distance_from_center, hoi[0].lane_pose.relative_heading)
            #lane_pose = lanesegment.lane_pose_from_SE2Transform(pose_object)
            #center_points.append(lane_pose.center_point)

        DFcolumns = ['TimestampSeconds', 'TimestampNanoSeconds','CenterDistance', 'RelHeading']
        essentialDF = pd.DataFrame(np.array(essentialInfo), columns = DFcolumns)
        fileString = str(trajectoryFile.split('.')[0]) + '.csv'
        essentialDF.to_csv(fileString, index=False)
        print('saved')

    if readImages is True:
        try:
            fileString = str(trajectoryFile.split('.')[0]) + '.csv'
            path =  str(trajectoryFile.split('.')[0]) + '/image_timestamps.csv'
            imagesDFcolumns = ['ImageName', 'Seconds', 'Nanoseconds']
            imagesDF = pd.read_csv(path)
            imagesDF.columns = imagesDFcolumns
            poseDF = pd.read_csv(fileString)

            assert len(imagesDF) == len(poseDF), 'The pose csv and the images csv have a different number of entries'

        except Exception as e:

            print(e)
