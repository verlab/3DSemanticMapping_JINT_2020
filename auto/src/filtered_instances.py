import cv2, numpy as np
from scipy.optimize import linear_sum_assignment
import math
from random import randint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#import pdb

class FilteredInstances:

    def __init__(self, className, radius, processNoise, measNoise, min_obs, groundtruths=[]):

        self.className = className

        # Each instance is a new kalman filter
        self.instances = []

        # Each element is the last prediction of an object
        self.predictions = []
        self.angles = []

        # Each element is a list of measurements (tuples of x,y) for each object 
        self.measurements = []

        # Each element color
        self.colors = []

        # Observations of each instance
        self.observations = []        

        # Radius, process noise, etc. 
        self.radius = radius
        self.processNoise = processNoise
        self.measNoise = measNoise

        # Graph List : A hashMap of geometry_msgs/Pose indexed by their id's
        self.posesMap = {}
        self.lastId = -1

        # Node list for each instance
        # Instance node id
        self.instancesNodeId = []

        self.min_obs = min_obs

        # List with the exact locations of objects, for doing experiments
        self.gts = groundtruths
        
    # for handling the cycle behaviour
    def handle_angle_diff(self, angle):
        angle = np.remainder(angle,2*np.pi)
        if angle >= np.pi:
            angle = -2*np.pi + angle
        else:
            if angle <= -np.pi:
                angle = 2*np.pi + angle

        return angle

    def euclidianDistance(self, x1, y1, x2, y2):
        distance = np.sqrt((x1 - x2)**2.0 + (y1 - y2)**2.0)
        return distance

    # Callback function to update the graph nodes.
    def updateGraphList(self, poses, poseids):
        min_diff = 0.0
        transform_matrices = {}
        for i in range(len(poseids)):
            pid = poseids[i]
            pose = poses[i]

            # New graph node added
            if pid not in self.posesMap:
                self.posesMap[pid] = pose
                self.lastId = pid

            # TODO old graph node: update
            else:
                old_pose = self.posesMap[pid]

                # find difference from previous position
                (row, pitch, yaw_old) = euler_from_quaternion([old_pose.orientation.x, old_pose.orientation.y, old_pose.orientation.z, old_pose.orientation.w])
                (row, pitch, yaw_curr) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                
                #Ro = quaternion_matrix()
                dr = (yaw_curr - yaw_old)
                dr=0
                dx = (pose.position.x - old_pose.position.x)
                dy = (pose.position.y - old_pose.position.y)

                if(abs(dr) > min_diff or abs(dx) > min_diff or abs(dy) > min_diff ):
                    transform_matrices[pid] = np.array([[np.cos(dr), -np.sin(dr), dx], [np.sin(dr), np.cos(dr), dy], [0, 0, 1]])
                    print 'Found adjustment!!'
                    #pdb.set_trace()

                # update pose
                self.posesMap[pid] = pose
        
        # Update instaces locations whose pose id is in transform_matrices
        for i in range(len(self.instancesNodeId)):
            instance_pose_id = self.instancesNodeId[i]
            if instance_pose_id in transform_matrices:
                transform_matrix = transform_matrices[instance_pose_id]
                x, y=self.predictions[i]
                position = np.array([[x],[y],[1]], dtype=np.float32)
                position_new = np.matmul(transform_matrix, position)
                x_new = position_new[0,0]
                y_new = position_new[1,0]
                self.instances[i].statePre[0,0] = x_new
                self.instances[i].statePre[1,0] = y_new

                # Update filter
                #self.instances[i].correct( np.array([[np.float32(x_new)],[np.float32(y_new)]]) )
                #tp = self.instances[i].predict()
                #self.predictions[i] = (tp[0], tp[1])
                self.predictions[i] = (x_new,y_new)

    def getErrorMatrix(self, meas_list):
        M = np.zeros((len(meas_list), len(self.predictions)), dtype=np.float32)
        for i in range(len(self.predictions)):
            x, y = self.predictions[i]
            for j in range(len(meas_list)):
                x_meas = np.float32(meas_list[j][0])
                y_meas = np.float32(meas_list[j][1])

                distance = np.sqrt((x_meas - x)**2.0 + (y_meas - y)**2.0)
                M[j, i] = distance
        
        return M
    
    def getGroundtruthErrorMatrix(self):
        # Select predictions with minimum number of observations
        preds = []
        for i in range(len(self.observations)):
            if self.observations[i] >= self.min_obs:
                preds.append(self.predictions[i])

        if len(self.gts) > 0 and len(preds)>0:
            M = np.zeros((len(self.gts), len(preds)), dtype=np.float32)
            for i in range(len(preds)):
                x, y = preds[i]
                for j in range(len(self.gts)):
                    x_gt = np.float32(self.gts[j][0])
                    y_gt = np.float32(self.gts[j][1])

                    distance = np.sqrt((x_gt - x)**2.0 + (y_gt - y)**2.0)
                    M[j, i] = distance
            return M

        else:
            return None

    def getMeanError(self):
        max_dist = self.radius
        preds = []
        mean_error = 0

        # Select predictions with minimum number of observations
        for i in range(len(self.observations)):
            if self.observations[i] >= self.min_obs:
                preds.append(self.predictions[i])

        instance_count = len(preds) 
        if len(self.gts) == 0 or len(preds) == 0: 
            return '0','0','0', str(instance_count)
               
        pred_indices = list(range(len(preds)))
        gts_indices = list(range(len(self.gts)))
        unmatchedObjs = 0
        costs = self.getGroundtruthErrorMatrix()
        costs2 = np.zeros((len(self.gts), 0))
        
        # Remove indices whose distance is too far from all groundtruths
        for i in range(len(preds)):
            ci = costs[:, i]
            remove_i = True
            for j in range(len(self.gts)):
                if ci[j] < max_dist:
                    remove_i = False
                    break
            
            if remove_i: 
                unmatchedObjs+=1
                pred_indices.remove(i)
            else:
                costs2 = np.c_[costs2, ci]

        # Perform instance association: rows -> gt indices; cols -> instances
        row_ind, col_ind = linear_sum_assignment(costs2)
        pred_tmp = list(pred_indices)
        for i in range(len(row_ind)):
            gt_index = row_ind[i]
            instance_index = pred_indices[col_ind[i]]

            # Test if assigned object has distances less than radius 

            pred_tmp.remove(instance_index)
            gts_indices.remove(gt_index)

            mean_error += costs2[row_ind[i], col_ind[i]]

        mean_error/=len(row_ind)
        falsePositives = len(pred_tmp)+unmatchedObjs
        falseNegatives = len(gts_indices)

        return str(instance_count), str(mean_error), str(falsePositives), str(falseNegatives)

    # add measurement list, for the case when more than one object is seen in the same frame
    def addMeasurementList(self, meas_list):
        if len(meas_list) == 0: 
            return 
        
        # Find association cost
        cost = self.getErrorMatrix(meas_list)

        row_ind, col_ind = linear_sum_assignment(cost)
        meas_indices = range(len(meas_list)) 
        unmatched_indices = [x for x in meas_indices if x not in row_ind]

        # update objects whose associated measurement lies within distance threshold
        for i in range(len(row_ind)):
            instance_index = col_ind[i]
            meas_index = row_ind[i]

            # Find distance
            meas = meas_list[meas_index]
            x = self.predictions[instance_index][0]
            y = self.predictions[instance_index][1]
            distance = math.sqrt(pow(x-meas[0],2) + pow(y-meas[1], 2))

            # if distance is small, add it as a measurement
            if distance < self.radius:
                # Save measurement at object 
                self.measurements[instance_index].append((meas[0], meas[1]))

                # Kalman correct and predict
                mp = np.array([ [np.float32(meas[0])],[np.float32(meas[1])] ])
                self.instances[instance_index].correct(mp)
                tp = self.instances[instance_index].predict()

                # Save last prediction of object in list
                self.predictions[instance_index] = (tp[0], tp[1])
                v = self.angles[instance_index]
                w = meas[2]
                d = self.handle_angle_diff(v - w)
                w = v - d
                persistence = 0.9
                self.angles[instance_index] = self.angles[instance_index]*persistence + w*(1-persistence)
                self.observations[instance_index] += 1.0

            # Add as instance later
            else:
                unmatched_indices.append(meas_index)

        # Add all unmatched indices
        for index in unmatched_indices:
            meas = meas_list[index]
            self.addNewInstance(meas)

    def addNewInstance(self, meas):
        
        print 'Adding new '+ self.className
        print '\n'

        mp = np.array([ [np.float32(meas[0])],[np.float32(meas[1])] ])            

        instance = cv2.KalmanFilter(2,2)
        instance.measurementMatrix = np.array([[1,0],[0,1]],np.float32)
        instance.transitionMatrix = np.array([[1,0],[0,1]],np.float32)
        instance.processNoiseCov = np.array([[1,0],[0,1]],np.float32) * self.processNoise
        instance.measurementNoiseCov = np.array([[1,0],[0,1]],np.float32) * self.measNoise
        instance.statePre = mp
        instance.errorCovPre = np.array([[1,0],[0,1]],np.float32) * 3          
            
        self.instances.append(instance)
        self.predictions.append((meas[0], meas[1]))
        self.angles.append(meas[2])
        self.measurements.append( [(meas[0], meas[1])] )
        self.colors.append( (randint(0, 255), randint(0, 255), randint(0, 255)) )
        self.observations.append(1.0)

        # Graph list
        self.instancesNodeId.append(self.lastId)

    def getMeanCovariance(self):
        num_instances = len(self.measurements)
        x_vars = []
        y_vars = []
        xy_vars = []
        max_x_var = 0
        max_y_var = 0
        max_xy_var = 0

        for i in range(num_instances):
            xs = []
            ys = []
            for x,y in self.measurements[i]:
                if not math.isnan(x) and (not math.isnan(y)):
                    xs.append(x)
                    ys.append(y)

            cov = np.cov(xs,ys)
            if( len(xs)> 1):
                x_vars.append(cov[0,0])
                y_vars.append(cov[1,1])
                xy_vars.append(cov[1,0])

        if(len(x_vars)>0):
            max_x_var = np.max(x_vars)
            max_y_var = np.max(y_vars)
            max_xy_var = np.max(xy_vars)

        return str(num_instances), str(round(max_x_var, 2)), str(round(max_y_var,2)), str(round(max_xy_var, 2))
