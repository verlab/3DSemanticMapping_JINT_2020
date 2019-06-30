import cv2, numpy as np
import math
from random import randint
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class FilteredInstances:

    def __init__(self, radius, processNoise, measNoise):
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

        self.radius = radius
        self.processNoise = processNoise
        self.measNoise = measNoise

        # Graph List : A hashMap of geometry_msgs/Pose indexed by their id's
        self.posesMap = {}
        self.lastId = -1

        # Node list for each instance
        # Instance node id
        self.instancesNodeId = []

        
    # for handling the cycle behaviour
    def handle_angle_diff(self, angle):
        angle = np.remainder(angle,2*np.pi)
        if angle >= np.pi:
            angle = -2*np.pi + angle
        else:
            if angle <= -np.pi:
                angle = 2*np.pi + angle

        return angle

    # Callback function to update the graph nodes.
    def updateGraphList(self, poses, poseids):
        min_diff = 0.00001
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

                dr = (yaw_curr - yaw_old)
                dx = (pose.position.x - old_pose.position.x)
                dy = (pose.position.y - old_pose.position.y)

                if(abs(dr) > min_diff or abs(dx) > min_diff or abs(dy) > min_diff ):
                    transform_matrices[pid] = np.array([[np.cos(dr), -np.sin(dr), dx], [np.sin(dr), np.cos(dr), dy], [0, 0, 1]])
                    print 'found adjustment!!'

                # update pose
                self.posesMap[pid] = pose
        
        # Update instaces locations whose pose id is in transform_matrices
        for i in range(len(self.instancesNodeId)):
            instance_pose_id = self.instancesNodeId[i]
            if instance_pose_id in transform_matrices:
                transform_matrix = transform_matrices[instance_pose_id]
                x = self.instances[i].statePre[0,0]
                y = self.instances[i].statePre[1,0]
                position = np.array([[x],[y],[1]])
                position_new = np.matmul(transform_matrix, position)
                x_new = position_new[0,0]
                y_new = position_new[1,0]
                self.instances[i].statePre[0,0] = x_new
                self.instances[i].statePre[1,0] = y_new

                # Update filter
                self.instances[i].correct( np.array([[np.float32(x_new)],[np.float32(y_new)]]) )
                tp = self.instances[i].predict()
                self.predictions[i] = (tp[0], tp[1])

    # add measurement list, for the case when more than one object is seen in the same frame
    def addMeasurementList(self, meas_list):
        if len(meas_list) == 0: 
            return 
    
        for m in meas_list:
            self.addMeasurement(m)

    # meas is the measurement, a tuple (x,y,...) in either integer or float format
    def addMeasurement(self, meas):
        addNew = True

        mp = np.array([ [np.float32(meas[0])],[np.float32(meas[1])] ])

        for i in range(len(self.instances)):
            x = self.predictions[i][0]
            y = self.predictions[i][1]
            d2 = pow(x-meas[0],2) + pow(y-meas[1], 2)
            d = math.sqrt(d2)

            if d < self.radius:
                addNew = False

                # Save measurement at object 
                self.measurements[i].append((meas[0], meas[1]))

                # Kalman correct and predict
                self.instances[i].correct(mp)
                tp = self.instances[i].predict()

                # Save last prediction of object in list
                self.predictions[i] = (tp[0], tp[1])
                v = self.angles[i]
                w = meas[2]
                d = self.handle_angle_diff(v - w)
                w = v - d
                self.angles[i] = self.angles[i]*0.8 + w*0.2

                self.observations[i] += 1.0

                break


        # Add new instance        
        if addNew:
            self.addNewInstance(meas)

    def addNewInstance(self, meas):
        
        print 'Adding new instance!'
        print 'pose id: '+ str(self.lastId) + ' (of '+ str(len(self.posesMap))+ ' poses)'
        #print str(self.posesMap[self.lastId])
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