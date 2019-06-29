import cv2, numpy as np
import math
from random import randint

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
        
    # for handling the cycle behaviour
    def handle_angle_diff(self, angle):
        angle = np.remainder(angle,2*np.pi)
        if angle >= np.pi:
            angle = -2*np.pi + angle
        else:
            if angle <= -np.pi:
                angle = 2*np.pi + angle

        return angle

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
                
                print "\nclass no.: %d" % (i)
                print "current angle: %f" % (self.angles[i])
                print "meas angle: %f" % (meas[2])
                
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