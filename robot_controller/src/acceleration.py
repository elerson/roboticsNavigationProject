#!/usr/bin/python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Quaternion, Point
import math
import numpy as np
import tf

imuData = Imu()


def imuCallback(data):
    global imuData
    imuData = data



def main():
    rospy.init_node("imu_pose_estimation")
    global imudData

    imuSubscriber = rospy.Subscriber("/imu/data", Imu, imuCallback)
    estimationPublisher = rospy.Publisher("/pose", Pose, queue_size=1)
    r = rospy.Rate(100.0)

    currentTime = rospy.Time.now()
    lastTime = rospy.Time.now()



    vx = 0.0
    vy = 0.0
    vz = 0.0

    x = 0.0
    y = 0.0
    z = 0.0

    while not rospy.is_shutdown():

        currentTime = rospy.Time.now()

        dt = (currentTime - lastTime).to_sec()

        quaternion = (
            imuData.orientation.x,
            imuData.orientation.y,
            imuData.orientation.z,
            imuData.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)


        x = euler[0]
        y = euler[1]
        z = euler[2]


        rx = np.array([
                    [1, 0, 0], 
                    [0, math.cos(x), -math.sin(x)],
                    [0, math.sin(x), math.cos(x)]
                    ])

        ry = np.array([
                    [math.cos(y), 0, math.sin(y)], 
                    [0, 1, 0],
                    [-math.sin(y), 0, math.cos(y)]
                    ])

        rz = np.array([ 
                    [math.cos(z), -math.sin(z), 0],
                    [math.sin(z), math.cos(z), 0],
                    [0, 0, 1]
                    ])

        rotationMatrix = np.dot(np.dot(rz,ry), rx)

        #linear_calibrated_acceleration_x = imuData.linear_acceleration.x# - (-0.05390782738476988)
        #linear_calibrated_acceleration_y = imuData.linear_acceleration.y #- (-0.09272071227431315)
        #linear_calibrated_acceleration_z = imuData.linear_acceleration.z #- (10.012223018646223)

        calibration = (0, 0, 0)

        rotated_accel = np.dot(rotationMatrix, np.array([imuData.linear_acceleration.x - calibration[0], imuData.linear_acceleration.y- calibration[1], imuData.linear_acceleration.z- calibration[2]]))

        print(rotated_accel)
        
        lastTime = currentTime
        
        r.sleep()

if __name__ == '__main__':
    main()
