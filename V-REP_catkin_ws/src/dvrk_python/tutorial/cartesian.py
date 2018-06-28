#!/usr/bin/env python

import roslib; roslib.load_manifest('dvrk_teleop')

from dvrk_python.robot import *
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from dvrk_planner.srv import *

import array

class CartesianControl():

    def __init__(self, robot_name, rate):

        self.robot_ = robot(robot_name)
        self.rate_ = rate
        self.rosRate_ = rospy.Rate(self.rate_) # Hz
        self.period_ = 1 / float(self.rate_)
        self.counter_ = 0
        poseDesired = PoseStamped();        
        poseCurrent = PoseStamped();
        poseA = Pose();
        poseA.position.x = 1; 
        poseDesired.pose = poseA;
        poseB = Pose();
        poseB.position.x = 0; 
        poseCurrent.pose = poseB;

        rospy.wait_for_service('plan')
        try:
            planner = rospy.ServiceProxy('plan', plan)
            resp = planner(poseDesired, poseCurrent)
            print resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # MAIN RUNNING THREAD #
    def run(self):
        
        '''# FIRST CONTROLLED MOTION TOWARDS GOAL
        for i in range(self.joint_number_):
            if i > (len(self.lines_[self.lines_index_]) - 1):
                self.desired_joint_list[i] = float(0)
            else:
                self.desired_joint_list[i] = self.lines_[self.lines_index_][i].item()

        self.robot_.move_cartesian_frame(self.desired_joint_list, interpolate=True)

        print("Starting trajectory")#
        # DIRECT TRAJECTORY CONTROL
        while not rospy.is_shutdown():
            if ( self.lines_index_ < self.num_lines_ ):
                #print "Current line: " + str(self.lines_index_)
               
                    

                self.robot_.move_joint_list(self.desired_joint_list, interpolate=False)
              

                self.lines_index_ += 1
                #print("palle")	


            else:
              
                pass

            self.counter_ += 1

            #time.sleep(self.period_)
            self.rosRate_.sleep() 
        pass '''

#################CLASS END#################
if __name__ == '__main__':
    try:
        print("... Starting dvrk_cartesian_control ...")

        var = None
        manip = None
        trj_ty = None
        while manip is None and trj_ty is None:
            var = raw_input("Please enter the manipulator name: ")

            if 'PSM1' in var:
                manip = 'PSM1'
            elif 'PSM2' in var:
                manip = 'PSM2'
            elif 'MTMR' in var:
                manip = 'MTMR'
            elif 'MTML' in var:
                manip = 'MTML'
            elif 'ECM' in var:
                manip = 'ECM'
            else:
                print("Wrong Manipulator ['ECM', 'MTM(L/R)', 'PSM(1/2)']")

        rate = 200
        #print("Starting {} with {} trajectory @ {} Hz".format(manip, rate))
        CartesianControl = CartesianControl(manip, rate)
        CartesianControl.run()

    except rospy.ROSInterruptException:
        pass
