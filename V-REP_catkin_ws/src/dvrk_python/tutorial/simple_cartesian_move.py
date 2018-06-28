"""In this example we will take a look at the how to move in cartesian space. Please note we can move a different distance than the one specified. We will show how the following methods world:
 * delta_move_cartesian_translation()
 * delta_move_cartesian_rotation
 * move_cartesian_translation
 * move_cartesian_rotation

Lets take a look:"""

from dvrk_python.robot import *
import math
import time
import numpy as np

def cartesian_move(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and we move the robot in castresian space accordingly.

    :param robotName: the name of the robot used """

    r = robot(robotName)
    print 'Starting position:'
    vec = np.array([0,0,0.1,0,0,0,0])
    desired_joint_list = vec.tolist()
    r.move_joint_list(desired_joint_list, interpolate=True)
    time.sleep(1)
    print r.get_desired_cartesian_position()

    list = [0.001, 0.0, 0.0]
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    time.sleep(0.01)
    r.delta_move_cartesian(list, False)
    
    '''rot_4 = Rotation()
    # PyKDL can create rotation matrices along axes
    rot_4.DoRotX(math.pi / 4.0) # rotate along x by 45 degrees
    r.delta_move_cartesian_rotation(rot_4, True)
    print 'After rotation along x by 45 degrees:'
    print r.get_desired_cartesian_position()
    time.sleep(1)
    rot_5 = Rotation()
    # PyKDL can create rotation matrices along axes
    rot_5.DoRotX(-(math.pi / 4.0)) # rotate along x by 45 degrees
    r.delta_move_cartesian_rotation(rot_5, True)
    print 'After rotation along x by -45 degrees:'
    print r.get_desired_cartesian_position()
    time.sleep(1)
    rot_6 = Rotation()
    # PyKDL can create rotation matrices along axes
    rot_6.DoRotY(math.pi / 4.0) # rotate along x by 45 degrees
    r.delta_move_cartesian_rotation(rot_6, True)
    print 'After rotation along x by 45 degrees:'
    print r.get_desired_cartesian_position()
    time.sleep(1)
    rot_7 = Rotation()
    # PyKDL can create rotation matrices along axes
    rot_7.DoRotY(-(math.pi / 4.0)) # rotate along x by 45 degrees
    r.delta_move_cartesian_rotation(rot_7, True)
    print 'After rotation along x by -45 degrees:'
    print r.get_desired_cartesian_position()
    time.sleep(1)
    rot_8 = Rotation()
    # PyKDL can create rotation matrices along axes
    rot_8.DoRotZ(math.pi / 4.0) # rotate along x by 45 degrees
    r.delta_move_cartesian_rotation(rot_8, True)
    print 'After rotation along x by 45 degrees:'
    print r.get_desired_cartesian_position()
    time.sleep(1)
    rot_8 = Rotation()
    # PyKDL can create rotation matrices along axes
    rot_8.DoRotZ(-(math.pi / 4.0)) # rotate along x by 45 degrees
    r.delta_move_cartesian_rotation(rot_8, True)
    print 'After rotation along x by -45 degrees:'
    print r.get_desired_cartesian_position()'''


    '''# True means direct command to the robot, without trajectory generation
    # this means we can't give a goal too far away from current position AND
    # this command doesn't wait for the robot to move so we need to add a sleep
    list_7 = [0.01,0.0,-0.15]
    r.move_cartesian_translation(list_7, False)
    time.sleep(3)
    print 'After, move to (0.01,0,-0.15)'
    print r.get_desired_cartesian_position()

    vec_8 = Vector(0.0,0.01,-0.15)
    r.move_cartesian_translation(vec_8, False)
    time.sleep(3)#in seconds
    print 'After, move to (0,0.01,-0.15):'
    print r.get_desired_cartesian_position()

    rot_9 = Rotation()
    rot_9.DoRotY(-math.pi/6.0)
    r.move_cartesian_rotation(rot_9,True)
    print 'After, rotation in y by -pi/6:'
    print r.get_desired_cartesian_position()'''

if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run cartesian_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        cartesian_move(sys.argv[1])
