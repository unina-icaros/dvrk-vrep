from dvrk_python.robot import *

def control():
  psm1 = robot('PSM1')
  joints = psm1.get_current_joint_position()
  print('PSM1 joints position is: ',joints)

  

  return

if __name__ == '__main__':
  print('sample.py')
  control()
