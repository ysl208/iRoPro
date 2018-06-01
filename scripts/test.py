#! /usr/bin/env python
  
from rapid_pbd_msgs.msg import ExecuteProgramGoal
from rapid_pbd_msgs.msg import ExecuteProgramAction
import actionlib
import rospy

def main():
    rospy.init_node('test')
# surface_perception.getLandmarks
# world.landmarks = surface_landmarks
# CreateGrid()
# world.grid = grid
# for i from 1 to 10
#  world.gridIndex = i
#  executeProgram(world)

# in program executor: if grid has points, then run same program for multiple positions
# 

    goal = ExecuteProgramGoal()
    goal.name = 't1'
    client = actionlib.SimpleActionClient('rapid_pbd/execute_program_action', ExecuteProgramAction)
    client.wait_for_server()
    client.send_goal_and_wait(goal)
    print 'done'


if __name__ == '__main__':
    main()
