#!/usr/bin/env python
import rospy
from baxter_interface import Gripper
from baxter_core_msgs.msg import (
    EndEffectorCommand,
)
import json
class UniversalGripper():
    def __init__(self,side,baxter=None):
        self.gripper =  Gripper(side)
        self.prepare()
        self.side = side
        self.opened = True
        self.outside_grip = True
        self.grippd = False
        self.state_pos =100
        self.weight_pub = rospy.Publisher("/robot/end_effector/"+side+"_gripper/command",
                                           EndEffectorCommand
                                          )
        self.baxter =baxter
        self.id = 65537
        self.touch_links = ["pedestal",side+"_gripper",side+"_gripper_base",side+"_hand_camera",side+"_hand_range",side+"_hand",side+"_wrist"]
        rospy.sleep(1)
        self.gripper_weight = 0.8
        self.object_weight = 0.3
        self.setWeight(0.8)
#         self.setObjectWeight()

    def setWeight(self,kg=0.8):        
        cmd = EndEffectorCommand()
        cmd.id = self.id
        cmd.command = "configure"
        cmd.args =''.join(['{"urdf":{ "name": "'+self.side+'_gripper","link":[{ "name": "'+self.side+'_hand"',
                           '}',
                           ',{ "name": "'+self.side+'_gripper_base","inertial":{ "mass":{ "value": '+str(kg)+'}',
                           ',"origin":{"xyz":[0.0, 0.0, 0.08]} }',
#                             ',"visual":{"origin":{"xyz":[0.0, 0.0, 0.0098]},"geometry":{"mesh":{"filename":"package://baxter_tasker/data/universalGripper.DAE","scale":[1,1,1]} } }',
                           '}]}}'])
#                            ',"collision":{"origin":{"xyz":[0.0, 0.0, 0.0098],"rpy":[0.0, 0.0, 0.0]},"geometry":{"cylinder":{"radius": 0.5,"length":0.6} } }',
#         print json.dumps(args)
        
        self.gripper_weight = kg
        self.weight_pub.publish(cmd)
        
    def setObjectWeight(self,kg=0.0):
        """
            NOT used yet as the gripper does not now when it gripped something
        """
        cmd = EndEffectorCommand()
        cmd.id = self.id
        cmd.command = "configure"
        cmd.args = ''.join(['{ "object":            { "name": "'+self.side+'_gripper",',
                           '"inertial": { "mass": { "value": '+str(kg)+'}, "origin":{ "xyz": [0.0, 0.0, 0.0] },',
                           ' "inertia": { "ixx": 0.0, "ixy": 0.0, "ixz": 0.0, "iyy": 0.0, "iyz": 0.0, "izz": 0.0 } } } }'])
        self.object_weight = kg
        self.weight_pub.publish(cmd)
        
    def prepare(self,ripples=5):
#         for i in range(ripples):
#             self.close(True,15,0.2)
#             self.open(True,0.2)
        self.close(True,15,1)
        self.open(True,3)
        self.close(True,0.01,2)
        self.open(True,0.7)
    
    def __setState(self, open):#true if opened ; false if closed
        if open is True:
            self.gripped = False
            self.state_pos =100
            self.opened = True
        else:
            self.gripped = True
            self.state_pos =0
            self.opened = False
    
    def open(self,blocking=False,open_time=1,timeout=5.0):
        print "opening"
        
        if self.opened is True:
            return
        self.close(True,0.00001,0.2,True)
        self.__setState(True)
        self.gripper.set_blow_off(open_time)
        cmd = EndEffectorCommand.CMD_RELEASE
        stop_test = lambda: (not self.gripper.sucking() and not self.gripper.blowing())
        self.gripper.command(
            cmd,
            block=blocking,
            test=stop_test,
            timeout=timeout,
            )
        if not self.baxter is None:
            self.baxter.scene.scene.remove_attached_object("right_gripper_base","right_object")
            rospy.sleep(0.1)
            self.baxter.scene.scene.remove_world_object("right_object")
        self.setWeight(self.gripper_weight-self.object_weight)
        
    def close(self,blocking=False,vac_thresh=15,timeout=10.0,suppress=False):
        
        print "sucking"
        if not self.baxter is None and suppress is False:
            pose = PS("right_gripper_base",[0.0,0.0,0.18]) 
            self.baxter.scene.scene.attach_box("right_gripper_base","right_object",pose,(0.06,0.06,0.12),self.touch_links)
        self.__setState(False)
        self.gripper.set_vacuum_threshold(vac_thresh)
        cmd = EndEffectorCommand.CMD_GO
        arguments = {"grip_attempt_seconds": timeout}
        self.gripper.command(
            cmd,
            block=blocking,
            test=self.gripper.vacuum,
            timeout=timeout,
            args=arguments,
            ) 
        if suppress is False:
            self.setWeight(self.gripper_weight+self.object_weight)
        print "sucked"
        
    def gripped(self):
        return self.gripped

    def command_position(self,value):
        if value < 50:
            self.close(False)
        else:
            
            self.open(False)
            
        return True
            
    def position(self):
        return self.state_pos
        
    def type(self):
        return 'suction'
        
if __name__ == "__main__":
    rospy.init_node("vac_gripper_test")
    gr = UniversalGripper("right",None)
#     gr.close(True,50,0.5)
#     gr.prepare()
  
#     gr.command_position(0)
#     rospy.sleep(1)
#     print gr.position()
#     gr.command_position(100)
#     print gr.position()
    # rospy.sleep(3)
#     gr.open(True,0.5)
    rospy.sleep(1)
