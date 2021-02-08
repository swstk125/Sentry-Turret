
import pybullet as p
import pybullet_data
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import math

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

turret = p.loadURDF(r"C:\Users\swastik\Desktop\ROBOTICS\turret\urdf\turret2.urdf")
friendly = p.loadURDF(r"C:\Users\swastik\Desktop\ROBOTICS\turret\friendly.xml", [5,3,0])

#joints info of turret
joints = p.getNumJoints(turret)
for joint in range(joints):
    print(joints)
    print(p.getJointInfo(turret,joint))
    print("\n")


vel1=0.1 #rad/sec
vel2=-0.1
maxforce=100 #newton

#camera_setup
dis = 2
width = 512
height = 512
fov = 360
aspect = width / height
near = 0.02
far = 5
upvector=[]
basepos, base_orient = p.getBasePositionAndOrientation(turret)
base_orient = p.getEulerFromQuaternion(base_orient)
x=basepos[0] + dis * math.cos(base_orient[2])         #calculating the coordinates for the target point we will pass in the computeViewMatrix
y=basepos[1] + dis * math.sin(base_orient[2])
for i in range(3):
    upvector.append(basepos[i])
upvector[2]=1
viewmatrix = p.computeViewMatrix([basepos[0], basepos[1], basepos[2]+2.5], [x, y, basepos[2]], upvector)
projectionmatrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

images= p.getCameraImage(width, 
                         height, 
                         viewmatrix, 
                         projectionmatrix, 
                         renderer=p.ER_BULLET_HARDWARE_OPENGL)
rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
cv2.imshow('rgb', rgb_opengl) 


while (1):
    keys = p.getKeyboardEvents()
    for k,v in keys.items():
        
        if(k== p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(turret, 0, p.VELOCITY_CONTROL, targetVelocity = vel1, force= maxforce )
            p.stepSimulation()

        if(k== p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            p.setJointMotorControl2(turret, 0, p.VELOCITY_CONTROL, targetVelocity = 0, force= maxforce )
            p.stepSimulation()

        if(k== p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(turret, 0, p.VELOCITY_CONTROL, targetVelocity = vel2, force= maxforce )
            p.stepSimulation()

        if(k== p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            p.setJointMotorControl2(turret, 0, p.VELOCITY_CONTROL, targetVelocity = 0, force= maxforce )
            p.stepSimulation()

        if(k== p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(turret, 1, p.VELOCITY_CONTROL, targetVelocity = vel1, force= maxforce )
            p.stepSimulation()

        if(k== p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            p.setJointMotorControl2(turret, 1, p.VELOCITY_CONTROL, targetVelocity = 0, force= maxforce )
            p.stepSimulation()

        if(k== p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(turret, 1, p.VELOCITY_CONTROL, targetVelocity = vel2, force= maxforce )
            p.stepSimulation()

        if(k== p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            p.setJointMotorControl2(turret, 1, p.VELOCITY_CONTROL, targetVelocity = 0, force= maxforce )
            p.stepSimulation()

        
    
      
p.disconnect()