import time
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-90.8)

planeId = p.loadURDF("plane.urdf")

robotId = p.loadURDF("body.urdf")


p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

frontLegSensorValues = np.zeros(1000)
backLegSensorValues = np.zeros(1000)

targetAngles = np.sin(np.linspace(-np.pi/4, np.pi/4, 1000) )

amplitude_fl = np.pi*2
frequency_fl = 10
phaseOffset_fl = 0

amplitude_bl = np.pi/4
frequency_bl = 10
phaseOffset_bl = np.pi/4


for i in range(1000):
    p.stepSimulation()
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = amplitude_fl * np.sin(frequency_fl * i + phaseOffset_fl),
        maxForce = 500)
    
    
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "BackLeg_Torso",
        controlMode = p.POSITION_CONTROL,
        targetPosition = amplitude_bl * np.sin(frequency_bl * i + phaseOffset_bl),
        maxForce = 500)
    
    time.sleep(1/360)
    
np.save("data/frontleg.npy",frontLegSensorValues)
np.save("data/backleg.npy",backLegSensorValues)

p.disconnect()