import constants as c
import numpy as np 
import pyrosim.pyrosim as pyrosim
import pybullet as p


class MOTOR:
    
    def __init__(self,jointName):
    
        self.jointName = jointName
        self.prepare_to_act()
        
    def prepare_to_act(self):
        self.amplitude = c.amplitude_bl
        self.offset = c.phaseOffset_bl
        self.frequency = c.frequency_bl
        
        if self.jointName == "BackLeg_Torso":
            self.frequency = self.frequency / 2
    
        self.motorValues = self.amplitude * np.sin(self.frequency * np.linspace(0, 2*np.pi, c.iterations) + self.offset)
        
    def set_value(self, robot, desiredAngle):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robot,
            jointName = self.jointName,
            controlMode = p.POSITION_CONTROL,
            targetPosition = desiredAngle,
            maxForce = 1000)
        
    def save_values(self):
        np.save("./data/motor-" + self.jointName + ".npy", self.motorValues)