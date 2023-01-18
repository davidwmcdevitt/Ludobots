from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pyrosim.pyrosim as pyrosim


class ROBOT:
    
    def __init__(self):
        
        
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.prepare_to_sense()
        self.prepare_to_act()

        
    def prepare_to_sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            print(linkName)
            self.sensors[linkName] = SENSOR(linkName)
            
    def sense(self, t):
        for i in self.sensors.values():
            i.get_value(t)
            
            
    def prepare_to_act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            print(jointName)
            self.motors[jointName] = MOTOR(jointName)
            
    def act(self, t):
        for i in self.motors.values():
            i.set_value(self.robotId, t)