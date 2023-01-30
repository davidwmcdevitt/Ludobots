from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK

class ROBOT:
    
    def __init__(self):
        
        
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.prepare_to_sense()
        self.prepare_to_act()
        self.nn = NEURAL_NETWORK("brain.nndf")
        self.robot = self.robotId
        
    def prepare_to_sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            #print(linkName)
            self.sensors[linkName] = SENSOR(linkName)
            
    def sense(self, t):
        for i in self.sensors.values():
            i.get_value(t)
            
            
    def prepare_to_act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            #print(jointName)
            self.motors[jointName] = MOTOR(jointName)
            
    def act(self, t):
        for neuron in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuron):
                desiredAngle = self.nn.Get_Value_Of(neuron)
                jointName = self.nn.Get_Motor_Neurons_Joint(neuron)
                self.motors[jointName].set_value(self.robotId, desiredAngle)
                #print(desiredAngle)
                
            #i.set_value(self.robotId, t)
            
    def think(self):
        self.nn.update()
        self.nn.Print()
        
    def get_fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        fh  = open("fitness.txt", "w")
        fh.write(str(xCoordinateOfLinkZero))
        fh.close()
        
        print(xCoordinateOfLinkZero)