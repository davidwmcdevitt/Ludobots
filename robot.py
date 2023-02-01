from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import constants as c


class ROBOT:
    
    def __init__(self,solutionID):
        
        
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.prepare_to_sense()
        self.prepare_to_act()
        self.nn = NEURAL_NETWORK("brain"+str(solutionID)+".nndf")
        os.system("rm brain"+str(solutionID)+".nndf")
        self.solutionID = solutionID

        
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
                desiredAngle = self.nn.Get_Value_Of(neuron) * c.motorJointRange
                jointName = self.nn.Get_Motor_Neurons_Joint(neuron)
                self.motors[jointName].set_value(self.robotId, desiredAngle)
                #Sprint(desiredAngle)
                
            #i.set_value(self.robotId, t)
            
    def think(self):
        self.nn.update()
        #self.nn.Print()
        
    def get_fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        fh  = open("tmp"+str(self.solutionID)+".txt", "w")
        fh.write(str(xCoordinateOfLinkZero))
        fh.close()
        os.system("mv tmp"+str(self.solutionID)+".txt fitness"+str(self.solutionID)+".txt")
        #print(xCoordinateOfLinkZero)