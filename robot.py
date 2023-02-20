from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import constants as c
import numpy as np
import random
import math

class ROBOT:
    
    def __init__(self,solutionID):
        
        
        #self.robotId = p.loadURDF("body.urdf")
        self.robotId = p.loadURDF("tardigrade" + str(solutionID) + ".urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.prepare_to_sense()
        self.prepare_to_act()
        #self.nn = NEURAL_NETWORK("brain"+str(solutionID)+".nndf")
        self.nn = NEURAL_NETWORK("tardigrade_noggin" + str(solutionID) + ".nndf")
        #os.system("rm brain"+str(solutionID)+".nndf")
        self.solutionID = solutionID

        
    def prepare_to_sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            #print(linkName)
            self.sensors[linkName] = SENSOR(linkName)
            
        #print(self.sensors)
            
    def sense(self, t):
        for i in self.sensors.values():
            i.get_value(t)
            
            
    def prepare_to_act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            #print(jointName)
            self.motors[jointName] = MOTOR(jointName)
            
    def act(self, t):
        
        wave = []

        for i in range(c.walking_cycle):
            wave.append(c.amplitude * np.sin(c.base_walking_frequency * i))
            
        wave_indices ={}
        
        for neuron in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuron):
            
                init_wave_pos = random.randint(0,c.walking_cycle)
                wave_indices[neuron] = init_wave_pos
        
        for neuron in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuron):
                #print(self.nn.Get_Value_Of(neuron))
                #amplitude_fl * np.sin(frequency_fl * i + phaseOffset_fl)
                #desiredAngle = np.sin(c.base_walking_frequency *self.nn.Get_Value_Of(neuron) * c.motorJointRange
                angle_index = wave_indices[neuron] + math.ceil(c.walking_cycle/10)
                #print(angle_index)
                if angle_index > c.walking_cycle - 1:
                    angle_index = angle_index - c.walking_cycle
                if angle_index < 0:
                    angle_index = angle_index + c.walking_cycle
                
                #desiredAngle = np.sin(c.base_walking_frequency *t) * c.motorJointRange
                desiredAngle = wave[angle_index]
                
                wave_indices[neuron] = angle_index
                
                #desiredAngle = self.nn.Get_Value_Of(neuron) * c.motorJointRange
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
        yCoordinateOfLinkZero = positionOfLinkZero[1]
        fitness = xCoordinateOfLinkZero
        fh  = open("tmp"+str(self.solutionID)+".txt", "w")
        fh.write(str(fitness))
        fh.close()
        os.system("mv tmp"+str(self.solutionID)+".txt fitness"+str(self.solutionID)+".txt")
        #print(xCoordinateOfLinkZero)