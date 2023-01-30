import numpy as np
import pyrosim
import random
import os
import random

class SOLUTION:
    
    def __init__(self):
        self.weights = np.random.rand(3, 2)
        self.weights = self.weights * 2 - 1
        #print(self.weights)
        pass
    
    def evaluate(self, mode):
        self.create_world()
        self.generate_body()
        self.generate_brain()
        os.system("python3 simulate.py " + mode)
        fh = open("fitness.txt", "r")
        self.fitness = float(fh.read())
        fh.close()
        
    
    def mutate(self):
        randomRow = random.randint(0,2)
        randomColumn = random.randint(0,1)
        self.weights[randomRow, randomColumn] = random.random()*2.0 - 1.0
    
    
    
    def create_world(self):
        pyrosim.Start_SDF("world.sdf")
        
        length = 1
        width = 1
        height = 1
        
        x = -2
        y = -2
        z = 0.5
        
        pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length,width,height])
        
        
        pyrosim.End()
        
        
        
    def generate_body(self):
        
        pyrosim.Start_URDF("body.urdf")
        
        length = 1
        width = 1
        height = 1
        
        x = 1.5
        y = 0
        z = 1.5
        
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z], size=[length,width,height])
        
        x = -0.5
        y = 0
        z = -0.5
        
        pyrosim.Send_Cube(name="BackLeg", pos=[x,y,z], size=[length,width,height])
        
        x = 1
        y = 0
        z = 1
        
        pyrosim.Send_Joint(name="BackLeg_Torso", parent="Torso", child="BackLeg", type="revolute", position=[x,y,z])
        
        x = 0.5
        y = 0
        z = -0.5
        
        pyrosim.Send_Cube(name="FrontLeg", pos=[x,y,z], size=[length,width,height])
        
        x = 2
        y = 0
        z = 1
        
        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[x,y,z])
        
    
    
        pyrosim.End()
        
     
        
        
    def generate_brain(self):
        
        pyrosim.Start_NeuralNetwork("brain.nndf")
        
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg")
        
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "BackLeg")
        
        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "BackLeg_Torso")
        
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")
        
        for currentRow in range(0,3):
            for currentColumn in range(3,5):
                
                num_sensors = 3
                currentColumn = currentColumn - num_sensors
                
                w = 1-random.random()*2
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+3, weight=self.weights[currentRow][currentColumn])
    
        pyrosim.End()
        
         