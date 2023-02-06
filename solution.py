import numpy as np
import pyrosim.pyrosim as pyrosim
import random
import os
import constants as c
import time

class SOLUTION:
    
    def __init__(self,myID):
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights = self.weights * 2 - 1
        #print(self.weights)
        self.myID = myID
    
    def set_ID(self, myID):
        self.myID = myID
        
    def evaluate(self,mode):
        self.create_world()
        self.generate_body()
        self.generate_brain()
        
        os.system("python simulate.py " + mode + " " + str(self.myID) + " 2&>1" + " &")
        
        #os.system("python3 simulate.py " + mode)
        fitnessFileName = "fitness"+str(self.myID)+".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        fh = open("fitness"+str(self.myID)+".txt", "r")
        self.fitness = float(fh.read())
        #print(self.fitness)
        fh.close()
        
    def start_simulation(self,mode):
        self.create_world()
        self.generate_body()
        self.generate_brain()
        
        os.system("python simulate.py " + mode + " " + str(self.myID) + " " + " &")
        
    
    def wait_for_simulation_to_end(self):
        fitnessFileName = "fitness"+str(self.myID)+".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        fh = open("fitness"+str(self.myID)+".txt", "r")
        self.fitness = float(fh.read())
        os.system("rm fitness"+str(self.myID)+".txt")
        #print(self.fitness)
        fh.close()
        
    def mutate(self):
        randomRow = random.randint(0,c.numSensorNeurons-1)
        randomColumn = random.randint(0,c.numMotorNeurons-1)
        #print(self.weights)
        self.weights[randomRow, randomColumn] = random.random()*2.0 - 1.0
        #print(self.weights)

    
        
    def create_world(self):
        pyrosim.Start_SDF("world.sdf")
    
        length = 1
        width = 1
        height = 1
        
        for i in range(1):
            x = random.randint(0, 10)
            y = random.randint(0, 10)
            z = 0.5
            
            name = "Box" + str(i)
            
            pyrosim.Send_Cube(name=name, pos=[x,y,z] , size=[length,width,height])
        
        
        pyrosim.End()
        
        
        
    def generate_body(self):
        
        pyrosim.Start_URDF("body.urdf")
        
        length = 1
        width = 1
        height = 1
        
        x = 0
        y = 0
        z = 1
        
        pyrosim.Send_Cube(name="Torso", pos=[x,y,z], size=[length,width,height])
        
        x = 0
        y = -0.5
        z = 0
        
        length = 0.2
        width = 1
        height = 0.2
        
        pyrosim.Send_Cube(name="BackLeg", pos=[x,y,z], size=[length,width,height])
        
        x = 0
        y = -0.5
        z = 1
        
        pyrosim.Send_Joint(name="BackLeg_Torso", parent="Torso", child="BackLeg", type="revolute", position=[x,y,z], jointAxis = "-1 0 0")
        
        x = 0
        y = 0.5
        z = 0
        
        length = 0.2
        width = 1
        height = 0.2
        
        pyrosim.Send_Cube(name="FrontLeg", pos=[x,y,z], size=[length,width,height])
        
        x = 0
        y = 0.5
        z = 1
        
        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[x,y,z], jointAxis = "-1 0 0")
        
        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg", type="revolute", position=[-0.5,0.0,1.0], jointAxis="0 1 0")
       
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0.0,0.0], size=[1.0,0.2,0.2])
        
        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg", type="revolute", position=[0.5,0.0,1.0], jointAxis="0 1 0")
        
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0.0,0.0], size=[1.0,0.2,0.2])
        
        
        pyrosim.Send_Joint(name="FrontLeg_FrontLowerLeg", parent="FrontLeg", child="FrontLowerLeg", type="revolute", position=[0.0,1.0,0.0], jointAxis="1 0 0")
        
        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0.0,0.0,-0.5], size=[0.2,0.2,1.0])
        
        pyrosim.Send_Joint(name="BackLeg_BackLowerLeg", parent="BackLeg", child="BackLowerLeg", type="revolute", position=[0.0,-1.0,0.0], jointAxis="1 0 0")
        
        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0.0,0.0,-0.5], size=[0.2,0.2,1.0])
        
        
        pyrosim.Send_Joint(name="LeftLeg_LeftLowerLeg", parent="LeftLeg", child="LeftLowerLeg", type="revolute", position=[-1.0,0.0,0.0], jointAxis="0 1 0")
        
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0.0,0.0,-0.5], size=[0.2,0.2,1.0])
        
        pyrosim.Send_Joint(name="RightLeg_RightLowerLeg", parent="RightLeg", child="RightLowerLeg", type="revolute", position=[1.0,0.0,0.0], jointAxis="0 1 0")
        
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0.0,0.0,-0.5], size=[0.2,0.2,1.0])
        
    
    
        pyrosim.End()
        
     
        
        
    def generate_brain(self):
        
        
        
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        
        
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLowerLeg")
        
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLowerLeg")
        
        pyrosim.Send_Sensor_Neuron(name=3, linkName="LeftLowerLeg")
        
        pyrosim.Send_Sensor_Neuron(name=4, linkName="RightLowerLeg")
        
        pyrosim.Send_Motor_Neuron(name=5, jointName="BackLeg_Torso")
        
        pyrosim.Send_Motor_Neuron(name=6, jointName="Torso_FrontLeg")
        
        pyrosim.Send_Motor_Neuron(name=7, jointName="Torso_LeftLeg")
        
        pyrosim.Send_Motor_Neuron(name=8, jointName="Torso_RightLeg")
        
        pyrosim.Send_Motor_Neuron(name=9, jointName="BackLeg_BackLowerLeg")
        
        pyrosim.Send_Motor_Neuron(name=10, jointName="FrontLeg_FrontLowerLeg")
        
        pyrosim.Send_Motor_Neuron(name=11, jointName="LeftLeg_LeftLowerLeg")
        
        pyrosim.Send_Motor_Neuron(name=12, jointName="RightLeg_RightLowerLeg")
        
        for currentRow in range(0,c.numSensorNeurons):
            for currentColumn in range(0,c.numMotorNeurons):
                #w = 1-random.random()*2
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+c.numSensorNeurons, weight=self.weights[currentRow][currentColumn])
                
        
        pyrosim.End()
        
        
        
         
