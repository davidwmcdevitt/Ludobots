import numpy as np
import pyrosim.pyrosim as pyrosim
import random
import os
import constants as c
import time

class SOLUTION:
    
    def __init__(self,myID):
        #self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        #self.weights = self.weights * 2 - 1
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
        self.generate_dna()
        self.generate_snake()
        #self.generate_body()
        #self.generate_brain()
        
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
        
        numLinks = self.dna['num_links']
        numSensors = sum(self.dna['sensors'])
        randomRow = random.randint(0,numSensors-1)
        randomColumn = random.randint(0,numLinks-1)
        #print(self.weights)
        self.weights[randomRow, randomColumn] = random.random()*2.0 - 1.0
        #print(self.weights)

    
        
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
        
    def generate_dna(self):
        
        self.dna = {}
        self.dna['num_links'] = random.randint(6,15)
        x_link = []
        y_link = []
        z_link = []
        sensors = []
        for i in range(self.dna['num_links']):
            x_link.append(random.random())
            y_link.append(random.random())
            z_link.append(random.random())
            #x_link.append(1)
            #y_link.append(1)
            #z_link.append(1)
            sensors.append(random.randint(0,1))
            
        self.dna['x_link'] = x_link
        self.dna['y_link'] = y_link
        self.dna['z_link'] = z_link
        self.dna['sensors'] = sensors
        
        self.dna['sensors'][0] = 1
        
    def generate_snake(self):
        
        pyrosim.Start_URDF("snek.urdf")
        
        numLinks = self.dna['num_links']
        numSensors = sum(self.dna['sensors'])
        
        #maxZ = np.argmax(self.dna['z_link'])
        
        pos_x = 0
        pos_y = 0
        pos_z = 0
        
        linkName = "Link0"
        
        i = 0
        pyrosim.Send_Cube(name= linkName, pos=[pos_x,pos_y,pos_z], size=[self.dna['x_link'][i],self.dna['y_link'][i],self.dna['z_link'][i]], col = "green")
        
        #pos_y = self.dna['y_link'][0] / 2
        
        numLinks -= 1
        for z in range(numLinks):
            i = z + 1
            
            parentName = linkName
            linkName = "Link" + str(i)
            
            pos_y = self.dna['y_link'][z] + self.dna['y_link'][i]/2
            
            col = "cyan"
            if self.dna['sensors'][i] == 1:
                col = "green"
                
            pyrosim.Send_Cube(name= linkName, pos=[pos_x,pos_y,pos_z], size=[self.dna['x_link'][i],self.dna['y_link'][i],self.dna['z_link'][i]], col = col)

            jointName= parentName + "_" + linkName
            joint_x = 0
            joint_y = self.dna['y_link'][z]
            joint_z = 0
            
            #print(joint_x,joint_y,joint_z)
            
            temp = random.randint(0,2)*2
            jointAxis ="0 0 0"
            jointAxis = jointAxis[:temp] + "1" + jointAxis[temp + 1:]
            #joint_types = ["revolute","continuous","prismatic","fixed","floating","planar"]
            joint_types = ["revolute","prismatic","fixed","floating","planar"]
            temp = random.randint(0,4)
            jointType = joint_types[temp]
            
            pyrosim.Send_Joint(name = jointName, parent = parentName, child = linkName, type=jointType, position=[joint_x,joint_y,joint_z], jointAxis=jointAxis)

                
        pyrosim.End()
        
        self.weights = np.random.rand(numSensors, numLinks+1)
        self.weights = self.weights * 2 - 1
        
        pyrosim.Start_NeuralNetwork("snek_noggin.nndf")
        
        neuron_id = 0
        
        linkName = ''
        
        for i in range(numLinks):
            
            if self.dna['sensors'][i] == 1:
                pyrosim.Send_Sensor_Neuron(name=neuron_id, linkName="Link" + str(i))
                neuron_id += 1
                
        
        for i in range(numLinks):
            
            parentName = linkName
            
            linkName = "Link" + str(i)
            
            if i != 0:
                
                jointName= parentName + "_" + linkName
                
                pyrosim.Send_Motor_Neuron(name=neuron_id, jointName=jointName)
                
                neuron_id += 1
            
        for currentRow in range(numSensors):
            for currentColumn in range(numLinks):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+numSensors, weight=self.weights[currentRow][currentColumn])
    
            
        pyrosim.End()
        
        
        
         
