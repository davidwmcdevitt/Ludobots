import numpy as np
import pyrosim.pyrosim as pyrosim
import random
import os
import constants as c
import time

class SOLUTION:
    
    def __init__(self,myID):
        '''HERE CHANGE'''
        #self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        #self.weights = self.weights * 2 - 1
        #print(self.weights)
        self.myID = myID
    
    def set_ID(self, myID):
        self.myID = myID
        
    def evaluate(self,mode):
        self.create_world()
        #self.generate_body()
        #self.generate_brain()
        self.generate_tardigrade(self.myID) 
        '''HERE CHANGE'''
        
        
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
        #self.generate_dna()
        #self.generate_snake()
        #self.generate_body()
        #self.generate_brain())
        self.generate_tardigrade(self.myID)
        
        os.system("python simulate.py " + mode + " " + str(self.myID) + " " + " &")
        
    
    def wait_for_simulation_to_end(self):
        fitnessFileName = "fitness"+str(self.myID)+".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.001)
        fh = open("fitness"+str(self.myID)+".txt", "r")
        self.fitness = float(fh.read())
        os.system("rm fitness"+str(self.myID)+".txt")
        #print(self.fitness)
        fh.close()
        
    def mutate(self):
        
        numSensors = sum(self.dna['legs']) * 2 + 1
        
        numMotors = sum(self.dna['legs']) * 2
        
        randomRow = random.randint(0,numSensors-1)
        randomColumn = random.randint(0,numMotors-1)
        #print(self.weights)
        self.weights[randomRow, randomColumn] = random.random()*2.0 - 1.0
        '''HERE CHANGE'''
        #print(self.weights)

    
        
    def create_world(self):
        pyrosim.Start_SDF("world.sdf")
    
        length = 1
        width = 1
        height = 1
        
        x = -2
        y = -2
        z = 0.5
        
        #pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length,width,height])
        
        
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
        self.dna['num_links'] = random.randint(6,30)
        #self.dna['num_links'] = 5
        x_link = []
        y_link = []
        z_link = []
        sensors = []
        for i in range(self.dna['num_links']):
            x_link.append(random.random())
            y_link.append(random.random())
            #z_link.append(random.random())
            #x_link.append(1)
            #y_link.append(1)
            z_link.append(1)
            sensors.append(random.randint(0,1))
            
        self.dna['x_link'] = x_link
        self.dna['y_link'] = y_link
        self.dna['z_link'] = z_link
        #self.dna['x_link'] = [1, 0.75, 0.5, 0.5, 0.25]
        #elf.dna['y_link'] = [1, 0.5, 0.25, 0.25, 0.25]
        #self.dna['z_link'] = [1, 1, 1, 1, 1]
        self.dna['sensors'] = sensors
        #self.dna['sensors'] = [1, 0, 0, 0, 1]
        
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
            
            #pos_y = self.dna['y_link'][z] + self.dna['y_link'][i]/2
            pos_y = self.dna['y_link'][i]/2
            if z == 0:
                pass
                #pos_y = 0
            
            col = "cyan"
            if self.dna['sensors'][i] == 1:
                col = "green"
                
            pyrosim.Send_Cube(name= linkName, pos=[pos_x,pos_y,pos_z], size=[self.dna['x_link'][i],self.dna['y_link'][i],self.dna['z_link'][i]], col = col)

            jointName= parentName + "_" + linkName
            joint_x = 0
            joint_y = self.dna['y_link'][z]
            joint_z = 0
            
            if z == 0:
                joint_y = self.dna['y_link'][z]/2
            if z == 1:
                joint_y = self.dna['y_link'][z]
            
            #print(joint_x,joint_y,joint_z)
            
            #temp = random.randint(0,1)*2
            jointAxis ="1 0 1"
            #jointAxis = jointAxis[:temp] + "1" + jointAxis[temp + 1:]
            #joint_types = ["revolute","continuous","prismatic","fixed","floating","planar"]
            joint_types = ["revolute","prismatic","fixed"]
            temp = random.randint(0,2)
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
        
        
    def generate_tardigrade(self, myID):
        
        ''' GENERATE DNA '''
        
        self.dna = {}
        self.dna['num_links'] = random.randint(2,8) 
        self.dna['size'] = random.randint(1,1) #This could be a continuous value later
        
        angles = []
        joints = []
        legs = []
        
        for i in range(self.dna['num_links']):
            #angles.append(random.choice(['left','right','straight']))
            angles.append('straight')
            joints.append(random.choice(['fixed','prismatic','revolute']))
            legs.append(random.randint(0,1))
         
        
        for angle in ['left','right']:
            
            if angles.count(angle) >= 3:
                count = angles.count(angle)
                
                for i in range(count - 2):
                    
                    temp = False
                    
                    while temp == False:
                        try:
                            angles[angles.index(angle, random.randint(0,len(angles)-1))] = 'straight'
                            temp = True
                        except ValueError:
                            pass
        
        angles[0] = 'head'
        legs[0] = 1
        legs[self.dna['num_links']-1] = 1
        angles[1] = 'straight'
        
        self.dna['angles'] = angles
        self.dna['joints'] = joints
        self.dna['legs'] = legs
        
        
        ''' GENERATE BODY '''
        
        
        pyrosim.Start_URDF("tardigrade" + str(myID) + ".urdf")
        
        numLinks = self.dna['num_links']
        
        legSize = self.dna['size'] / 6
        
        for i in range(numLinks):
            
            if self.dna['angles'][i] == 'head':
                
                pyrosim.Send_Cube(name= "Head", pos=[0 ,0 ,2*self.dna['size']], size=[self.dna['size'],self.dna['size'],self.dna['size']], col = "green")
                
                pyrosim.Send_Joint(name = "Head_Link0", parent = "Head", child = "Link0", type="fixed", position=[0,0,self.dna['size']], jointAxis="1 0 1")
                
                pyrosim.Send_Cube(name= "Link0", pos=[0,0,0], size=[self.dna['size'],self.dna['size'],self.dna['size']], col = "cyan")
                
                #pyrosim.Send_Joint(name = "Link0_Link0zNode", parent = "Link0", child = "Link0zNode", type = "revolute", position = [0,0,0], jointAxis="0 0 1")
                
                #pyrosim.Send_Joint(name = "Link0_Link0yNode", parent = "Link0", child = "Link0yNode", type = "revolute", position = [0,0,0], jointAxis="0 1 0")
                
                #pyrosim.Send_Cube(name= "Link0zNode", pos=[0,0,0.1], size=[0.1,0.1,0.1], col = "cyan")
                
                #pyrosim.Send_Cube(name= "Link0yNode", pos=[0,0.1,0], size=[0.1,0.1,0.1], col = "cyan")
                
                pyrosim.Send_Joint(name = "Link0_Link0Right", parent = "Link0", child = "Link0Right", type = "revolute", position = [0,0,0], jointAxis="0 1 1")
                #pyrosim.Send_Joint(name = "Link0yNode_Link0Right", parent = "Link0yNode", child = "Link0Right", type = "revolute", position = [0,0,0], jointAxis="0 1 0")
                #pyrosim.Send_Joint(name = "Link0zNode_Link0Right", parent = "Link0zNode", child = "Link0Right", type = "revolute", position = [0,0,0], jointAxis="0 1 1")
                
                pyrosim.Send_Cube(name = "Link0Right", pos = [self.dna['size'],0,0], size = [self.dna['size'], legSize, legSize], col = "cyan")
                
                pyrosim.Send_Joint(name = "Link0_Link0Left", parent = "Link0", child = "Link0Left", type = "revolute", position = [0,0,0], jointAxis="0 1 1")
                #pyrosim.Send_Joint(name = "Link0yNode_Link0Left", parent = "Link0yNode", child = "Link0Left", type = "revolute", position = [0,0,0], jointAxis="0 1 0")
                #pyrosim.Send_Joint(name = "Link0zNode_Link0Left", parent = "Link0zNode", child = "Link0Left", type = "revolute", position = [0,0,0], jointAxis="0 1 1")
                
                pyrosim.Send_Cube(name = "Link0Left", pos = [-self.dna['size'],0,0], size = [self.dna['size'], legSize, legSize], col = "cyan")
                
                pyrosim.Send_Joint(name = "Link0Right_Link0RightFoot", parent = "Link0Right", child = "Link0RightFoot", type = "fixed", position = [self.dna['size'],0,self.dna['size']/2], jointAxis="1 0 1")
                
                pyrosim.Send_Cube(name = "Link0RightFoot", pos = [self.dna['size']/2,0,-self.dna['size']], size = [legSize, legSize, self.dna['size']], col = "green")
                
                pyrosim.Send_Joint(name = "Link0Left_Link0LeftFoot", parent = "Link0Left", child = "Link0LeftFoot", type = "fixed", position = [self.dna['size'],0,self.dna['size']/2], jointAxis="1 0 1")
                
                pyrosim.Send_Cube(name = "Link0LeftFoot", pos = [-self.dna['size']*2.5,0,-self.dna['size']], size = [legSize, legSize, self.dna['size']], col = "green")
                
                parentName = "Link0"
                
                
            if self.dna['angles'][i] == 'straight':
                
                linkName = "Link" + str(i)
                
                pyrosim.Send_Joint(name = parentName + "_" + linkName, parent = parentName, child = linkName, type = "fixed", position =[0,self.dna['size'],0], jointAxis = "0 1 0")
                
                pyrosim.Send_Cube(name= linkName, pos=[0,0,0], size=[self.dna['size'],self.dna['size'],self.dna['size']], col = "cyan")
                
                if self.dna['legs'][i] == 1:
                    
                    #pyrosim.Send_Joint(name = linkName +"_" + linkName + "zNode", parent = linkName, child = linkName + "zNode", type = "revolute", position = [0,0,0], jointAxis="0 0 1")
                
                    #pyrosim.Send_Joint(name = linkName +"_" + linkName + "yNode", parent = linkName, child = linkName + "yNode", type = "revolute", position = [0,0,0], jointAxis="0 1 0")
                
                    #pyrosim.Send_Cube(name= linkName + "zNode", pos=[0,0,0.1], size=[0.1,0.1,0.1], col = "cyan")
                    
                    #pyrosim.Send_Cube(name= linkName + "yNode", pos=[0,0.1,0], size=[0.1,0.1,0.1], col = "cyan")
                    
                    pyrosim.Send_Joint(name = linkName + "_" + linkName + "Right", parent = linkName, child = linkName + "Right", type = "revolute", position = [0,0,0], jointAxis="0 1 1")
                    #pyrosim.Send_Joint(name = linkName + "yNode_" + linkName + "Right", parent = linkName + "yNode", child = linkName + "Right", type = "revolute", position = [0,0,0], jointAxis="0 1 0")
                    #pyrosim.Send_Joint(name = linkName + "zNode_" + linkName + "Right", parent = linkName+ "zNode", child = linkName + "Right", type = "revolute", position = [0,0,0], jointAxis="0 1 1")
                    
                    pyrosim.Send_Cube(name = linkName + "Right", pos = [self.dna['size'],0,0], size = [self.dna['size'], legSize, legSize], col = "cyan")
                    
                    pyrosim.Send_Joint(name = linkName + "_" + linkName + "Left", parent = linkName, child = linkName + "Left", type = "revolute", position = [0,0,0], jointAxis="0 1 1")
                    
                    #pyrosim.Send_Joint(name = "Link0_Link0Left", parent = "Link0", child = "Link0_Left", type = "revolute", position = [self.dna['size'],0,self.dna['size']/2], jointAxis="0 0 1")
                    #pyrosim.Send_Joint(name = linkName + "yNode_" + linkName + "Left", parent = linkName + "yNode", child = linkName + "Left", type = "revolute", position = [0,0,0], jointAxis="0 1 0")
                    #pyrosim.Send_Joint(name = linkName + "zNode_" + linkName + "Left", parent = linkName+ "zNode", child = linkName + "Left", type = "revolute", position = [0,0,0], jointAxis="0 1 1")
                    
                    pyrosim.Send_Cube(name = linkName + "Left", pos = [-self.dna['size'],0,0], size = [self.dna['size'], legSize, legSize], col = "cyan")
                    
                    pyrosim.Send_Joint(name = linkName+"Right_"+linkName+"RightFoot", parent = linkName+"Right", child = linkName+"RightFoot", type = "fixed", position = [self.dna['size'],0,self.dna['size']/2], jointAxis="1 0 1")
                    
                    pyrosim.Send_Cube(name = linkName+"RightFoot", pos = [self.dna['size']/2,0,-self.dna['size']], size = [legSize, legSize, self.dna['size']], col = "green")
                    
                    pyrosim.Send_Joint(name = linkName+"Left_"+linkName+"LeftFoot", parent = linkName+"Left", child = linkName+"LeftFoot", type = "fixed", position = [self.dna['size'],0,self.dna['size']/2], jointAxis="1 0 1")
                    
                    pyrosim.Send_Cube(name = linkName+"LeftFoot", pos = [-self.dna['size']*2.5,0,-self.dna['size']], size = [legSize, legSize, self.dna['size']], col = "green")
                    
                parentName = linkName
                
            if self.dna['angles'][i] == 'left':
                pass
            if self.dna['angles'][i] == 'right':
                pass
            
        pyrosim.End()
            
            
        ''' GENERATE BRAIN '''
        
        pyrosim.Start_NeuralNetwork("tardigrade_noggin" + str(self.myID) + ".nndf")
        
        numSensors = sum(self.dna['legs']) * 2 + 1
        
        numMotors = sum(self.dna['legs']) * 2
        
        self.weights = np.random.rand(numSensors, numMotors)
        self.weights = self.weights * 2 - 1
        
        neuron_id = 0
        
        pyrosim.Send_Sensor_Neuron(name=neuron_id, linkName="Head")
        neuron_id += 1
        
        for i in range(numLinks):
            
            linkName = "Link" + str(i)
            
            if self.dna['angles'][i] == 'head':
                pass
                
                
            if self.dna['legs'][i] == 1:
                pyrosim.Send_Sensor_Neuron(name=neuron_id, linkName= linkName + "RightFoot")
                neuron_id += 1
                pyrosim.Send_Sensor_Neuron(name=neuron_id, linkName= linkName + "LeftFoot")
                neuron_id += 1
            
            
            
        for i in range(numLinks):
            
            linkName = "Link" + str(i)
                
            if self.dna['legs'][i] == 1:
                    
                jointName= linkName + "_" + linkName + "Right"
                
                pyrosim.Send_Motor_Neuron(name=neuron_id, jointName=jointName)
                
                neuron_id += 1
                        
                #jointName= linkName + "zNode_" + linkName + "Right"
                
                #pyrosim.Send_Motor_Neuron(name=neuron_id, jointName=jointName)
                
                #neuron_id += 1
                        
                jointName= linkName + "_" + linkName + "Left"
                
                pyrosim.Send_Motor_Neuron(name=neuron_id, jointName=jointName)
                
                neuron_id += 1
                        
                #jointName= linkName + "zNode_" + linkName + "Left"
                
                #pyrosim.Send_Motor_Neuron(name=neuron_id, jointName=jointName)
                
                #neuron_id += 1
            
        for currentRow in range(numSensors):
            for currentColumn in range(numMotors):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+numSensors, weight=self.weights[currentRow][currentColumn])
    
        
        pyrosim.End()
        
            
         
