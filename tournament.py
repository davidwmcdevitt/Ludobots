#pip install torch==1.10.0+cu111 torchvision==0.11.1+cu111 torchaudio==0.10.0+cu111 -f https://download.pytorch.org/whl/cu111/torch_stable.html


import time
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import constants as c
import torch
import torch.nn.functional as F
import torch.nn as nn
import torch.optim as optim
import random 
from torch.utils.data import DataLoader, TensorDataset
import pickle

class brain_model(nn.Module):
  def __init__(self,vocab, context_size, embedding_size, hidden_size, num_layers):
    super(brain_model, self).__init__()

    self.vocab = vocab
    self.embedding_size = embedding_size
    self.vocab_size = len(self.vocab)
    self.context_size = context_size
    self.num_layers = num_layers

    self.embedding = nn.Embedding(self.vocab_size, self.embedding_size, padding_idx=0)

    self.lstm = nn.LSTM(embedding_size, hidden_size, num_layers = num_layers, batch_first = True)
    self.fc1 = nn.Linear(hidden_size, self.vocab_size)
    self.dropout = nn.Dropout(0.2)

  def forward(self, inputs):
    out = self.embedding(inputs)
    out = self.dropout(out)
    lstm_out, (ht, ct) = self.lstm(out)
    
    return self.fc1(ht[-1])

vocab = list(range(1000))

loss_function = nn.CrossEntropyLoss()

model = brain_model(vocab, context_size = 296, embedding_size = 100 ,hidden_size = 128, num_layers = 2)


def initialize_challenger(form):
    
    if form == 0:
    
        dna = [random.randint(0,999) for _ in range(296)]
    
    return dna

def mutate_challenger(dna, model):
        
        
    training_data = torch.load('training_data/training_data.pt')
    
    dna_tensor = torch.tensor(dna)
    
    try:
        training_data = torch.cat((training_data,dna_tensor.unsqueeze(0)),dim = 0)
    except RuntimeError:
        training_data = torch.stack((training_data,dna_tensor),dim = 0)
    
    torch.save(training_data, 'training_data/training_data.pt', pickle_protocol=4, _use_new_zipfile_serialization=True)   
    
    train_loader = DataLoader(training_data, batch_size = training_data.shape[0], shuffle = False)  
    
    
    
    optimizer = optim.Adam(model.parameters(), lr = 0.1/ training_data.shape[0])
     
        
    model.train()
    
    for index, brain in enumerate(train_loader):
        feed = brain[:,0:296]
        target = brain[:,296]
    
        output = model(feed)
    
        loss = loss_function(output, target)
        loss.backward()
        optimizer.step()
    
        optimizer.zero_grad()
    
    model.eval()
    initial_token = torch.tensor([1]) # Provide an initial token of 1 (using index 2 since vocab index 0 is for padding and index 1 is for 0)
    #hidden = (torch.zeros(num_layers, 1, hidden_size), torch.zeros(num_layers, 1, hidden_size)) # Initialize the hidden state
    output_sequence = []
    with torch.no_grad():
       for i in range(296):
            output = model(initial_token)
            k = 2
            _, predicted = torch.max(output, dim=0)
            if len(output_sequence) > 0 :
              if predicted == output_sequence[len(output_sequence)-1]:
                predicted = torch.topk(output, k =k).indices[k-1]
                k+=1
            output_sequence.append(predicted.item())
            initial_token = torch.cat((initial_token,predicted.unsqueeze(0)))
            
    return output_sequence
    

    
def build_challenger(left_dna, right_dna):
    
    left_dna = [x / 1000 for x in left_dna]
        
    right_dna = [x / 1000 for x in right_dna]
  
    pyrosim.Start_URDF("matchup.urdf")
    
    sides = ["Left","Right"]
    
    pyrosim.Send_Cube(name= "Base", pos=[0, 0, 0.25], size = [5, 10, 0.25], col = "white")
    
    for side in sides:
        
        side_joints = []
        side_cubes = []
        
        if side == "Left":
            g = 1
            col = "green"
            eyecol = "red"
        else:
            g = -1
            col = "cyan"
            eyecol = "purple"
            
    
        pyrosim.Send_Joint(name = "Base_Root"+side, parent = "Base", child = "Root"+side, type = "fixed", position = [0,0,0], jointAxis="1 0 1")
                    
        pyrosim.Send_Cube(name= "Root"+side, pos=[0, g*5, 0.5], size = [10, 10, 0.25], col = col)
        
        side_cubes.append("Root"+side)
    
        pyrosim.Send_Joint(name = "Root"+side+"_SpineBase"+side, parent = "Root"+side, child = "SpineBase"+side, type = "prismatic", position = [0,0,0], jointAxis = "1 0 0")
           
        side_joints.append("Root"+side+"_SpineBase"+side)

        pyrosim.Send_Cube(name= "SpineBase"+side, pos=[0, g*2.5, 0.75], size = [1, 1, 0.5], col = col)
        
        side_cubes.append("SpineBase"+side)
        
        #Spine
        parent = "SpineBase"+side
        jointPos = [0,g*2.5,0.5]
        
        for spine_link in range(5):
            
            spine_link = str(spine_link)
            
            pyrosim.Send_Joint(name=parent+"_"+"Spine"+spine_link+side, parent = parent, child = "Spine"+spine_link+side, type = "revolute", position = jointPos, jointAxis = "1 1 0")
            
            side_joints.append(parent+"_"+"Spine"+spine_link+side)
            
            pyrosim.Send_Cube(name = "Spine"+spine_link+side, pos = [0,0,1], size = [0.5,0.5,1], col = col)
            
            side_cubes.append("Spine"+spine_link+side)
            
            parent = "Spine"+spine_link+side
            
            jointPos = [0,0,1]
            
        pyrosim.Send_Joint(name = parent+"_Head"+side, parent = parent, child = "Head"+side, type = "fixed", position = [0,0,1], jointAxis = "0 0 0")
        
        pyrosim.Send_Cube(name="Head"+side, pos = [0,0,1], size = [1,1,1], col = col)
        
        if side == "Left":
            leftHead_index = len(side_cubes) + 1
        else:
            rightHead_index =  len(left_cubes) + len(side_cubes) + 1
        
        pyrosim.Send_Joint(name = "Head"+side+"_LeftEye"+side, parent = "Head"+side, child = "LeftEye"+side, type = "fixed", position = [0,0,1], jointAxis = "0 0 0")
        
        pyrosim.Send_Cube(name="LeftEye"+side, pos = [0.25,-1*g*0.5,0.25], size = [0.25,0.25,0.25], col = eyecol)
        
        pyrosim.Send_Joint(name = "Head"+side+"_RightEye"+side, parent = "Head"+side, child = "RightEye"+side, type = "fixed", position = [0,0,1], jointAxis = "0 0 0")
        
        pyrosim.Send_Cube(name="RightEye"+side, pos = [-0.25,-1*g*0.5,0.25], size = [0.25,0.25,0.25], col = eyecol)
        
        arms = ["LeftArm","RightArm"]
        
        for arm in arms:
        
            parent = "Spine4"+side
            
            if arm == "LeftArm":
                z = 1
                jointPos =  [0,0,0]
                armPos = [0.5*z,0,0]
            else:
                z = -1
                jointPos =  [0,0,0]
                armPos = [0.5*z,0,0]
                
            for arm_link in range(5):
                
                arm_link = str(arm_link)
                    
                suffix =  arm+arm_link+side
                    
                pyrosim.Send_Joint(name = parent+"_Arm"+suffix, parent = parent, child = "Arm"+suffix, type = "revolute", position = jointPos, jointAxis = "1 1 0")
                
                side_joints.append(parent+"_Arm"+suffix)
                
                pyrosim.Send_Cube(name = "Arm"+suffix, pos = armPos, size = [1,0.25,0.25], col = col)
                
                side_cubes.append("Arm"+suffix)
                
                parent = "Arm"+suffix
                
                jointPos = [1*z,0,0]
                armPos = [0.5*z,0,0]
                
            pyrosim.Send_Joint(name = parent+"_Paddle"+arm+side, parent = parent , child = "Paddle"+arm+side, type="fixed", position = jointPos, jointAxis ="0 1 0")
            
            pyrosim.Send_Cube(name = "Paddle"+arm+side, pos = armPos, size = [1,0.25,1], col = col)
            
            side_cubes.append("Paddle"+arm+side)
            
    
        if side == "Left":
            left_cubes = side_cubes
            left_joints = side_joints
        else:
            right_cubes = side_cubes
            right_joints = side_joints
    
   
    
    
    pyrosim.End()
    
    #print(left_cubes)
    
    ''' GENERATE BRAIN '''
    
    pyrosim.Start_NeuralNetwork("matchup_noggin.nndf")
    
    
    numSensorsLeft = len(left_cubes)
    numMotorsLeft = len(left_joints)
    numSensorsRight = len(right_cubes)
    numMotorsRight = len(right_joints)
    
    #print(numSensorsLeft)
    #print(numMotorsLeft)
    
    numSensors = numSensorsLeft+numSensorsRight
    numMotors = numMotorsLeft+numMotorsRight
    
    numHiddenLayersLeft = 2
    numHiddenLayersRight = 2
    
    HiddenLayerSize = 8
    
    numSynapsesLeft = numSensorsLeft + numMotorsLeft + numHiddenLayersLeft * HiddenLayerSize + 3
    numSynapsesRight = numSensorsRight + numMotorsRight + numHiddenLayersRight * HiddenLayerSize + 3
    
    left_sensor_weights = np.array(left_dna[0:numSensorsLeft*HiddenLayerSize])
    left_sensor_weights = np.reshape(left_sensor_weights,(numSensorsLeft, HiddenLayerSize))
    
    #left_sensor_weights = np.random.rand(numSensorsLeft, HiddenLayerSize)
    #left_sensor_weights = left_sensor_weights * 2 - 1
    
    left_motor_weights = np.array(left_dna[numSensorsLeft*HiddenLayerSize:numSensorsLeft*HiddenLayerSize + numMotorsLeft*HiddenLayerSize])
    left_motor_weights = np.reshape(left_motor_weights,(HiddenLayerSize, numMotorsLeft))
    
    #left_motor_weights = np.random.rand(HiddenLayerSize, numMotorsLeft)
    #left_motor_weights = left_motor_weights * 2 - 1
    
    neuron_name = 0
    
    for i in left_cubes:
        pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = i)
        neuron_name +=1
    
    for i in left_joints:
        pyrosim.Send_Motor_Neuron(name = neuron_name, jointName = i)
        if i == "RootLeft_SpineBaseLeft":
            leftroot_neuron = neuron_name
        neuron_name +=1
        
    for i in range(numHiddenLayersLeft):
        for z in range(HiddenLayerSize):
            pyrosim.Send_Hidden_Neuron(name = neuron_name)
            neuron_name +=1
            
       
    pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = "HeadLeft")   
    neuron_name +=1
            
    #Sensors to Hidden
    for sensorNum in range(numSensorsLeft):
        for hiddenNum in range(HiddenLayerSize):
            pyrosim.Send_Synapse(sourceNeuronName=sensorNum, targetNeuronName=hiddenNum+numSensorsLeft+numMotorsLeft, weight=left_sensor_weights[sensorNum][hiddenNum])
    
    
    #Eyes to Hidden
    
    left_eye_weights = np.array(left_dna[numSensorsLeft*HiddenLayerSize + numMotorsLeft*HiddenLayerSize:numSensorsLeft*HiddenLayerSize + numMotorsLeft*HiddenLayerSize + 2*HiddenLayerSize])
    left_eye_weights = np.reshape(left_eye_weights,(HiddenLayerSize, 2))
    
    #left_eye_weights = np.random.rand(HiddenLayerSize, 2)
    #left_eye_weights = left_eye_weights * 2 - 1
      
    pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = "LeftEyeLeft")   
    
    for hiddenNum in range(HiddenLayerSize):
        pyrosim.Send_Synapse(sourceNeuronName=neuron_name, targetNeuronName=hiddenNum+numSensorsLeft+numMotorsLeft, weight =left_eye_weights[hiddenNum,0])
    
    neuron_name +=1
    
    pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = "RightEyeLeft")  

    for hiddenNum in range(HiddenLayerSize):
        pyrosim.Send_Synapse(sourceNeuronName=neuron_name, targetNeuronName=hiddenNum+numSensorsLeft+numMotorsLeft, weight =left_eye_weights[hiddenNum,1])
     
    neuron_name +=1
    
    #Hidden to Motors
    for hiddenNum in range(HiddenLayerSize):
        for motorNum in range(numMotorsLeft):
            pyrosim.Send_Synapse(sourceNeuronName=hiddenNum+numSensorsLeft+numMotorsLeft+3, targetNeuronName=motorNum+numSensorsLeft, weight = left_motor_weights[hiddenNum,motorNum])
       
    
    
    ### Right Brain
    right_sensor_weights = np.array(right_dna[0:numSensorsRight*HiddenLayerSize])
    right_sensor_weights = np.reshape(right_sensor_weights,(numSensorsRight, HiddenLayerSize))
    
    right_motor_weights = np.array(left_dna[numSensorsRight*HiddenLayerSize:numSensorsRight*HiddenLayerSize + numMotorsRight*HiddenLayerSize])
    right_motor_weights = np.reshape(right_motor_weights,(HiddenLayerSize, numMotorsRight))
    
    for i in right_cubes:
        pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = i)
        neuron_name +=1
    
    for i in right_joints:
        pyrosim.Send_Motor_Neuron(name = neuron_name, jointName = i)
        if i == "RootRight_SpineBaseRight":
            rightroot_neuron = neuron_name
        neuron_name +=1
        
    for i in range(numHiddenLayersRight):
        for z in range(HiddenLayerSize):
            pyrosim.Send_Hidden_Neuron(name = neuron_name)
            neuron_name +=1
     
    pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = "HeadRight")   
    neuron_name +=1
    
    #Sensors to Hidden
    for sensorNum in range(numSensorsRight):
        for hiddenNum in range(HiddenLayerSize):
            pyrosim.Send_Synapse(sourceNeuronName=sensorNum+numSynapsesLeft, targetNeuronName=hiddenNum+numSensorsRight+numMotorsRight+numSynapsesLeft, weight=right_sensor_weights[sensorNum][hiddenNum])
    
    
    #Eyes to Hidden
    right_eye_weights = np.array(right_dna[numSensorsRight*HiddenLayerSize + numMotorsRight*HiddenLayerSize:numSensorsRight*HiddenLayerSize + numMotorsRight*HiddenLayerSize + 2*HiddenLayerSize])
    right_eye_weights = np.reshape(right_eye_weights,(HiddenLayerSize, 2))
    
    #right_eye_weights = np.random.rand(HiddenLayerSize, 2)
    #right_eye_weights = right_eye_weights * 2 - 1
      
    pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = "LeftEyeRight")   
    
    for hiddenNum in range(HiddenLayerSize):
        pyrosim.Send_Synapse(sourceNeuronName=neuron_name, targetNeuronName=hiddenNum+numSensorsLeft+numMotorsLeft+numSynapsesLeft, weight =right_eye_weights[hiddenNum,0])
    
    neuron_name +=1
    
    pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = "RightEyeRight")   

    for hiddenNum in range(HiddenLayerSize):
        pyrosim.Send_Synapse(sourceNeuronName=neuron_name, targetNeuronName=hiddenNum+numSensorsLeft+numMotorsLeft+numSynapsesLeft, weight =right_eye_weights[hiddenNum,1])
    
    neuron_name +=1
    

    #Hidden to Motors
    for hiddenNum in range(HiddenLayerSize):
        for motorNum in range(numMotorsRight):
            pyrosim.Send_Synapse(sourceNeuronName=hiddenNum+numSensorsRight+numMotorsRight+numSynapsesLeft+3, targetNeuronName=motorNum+numSensorsRight+numSynapsesLeft, weight = right_motor_weights[hiddenNum,motorNum])
            
        
     
        
        
        
        
        
       
    '''
    
    weights = np.random.rand(numSensors, numMotors)
    weights = weights * 2 - 1
    
    for i in right_cubes:
        pyrosim.Send_Sensor_Neuron(name = neuron_name, linkName = i)
        neuron_name +=1
    
    for i in left_joints:
        pyrosim.Send_Motor_Neuron(name = neuron_name, jointName = i)
        if i == "RootLeft_SpineBaseLeft":
            leftroot_neuron = neuron_name
        neuron_name +=1
        
    for i in right_joints:
        pyrosim.Send_Motor_Neuron(name = neuron_name, jointName = i)
        if i == "RootRight_SpineBaseRight":
            rightroot_neuron = neuron_name
        neuron_name +=1
    
    for currentRow in range(0,numSensors):
        for currentColumn in range(0,numMotors):
            #w = 1-random.random()*2
            pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+numSensors, weight=weights[currentRow][currentColumn])
    '''
    pyrosim.End()
    
    #print("Brain Size:")
    #print(left_sensor_weights.size+left_motor_weights.size+left_eye_weights.size)
    
    return leftroot_neuron, rightroot_neuron, leftHead_index, rightHead_index
    

    
 
    

def start_world(direct):
    
    if direct == True:
        physicsClient = p.connect(p.DIRECT)
    else:
        physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    p.setGravity(0,0,-90.8)
    
    planeId = p.loadURDF("plane.urdf")
    
    p.loadSDF("world.sdf")
    
    leftAlive = True
    rightAlive = True
    
    return leftAlive, rightAlive
    
    
def prepare_matchup():
    
    robot = p.loadURDF("matchup.urdf", flags = p.URDF_USE_SELF_COLLISION)
    #robot = p.loadURDF("matchup.urdf")
    brain = NEURAL_NETWORK("matchup_noggin.nndf")
    
    pyrosim.Prepare_To_Simulate(robot)
    
    sensors = {}
    for linkName in pyrosim.linkNamesToIndices:
        linkIndex = list(pyrosim.linkNamesToIndices.keys()).index(linkName)
        #print(linkName)
        sensors[linkName] = SENSOR(linkName,linkIndex)
        
    #print(sensors)
    
    motors = {}
    for jointName in pyrosim.jointNamesToIndices:
        #print(jointName)
        motors[jointName] = MOTOR(jointName)
        
    #print(motors)   
    
    return sensors, motors, brain, robot
            

def sense(t, sensors,robot,leftHead_index, rightHead_index):
    for i in sensors.values():
        i.get_value(t,robot,leftHead_index, rightHead_index)
        
        
def think(brain):
    brain.update()


def act(t, motors, brain, robot,leftroot_neuron, rightroot_neuron):
    
    for neuron in brain.Get_Neuron_Names():
        
        if brain.Is_Motor_Neuron(neuron):
            
            if neuron == rightroot_neuron or neuron == leftroot_neuron:
                desiredAngle = 100*brain.Get_Value_Of(neuron)
            else:
                desiredAngle = brain.Get_Value_Of(neuron)
            
            jointName = brain.Get_Motor_Neurons_Joint(neuron)
            motors[jointName].set_value(robot, desiredAngle)
            
def check(robot, leftHead_index, rightHead_index):
    
    rightHeadState = p.getLinkState(robot,rightHead_index)
    positionOfRightHead = rightHeadState[0]
    
    zCoordinateOfRightHead = positionOfRightHead[2]
        
    leftHeadState = p.getLinkState(robot,leftHead_index)
    positionOfLeftHead = leftHeadState[0]
    
    zCoordinateOfLeftHead = positionOfLeftHead[2]
    
    if zCoordinateOfLeftHead <= 3:
        leftAlive = False
    else:
        leftAlive = True
    
    #print(zCoordinateOfRightHead)
    if zCoordinateOfRightHead <= 3:
        
        rightAlive = False
    else:
        rightAlive = True
        

    
    if pyrosim.Get_Touch_Sensor_Value_For_Link("HeadLeft") == 1:
        leftAlive = False
    if pyrosim.Get_Touch_Sensor_Value_For_Link("HeadRight") == 1:
        rightAlive = False
    
    return leftAlive, rightAlive
            

scoreLeft = 0
scoreRight = 0
          
left_dna=initialize_challenger(0)
right_dna=initialize_challenger(0)

gen = 0

mutation_type = "LSTM"
#mutation_type = "Simple"

for i in range(0):     
    print("Round" + str(i))
    
    leftroot_neuron, rightroot_neuron, leftHead_index, rightHead_index = build_challenger(left_dna, right_dna)
    
    leftAlive, rightAlive = start_world(direct= True)
    sensors, motors, brain, robot = prepare_matchup()
     
    t = 0
    while leftAlive == True and rightAlive == True:
        p.stepSimulation()
        
        sense(t, sensors, robot, leftHead_index, rightHead_index)
        think(brain)
        act(t, motors, brain, robot,leftroot_neuron, rightroot_neuron)
        leftAlive, rightAlive = check(robot, leftHead_index, rightHead_index)
        #time.sleep(1/5000)
        
        t+=1
        if t == 10000:
            leftAlive = False
            rightAlive = False
     
    if mutation_type == "LSTM":
        if leftAlive == False:
            
            if rightAlive == True:
            
                left_tensor = torch.tensor(left_dna)
                left_tensor = torch.cat((torch.tensor([0]),left_tensor))
                
                left_dna = mutate_challenger(left_tensor,model)
                gen +=1
                print(gen)
            else:
                left_dna=initialize_challenger(0)
                gen +=1
                print(gen)
            
        else:
            
            if rightAlive == False:
            
                left_tensor = torch.tensor(left_dna)
                left_tensor = torch.cat((torch.tensor([1]),left_tensor))
                
                _ = mutate_challenger(left_tensor,model)
                gen +=1
                print(gen)
            else:
                left_dna=initialize_challenger(0)
                gen +=1
                print(gen)
            
        if rightAlive == False:
            
            if leftAlive == True:
                
                right_tensor = torch.tensor(right_dna)
                right_tensor = torch.cat((torch.tensor([0]),right_tensor))
                
                right_dna = mutate_challenger(right_tensor,model)
                gen +=1
                print(gen)
            else:
                right_dna=initialize_challenger(0)
                gen +=1
                print(gen)
            
        else:
            
            if leftAlive == False:
                
                right_tensor = torch.tensor(right_dna)
                right_tensor = torch.cat((torch.tensor([1]),right_tensor))
                
                _ = mutate_challenger(left_tensor,model)
                gen +=1
                print(gen)
            else:
                right_dna=initialize_challenger(0)
                gen +=1
                print(gen)
        torch.save(model.state_dict(), "training_data/brain_model_"+str(int(time.time()))+".pt")     
        with open('training_data/lstmWiggle_'+str(int(time.time()))+'.pkl', 'wb') as f:
            pickle.dump(right_dna, f)
            
        time.sleep(1)
        with open('training_data/lstmWiggle_'+str(int(time.time()))+'.pkl', 'wb') as f:
            pickle.dump(left_dna, f)   
            
    if mutation_type == "Simple":
        if leftAlive == False:
            indices = random.sample(range(len(left_dna)), 10)
            for i in indices:
                left_dna[i] = random.randint(2, 1000)
            
            gen +=1
            print(gen)

        if rightAlive == False:
            indices = random.sample(range(len(right_dna)), 10)
            for i in indices:
                right_dna[i] = random.randint(2, 1000)
                
            gen +=1
            print(gen)

        with open('training_data/organicWiggle_'+str(int(time.time()))+'.pkl', 'wb') as f:
            pickle.dump(right_dna, f)
            
        time.sleep(1)
        with open('training_data/organicWiggle_'+str(int(time.time()))+'.pkl', 'wb') as f:
            pickle.dump(left_dna, f)   
  
    
'''
leftroot_neuron, rightroot_neuron, leftHead_index, rightHead_index = build_challenger(left_dna, right_dna)   

leftAlive, rightAlive = start_world(direct= False)

sensors, motors, brain, robot = prepare_matchup()
 
t = 0
while leftAlive == True and rightAlive == True:
    p.stepSimulation()
    
    sense(t, sensors, robot, leftHead_index, rightHead_index)
    think(brain)
    act(t, motors, brain, robot,leftroot_neuron, rightroot_neuron)
    leftAlive, rightAlive = check(robot, leftHead_index, rightHead_index)
    #time.sleep(1/5000)
    
    t+=1
    
    '''


    