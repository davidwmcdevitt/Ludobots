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




def initialize_challenger(formtype,challengerID):
    
    dna = {}
    
    
    '''
    DNA Pieces
    
    Base
    Fractal Straight
    Fractal Right
    Fractal Left
    Fractal Front
    Fractal Back
    Fractal Both
    Restore
    
    '''
    
    '''
    Formtypes designate initial DNA states - for the time being everything is 
    going to be initialized from a single body in the middle of the platform.
    
    To-do: Add different spawn locations, growth directions(?) + brainstorm
    '''
    
    dna['challengerID'] = challengerID
    dna['spine_links'] = 8
    dna['head_x'] = 0
    dna['head_y'] = 0
    
    return dna

def mutate_challenger(dna):
    '''
    Takes a challenger DNA and inserts a mutation. Mutation options include
        1. Growing taller
        2. Adding limbs
        3. Making limbs longer
        4. Changing joint types on limbs
        5. Motor Force
        6. Joint range
    '''
    return dna
    

    
def build_challenger(left_dna, right_dna):
  
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
    
        pyrosim.Send_Joint(name = "Root"+side+"_SpineBase"+side, parent = "Root"+side, child = "SpineBase"+side, type = "prismatic", position = [0,0,0], jointAxis = "1 1 0")
           
        side_joints.append("Root"+side+"_SpineBase"+side)

        pyrosim.Send_Cube(name= "SpineBase"+side, pos=[0, g*5, 0.75], size = [1, 1, 0.5], col = col)
        
        side_cubes.append("SpineBase"+side)
        
        #Spine
        parent = "SpineBase"+side
        jointPos = [0,g*5,0.5]
        
        for spine_link in range(5):
            
            spine_link = str(spine_link)
            
            pyrosim.Send_Joint(name=parent+"_"+"Spine"+spine_link+side, parent = parent, child = "Spine"+spine_link+side, type = "revolute", position = jointPos, jointAxis = "0 1 0")
            
            side_joints.append(parent+"_"+"Spine"+spine_link+side)
            
            pyrosim.Send_Cube(name = "Spine"+spine_link+side, pos = [0,0,1], size = [0.5,0.5,1], col = col)
            
            side_cubes.append("Spine"+spine_link+side)
            
            parent = "Spine"+spine_link+side
            
            jointPos = [0,0,1]
            
        pyrosim.Send_Joint(name = parent+"_Head"+side, parent = parent, child = "Head"+side, type = "fixed", position = [0,0,1], jointAxis = "0 0 0")
        
        pyrosim.Send_Cube(name="Head"+side, pos = [0,0,1], size = [1,1,1], col = col)
        
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
                    
                pyrosim.Send_Joint(name = parent+"_Arm"+suffix, parent = parent, child = "Arm"+suffix, type = "revolute", position = jointPos, jointAxis = "1 1 1")
                
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
    
    print(left_cubes)
    
    ''' GENERATE BRAIN '''
    
    pyrosim.Start_NeuralNetwork("matchup_noggin.nndf")
    
    
    numSensorsLeft = len(left_cubes)
    numMotorsLeft = len(left_joints)
    numSensorsRight = len(right_cubes)
    numMotorsRight = len(right_joints)
    
    numSensors = numSensorsLeft+numSensorsRight
    numMotors = numMotorsLeft+numMotorsRight
    
    numHiddenLayersLeft = 1
    numHiddenLayersRight = 1
    
    HiddenLayerSize = 10
    
    numSynapsesLeft = numSensorsLeft + numMotorsLeft + numHiddenLayersLeft * HiddenLayerSize
    numSynapsesRight = numSensorsRight + numMotorsRight + numHiddenLayersRight * HiddenLayerSize
    
    weights = np.random.rand(numSensors, numMotors)
    weights = weights * 2 - 1
    
    left_sensor_weights = np.random.rand(numSensorsLeft, HiddenLayerSize)
    left_sensor_weights = left_sensor_weights * 2 - 1
    
    left_motor_weights = np.random.rand(HiddenLayerSize, numMotorsLeft)
    left_motor_weights = left_motor_weights * 2 - 1
    
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
            
    #Sensors to Hidden
    for sensorNum in range(numSensorsLeft):
        for hiddenNum in range(HiddenLayerSize):
            pyrosim.Send_Synapse(sourceNeuronName=sensorNum, targetNeuronName=hiddenNum+numSensorsLeft+numMotorsLeft, weight=left_sensor_weights[sensorNum][hiddenNum])
    
    #Hidden to Hidden         
    if numHiddenLayersLeft > 1:
        pass
    

    #Hidden to Motors
    for hiddenNum in range(HiddenLayerSize):
        for motorNum in range(numMotorsLeft):
            pyrosim.Send_Synapse(sourceNeuronName=hiddenNum+numSensorsLeft+numMotorsLeft, targetNeuronName=motorNum+numSensorsLeft, weight = left_motor_weights[hiddenNum,motorNum])
       
    
    
    
    right_sensor_weights = np.random.rand(numSensorsRight, HiddenLayerSize)
    right_sensor_weights = right_sensor_weights * 2 - 1
    
    right_motor_weights = np.random.rand(HiddenLayerSize, numMotorsRight)
    right_motor_weights = right_motor_weights * 2 - 1
    
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
            
    #Sensors to Hidden
    for sensorNum in range(numSensorsRight):
        for hiddenNum in range(HiddenLayerSize):
            pyrosim.Send_Synapse(sourceNeuronName=sensorNum+numSynapsesLeft, targetNeuronName=hiddenNum+numSensorsRight+numMotorsRight+numSynapsesLeft, weight=right_sensor_weights[sensorNum][hiddenNum])
    
    #Hidden to Hidden         
    if numHiddenLayersRight > 1:
        pass
    

    #Hidden to Motors
    for hiddenNum in range(HiddenLayerSize):
        for motorNum in range(numMotorsRight):
            pyrosim.Send_Synapse(sourceNeuronName=hiddenNum+numSensorsRight+numMotorsRight+numSynapsesLeft, targetNeuronName=motorNum+numSensorsRight+numSynapsesLeft, weight = right_motor_weights[hiddenNum,motorNum])
            
        
        
        
        
        
        
        
        
       
    '''
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
    
    return leftroot_neuron, rightroot_neuron
    

    
 
    

def start_world():
        
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    p.setGravity(0,0,-90.8)
    
    planeId = p.loadURDF("plane.urdf")
    
    p.loadSDF("world.sdf")
    
def prepare_matchup():
    
    #robot = p.loadURDF("matchup.urdf", flags = p.URDF_USE_SELF_COLLISION)
    robot = p.loadURDF("matchup.urdf")
    brain = NEURAL_NETWORK("matchup_noggin.nndf")
    
    pyrosim.Prepare_To_Simulate(robot)
    
    sensors = {}
    for linkName in pyrosim.linkNamesToIndices:
        #print(linkName)
        sensors[linkName] = SENSOR(linkName)
        
    #print(sensors)
    
    motors = {}
    for jointName in pyrosim.jointNamesToIndices:
        #print(jointName)
        motors[jointName] = MOTOR(jointName)
        
    #print(motors)   
    
    return sensors, motors, brain, robot
            

def sense(t, sensors):
    for i in sensors.values():
        i.get_value(t)
        
        
def think(brain):
    brain.update()


def act(t, motors, brain, robot,leftroot_neuron, rightroot_neuron):
    
    for neuron in brain.Get_Neuron_Names():
        
        if brain.Is_Motor_Neuron(neuron):
            
            if neuron == rightroot_neuron or neuron == leftroot_neuron:
                desiredAngle = 10*brain.Get_Value_Of(neuron)
            else:
                desiredAngle = 0.5*brain.Get_Value_Of(neuron)
            
            
            ''' Kill Code
            
            if pyrosim.Get_Touch_Sensor_Value_For_Link("Head") == 1 or self.alive == False:
                desiredAngle = 0
                self.alive = False
                '''
            
            jointName = brain.Get_Motor_Neurons_Joint(neuron)
            motors[jointName].set_value(robot, desiredAngle)
            
        
        
left_dna=initialize_challenger(0,1)
right_dna=initialize_challenger(0,2)

leftroot_neuron, rightroot_neuron = build_challenger(left_dna, right_dna)

start_world()
sensors, motors, brain, robot = prepare_matchup()

 
t = 0
while True:
    p.stepSimulation()
    
    sense(t, sensors)
    think(brain)
    act(t, motors, brain, robot,leftroot_neuron, rightroot_neuron)
    
    time.sleep(1/5000)
    
    t+=1