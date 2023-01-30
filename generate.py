import pyrosim.pyrosim as pyrosim
import random


def create_world():
    pyrosim.Start_SDF("world.sdf")
    
    length = 1
    width = 1
    height = 1
    
    x = -2
    y = -2
    z = 0.5
    
    pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length,width,height])
    
    
    pyrosim.End()
    
    
    
def generate_body():
    
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
    
 
    
    
def generate_brain():
    
    pyrosim.Start_NeuralNetwork("brain.nndf")
    
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    
    pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg")
    
    pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "BackLeg")
    
    pyrosim.Send_Motor_Neuron(name = 3 , jointName = "BackLeg_Torso")
    
    pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")
    
    for i in range(0,3):
        for j in range(3,5):
            w = random.random()*2-1
            pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j, weight=w)

    pyrosim.End()
    
     
create_world()
generate_body()
generate_brain()

