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
    
    
def build_challenger(dna, side):
    
    if side == "left":
        base_y = 5
        col = "green"
    else:
        base_y = -5
        col= "cyan"
    
    challengerID = dna['challengerID']
    
    pyrosim.Start_URDF("challenger" + str(challengerID) + ".urdf")
    
    pyrosim.Send_Cube(name= "Base", pos=[0, base_y/2, 0.25], size = [5, 5, 0.25], col = col)
    
    
    pyrosim.Send_Joint(name = "Base_Head", parent = "Base", child = "Head", type = "fixed", position = [0,0,0], jointAxis="1 0 1")
                    
    
    pyrosim.Send_Cube(name= "Head", pos=[0, base_y/2, 0.75], size = [1, 1, 1], col = col)
    
    pyrosim.End()
    
    

def start_world():
        
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    p.setGravity(0,0,-90.8)
    
    planeId = p.loadURDF("plane.urdf")
    
    p.loadSDF("world.sdf")
    
def load_challengers():
    
    p.loadURDF("challenger1.urdf")
    p.loadURDF("challenger2.urdf")
    


    
    
left_dna=initialize_challenger(0,1)
right_dna=initialize_challenger(0,2)

build_challenger(left_dna, "left")
build_challenger(right_dna, "right")

start_world()
load_challengers()
