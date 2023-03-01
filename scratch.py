


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

def initialize_challenger(formtype):
    
    dna = {}
    
    '''
    Formtypes designate initial DNA states - for the time being everything is 
    going to be initialized from a single body in the middle of the platform.
    
    To-do: Add different spawn locations, growth directions(?) + brainstorm
    '''
    
    dna['height'] = 10
    
    return dna

def mutate_challenger(dna):
    
    '''
    Takes a challenger DNA and inserts a mutation. Mutation options include
        1. Growing taller
        2. Adding limbs
        3. Making limbs longer
        4. Changing joint types on limbs
    '''
    
    
      
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
    

class CHALLENGER:
    
    def __init__(self,myID,side,dna):
        self.myID = myID
        self.alive = True
        self.side = side
        self.dna = dna
    
    
    

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-90.8)

planeId = p.loadURDF("plane.urdf")

challenger_1 = CHALLENGER(1)
challenger_1.generate_tardigrade()

challenger_1.prepare_for_battle()

#p.loadURDF("tardigrade1.urdf")

p.loadSDF("world.sdf")
'''
for i in range(c.iterations):
    p.stepSimulation()
    #alive = self.robot.check_head(i)
    challenger_1.sense(i)
    challenger_1.think()
    challenger_1.act(i)
    time.sleep(1/500)
'''

