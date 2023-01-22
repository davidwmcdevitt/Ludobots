from world import WORLD
from robot import ROBOT
import time
import pybullet as p
import pybullet_data
import constants as c


class SIMULATION:
    
    def __init__(self):
        
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-90.8)
        
        self.world = WORLD()
        self.robot = ROBOT()
        
        
    def run(self):
                
        for i in range(c.iterations):
            #print(i)
            p.stepSimulation()
            self.robot.sense(i)
            self.robot.think()
            self.robot.act(i)
            time.sleep(1/60)
            
    def __del__(self):        
        p.disconnect()
        self.robot.motors.save_values()
        self.robot.sensors.save_values()