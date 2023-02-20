from world import WORLD
from robot import ROBOT
import time
import pybullet as p
import pybullet_data
import constants as c


class SIMULATION:
    
    def __init__(self,directOrGui,solutionID):
                
        mode = p.GUI # p.DIRECT
        if directOrGui == "DIRECT":
            mode = p.DIRECT
        self.physicsClient = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-90.8)
        
        self.world = WORLD()
        self.robot = ROBOT(solutionID)
        
        
    def run(self):
                
        for i in range(c.iterations):
            #print(i)
            p.stepSimulation()
            #alive = self.robot.check_head(i)
            self.robot.sense(i)
            self.robot.think()
            self.robot.act(i)
            time.sleep(1/500)
            
    def __del__(self):        
        p.disconnect()
        #self.robot.motors.save_values()
        #self.robot.sensors.save_values()
        
    def get_fitness(self):
        self.robot.get_fitness()
        #print("check1")