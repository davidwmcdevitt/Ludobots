import constants as c
import numpy as np
import pyrosim.pyrosim as pyrosim

class SENSOR:
    
    def __init__(self , linkName):
        self.linkName = linkName
        
        self.values = np.zeros(c.iterations)
        
    def get_value(self, linkName):
        self.values[linkName] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        #print(self.values)
        
    def save_values(self):
        np.save("./data/sensor-" + self.linkName + ".npy", self.values)