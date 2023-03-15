import constants as c
import numpy as np
import pyrosim.pyrosim as pyrosim
import pybullet as p

class SENSOR:
    
    def __init__(self , linkName, linkIndex):
        self.linkName = linkName
        self.linkIndex = linkIndex
        
        self.values = np.zeros(c.iterations)
        
    def get_value(self, t, robot,leftHead_index, rightHead_index):
        
        if "Left" in self.linkName:
            if "Eye" in self.linkName:
                
                rightHeadState = p.getLinkState(robot,leftHead_index)
                positionOfRightHead = rightHeadState[0]
                
                xCoordinateOfRightHead = positionOfRightHead[0]
                yCoordinateOfRightHead = positionOfRightHead[1]
                zCoordinateOfRightHead = positionOfRightHead[2]
                
                
                eyeState = p.getLinkState(robot,self.linkIndex)
                eyePosition = eyeState[0]
                xCoordinateOfEye = eyePosition[0]
                yCoordinateOfEye = eyePosition[1]
                zCoordinateOfEye = eyePosition[2]
                
                distance = (xCoordinateOfRightHead-xCoordinateOfEye)**2 + (yCoordinateOfRightHead-yCoordinateOfEye)**2 + (zCoordinateOfRightHead-zCoordinateOfEye)**2
                
                distance = distance ** 0.5
                
                self.values[t] = distance
    
            else:
                
                self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
                
        else:
            if "Eye" in self.linkName:
                
                leftHeadState = p.getLinkState(robot,rightHead_index)
                positionOfLeftHead = leftHeadState[0]
                
                xCoordinateOfLeftHead = positionOfLeftHead[0]
                yCoordinateOfLeftHead = positionOfLeftHead[1]
                zCoordinateOfLeftHead = positionOfLeftHead[2]
                
                
                eyeState = p.getLinkState(robot,self.linkIndex)
                eyePosition = eyeState[0]
                xCoordinateOfEye = eyePosition[0]
                yCoordinateOfEye = eyePosition[1]
                zCoordinateOfEye = eyePosition[2]
                
                distance = (xCoordinateOfLeftHead-xCoordinateOfEye)**2 + (yCoordinateOfLeftHead-yCoordinateOfEye)**2 + (zCoordinateOfLeftHead-zCoordinateOfEye)**2
                
                distance = distance ** 0.5
                #print(distance)
                self.values[t] = distance
    
            else:
                
                self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
                
        #print(self.linkName)
        #print(self.values)
        
    def save_values(self):
        np.save("./data/sensor-" + self.linkName + ".npy", self.values)