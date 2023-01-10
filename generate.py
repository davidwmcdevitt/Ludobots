import pyrosim.pyrosim as pyrosim


pyrosim.Start_SDF("boxes.sdf")

length = 1
width = 1
height = 1

x = 0
y = 0
z = 0.5

for v in range(5):
    for g in range(5):
        for i in range(10):
            pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length,width,height])
            z = z+1
            
            length = length * 0.9
            height = height * 0.9
            width = width * 0.9
        
        z = 0.5
        x = x+1
        
        length = 1
        width = 1
        height = 1
    
    y = y + 1
    x = 0



#x = 1
#y = 1
#z = 1.5
#pyrosim.Send_Cube(name="Box2", pos=[x,y,z] , size=[length,width,height])


pyrosim.End()