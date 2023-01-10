import pyrosim.pyrosim as pyrosim


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
    
    
    
def create_robot():
    pyrosim.Start_URDF("body.urdf")
    
    length = 1
    width = 1
    height = 1
    
    x = 0
    y = 0
    z = 0.5
    
    pyrosim.Send_Cube(name="BackLeg", pos=[x,y,z], size=[length,width,height])
    
    x = 0
    y = 0.5
    z = 1
    
    pyrosim.Send_Joint(name="BackLeg_Torso", parent="BackLeg", child="Torso", type="revolute", position=[x,y,z])
    
    x = 0
    y = 0.5
    z = 0.5
    
    pyrosim.Send_Cube(name="Torso", pos=[x,y,z], size=[length,width,height])
    
    x = 0
    y = 0.5
    z = -0.5
    
    pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[x,y,z])
    
    x = 0
    y = 1
    z = 0
    
    pyrosim.Send_Cube(name="FrontLeg", pos=[x,y,z], size=[length,width,height])

    pyrosim.End()
    
    
create_world()
create_robot()