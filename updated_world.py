import pyrosim2.pyrosim as pyrosim

sim = pyrosim.Simulator( play_paused=True , eval_time=1000 )

sim.send_cylinder( x=0 , y=0 , z=1.5 , length=1.0 , radius=0.1 )

sim.start()

sim.wait_to_finish()