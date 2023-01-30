import os
from hillclimber import HILL_CLIMBER
from parallelhc import PARALLEL_HILL_CLIMBER

for i in range(1):
    
    phc = PARALLEL_HILL_CLIMBER()
    phc.evolve()
    
    phc.show_best()
    
    #os.system("python3 generate.py")
    #os.system("python3 simulate.py")