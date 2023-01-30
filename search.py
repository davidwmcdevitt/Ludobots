import os
from hillclimber import HILL_CLIMBER

for i in range(1):
    
    hc = HILL_CLIMBER()
    hc.evolve()
    
    hc.show_best()
    
    #os.system("python3 generate.py")
    #os.system("python3 simulate.py")