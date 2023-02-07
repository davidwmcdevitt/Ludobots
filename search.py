import os
from hillclimber import HILL_CLIMBER
from parallelhc import PARALLEL_HILL_CLIMBER
from reef import REEF

def run_ludo():
    
    phc = PARALLEL_HILL_CLIMBER()
    phc.evolve()
    
    phc.show_best()
