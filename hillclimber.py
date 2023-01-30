from solution import SOLUTION
import copy
import constants as c

class HILL_CLIMBER:
    
    def __init__(self):
        self.parent = SOLUTION()
        self.parent.evaluate("GUI")
    
    
    def evolve(self):
        for currentGeneration in range(c.numberOfGenerations):
            #print(currentGeneration)
            self.evolve_for_one_generation("DIRECT")
    
    def evolve_for_one_generation(self, mode):
        self.spawn()
        self.mutate()
        self.child.evaluate(mode)
        self.select()
        self.Print()
        
    
    def spawn(self):
        self.child = copy.deepcopy(self.parent)
    
    def mutate(self):
        self.child.mutate()
        #print(self.parent.weights)
        #print(self.child.weights)
        #exit()
    
    def select(self):
        if self.child.fitness < self.parent.fitness:
            #print("HIT")
            self.parent = self.child
            
    def Print(self):
        print("Fitness of Parent and Child")
        print(self.parent.fitness, self.child.fitness)
        
    def show_best(self):
        self.parent.evaluate("GUI")
        
