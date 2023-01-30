from solution import SOLUTION
import constants as c
import copy


class HILL_CLIMBER:
    
    def __init__(self):
        
        self.parent = SOLUTION()
        self.parent.evaluate("DIRECT")
    
    def evolve(self):
        for currentGeneration in range(c.numberOfGenerations):
            print(currentGeneration)
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

    def select(self):
        if self.child.fitness < self.parent.fitness:
            print(self.child.fitness)
            self.parent = self.child

    def Print(self):
        print("Fitness of Parent and Child")
        print(self.parent.fitness, self.child.fitness)
        
    def show_best(self):
        self.parent.evaluate("GUI")