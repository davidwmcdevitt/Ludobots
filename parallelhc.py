from solution import SOLUTION
import copy
import constants as c
import os 
import numpy as np

class PARALLEL_HILL_CLIMBER:
    
    def __init__(self):
        
        #os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        
        
        self.parents = {}
        self.nextAvailableID = 0
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
            
        #print(self.parents)
        #self.parent.evaluate("GUI")
    
    
    def evolve(self):
        
        self.evaluate(self.parents)
        
            
    
        for currentGeneration in range(c.numberOfGenerations):
            #print(currentGeneration)
            self.evolve_for_one_generation("DIRECT")
            
    
    def evolve_for_one_generation(self, mode):
        
        
        self.spawn()
        self.mutate()
        self.evaluate(self.children)
        self.select()
        self.Print()
        
    
    def spawn(self):
        self.children = {}
        for i in range(c.populationSize):
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
            #print(self.children)
    
    def mutate(self):
        for i in range(c.populationSize):
            self.children[i].mutate()
    
    def select(self):
        for i in self.parents.keys():
            if self.children[i].fitness > self.parents[i].fitness:
                self.parents[i] = self.children[i]
            
    def Print(self):
        print()
        for i in self.parents.keys():
            print(self.parents[i].fitness, self.children[i].fitness)
        print()
        
    def show_best(self):
        m = np.argmin([self.parents[i].fitness for i in self.parents.keys()])
        print("Fitness: " + str(self.parents[m].fitness))
        self.parents[m].start_simulation("GUI")
        #self.parent.evaluate("GUI")
        
    def evaluate(self,solutions):
        for i in range(c.populationSize):
            solutions[i].start_simulation("DIRECT")
        
        for i in range(c.populationSize):
            solutions[i].wait_for_simulation_to_end()
            #print(self.parents[i].fitness)
        
