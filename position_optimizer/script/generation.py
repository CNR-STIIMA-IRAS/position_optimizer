#!/usr/bin/env python
from individual import Individual
import random
import numpy as np
import copy
import rospy

class Generation:

    def __init__(self, n_ind):
        self.population = []
        for i in range (1,n_ind+1):
            self.population.append(Individual(i))

    def probability(self):
        total_fitness = 0

        for ind in self.population:
            total_fitness += ind.fitness

        if total_fitness == 0:
            rospy.logerr("fitness is zero. Not allowed!")
            return False

        for ind in self.population:
            prob = ind.get_fitness()/total_fitness
            ind.set_probability(prob)
        return True

    def distribution(self):
        self.population.sort(key=lambda x: x.prob, reverse=False)

    def rank(self):
        self.distribution()
        n = len(self.population)

        # for i in range(1, n+1):
        #     self.population[i-1].rank_probability = float(i)/(n*(n-1))

        for i in range(0, n):
            self.population[i].rank_probability = (1-self.pc)**(n-i)*self.pc

    def set_pc(self,pc):
        self.pc = pc

    def get_ind(self, method = "rank"):

        if method == "roulette_wheel":
            ## roulette wheel fitness
            max_fit = sum(x.fitness for x in self.population)
            pick = random.uniform(0, max_fit)
            current = 0
            for ind in self.population:
                current += ind.fitness
                if current > pick:
                    return (ind)

        if method == "roulette_wheel_probability":
            # roulette wheel probability
            max_prob = sum(x.prob for x in self.population)
            pick = random.uniform(0, max_prob)
            current = 0
            for ind in self.population:
                current += ind.prob
                if current > pick:
                    return (ind)

        # rank selection
        if method == "rank":
            max_prob = sum(x.rank_probability for x in self.population)
            pick = random.uniform(0, max_prob)
            current = 0
            for ind in self.population:
                current += ind.rank_probability
                if current > pick:
                    return (ind)


    def tournament(self):
        n_inds = len(self.population)/10
        inds = []
        for i in range(1, n_inds):
            num = random.randint(0, len(self.population)-1)
            jj = self.population[num]
            inds.append(jj)

        inds.sort(key=lambda x: x.fitness, reverse=False)

        return (inds[-1])

    def crossover(self,mum,dad):

        child1 = copy.deepcopy(mum)
        child2 = copy.deepcopy(dad)

        for i in range(0,len(child1.chromosome)-1):
            if bool(random.getrandbits(1)):
                child1.chromosome[i] = copy.deepcopy(dad.chromosome[i])
                child2.chromosome[i] = copy.deepcopy(mum.chromosome[i])

        child1.chromosome_to_pose()
        child2.chromosome_to_pose()

        return child1, child2

    def get_far_ind(self, dad):
        pop = copy.deepcopy(self.population)

        dad_c = np.array(dad.chromosome)

        for i in pop:
            ind_c = np.array(i.chromosome)
            i.differonte = np.linalg.norm(dad_c-ind_c)
            i.fitness = i.fitness + i.differonte

        pop.sort(key=lambda x: x.fitness, reverse=False)

        n = len(pop)
        for i in range(0, n):
            pop[i].rank_probability = (1-self.pc)**(n-i)*self.pc

        max_prob = sum(x.rank_probability for x in pop)
        pick = random.uniform(0, max_prob)
        current = 0
        for ind in pop:
            current += ind.rank_probability
            if current > pick:
                return (ind)

    def selection(self):
        self.children = []

        if rospy.has_param("selection_method"):
            method = rospy.get_param("selection_method")
        else:
            rospy.logwarn("%s/%s not found ! default rank", rospy.get_namespace(), "selection_method")
            method = "rank"

        for i in range(0,len(self.population)/2):
            if method is "rank" or "roulette_wheel" or "roulette_wheel_probability":
                dad = self.get_ind(method)
            elif method == "tournament":
                dad = self.tournament()
            else:
                rospy.logerr("selection method not found. something wrong is going on")

            counter = 0
            while True:
                if counter > 1000:
                    rospy.logerr("More than 1000 times mum and dad chromosomes were exactly the same during selection process. is it ok?")

                if method is "rank" or "roulette_wheel" or "roulette_wheel_probability":
                    mum = self.get_ind(method)
                # mum = self.get_far_ind(dad)
                elif method == "tournament":
                    mum = self.tournament()

                if mum.chromosome != dad.chromosome:
                    counter = counter+1
                    break

            son, daughter = self.crossover(mum,dad)
            self.children.append(son)
            self.children.append(daughter)

        best = copy.deepcopy(self.population[-1])
        self.population = self.children
        self.population.append(best)
        for i in range(0,len(self.population)):
            self.population[i].id = i+1
            self.population[i].prob = 0
            self.population[i].fitness = 0

    def mutation(self, ind):
            self.population[ind].mutate()


if __name__ == "__main__":

    n_ind = 10
    generation = Generation(n_ind)
