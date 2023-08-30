import numpy as np
from deap import creator, base, tools, algorithms


class Algorithm:

    def __init__(self, fitness, population=None, populationSize=10):
        if population is None:
            population = []
        self.minKP = 0.01
        self.maxKP = 8.0
        self.minKI = 0.01
        self.maxKI = 0.5
        self.minKD = 0.01
        self.maxKD = 1.0

        self.cxpb = 0.8
        self.mutpb = 0.1

        self.populationSize = populationSize
        self.fitness = fitness
        self.population = population
        self.toolbox = self.toolbox()

    def randomKi(self):
        return np.power(10, np.random.uniform(np.log10(self.minKI), np.log10(self.maxKI)))

    def randomKp(self):
        return np.random.uniform(self.minKP, self.maxKP)

    def randomKd(self):
        return np.random.uniform(self.minKD, self.maxKD)

    def getFitness(self, individual, population):
        return self.fitness[population.index(individual)]

    def evaluate(self, individual, population):
        return self.getFitness(individual, population)

    def toolbox(self):
        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMin)

        toolbox = base.Toolbox()

        toolbox.register("attr_kp", self.randomKp)
        toolbox.register("attr_ki", self.randomKi)
        toolbox.register("attr_kd", self.randomKd)

        toolbox.register("individual", tools.initCycle, creator.Individual,
                         (toolbox.attr_kp, toolbox.attr_ki, toolbox.attr_kd), n=1)
        toolbox.register("population", tools.initRepeat, list, toolbox.individual)

        toolbox.register("evaluate", self.evaluate)

        #  toolbox.register("select", tools.selTournament, tournsize=3)
        toolbox.register("select", tools.selStochasticUniversalSampling)
        toolbox.register("mate", tools.cxOnePoint)
        toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=[(self.maxKP - self.minKP) / 4,
                                                                   (self.maxKI - self.minKI) / 4,
                                                                   (self.maxKD - self.minKD) / 4],
                         indpb=self.mutpb)

        return toolbox

    def getPopulation(self):
        population = self.toolbox.population(n=self.populationSize)
        self.population = population
        return population

    def evolution(self):
        lambda_ = self.populationSize
        ngen = 1

        population = list(self.population)

        for individual in population:
            individual.fitness.values = [self.toolbox.evaluate(individual, population)]

        for g in range(ngen):
            offspring = algorithms.varOr(self.population, self.toolbox, lambda_, self.cxpb, self.mutpb)
        self.population[:] = offspring

        return offspring
