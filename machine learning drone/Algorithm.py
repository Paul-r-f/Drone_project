import numpy as np
import pandas as pd
from deap import creator, base, tools, algorithms


class Algorithm:

    def __init__(self, fitness, population=None, populationSize=12):
        if population is None:
            population = []
        self.minKP = 0.01
        self.maxKP = 0.8
        self.minKI = 0.001
        self.maxKI = 0.02
        self.minKD = 0.01
        self.maxKD = 0.3

        self.cxpb = 0.8
        self.mutpb = 0.2

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
        # load some good examples or the whole last population, because battery runs out quickly
        csv_file_path = "/Users/paulfay/Documents/Arduino/machine learning drone/Fitness/PID_fitness_15_d_6g.csv"
        try:
            df = pd.read_csv(csv_file_path)
        except FileNotFoundError:
            print("file could not be read.")
            exit()
        sorted_df = df.sort_values(by=df.columns[3], ascending=True)
        # if you want your whole last population:
        top_values = df.iloc[-int(self.populationSize):, :3].values
        # if you want just some best of the last population:
        #top_values = sorted_df.iloc[:3, :3].values #change how many you want from last population
        for i in range(min(len(population), len(top_values))):
            population[i][:3] = top_values[i]
        # if you don't want examples, just comment out this part

        self.population = population
        return population

    def evolution(self):
        lambda_ = self.populationSize
        ngen = 1

        population = list(self.population)

        for individual in population:
            individual.fitness.values = [self.toolbox.evaluate(individual, population)]

        for g in range(ngen):
            best_previous = tools.selBest(self.population, 1)[0]
            offspring = algorithms.varOr(self.population, self.toolbox, lambda_, self.cxpb, self.mutpb)
            # Elitism
            offspring[0] = best_previous
            # less offspring in every generation to have more genetic variance in the beginning and less in the end
            num_to_keep = int(self.populationSize * 0.9)
            offspring = tools.selBest(offspring, num_to_keep)
        self.population[:] = offspring

        return offspring
