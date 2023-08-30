import serial
import time
from Algorithm import Algorithm
import numpy as np
import pandas as pd

# for arduino communication
arduino_port = '/dev/cu.usbmodem145201'
baud_rate = 9600
arduino = serial.Serial(port=arduino_port, baudrate=baud_rate, timeout=.1)

# for genetic algorithm
populationSize = 12
fitness = np.zeros(populationSize)

algorithm = Algorithm(fitness, populationSize=populationSize)

population = algorithm.getPopulation()


def hypothesis_tester(i):
    return population[i]


def fitness_reader(x, y, z, i):
    fitness[i] = abs(x) + abs(y) + abs(z)


def evolution_process():
    global population

    index = min(range(len(fitness)), key=lambda i: fitness[i])

    chromosome = population[index]

    algo = Algorithm(fitness, population=population, populationSize=populationSize)
    population = algo.evolution()

    kp = chromosome[0]
    ki = chromosome[1]
    kd = chromosome[2]

    return [kp, ki, kd, fitness[index]]


def send_data_to_arduino(data):
    try:
        arduino.write(data.encode('utf-8'))
        arduino.flush()
    except Exception as e:
        print(f"Error while sending data to Arduino: {e}")


def receive_data_from_arduino():
    try:
        data = arduino.readline()
        data_dec = data.decode('utf-8')
        return data_dec
    except Exception as e:
        print(f"Error while receiving data from arduino: {e}")
        return None


if __name__ == '__main__':
    g = 0
    PID_fitness = pd.DataFrame(columns=['Kp', 'Ki', 'Kd', 'fitness'])
    Best_fitness = 500
    print("Start of Evolution")
    Kp = 100
    Kd = 100
    Ki = 100
    int_string = f"{Kp};{Kd};{Ki}"
    send_data_to_arduino(int_string)
    time.sleep(5.8)
    value = arduino.readline()
    print("testlauf:", value)

    # Begin the evolution with g = 6 generations
    counter = 0
    while (Best_fitness > 50) and (g < 4):
        for pop_index in range(len(population)):
            individual = hypothesis_tester(pop_index)
            Kp = int(round(individual[0]*100))
            Ki = int(round(individual[1]*1000))
            Kd = int(round(individual[2]*100))
            print("Kp:", individual[0])
            print("Ki:", individual[1])
            print("Kd:", individual[2])
            int_string = f"{Kp};{Ki};{Kd}"
            send_data_to_arduino(int_string)
            time.sleep(5.8)  # wait till drone tested completely
            fitness_float = 0.0
            while fitness_float < 0.1:
                fitness_individuum = receive_data_from_arduino()  # receive fitness of individuum
                if fitness_individuum:
                    fitness_float = float(fitness_individuum)
                    print("fitness:", fitness_float)
                else:
                    fitness_float = 0.0
                    print("Ungültige oder leere Zeichenkette")
            fitness_reader(fitness_float, 0, 0, pop_index)  # just pitch fitness here, because just 1 DOF
            PID_fitness.loc[counter] = [Kp / 100, Ki / 1000, Kd / 100, fitness_float]
            counter += 1
        [Best_Kp, Best_Ki, Best_Kd, Best_fitness] = evolution_process()
        print("Generation: ", g, "Kp: ", Best_Kp, "Ki: ", Best_Ki, "Kd: ", Best_Kd, "fitness: ", Best_fitness)
        g = g + 1
    PID_fitness.to_csv('PID_fitness.csv', index=False)
