# Drone_project
In the folder machine learning drone you will find some python scripts and folders with the arduino code. 

For Testing the connection:
- Use the python script test.py
- Use the arduino code in the folder test_connection
This will just test the serial connection between the drone and your computer. If you are able to connect the drone to the computer, you can also check if the drone is working in generall.

Testing the drone:
- use PID_balance_arduino
 and load it on the drone. There you can input your own values for kp,ki,kd and see how the drone behaves.

Machine Learning:
- the main.py and Algorithm.py is needed for the machine learning part.
- use PID_balance_arduino_machine_learning for the arduino to run
- After uploading the code to the drone, connect it to the battery and see if its running
- Start the main.py script
  
After that the drone should get their PID constants from the Python script, test them for 5 seconds and give back the fitness (the cummulated error or you can add an error term form quick movements (this did not help in my case, but I left it in the code)).
Also you can adjust the population size, the crossover and mutation, if you want to have elitism or not and if you want to shrink the population every step in the evolution. Also I added the possibility to load results from the previous run. You could load the whole last population or just the best individuums.
Then let it run for a few minutes, till it reaches its desired generation or gets a fitness, which you find good.

After the run, every PID value and its fitness is saved into a CSV file. You can find those in the folder Fitness. I tried some combinations. The 6g and +4g ones are based on another. So I first did a run for 6 generations, than for 4. One of the for with a mutation of 5%, the other one of 20%, just because I was interested. All the others and many more which I have not saved, were from runs before. In those I wanted to get a feeling for the ranges of the PID constants, the crossover and the mutation. So the 6g and +4g were the final results from my lastes testing


Also I added 3 Videos which show how the training of the EA was done.
