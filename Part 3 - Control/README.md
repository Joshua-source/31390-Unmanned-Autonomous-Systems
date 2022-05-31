# PART 3: Control
### Exercise 3.1?
Using the non-linear model of Exercise 2.1 (or 2.2), close a feedback control loop to control
the attitude, to allow setting roll, pitch, yaw and altitude references. Choose the gains of
the PID by tuning on the linearized system, by minimizing the settling time while ensuring
that the controlled system is critically damped (no overshoot), and plot the step response
for a step of:

• 1. Θ? = [10, 0, 0]T[deg], z? = 0[m]

• 2. Θ? = [0, 10, 0]T[deg], z? = 0[m]

• 3. Θ? = [0, 0, 10]T[deg], z? = 0[m]

• 4. Θ? = [0, 0, 0]T[deg], z? = 1[m].

Show your results and explain them.
### Exercise 3.2
Apply the controller of Exercise 3.1 to the non-linear model and compare the step responses
for the controller applied to the linearized model and to the non-linear one. Show the
comparison and explain the differences for each of the references setpoints of Exercise 3.1.
### Exercise 3.3
Close a position control loop for the x and y components. Tune the gains using proper
considerations and showing an appropriate methodology. Explain the method used and
discuss your results.
