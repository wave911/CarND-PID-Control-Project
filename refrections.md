# P, D, I factors in PID controller

 - P component in this project defines the proportionality factor between cross-track error and output value (in our case - steer value). Changing its value affects on car's amplitude oscillations, like sinus trajectory.
 - D component in this project defines the ability of PID controller to be able to converge the output value to the target value. It uses the derivative feature and creates a direction to the target. But it doesn't allow to affect the speed of this convergence. In a very ideal situation - P and D are enough to calculate the steer value.
 - I component indeed allows us to affect this convergence speed. But also it affects sensetive and choosing a wrong value may make the car too sensetive to the small road change.

Initially I've chosen the initial values for P, D, I (0.225, 0.0004, 4.0). Then to improve convergence I tried to decrease I value. I stopped at 2. Then I decrease the P value, to 0.10 and it decreased the amplitude oscilattions.
I failed with twiddle. So I did it manually (
