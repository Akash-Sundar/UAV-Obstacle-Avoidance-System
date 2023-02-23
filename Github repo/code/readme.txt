I have implemented minimum snap trajectory to produce favourable results in the given test maps. I have also coded in minimum jerk trajectory and constant speed trajectory.

I generated sparse points from my graph search generated points by implementing the RDP algorithm (As recommended on Ed) with minor modifications to manually insert additional waypoints along points in a straight line separated by long distances.

I have generated my own test map such that the quadroptor first has to pass through a window, then manuever through a maze and finally perform an up and down manuever to reach the goal.

I have used a speed of 10 m/s but identified that the given test cases can be passed with speeds up to 13 m/s. However, in consideration of more complex routes, I limited it with a safety factor of 0.2 circa.

I have referenced multiple github repositories to understand how the A and B matrices for minimum snap were formulated. I identified the repeating structure of a matrix of derivatives and some outlying elements. I coded in the two cases separately.

The different modes can be implemented by changing appropriate values in the params.py file.