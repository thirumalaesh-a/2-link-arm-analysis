**A BASIC STUDY ON ROBOT ARM MANIPULATION**  
This code was written sometime around 2021, as a tool to better understand robot arm kinematics and manipulation. 

```
python3 manipulability.py  
```
   Generates plots to test if the robot can reach a certain point in space - It should exit if the location is reachable, else I'm guessing it would run for a long time.  
   
```
python3 twolink_planar_Inverse_kinematics.py  
```   
   Computes the inverse kinematics, plots the errors and gains  
```
python3 Velocity_2l2d.py 
```     
   Does some analysis on the velocities and plots the trajectory of the robot  
```
python3 Workspace.py
```  
   Maps out the entire workspace of the robot  
