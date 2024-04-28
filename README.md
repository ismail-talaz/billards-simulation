# Billards Simulation

### Usage

-  *initial-state.json* and *snapshot-times.txt* must be in the same directory as *main.cpp*
-  Snapshot times must be entered in the txt file in ascending order.
-  The simulation starts with the execution of *main.cpp*
-  The output is saved in the *output.json* file.
  

### Capability

The programme can handle three situations : 
1. **Base**: No collisions between balls or with the walls
2.  **Intermediate**: Collisions with the walls but no collisions between balls.
3.  **Advanced**: Collisions between balls and with the walls.

### Logic

From the moment the programme starts, it calculates the potential collision times of all the balls with each other and with the walls and finds the closest one. It advances the time by updating the information of the balls until the calculated time. It performs the collision at the travelled time. It continues the simulation by continuously performing the closest collision. During this process, when the snapshot is closer to the desired time, it goes to that moment and saves the instantaneous information of the balls for later output. The programme has been tested in many edge cases such as Newton's Cradle.

### Dependency

- The [nlohmann JSON](https://github.com/nlohmann/json) library is used to handle the JSON files in C++.
