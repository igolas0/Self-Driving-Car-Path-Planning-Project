# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[image1]: ./pics/miles1.png "Miles example 1"   
[image2]: ./pics/miles2.png "Miles example 2"   
[image3]: ./pics/miles_record.png "Miles record"   

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

### Approach
For this Path Planning Project a perfect/ideal controller model was asumed. A splines library was used to smooth the path points that were then fed to the car agent in the simulator. To manage the lane changing behaviour a state machine and a scoring system to optimize the decisions were implemented. The logic used was relatively simple: the state machine keeps driving the same lane until it encounters a front car. Then it transitions into the "Prepare Lane Change" state which checks the viability of lane changes by surveying the surrounding space available in the side lanes based on the data coming from sensor fusion. Then a score is given to each viable decision based on the expectation of fast advancing in that lane. If the best decision involves a lane change, the target lane is fed to the path points generation process (using splines), which will output a smooth transition to that lane. 

### Description of the code
The meat of the project is implemented in 'src/main.cpp'. 
Up until line 164 we include the libraries we want to use (including the very helpful spline library) and define some helpful functions, mostly to convert distances or between coordinate systems (Cartesian and Frenet).
The main program starts in line 166. In the lines of code 204 to 210 we initialize three important variables. One for our target lane to drive, target velocity and the state machine variable for controlling the decision making. We pass those variables to the lambda expression in the next line which iterates on every communication step with the simulator.

For this project we use a perfect control model. We feed the car agent with a series of points to visit next with no delay (other than computing time). The car visits one point every 0.02 seconds so we influence the speed indirectly by setting these points closer or further apart. 

In lines 251 and 252 we initialize two vectors of doubles to later define and store the x and y values corresponding to the path we want the car to follow. Also we use the remaining non-visited points of the previous time step as a starting point (lines 258-261). 

Next, we loop over sensor fusion data searching for cars in front of us driving the same lane (lines 277 to 296). If our projected path collides with the future estimated position of the front car we adapt our speed accordingly (lines 299-302). Else if the front car is driving faster than us or the lane is free we accelerate until just below the speed limit and stay there. (the increments of 0.224mph make sure that we do not surpass the max acc limit of the simulator).

The "brain" of the decision making is implemented from lines 320 to 466. A state machine is implemented via a switch statement that manages lane changing behaviour. The variable ´menuItem´ can take the values from 1 to 4. It starts at 1 representing "keep driving the current lane" until we spot a front car driving slower than us. Then we transition to menuItem = 2 and run over the "Prepare Lane Change" logic. Basically we check the viability of left and right lane changes based on the available space found in the side lanes (taking into account not only current vehicle positions but also projected straight lane keeping driving). In lines of code 332 to 372 we perform this check for a potential left lane change by looping over sensor fusion data. The same is done for a potential right lane change (lines 379-415).

To meet the final decision (Keep Lane, Left Lane Change or Right LC) we set a scoring system based on the possibility of advancing fast in that lane -lookahed of 10 seconds- (lines 420-454). The highest score between the feasible manouvers determines our decision. In the case that "Keep Lane" is the best option or neither lane change is feasible we keep "Preparing for a Lane Change", else we transition into menuItem 3 or 4 (Left Lane Change or Right Lane Change). In those cases we simply change our target lane which will impact the path generation process below.

Our path generation is implemented in the lines of code 469 to 571. First we define two vectors of doubles as helper points to store our path. As a starting point we use the non-visited points of the previous time step as a a starting path for the current time step and build later on top of it (lines 490 to 505). In the case that we are starting from scratch or the whole path was visited (which should happen rarely) we set the starting point of our path based on current car position and orientation (lines 477-489).

In lines of code 507 to 518 we set target points to visit in the target lane (at 30m, 60m and 90m from our current position in Frenet coordinates), convert them to Cartesian coordinates and store them in our helper point vectors.

Next we transform the stored points into the local coordinates of the car and create a spline based on these points (lines 521 to 529). We will use this spline to draw on top of it our target path. We start by filling up our ´next_x_vals' and ´next_y_vals´ with the previous path (lines 536-540). We fill then the rest of the path up to 50 points by "drawing" points on top of the previous defined spline. We compute the needed space between the target points so that the car travels at the desired speed and convert back to globalCartesian coordinates before storing these points (lines 542-568). Finally in lines 570 and 571 we send the desired path to the simulator. 

### Results

Though being a relative simple model it achieves satisfying results most of the time, being able to drive often for longer than 10 miles without incidents. Here are two screenshots of consecutive runs for 10+ miles:

![alt text][image1]

![alt text][image2]

The best result achieved was almost 30 miles driven (27 miles) after a bit more than half an hour:

![alt text][image3]

The resulting decision making is able to make lane changes to advance fast that I would make myself (most of the time). Still, sometimes mistakes do happen and there are a lot of areas that could be improved and we do detail in the next section.

### Further work

Here is a list of identified issues which would need being solved:

- The model seems to be more mistake prone when driving along congestionated traffic. Especially problematic seems to be taking into account vehicles which are performing a lane change into our potential target lane.
- Only the traffic in the immediate side lanes is considered. Thus a double lane change to the right or left is not immediately considered. 
- The model asumes every other car will keep driving the same lane. A more sofisticated prediction model would be needed to make more complex and accurate decisions on edge cases.
- Sometimes the model would perform a lane change (e.g. to the left) to a slightly faster advancing lane while the best move would be to wait for a fast car on the right lane to pass by and then perform a right lane change behind it. Modeling this kind of behaviour would need as stated above a more sofisticated prediction model and decision logic. 
- The car seemed sometimes somehow overly cautious before making clear lane changing decision.
- While driving in congestionated traffic in occasions it loops very fast over left and right lane changes. Mostly when the car in front of us inside our new target lane suddenly hits the brakes, but later continues to advance faster than current lane and then hits the brakes again...

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

