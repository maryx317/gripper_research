# Fall 2021 Gripper Research 

## Problem and goals
In physics simulators, picking things up with a gripper is very difficult. In PyBullet specifically, simulations are usually made with rigid bodies, a fully solid object where the deformation is zero or close to zero. If one were to create a gripper with these objects and  attempt to pick up another rigid body, it would be very slippery and close to impossible to pick it up like a normal human hand would. Take a glass cup for example. Our human hands could easily grip around it and pick it up vertically. A simulated gripper made of rigid bodies could attempt to do the same thing, but the glass cup would fall right through. The difference between the human hand and a rigid body gripper is that a human hand is fleshy and our fingerprints help create more points of contact and improve grip. How can we modify a rigid body gripper to do the same thing a human hand can with the same kind of movement?

With that in mind, the goal of this research project is to better simulate grip in PyBullet. This would include figuring out how to make a simple gripper with “finger padding” that can emulate our hands gripping things. 

*For more information on PyBullet, visit PyBullet’s website at https://pybullet.org*

## Results

### Setting the foundation: simple gripper
I started out with just creating a simple gripper to get used to using PyBullet and creating joints and objects. This gripper is shown in the GIF below. This gripper will be the base to be improved upon throughout the solutions following this sections.

In setting this gripper up, I also created a way to evaluate the grippers. When the simulation method is run, I can set how many simulations to run. In each simulation, the sphere being picked up will be randomly placed on the ground, and the method will count how many times the ball was successfully picked up. 

With this evaluation method, this basic simple gripper had a success rate of 27%.

![solid_gripper_0_units](https://user-images.githubusercontent.com/30352267/145732871-fd206295-1444-4643-bb89-6523de294f72.gif)
    
### Solution 1: solid body units
The first solution includes adding units to the arms of the gripper. The concept here is to increase the number of contact points the gripper has to the ball. The units are controlled with a prismatic joint that allows them to be pushed into the arm of the gripper, much like how human flesh works. 

In this section, I tested pad sizes of 1x1, 2x2, 3x3, and 4x4 sets of units. Reference the two GIFS below for the grippers with 2x2 and 4x4 sets of units. 

#### Optimization
Here, I also attempted to use CMA-ES optimization to optimize various parameters of the gripper. The three that I specifically optimized were:
- How much the units could move into the arm
- Effort (amount of force) of the arm that grabs the ball

Surprisingly, the optimization of these all ended out to be 0. 

2x2 sets of units: 75% success rate

![solid_gripper_2_units](https://user-images.githubusercontent.com/30352267/145732872-f939aa01-8458-4d7c-a426-f9073e345b64.gif)
    
4x4 sets of units: 76% success rate

![solid_gripper_4_units](https://user-images.githubusercontent.com/30352267/145732874-9534f7a2-721c-4de1-be72-de6f9d8562d5.gif)

#### Conclusion
Compared to the 27% success rate of the base gripper with no units, the success rate of this solution shows that this solution was pretty successful. Increasing the amount of contact points the gripper has to the ball does increase the success rate quite a bit. 

### Solution 2: soft body pads
After evaluating the first solution, I wanted to explore using soft bodies to make the gripper more successful. Soft bodies not only emulate flesh, it also drastically increases the amount of contact points the gripper makes to the ball, which (as shown in the first solution) does increase the success rate of this gripper. 

Here, I tested two different sized pads, one small thin one and one bigger pad. Before evaluating either, I also created identical solid body versions of these grippers to ensure that soft bodies does improve the success rate of the gripper. 

Solid body gripper with a small pad: 32% success rate

Soft body gripper with a small pad: 45.99% success rate

![soft_gripper_small_pad](https://user-images.githubusercontent.com/30352267/145732870-f9e7a1f6-124f-4c54-983e-384a35023753.gif)
    
Solid body gripper with a big pad: 74% success rate

![solid_gripper_big_pad](https://user-images.githubusercontent.com/30352267/145732877-193d0b82-7a3c-4a65-96a1-48f6404316ee.gif)
    
Soft body gripper with a big pad: 90-94% success rate

![soft_gripper_big_pad](https://user-images.githubusercontent.com/30352267/145732956-3c982f84-6021-4bc3-b883-5edd97204bbc.gif)
    
#### Conclusion
As seen in the success rates here, soft bodies definitely improved the success rate of the gripper. The big pad even had a 90+% success rate. The only reason why it did not have 100% success rate is that some of the randomly placed balls were too far for the gripper to fully grab onto. To solve this, a possible solution would be an even bigger pad. 

Either way, this provides a good direction to go in improving grip in physics simulators.
