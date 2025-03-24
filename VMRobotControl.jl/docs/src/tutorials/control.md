# Virtual Model Control of a Real Robot using ROS

This library is not just for simulation, but can be used for realtime control of torque-controlled
robots. The library aims to avoid any allocations so that it can be used in tight loops without
invoking the garbage collector.

We provide a way to communicate over ros using the python script `./ros/rospy_client.py`. This can 
communicate with a julia controller using TCP/UDP sockets. 

To use it, run this script with python 3, providing the required command line arguments. These are
the number of degrees of freedom of the robot (the length of the torque/effort vector), the name of
the name of the topic to send joint torque commands to, and the name of the topic to subscribe to 
joint state updates on.
All command line arguments can be viewed by calling with `--help`.

The ROS interface was written and tested with the help of Yi Zhang to control the Sciurus robot.

# Implementing your own realtime control

The library provides an interface for realtime control via the two methods:
- [`new_control_cache`](@ref) and
- [`control_step!`](@ref)
New control cache creates a cache bundle: a structure containing the mechanism and a cache for 
storing intermediate computations.

The step function should be called once per loop, and needs to be provided with the current state:
the time `t`, joint configurations `q`, and joint velocities `q̇`.
Then the effect of the virtual mechanism system is applied in two steps:
- The virtual mechanism advances in time to `t` (by taking one euler step)
- The new demanded robot torques are returned

Any component added to the robot is treated as part of the model of the robot, not as part of the 
controller. Therefore, any component added directly to the robot is ignored by `control_step!` and 
does not generate any forces/torques.
If you wish to create a components that acts upon the robot, it must be added to the 
`VirtualMechanismSystem` and may use coordinates from the robot by prefixing the id with `".robot."`.

For example, if I have a robot with coordinate `"tip_error"`, I can create a spring using coordinate
`"tip_error"` by using `".robot.tip_error"`:
```
vms = VirtualMechanismSystem("MyVMS", robot, virtual_mechanism)
spring = LinearSpring(1000.0, ".robot.tip_error")
add_component!(vms, spring; id="Tip spring")
```
If instead I did
```
spring = LinearSpring(1000.0, "tip_error")
add_component!(robot, spring; id="Tip spring")
vms = VirtualMechanismSystem("MyVMS", robot, virtual_mechanism)
```
then the controller will not apply a force due to the spring, because it thinks the spring already 
exists and is part of the actual robot.
