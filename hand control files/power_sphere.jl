using Revise
using GeometryBasics: Vec3f, Point3f
using LinearAlgebra
using StaticArrays
using VMRobotControl
using VMRobotControl.Splines: CubicSpline
using DifferentialEquations
#using MeshIO

module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
include(joinpath(module_path, "ros/ROS.jl"))

###### URDF PARSING #####

using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end

print("parsing robot URDF... ")

module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])

shadow_cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
shadow_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), shadow_cfg)

println("URDF parsed !")

print("parsing virtual mechanism URDF ...")

vm_cfg = URDFParserConfig(;suppress_warnings=true) 
# For the moment the urdfs are the same but we might want to change the properties of the virtual robot
vm_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), vm_cfg) 

println("URDF parsed !")

##### COMPLEMENTING THE VIRTUAL ROBOT #####

print("Building the virtual robot...")

# Gravity Compensation and joint limits/damping
add_gravity_compensation!(vm_robot, VMRobotControl.DEFAULT_GRAVITY)

joint_limits = vm_cfg.joint_limits

for joint_id in keys(joints(vm_robot))
    limits = joint_limits[joint_id]
    isnothing(limits) && continue
    add_coordinate!(vm_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
    @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
    #add_deadzone_springs!(vm_robot, 50.0, (limits.lower+0.1, limits.upper-0.1), "$(joint_id)_coord")
    add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
end

println("Robot built !")

####### VIRTUAL MECHANISM SYSTEM #######

print("Building the virtual mechanisms...")

vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)
root = root_frame(vms.robot)

K = 0.01
D = 0.001

#Lightly constraint some joints to avoid unwanted motions
add_component!(vms, LinearSpring(0.001, ".virtual_mechanism.rh_LFJ5_coord"); id = "lf j5 angular spring")
add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ1_coord"); id = "wr j1 angular spring")
add_component!(vms, LinearDamper(0.01, ".virtual_mechanism.rh_WRJ1_coord"); id = "wr j1 angular damper")

# PHASE 1 : FINGERS SPACING

add_coordinate!(vms, ConstCoord(0.0); id = "angular spring length")

#ff mf spacing
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_MFJ4_coord", ".virtual_mechanism.rh_FFJ4_coord"); id="ff mf j4 angular diff")
add_coordinate!(vms, CoordDifference("ff mf j4 angular diff", "angular spring length") ; id="ff mf j4 angular error")
add_component!(vms, LinearSpring(K, "ff mf j4 angular error"); id="ff mf angular spring")
add_component!(vms, LinearDamper(D, "ff mf j4 angular error"); id="ff mf angular damper")

#mf rf spacing
add_coordinate!(vms, CoordSum(".virtual_mechanism.rh_RFJ4_coord", ".virtual_mechanism.rh_MFJ4_coord"); id="mf rf j4 angular diff")
add_coordinate!(vms, CoordSum("mf rf j4 angular diff", "angular spring length") ; id="mf rf j4 angular error")
add_component!(vms, LinearSpring(K, "mf rf j4 angular error"); id="mf rf angular spring")
add_component!(vms, LinearDamper(D, "mf rf j4 angular error"); id="mf rf angular damper")

#rf lf spacing
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_RFJ4_coord", ".virtual_mechanism.rh_LFJ4_coord"); id="rf lf j4 angular diff")
add_coordinate!(vms, CoordDifference("rf lf j4 angular diff", "angular spring length") ; id="rf lf j4 angular error")
add_component!(vms, LinearSpring(K, "rf lf j4 angular error"); id="rf lf angular spring")
add_component!(vms, LinearDamper(D, "rf lf j4 angular error"); id="rf lf angular damper")

#th spacing
add_coordinate!(vms, ConstCoord(0.0); id="th spring length")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_THJ4_coord", "th spring length"); id="th j4 error")
add_component!(vms, LinearSpring(K, "th j4 error"); id="th j4 spring")

# PHASE 2 : GRASPING : EACH JOINTS SHOULD GO TO ITS FINAL POSITION, BUT AT DIFFERENT TIMESCALES

add_coordinate!(vms, ConstCoord(0.0); id="J3 target angle") # First that should be activated
add_coordinate!(vms, ConstCoord(0.0); id="J2 target angle") # Second 
add_coordinate!(vms, ConstCoord(0.0); id="J1 target angle") # Last

# First finger
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_FFJ3_coord", "J3 target angle"); id="ff j3 angle error")
add_component!(vms, LinearSpring(K, "ff j3 angle error"); id="ff j3 spring")
add_component!(vms, LinearDamper(D, "ff j3 angle error"); id="ff j3 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_FFJ2_coord", "J2 target angle"); id="ff j2 angle error")
add_component!(vms, LinearSpring(K, "ff j2 angle error"); id="ff j2 spring")
add_component!(vms, LinearDamper(D, "ff j2 angle error"); id="ff j2 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_FFJ1_coord", "J1 target angle"); id="ff j1 angle error")
add_component!(vms, LinearSpring(K, "ff j1 angle error"); id="ff j1 spring")
add_component!(vms, LinearDamper(D, "ff j1 angle error"); id="ff j1 damper")

# Middle Finger
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_MFJ3_coord", "J3 target angle"); id="mf j3 angle error")
add_component!(vms, LinearSpring(K, "mf j3 angle error"); id="mf j3 spring")
add_component!(vms, LinearDamper(D, "mf j3 angle error"); id="mf j3 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_MFJ2_coord", "J2 target angle"); id="mf j2 angle error")
add_component!(vms, LinearSpring(K, "mf j2 angle error"); id="mf j2 spring")
add_component!(vms, LinearDamper(D, "mf j2 angle error"); id="mf j2 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_MFJ1_coord", "J1 target angle"); id="mf j1 angle error")
add_component!(vms, LinearSpring(K, "mf j1 angle error"); id="mf j1 spring")
add_component!(vms, LinearDamper(D, "mf j1 angle error"); id="mf j1 damper")

# Ring Finger
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_RFJ3_coord", "J3 target angle"); id="rf j3 angle error")
add_component!(vms, LinearSpring(K, "rf j3 angle error"); id="rf j3 spring")
add_component!(vms, LinearDamper(D, "rf j3 angle error"); id="rf j3 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_RFJ2_coord", "J2 target angle"); id="rf j2 angle error")
add_component!(vms, LinearSpring(K, "rf j2 angle error"); id="rf j2 spring")
add_component!(vms, LinearDamper(D, "rf j2 angle error"); id="rf j2 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_RFJ1_coord", "J1 target angle"); id="rf j1 angle error")
add_component!(vms, LinearSpring(K, "rf j1 angle error"); id="rf j1 spring")
add_component!(vms, LinearDamper(D, "rf j1 angle error"); id="rf j1 damper")

# Little Finger
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_LFJ3_coord", "J3 target angle"); id="lf j3 angle error")
add_component!(vms, LinearSpring(K, "lf j3 angle error"); id="lf j3 spring")
add_component!(vms, LinearDamper(D, "lf j3 angle error"); id="lf j3 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_LFJ2_coord", "J2 target angle"); id="lf j2 angle error")
add_component!(vms, LinearSpring(K, "lf j2 angle error"); id="lf j2 spring")
add_component!(vms, LinearDamper(D, "lf j2 angle error"); id="lf j2 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_LFJ1_coord", "J1 target angle"); id="lf j1 angle error")
add_component!(vms, LinearSpring(K, "lf j1 angle error"); id="lf j1 spring")
add_component!(vms, LinearDamper(D, "lf j1 angle error"); id="lf j1 damper")

# Thumb 
add_coordinate!(vms, ConstCoord(0.0); id="th J5 target angle") # First that should be activated
add_coordinate!(vms, ConstCoord(0.0); id="th J2 target angle") # Second 
add_coordinate!(vms, ConstCoord(0.0); id="th J1 target angle") # Last

add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_THJ5_coord", "th J5 target angle"); id="th j5 angle error")
add_component!(vms, LinearSpring(K, "th j5 angle error"); id="th j5 spring")
add_component!(vms, LinearDamper(D, "th j5 angle error"); id="th j5 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_THJ2_coord", "th J2 target angle"); id="th j2 angle error")
add_component!(vms, LinearSpring(K, "th j2 angle error"); id="th j2 spring")
add_component!(vms, LinearDamper(D, "th j2 angle error"); id="th j2 damper")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_THJ1_coord", "th J1 target angle"); id="th j1 angle error")
add_component!(vms, LinearSpring(K, "th j1 angle error"); id="th j1 spring")
add_component!(vms, LinearDamper(D, "th j1 angle error"); id="th j1 damper")


println("Virtual Mechanism Built !")

print("Linking real robot and virtual robot ...")

joint_limits = shadow_cfg.joint_limits

for joint_id in keys(joints(shadow_robot))
    # "limits" are here used simply to identify the joints that actually move with respect to the fixed joints
    limits = joint_limits[joint_id]
    isnothing(limits) && continue

    add_coordinate!(shadow_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
    add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord");id="$(joint_id) coord diff")
    add_component!(vms, LinearSpring(0.0, "$(joint_id) coord diff"); id = "$(joint_id) coord spring")
    add_component!(vms, LinearDamper(0.00, "$(joint_id) coord diff"); id = "$(joint_id) coord damper")
end

println("Linked !")

print("Definition of the control and setup functions....")

function update_power_sphere_coord(args, cache, coord)
    angular_spring_length_id, th_spring_length_id, j3_target_angle_id, j2_target_angle_id, j1_target_angle_id, th_j5_target_angle_id,
    th_j2_target_angle_id, th_j1_target_angle_id = args

    # PHASE 1 : FINGERS SPACING
    #update the angle spacing the fingers, and the thumb
    spacing_end_coord = 0.3
    if coord < spacing_end_coord
        fingers_angle_min = 0.0
        fingers_angle_max = 0.5
        fingers_angle_value = fingers_angle_min + (fingers_angle_max - fingers_angle_min)*(coord/spacing_end_coord)
        cache[angular_spring_length_id] = remake(cache[angular_spring_length_id] ; coord_data = ConstCoord(fingers_angle_value))
        th_angle_min = 0.0
        th_angle_max = 1.22
        th_angle_value = th_angle_min + (th_angle_max - th_angle_min)*(coord/spacing_end_coord)
        cache[th_spring_length_id] = remake(cache[th_spring_length_id] ; coord_data = ConstCoord(th_angle_value))
    end

    # PHASE 2 : GRASPING

    #Start by metacarpophalangeal joints (MCP joints)
    j3_start_coord = 0.3
    j3_end_coord = 0.8
    if coord >= j3_start_coord && coord <= j3_end_coord

        j3_coord_value = (coord - j3_start_coord)/(j3_end_coord - j3_start_coord) 

        #fingers 
        j3_angle_min = 0.0
        j3_angle_max = 1.57
        j3_angle_value = j3_angle_min + (j3_angle_max - j3_angle_min)*j3_coord_value
        cache[j3_target_angle_id] = remake(cache[j3_target_angle_id]; coord_data = ConstCoord(j3_angle_value))

    end

    #shift metacarpophalangeal joints (MCP joints) of the thumb
    th_j3_start_coord = 0.6
    th_j3_end_coord = 0.9
    if coord >= th_j3_start_coord && coord <= th_j3_end_coord

        th_j3_coord_value = (coord - th_j3_start_coord)/(j3_end_coord - th_j3_start_coord) 

        #thumb
        th_j5_angle_min = 0.0
        th_j5_angle_max = 1.05
        th_j5_angle_value = th_j5_angle_min + (th_j5_angle_max - th_j5_angle_min)*th_j3_coord_value
        cache[th_j5_target_angle_id] = remake(cache[th_j5_target_angle_id] ; coord_data = ConstCoord(th_j5_angle_value))
    end

    #Followed by proximal interphalangeal joints (PIP joints)
    j2_start_coord = 0.5
    j2_end_coord = 0.9
    if coord >= j2_start_coord && coord <= j2_end_coord

        j2_coord_value = (coord - j2_start_coord)/(j2_end_coord - j2_start_coord) 

        #fingers
        j2_angle_min = 0.0
        j2_angle_max = 1.57
        j2_angle_value = j2_angle_min + (j2_angle_max - j2_angle_min)*j2_coord_value
        cache[j2_target_angle_id] = remake(cache[j2_target_angle_id]; coord_data = ConstCoord(j2_angle_value))

        #thumb
        th_j2_angle_min = 0.0
        th_j2_angle_max = 0.7
        th_j2_angle_value = th_j2_angle_min + (th_j2_angle_max - th_j2_angle_min)*j2_coord_value
        cache[th_j2_target_angle_id] = remake(cache[th_j2_target_angle_id] ; coord_data = ConstCoord(th_j2_angle_value))
    end

    #And then distal interphalangeal joints (DIP joints)
    j1_start_coord = 0.7
    j1_end_coord = 1.0
    if coord >= j1_start_coord && coord <= j1_end_coord

        j1_coord_value = (coord - j1_start_coord)/(j1_end_coord - j1_start_coord) 

        #fingers
        j1_angle_min = 0.0
        j1_angle_max = 1.57
        j1_angle_value = j1_angle_min + (j1_angle_max - j1_angle_min)*j1_coord_value
        cache[j1_target_angle_id] = remake(cache[j1_target_angle_id]; coord_data = ConstCoord(j1_angle_value))

        #thumb
        th_j1_angle_min = 0.0
        th_j1_angle_max = 1.57
        th_j1_angle_value = th_j1_angle_min + (th_j1_angle_max - th_j1_angle_min)*j1_coord_value
        cache[th_j1_target_angle_id] = remake(cache[th_j1_target_angle_id] ; coord_data = ConstCoord(th_j1_angle_value))
    end

    nothing
end

function f_setup(cache) 

    return  (get_compiled_coordID(cache, "angular spring length"), get_compiled_coordID(cache, "th spring length"), get_compiled_coordID(cache, "J3 target angle"),
            get_compiled_coordID(cache, "J2 target angle"), get_compiled_coordID(cache, "J1 target angle"), get_compiled_coordID(cache, "th J5 target angle"),
            get_compiled_coordID(cache, "th J2 target angle"), get_compiled_coordID(cache, "th J1 target angle"))
end

function f_control(cache, t, args, extra)

    t_start = 20.
    t_end = 30. #we want to reach the end position at t = t_end 
    
    if t >= t_start && t <= t_end 
        coord_value = (t - t_start) /(t_end -t_start)
        update_power_sphere_coord(args, cache, coord_value)
    end
    nothing
end

println("Defined")

# Compile the virtual mechanism system, and run the controller via ROS
# Make sure rospy_client.py is running first.
println("Connecting to ROS client...")
cvms = compile(vms)
qᵛ = zero_q(cvms.virtual_mechanism)

joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                "rh_THJ3", "rh_THJ4", "rh_THJ5"]


with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
    ros_vm_position_controller(connection, cvms, qᵛ, joint_names; f_control, f_setup, E_max=10.0)
end