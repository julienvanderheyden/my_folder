using Revise
using GeometryBasics: Vec3f, Point3f
using LinearAlgebra
using StaticArrays
using VMRobotControl
using VMRobotControl.Splines: CubicSpline
using DifferentialEquations
#using MeshIO
include("functions.jl")

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
    add_deadzone_springs!(vm_robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
    add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
end

println("Robot built !")

####### VIRTUAL MECHANISM SYSTEM #######

print("Building the virtual mechanisms...")

vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)
root = root_frame(vms.robot)

# m = compile(robot)
# kcache = new_kinematics_cache(m)  

# cart_init_pos = SVector(0.011, -0.01, 0.442)

# #Linking the four fingers to the same point, with non-zero equilibrium springs

K = 0.01
D = 0.001

#lightly constraining some joints to avoid unwanted motions
add_component!(vms, LinearSpring(0.001, ".virtual_mechanism.rh_LFJ5_coord"); id = "lf j5 angular spring")
add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_FFJ4_coord"); id = "ff j4 angular spring")
add_component!(vms, LinearSpring(0.001, ".virtual_mechanism.rh_MFJ4_coord"); id = "mf j4 angular spring")
add_component!(vms, LinearSpring(0.001, ".virtual_mechanism.rh_RFJ4_coord"); id = "rf j4 angular spring")
add_component!(vms, LinearSpring(0.001, ".virtual_mechanism.rh_LFJ4_coord"); id = "lf j4 angular spring")
#add_component!(vms, LinearSpring(0.001, ".virtual_mechanism.rh_WRJ1_coord"); id = "wr j1 angular spring")
#add_component!(vms, LinearSpring(0.0001, ".virtual_mechanism.rh_FFJ3_coord"); id = "ff j3 angular spring")


# THUMB HANDLING 
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_thtip"); id ="th position")
add_coordinate!(vms, FramePoint(".virtual_mechanism.rh_ffmiddle", SVector(0.012,0.0,0.0)); id= "th target position")
add_coordinate!(vms, CoordDifference("th position", "th target position"); id = "th target dist")
add_coordinate!(vms, CoordNorm("th target dist"); id="th target norm")
add_coordinate!(vms, ConstCoord(0.0); id="th spring length")
add_coordinate!(vms, CoordDifference("th target norm", "th spring length"); id = "th position error")

add_component!(vms, LinearSpring(0.0, "th position error"); id="th position spring")
add_component!(vms, LinearDamper(0.1, "th position error"); id="th position damper")

add_coordinate!(vms, ConstCoord(1.5); id="th j1 target angle")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_THJ1_coord", "th j1 target angle"); id="th j1 angle error")
add_component!(vms, LinearSpring(0.0, "th j1 angle error"); id="th j1 angular spring")

add_coordinate!(vms, ConstCoord(1.57); id = "ff j2 angle target")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_FFJ2_coord", "ff j2 angle target") ; id ="ff j2 angle error")
add_component!(vms, LinearSpring(0.0, "ff j2 angle error"); id = "ff j2 spring")

# add_coordinate!(vms, ConstCoord(1.00); id="th j5 target angle")
# add_coordinate!(vms, JointSubspace(".robot.rh_THJ5"); id= "th j5 angle")
# add_coordinate!(vms, CoordDifference("th j5 target angle", "th j5 angle"); id="th j5 angle error")

# add_component!(vms, LinearSpring(100.0, "th j5 angle error"); id="th j5 angular spring")
# add_component!(vms, LinearDamper(10.0, "th j5 angle error"); id="th j5 angular damper")

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

function update_lateral_pinch_coord(args, cache, coord)
    #target_rail_id, th_spring_length_id, th_j5_target_angle_id = args
    th_position_spring_id, th_target_pos_id, ffmiddle_frame_id, ff_j2_spring_id, th_j1_angular_spring = args

    #update the length of the spring between the thumb and the ff finger

    # First phase : the thumb converges to the top of the phalanx
    if coord < 0.6
        th_stiff_max = 0.5
        th_stiff_min = 0.0
        th_stiff_value = th_stiff_min + (th_stiff_max - th_stiff_min)*(coord/0.6) 
        cache[th_position_spring_id] = remake(cache[th_position_spring_id] ; stiffness = th_stiff_value)

    # phase 2 : a force is applied to go "into" the phalanx
    elseif coord > 0.8
        cache[th_target_pos_id] = remake(cache[th_target_pos_id]; coord_data = FramePoint(ffmiddle_frame_id, SVector(0.0,0.,0.)))
        th_j1_stiff_max = 0.01
        th_j1_stiff_min = 0.0
        th_j1_stiff_value = th_j1_stiff_min + (th_j1_stiff_max - th_j1_stiff_min)*((coord - 0.8)/(1.0 - 0.8))
        cache[th_j1_angular_spring] = remake(cache[th_j1_angular_spring] ; stiffness = th_j1_stiff_value)
    end

    #update the spring of the second phalanx
    j2_activation_coord = 0.3
    if coord > j2_activation_coord
        stiff_max = 0.01
        stiff_min = 0.0
        stiff_value = stiff_min + (stiff_max - stiff_min)*((coord - j2_activation_coord)/(1.0 - j2_activation_coord))
        cache[ff_j2_spring_id] = remake(cache[ff_j2_spring_id] ; stiffness = stiff_value)
    end 

    # #update the angle of the thumb
    # angle_max = 1.0
    # angle_min = 0.0
    # angle_value = angle_max - (angle_max - angle_min)*coord
    # cache[th_j5_target_angle_id] = remake(cache[th_j5_target_angle_id] ; coord_data = ConstCoord(angle_value))

    nothing
end

function f_setup(cache) 
    return  (get_compiled_componentID(cache, "th position spring"), get_compiled_coordID(cache, "th target position"), 
            get_compiled_frameID(cache, ".virtual_mechanism.rh_ffmiddle"), get_compiled_componentID(cache, "ff j2 spring"),
            get_compiled_componentID(cache, "th j1 angular spring"))#, get_compiled_coordID(cache, "th j5 target angle"))
end

function f_control(cache, t, args, extra)

    t_start = 10.
    t_end = 20. #we want to reach the end position at t = t_end 
    
    if t >= t_start && t <= t_end 
        coord_value = (t - t_start) /(t_end -t_start)
        update_lateral_pinch_coord(args, cache, coord_value)
    end
end

println("Defined")

# Compile the virtual mechanism system, and run the controller via ROS
# Make sure rospy_client.py is running first.
println("Connecting to ROS client...")
cvms = compile(vms)
qᵛ = generate_q_init(cvms; mf=true, rf=true, lf=true)
qᵛ[5] = 1.57
qᵛ[4] = 1.0
qᵛ[23] = -0.7

joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                "rh_THJ3", "rh_THJ4", "rh_THJ5"]


with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
    ros_vm_position_controller(connection, cvms, qᵛ, joint_names; f_control, f_setup, E_max=10.0)
end