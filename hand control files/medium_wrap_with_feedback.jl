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

for joint_id in keys(joints(shadow_robot))
    # "limits" are here used simply to identify the joints that actually move with respect to the fixed joints
    limits = joint_limits[joint_id]
    isnothing(limits) && continue

    add_coordinate!(shadow_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
end

add_coordinate!(shadow_robot, CoordSum("rh_FFJ1_coord", "rh_FFJ2_coord"); id="rh_FFJ0_coord")
add_coordinate!(shadow_robot, CoordSum("rh_MFJ1_coord", "rh_MFJ2_coord"); id="rh_MFJ0_coord")
add_coordinate!(shadow_robot, CoordSum("rh_RFJ1_coord", "rh_RFJ2_coord"); id="rh_RFJ0_coord")
add_coordinate!(shadow_robot, CoordSum("rh_LFJ1_coord", "rh_LFJ2_coord"); id="rh_LFJ0_coord")


println("URDF parsed !")

print("parsing virtual mechanism URDF ...")

vm_cfg = URDFParserConfig(;suppress_warnings=true) 
# For the moment the urdfs are the same but we might want to change the properties of the virtual robot
vm_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), vm_cfg) 

println("URDF parsed !")

##### COMPLEMENTING THE VIRTUAL ROBOT #####

print("Building the virtual robot...")

# Gravity Compensation and joint limits/damping

joint_limits = vm_cfg.joint_limits

for joint_id in keys(joints(vm_robot))
    # println(joint_id)
    limits = joint_limits[joint_id]
    isnothing(limits) && continue
    # println(limits.upper)
    # println(limits.lower)
    # println("\n")
    add_coordinate!(vm_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
    @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
    #add_deadzone_springs!(vm_robot, 50.0, (limits.lower+0.1, limits.upper-0.1), "$(joint_id)_coord")
    add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
end

add_coordinate!(vm_robot, CoordSum("rh_FFJ1_coord", "rh_FFJ2_coord"); id="rh_FFJ0_coord")
add_coordinate!(vm_robot, CoordSum("rh_MFJ1_coord", "rh_MFJ2_coord"); id="rh_MFJ0_coord")
add_coordinate!(vm_robot, CoordSum("rh_RFJ1_coord", "rh_RFJ2_coord"); id="rh_RFJ0_coord")
add_coordinate!(vm_robot, CoordSum("rh_LFJ1_coord", "rh_LFJ2_coord"); id="rh_LFJ0_coord")

println("Robot built !")

####### VIRTUAL MECHANISM SYSTEM #######

print("Building the virtual mechanisms...")

# CREATION OF THE RAIL

track_points = Matrix{Float64}([0.011 -0.01 0.442; 0.011  -0.075  0.41 ; 0.011 -0.065 0.32; 0.011 -0.0275 0.28; 0.011 -0.01 0.34])
spline = CubicSpline(track_points)

add_frame!(vm_robot; id = "cart_frame")
add_joint!(vm_robot, Rail(spline, zero(Transform{Float64})); parent=root_frame(vm_robot), child="cart_frame", id="RailJoint")
add_coordinate!(vm_robot, FrameOrigin("cart_frame"); id="cart position")
add_coordinate!(vm_robot, JointSubspace("RailJoint");  id="CartDistance")
add_coordinate!(vm_robot, ConstCoord(0.0); id = "Cart target position")
add_coordinate!(vm_robot, CoordDifference("CartDistance", "Cart target position"); id ="Cart position error")
add_component!(vm_robot, LinearInerter(1.0, "cart position");  id="CartInertance") # Cart mass
add_component!(vm_robot, LinearSpring(1.0, "Cart position error"); id = "cart positioning spring")
add_component!(vm_robot, LinearDamper(1.0, "CartDistance"); id="CartDamper");

add_gravity_compensation!(vm_robot, VMRobotControl.DEFAULT_GRAVITY)


vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)

# MOTION MECHANISMS
m = compile(vm_robot)
kcache = new_kinematics_cache(m)  
cart_init_pos = SVector(0.011, -0.01, 0.442)

K = SMatrix{3, 3}(1., 0., 0., 0., 1., 0., 0., 0., 1.)
D = SMatrix{3, 3}(0.1, 0., 0., 0., 0.1, 0., 0., 0., 0.1)

#lightly constraining some joints to avoid unwanted motions
add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_LFJ5_coord"); id = "lf j5 angular spring")
add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_FFJ4_coord"); id = "ff j4 angular spring")
add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_MFJ4_coord"); id = "mf j4 angular spring")
add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_RFJ4_coord"); id = "rf j4 angular spring")
add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_LFJ4_coord"); id = "lf j4 angular spring")
add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ1_coord"); id = "wr j1 angular spring")

#Linking fingers to this target 
rh_fftip_frame_id = get_compiled_frameID(m, "rh_fftip")
fftip_transform = get_transform(kcache, rh_fftip_frame_id)
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_fftip"); id="ff position")
add_coordinate!(vms, FramePoint(".virtual_mechanism.cart_frame", fftip_transform.origin - cart_init_pos); id = "ff Target position") 
add_coordinate!(vms, CoordDifference("ff Target position", "ff position"); id="ff position error")

add_component!(vms, LinearSpring(K, "ff position error");           id="ff position spring")
add_component!(vms, LinearDamper(D, "ff position error");           id="ff position damper")

rh_mftip_frame_id = get_compiled_frameID(m, "rh_mftip")
mftip_transform = get_transform(kcache, rh_mftip_frame_id)
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_mftip"); id="mf position")
add_coordinate!(vms, FramePoint(".virtual_mechanism.cart_frame", mftip_transform.origin - cart_init_pos); id = "mf Target position") 
add_coordinate!(vms, CoordDifference("mf Target position", "mf position"); id="mf position error")

add_component!(vms, LinearSpring(K, "mf position error");           id="mf position spring")
add_component!(vms, LinearDamper(D, "mf position error");           id="mf position damper")

rh_rftip_frame_id = get_compiled_frameID(m, "rh_rftip")
rftip_transform = get_transform(kcache, rh_rftip_frame_id)
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_rftip"); id="rf position")
add_coordinate!(vms, FramePoint(".virtual_mechanism.cart_frame", rftip_transform.origin - cart_init_pos); id = "rf Target position") 
add_coordinate!(vms, CoordDifference("rf Target position", "rf position"); id="rf position error")

add_component!(vms, LinearSpring(K, "rf position error");           id="rf position spring")
add_component!(vms, LinearDamper(D, "rf position error");           id="rf position damper")

rh_lftip_frame_id = get_compiled_frameID(m, "rh_lftip")
lftip_transform = get_transform(kcache, rh_lftip_frame_id)
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_lftip"); id="lf position")
add_coordinate!(vms, FramePoint(".virtual_mechanism.cart_frame", lftip_transform.origin - cart_init_pos); id = "lf Target position") 
add_coordinate!(vms, CoordDifference("lf Target position", "lf position"); id="lf position error")

add_component!(vms, LinearSpring(K, "lf position error");           id="lf position spring")
add_component!(vms, LinearDamper(D, "lf position error");           id="lf position damper")

# THUMB HANDLING 
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_thtip"); id ="th position")
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_ffmiddle"); id= "ff middle position")
add_coordinate!(vms, CoordDifference("th position", "ff middle position"); id = "th ff dist")
add_coordinate!(vms, CoordNorm("th ff dist"); id="th ff norm")
add_coordinate!(vms, ConstCoord(0.12); id="th spring length")
add_coordinate!(vms, CoordDifference("th ff norm", "th spring length"); id = "th position error")

add_component!(vms, LinearSpring(0.1, "th position error"); id="th position spring")
add_component!(vms, LinearDamper(0.1, "th position error"); id="th position damper")

add_coordinate!(vms, ConstCoord(1.22); id="th j4 target angle")
add_coordinate!(vms, JointSubspace(".virtual_mechanism.rh_THJ4"); id= "th j4 angle")
add_coordinate!(vms, CoordDifference("th j4 target angle", "th j4 angle"); id="th j4 angle error")

add_component!(vms, LinearSpring(0.01, "th j4 angle error"); id="th j4 angular spring")
add_component!(vms, LinearDamper(0.001, "th j4 angle error"); id="th j4 angular damper")


println("Virtual Mechanism Built !")

print("Linking real robot and virtual robot ...")

feedback_stiffness = 0.0005

# START BY LINKING UNCOUPLED JOINTS
joint_limits = shadow_cfg.joint_limits
uncoupled_joints = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ3", "rh_MFJ4", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4","rh_THJ5"]


for joint_id in uncoupled_joints
    add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord");id="$(joint_id) coord diff")
    # use deadzone springs instead of linear springs to take the mismatches into account
    add_deadzone_springs!(vms, feedback_stiffness, (-0.05, 0.05), "$(joint_id) coord diff") 
    #add_component!(vms, LinearSpring(feedback_stiffness, "$(joint_id) coord diff"); id="$(joint_id) feedback spring")
    #add_component!(vms, LinearDamper(0.00, "$(joint_id) coord diff"); id = "$(joint_id) coord damper")  no damping for the moment
end


# LINKING COUPLED JOINTS

coupled_joints = ["rh_FFJ0", "rh_MFJ0", "rh_LFJ0"] # RFJ0 is removed because of hardware failure : no feedback on this joint

for joint_id in coupled_joints
    add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord");id="$(joint_id) coord diff")
    # use deadzone springs instead of linear springs to take the mismatch into account
    add_deadzone_springs!(vms, feedback_stiffness, (-0.05, 0.05), "$(joint_id) coord diff") 
    #add_component!(vms, LinearSpring(feedback_stiffness, "$(joint_id) coord diff"); id="$(joint_id) feedback spring")
    #add_component!(vms, LinearDamper(0.00, "$(joint_id) coord diff"); id = "$(joint_id) coord damper")  no damping for the moment  
end

println("Linked !")

print("Definition of the control and setup functions....")

function update_medium_wrap_coord(args, cache, coord)
    target_rail_id, th_spring_length_id, th_j4_angular_spring_id, th_j4_target_angle_id = args
    # target_rail_id = args

    #update the cart position on the track
    rail_min = 0
    rail_max = 0.25
    rail_value = rail_min + (rail_max - rail_min)*coord # linear proportion
    cache[target_rail_id] = remake(cache[target_rail_id] ; coord_data=ConstCoord(rail_value))

    #update the length of the spring between the thumb and the ff finger
    th_spring_end_coord = 0.6
    if coord <= th_spring_end_coord
        length_max = 0.12
        length_min = 0.0
        length_value = length_max - (length_max - length_min)*(coord/th_spring_end_coord)
        cache[th_spring_length_id] = remake(cache[th_spring_length_id] ; coord_data = ConstCoord(length_value))
    end

    #update the angle of the thumb
    th_angle_end_coord = 0.4
    if coord <= th_angle_end_coord 
        angle_max = 1.22
        angle_min = 0.8
        angle_value = angle_max - (angle_max - angle_min)*(coord/th_angle_end_coord)
        cache[th_j4_target_angle_id] = remake(cache[th_j4_target_angle_id] ; coord_data = ConstCoord(angle_value))
    end

    #update the stiffness of the angular spring
    th_stiff_end_coord = 0.5
    if coord <= th_stiff_end_coord
        stiff_max = 0.01
        stiff_min = 0.0
        stiff_value = stiff_max - (stiff_max - stiff_min)*(coord/th_stiff_end_coord)
        cache[th_j4_angular_spring_id] = remake(cache[th_j4_angular_spring_id] ; stiffness = stiff_value)
    end

    nothing
end

function f_setup(cache) 
    return (get_compiled_coordID(cache, ".virtual_mechanism.Cart target position") , get_compiled_coordID(cache, "th spring length"), 
    get_compiled_componentID(cache, "th j4 angular spring"), get_compiled_coordID(cache, "th j4 target angle"))
end

function f_control(cache, t, args, extra)

    t_start = 2.
    t_end = 10. #we want to reach the end position at t = t_end 
    
    if t >= t_start && t <= t_end 
        coord_value = (t - t_start)/(t_end - t_start)
        update_medium_wrap_coord(args, cache, coord_value)
    end
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
#with_rospy_connection("172.29.130.141", ROSPY_LISTEN_PORT, 24, 48) do connection
    ros_vm_position_controller(connection, cvms, qᵛ, joint_names; f_control, f_setup, E_max=10.0)
end
