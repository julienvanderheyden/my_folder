using Revise
using GeometryBasics: Vec3f, Point3f
using LinearAlgebra
using StaticArrays
using VMRobotControl
using VMRobotControl.Splines: CubicSpline
#using DifferentialEquations
#using MeshIO

module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
include(joinpath(module_path, "ros/ROS.jl"))

###### URDF PARSING #####

using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end

print("parsing robot URDF...  ")

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

println("Robot built !")

####### VIRTUAL MECHANISM SYSTEM #######

print("Building the virtual mechanisms...")

vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)

# MOTION MECHANISMS
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_fftip"); id="fftip position")
add_coordinate!(vms, FrameOrigin(".virtual_mechanism.rh_palm"); id="palm position")
add_coordinate!(vms, CoordDifference("fftip position", "palm position"); id="fftip position error")

#add_component!(vms, LinearSpring(0.001, "fftip position error"); id="fftip position spring")
#add_component!(vms, LinearDamper(0.0001, "fftip position error"); id="fftip position damper")

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
    ros_vm_position_controller(connection, cvms, qᵛ, joint_names; E_max=10.0)
end