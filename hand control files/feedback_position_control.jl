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

print("parsing robot URDF...  ")

module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])

shadow_cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
shadow_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), shadow_cfg)

joint_limits = shadow_cfg.joint_limits

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
    #add_deadzone_springs!(vm_robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
    #add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
end

add_coordinate!(vm_robot, CoordSum("rh_FFJ1_coord", "rh_FFJ2_coord"); id="rh_FFJ0_coord")
add_coordinate!(vm_robot, CoordSum("rh_MFJ1_coord", "rh_MFJ2_coord"); id="rh_MFJ0_coord")
add_coordinate!(vm_robot, CoordSum("rh_RFJ1_coord", "rh_RFJ2_coord"); id="rh_RFJ0_coord")
add_coordinate!(vm_robot, CoordSum("rh_LFJ1_coord", "rh_LFJ2_coord"); id="rh_LFJ0_coord")

println("Robot built !")

####### VIRTUAL MECHANISM SYSTEM #######

print("Building the virtual mechanisms...")

vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)
root = root_frame(vms.virtual_mechanism)

# MOTION MECHANISMS

# add_coordinate!(vms, FramePoint(".virtual_mechanism.$root", SVector(0.03, -0.02, 0.3));        id="Target position")
# add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_fftip_mass_coord", "Target position"); id="Position error");

# K = SMatrix{3, 3}(0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.01)
# add_component!(vms, LinearSpring(K, "Position error");  id="Linear Spring")
# D = SMatrix{3, 3}(0.001, 0., 0., 0., 0.001, 0., 0., 0., 0.001)
# add_component!(vms, LinearDamper(D, "Position error");  id="Linear Damper")

add_coordinate!(vms, ConstCoord(1.57); id="target")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_MFJ1_coord", "target"); id="mfj1 pos error")
add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_MFJ2_coord", "target"); id="mfj2 pos error")

add_component!(vms, LinearSpring(0.0001, "mfj1 pos error");  id="mfj1 spring")
add_component!(vms, LinearSpring(0.0001, "mfj2 pos error");  id="mfj2 spring")

println("Virtual Mechanism Built !")

print("Linking real robot and virtual robot ...")

feedback_stiffness = 0.00001
# feedback_damping = 0.0
# feedback_damping = 0.00001
feedback_stiffness = 0.0

# START BY LINKING UNCOUPLED JOINTS
joint_limits = shadow_cfg.joint_limits
uncoupled_joints = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ3", "rh_MFJ4", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4","rh_THJ5"]


for joint_id in uncoupled_joints
    add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord");id="$(joint_id) coord diff")
    # use deadzone springs instead of linear springs to take the mismatches into account
    add_deadzone_springs!(vms, feedback_stiffness, (-0.05, 0.05), "$(joint_id) coord diff") 

    # implement "deadzone" damping using two rectified dampers (transition is linear, not sharp)
    add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (0.045, 0.055), false, false); id = "$(joint_id) feedback damper 1")
    add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (-0.055, -0.045), true, false); id = "$(joint_id) feedback damper 2")
    #add_component!(vms, LinearSpring(feedback_stiffness, "$(joint_id) coord diff"); id="$(joint_id) feedback spring")
    #add_component!(vms, LinearDamper(0.00, "$(joint_id) coord diff"); id = "$(joint_id) coord damper")  no damping for the moment
end


# LINKING COUPLED JOINTS

coupled_joints = ["rh_FFJ0", "rh_MFJ0", "rh_RFJ0" ,"rh_LFJ0"]

for joint_id in coupled_joints
    add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord");id="$(joint_id) coord diff")
    # use deadzone springs instead of linear springs to take the mismatch into account
    add_deadzone_springs!(vms, feedback_stiffness, (-0.1, 0.1), "$(joint_id) coord diff")
    
    add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (0.09, 0.11), false, false); id = "$(joint_id) feedback damper 1")
    add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (-0.11, -0.09), true, false); id = "$(joint_id) feedback damper 2")
    #add_component!(vms, LinearSpring(feedback_stiffness, "$(joint_id) coord diff"); id="$(joint_id) feedback spring")
    #add_component!(vms, LinearDamper(0.00, "$(joint_id) coord diff"); id = "$(joint_id) coord damper")  no damping for the moment  
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