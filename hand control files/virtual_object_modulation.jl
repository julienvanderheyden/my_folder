using Revise
using GeometryBasics: Vec3f, Point3f
using LinearAlgebra
using StaticArrays
using VMRobotControl
using VMRobotControl: remake
using VMRobotControl.Splines: CubicSpline
using DifferentialEquations
include("functions.jl")
#using MeshIO

module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
include(joinpath(module_path, "ros/ROS.jl"))

###### URDF PARSING #####

using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end

const MISMATCH_DEADZONE = 0.15

mutable struct FingerModulationState
    radius::Float64
    equilibrium::Bool
    contact_detected::Bool
    modulation_activated::Bool
    modulation_stopped::Bool
    activation_time::Float64
    stopping_time::Float64
end

FingerModulationState(initial_radius::Float64) = FingerModulationState(initial_radius, false, false, false, false, 0.0, 0.0)

struct FingerConfig
    attracted_frames::Vector{String}       # mass coord IDs used for attraction springs
    attracted_frames_names::Vector{String} # short names for those frames
    repulsed_frames::Vector{String}        # mass coord IDs used for repulsion
    repulsed_frames_names::Vector{String}  # short names for those frames
    coupled_joints::Vector{String}         # joints where J0 = J1 + J2
    uncoupled_joints::Vector{String}       # standard individual joints
end


const FINGER_CONFIGS = Dict{String, FingerConfig}(

    "ff" => FingerConfig(
        ["rh_ffdistal_mass_coord", "rh_ffmiddle_mass_coord", "rh_ffproximal_mass_coord"],
        ["ffdistal", "ffmiddle", "ffprox"],
        ["rh_fftip_mass_coord", "rh_ffmiddle_mass_coord",
         "rh_ffproximal_mass_coord", "rh_ffdistal"],
        ["fftip", "ffmiddle", "ffprox", "ffdistal"],
        ["rh_FFJ0"],
        ["rh_FFJ3", "rh_FFJ4"],
    ),

    "mf" => FingerConfig(
        ["rh_mfdistal_mass_coord", "rh_mfmiddle_mass_coord", "rh_mfproximal_mass_coord"],
        ["mfdistal", "mfmiddle", "mfprox"],
        ["rh_mftip_mass_coord", "rh_mfmiddle_mass_coord",
         "rh_mfproximal_mass_coord", "rh_mfdistal"],
        ["mftip", "mfmiddle", "mfprox", "mfdistal"],
        ["rh_MFJ0"],
        ["rh_MFJ3", "rh_MFJ4"],
    ),

    "rf" => FingerConfig(
        ["rh_rfdistal_mass_coord", "rh_rfmiddle_mass_coord", "rh_rfproximal_mass_coord"],
        ["rfdistal", "rfmiddle", "rfprox"],
        ["rh_rftip_mass_coord", "rh_rfmiddle_mass_coord",
         "rh_rfproximal_mass_coord", "rh_rfdistal"],
        ["rftip", "rfmiddle", "rfprox", "rfdistal"],
        ["rh_RFJ0"],
        ["rh_RFJ3", "rh_RFJ4"],
    ),

    "lf" => FingerConfig(
        ["rh_lfdistal_mass_coord", "rh_lfmiddle_mass_coord", "rh_lfproximal_mass_coord"],
        ["lfdistal", "lfmiddle", "lfprox"],
        ["rh_lftip_mass_coord", "rh_lfmiddle_mass_coord",
         "rh_lfproximal_mass_coord", "rh_lfdistal"],
        ["lftip", "lfmiddle", "lfprox", "lfdistal"],
        ["rh_LFJ0"],
        ["rh_LFJ3", "rh_LFJ4", "rh_LFJ5"],
    ),

    "th" => FingerConfig(
        ["rh_thdistal_mass_coord", "rh_thmiddle_mass_coord"],
        ["thdistal", "thmiddle"],
        ["rh_thtip_mass_coord", "rh_thmiddle_mass_coord",
         "rh_thdistal"],
        ["thtip", "thmiddle", "thdistal"],
        [],                                          # thumb has no coupled J0
        ["rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"],
    ),
)



function virtual_object_modulation(cylinder_radius, feedback_stiffness, feedback_damping)
    print("parsing robot URDF... ")

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

    add_coordinate!(vm_robot, FrameOrigin("rh_ffdistal"); id="rh_ffdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_mfdistal"); id="rh_mfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_rfdistal"); id="rh_rfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_lfdistal"); id="rh_lfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thdistal"); id="rh_thdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_ffproximal"); id="rh_ffproximal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thmiddle"); id="rh_thmiddle")

    println("URDF parsed !")

    ##### COMPLEMENTING THE VIRTUAL ROBOT #####

    print("Building the virtual robot...")

    #joint limits/damping
    joint_limits = vm_cfg.joint_limits

    for joint_id in keys(joints(vm_robot))
        limits = joint_limits[joint_id]
        isnothing(limits) && continue
        add_coordinate!(vm_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
        @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
        add_deadzone_springs!(vm_robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
        add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
    end

    add_coordinate!(vm_robot, CoordSum("rh_FFJ1_coord", "rh_FFJ2_coord"); id="rh_FFJ0_coord")
    add_coordinate!(vm_robot, CoordSum("rh_MFJ1_coord", "rh_MFJ2_coord"); id="rh_MFJ0_coord")
    add_coordinate!(vm_robot, CoordSum("rh_RFJ1_coord", "rh_RFJ2_coord"); id="rh_RFJ0_coord")
    add_coordinate!(vm_robot, CoordSum("rh_LFJ1_coord", "rh_LFJ2_coord"); id="rh_LFJ0_coord")

    println("Robot built !")

    print("Building the virtual mechanisms...")

    m = compile(vm_robot)
    kcache = new_kinematics_cache(m)  
    medium_wrap_preshape = zeros(24)
    medium_wrap_preshape[21] = 1.2 # thumb extended
    kinematics!(kcache, 0.0, medium_wrap_preshape)

    if cylinder_radius < 0.015
        # add one centimeter to the radius to avoid intersection with the fingers 
        rh_ffknuckle_frame_id = get_compiled_frameID(m, "rh_ffknuckle")
        ffknuckle_transform = get_transform(kcache, rh_ffknuckle_frame_id)
    
        cylinder_position = SVector(0.0, -0.03, ffknuckle_transform.origin[3] - cylinder_radius - 0.007)
    else
        # Get the positions of the finger tips
        rh_fftip_frame_id = get_compiled_frameID(m, "rh_fftip")
        fftip_transform = get_transform(kcache, rh_fftip_frame_id)
        p11 = [fftip_transform.origin[2], fftip_transform.origin[3]]  

        rh_ffmiddle_frame_id = get_compiled_frameID(m, "rh_ffmiddle")
        ffmiddle_transform = get_transform(kcache, rh_ffmiddle_frame_id)
        p12 = [ffmiddle_transform.origin[2], ffmiddle_transform.origin[3]]

        rh_thtip_frame_id = get_compiled_frameID(m, "rh_thtip")
        thtip_transform = get_transform(kcache, rh_thtip_frame_id)
        p21 = [thtip_transform.origin[2], thtip_transform.origin[3]]

        rh_thmiddle_frame_id = get_compiled_frameID(m, "rh_thmiddle")
        thmiddle_transform = get_transform(kcache, rh_thmiddle_frame_id)
        p22 = [thmiddle_transform.origin[2], thmiddle_transform.origin[3]]

        # add one centimeter to the radius to avoid intersection with the fingers
        cylinder_position = circle_center_tangent_to_lines(p11, p12, p21, p22, cylinder_radius + 0.01)
        cylinder_position = SVector(0.0, cylinder_position[1], cylinder_position[2])  # Convert to SVector
    end
    
    attracted_frames = ("rh_lfdistal_mass_coord", "rh_lfmiddle_mass_coord", "rh_lfproximal_mass_coord", "rh_rfdistal_mass_coord", 
    "rh_rfmiddle_mass_coord", "rh_rfproximal_mass_coord", "rh_mfdistal_mass_coord", "rh_mfmiddle_mass_coord",
    "rh_mfproximal_mass_coord", "rh_ffdistal_mass_coord", "rh_ffmiddle_mass_coord", "rh_ffproximal_mass_coord",
    "rh_thdistal_mass_coord", "rh_thmiddle_mass_coord") #, "rh_thproximal_mass_coord" , "rh_palm_mass_coord")

    #attracted_frames = ("rh_ffdistal_mass_coord", "rh_ffmiddle_mass_coord", "rh_ffproximal_mass_coord") #, "rh_thproximal_mass_coord" , "rh_palm_mass_coord")

    attracted_frames_names = ("lfdistal", "lfmiddle", "lfprox", "rfdistal", "rfmiddle", "rfprox", "mfdistal", "mfmiddle", "mfprox", "ffdistal", "ffmiddle", 
    "ffprox", "thdistal", "thmiddle") #, "thprox", "palm")

    # attracted_frames_names = ("ffdistal", "ffmiddle", "ffprox") #, "thprox", "palm")
    
    # CYLINDER PRISMATIC JOINTS
    
    for i in 1:length(attracted_frames)
        add_frame!(vm_robot; id="center_frame_$(attracted_frames_names[i])")
        frame_pos = configuration(kcache, get_compiled_coordID(kcache, attracted_frames[i]))
        add_joint!(vm_robot, Rigid(Transform(SVector(frame_pos[1], cylinder_position[2], cylinder_position[3]))); parent=root_frame(vm_robot), child="center_frame_$(attracted_frames_names[i])", id="root_joint_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="prism_frame_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Prismatic(SVector(1.0,0.0,0.0)); parent="center_frame_$(attracted_frames_names[i])", child="prism_frame_$(attracted_frames_names[i])", id="prism_joint_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="revo_frame_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Revolute(SVector(1.0,0.0,0.0)); parent="prism_frame_$(attracted_frames_names[i])", child="revo_frame_$(attracted_frames_names[i])", id = "revo_joint_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="ee_frame_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Rigid(Transform(SVector(0.0,0.0,cylinder_radius))); parent ="revo_frame_$(attracted_frames_names[i])", child ="ee_frame_$(attracted_frames_names[i])", id = "fixed_joint_$(attracted_frames_names[i])")
    
        add_coordinate!(vm_robot, FrameOrigin("ee_frame_$(attracted_frames_names[i])"); id="$(attracted_frames_names[i]) ee position")
        add_component!(vm_robot, PointMass(0.01, "$(attracted_frames_names[i]) ee position"); id="$(attracted_frames_names[i]) ee mass")
    
        add_coordinate!(vm_robot, JointSubspace("prism_joint_$(attracted_frames_names[i])"); id="prism_joint_$(attracted_frames_names[i])")
        add_component!(vm_robot, LinearDamper(0.1, "prism_joint_$(attracted_frames_names[i])"); id="prism_joint_$(attracted_frames_names[i])_damper")
        add_coordinate!(vm_robot, JointSubspace("revo_joint_$(attracted_frames_names[i])"); id="revo_joint_$(attracted_frames_names[i])")
        #add_component!(vm_robot, LinearDamper(0.005, "revo_joint_$(attracted_frames_names[i])"); id="revo_joint_$(attracted_frames_names[i])_damper")    
    
        add_coordinate!(vm_robot, FrameOrigin("center_frame_$(attracted_frames_names[i])"); id="center_frame_$(attracted_frames_names[i])")
        add_coordinate!(vm_robot, FrameOrigin("prism_frame_$(attracted_frames_names[i])"); id="prism_frame_$(attracted_frames_names[i])")
        add_coordinate!(vm_robot, CoordDifference("center_frame_$(attracted_frames_names[i])", "prism_frame_$(attracted_frames_names[i])"); id="$(attracted_frames_names[i])_prismatic_error")
        comeback_stiffness = 0.1
        comeback_stiffness_matrix = SMatrix{3, 3}(comeback_stiffness, 0., 0., 0., comeback_stiffness, 0., 0., 0., comeback_stiffness)
        add_component!(vm_robot, LinearSpring(comeback_stiffness_matrix, "$(attracted_frames_names[i])_prismatic_error"); id = "$(attracted_frames_names[i])_comeback_spring")
    end

    add_gravity_compensation!(vm_robot, VMRobotControl.DEFAULT_GRAVITY)
    
    vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)

    # HAND MOTION

    D = SMatrix{3, 3}(0.05, 0., 0., 0., 0.05, 0., 0., 0., 0.05)

    base_stiffness = 0.05
    #phalanx scaling : 
    # for linear scaling : 
    # > 0 means that the proximal stiffness is higher than the distal stiffness
    # < 0 means that the distal stiffness is higher than the proximal stiffness
    # phalanx_scaling_factor = - 0.5
    # finger_scaling_factor = 1.0


    # for geometric scaling :
    # > 1 means that the proximal stiffness is higher than the distal stiffness
    # < 1 means that the distal stiffness is higher than the proximal stiffness
    phalanx_scaling_factor = 0.5
    finger_scaling_factor = 1.5
    
    
    stiffnesses = generate_stiffnesses_geometric_scaling(base_stiffness, phalanx_scaling_factor, finger_scaling_factor)
    
    damping_decay_rate = 161 # 20% of damping at |z| = 0.01
    exponential_damping_coeff = 0.1
    exponential_damping_matrix = SMatrix{3, 3}(exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff)
    
    for i in 1:length(attracted_frames)
        K = SMatrix{3, 3}(stiffnesses[i], 0., 0., 0., stiffnesses[i], 0., 0., 0., stiffnesses[i])
        #K = SMatrix{3, 3}(base_stiffness, 0., 0., 0., base_stiffness, 0., 0., 0., base_stiffness)
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(attracted_frames_names[i]) ee position", ".virtual_mechanism.$(attracted_frames[i])"); id = "ee $(attracted_frames_names[i]) diff")
        add_component!(vms, LinearSpring(K, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) spring")
        add_component!(vms, LinearDamper(D, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) damper")
        add_component!(vms, ExponentialDamper(exponential_damping_matrix, "ee $(attracted_frames_names[i]) diff", damping_decay_rate); id = "ee $(attracted_frames_names[i]) exp damper")
    end

    add_component!(vms, LinearDamper(SMatrix{3, 3}(10.0, 0., 0., 0., 10.0, 0., 0., 0., 10.0), "ee thmiddle diff"); id = "thmiddle massive damper")
    
    #lightly constraint some joints to avoid unwanted motions 
    # add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_FFJ4_coord"); id = "ff j4 angular spring")
    # add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_MFJ4_coord"); id = "mf j4 angular spring")
    # add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_RFJ4_coord"); id = "rf j4 angular spring")
    # add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_LFJ4_coord"); id = "lf j4 angular spring")
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ1_coord"); id = "wr j1 angular spring")
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ2_coord"); id = "wr j2 angular spring")

    # CYLINDER COLLISION MODEL  

    # add_coordinate!(vms,  ConstCoord(cylinder_position);  id="cylinder position")
    # add_coordinate!(vms, ConstCoord(cylinder_radius); id="cylinder radius")
    
    add_coordinate!(vms, FramePoint(".virtual_mechanism.rh_palm", SVector(0. , 0., 0.07)); id="second palm point")
    
    repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_mftip_mass_coord", ".virtual_mechanism.rh_rftip_mass_coord",".virtual_mechanism.rh_lftip_mass_coord" , 
                        ".virtual_mechanism.rh_thtip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord",".virtual_mechanism.rh_mfmiddle_mass_coord", ".virtual_mechanism.rh_rfmiddle_mass_coord",
                        ".virtual_mechanism.rh_lfmiddle_mass_coord",  ".virtual_mechanism.rh_thmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord", ".virtual_mechanism.rh_mfproximal_mass_coord",
                        ".virtual_mechanism.rh_rfproximal_mass_coord", ".virtual_mechanism.rh_lfproximal_mass_coord", ".virtual_mechanism.rh_thproximal_mass_coord", ".virtual_mechanism.rh_palm_mass_coord", "second palm point",
                        ".virtual_mechanism.rh_ffdistal", ".virtual_mechanism.rh_mfdistal", ".virtual_mechanism.rh_rfdistal", ".virtual_mechanism.rh_lfdistal", ".virtual_mechanism.rh_thdistal", ".virtual_mechanism.rh_thmiddle")

    # repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord",".virtual_mechanism.rh_ffdistal")

    repulsed_frames_names = ("fftip", "mftip", "rftip", "lftip", "thtip", "ffmiddle", "mfmiddle", "rfmiddle", "lfmiddle", "thmiddle", "ffprox", 
                    "mfprox", "rfprox", "lfprox", "thprox", "palm", "palm2", "ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal", "thmiddle2")

    # repulsed_frames_names = ("fftip", "ffmiddle",  "ffprox", "ffdistal")
    
    for i in 1:length(repulsed_frames)
        frame = repulsed_frames[i]
        # make radius and position frame-dependent to be able to modulate the cylinder properties during the motion 
        add_coordinate!(vms, ConstCoord(cylinder_radius); id = "$(repulsed_frames_names[i]) cylinder radius")
        add_coordinate!(vms, ConstCoord(cylinder_position); id = "$(repulsed_frames_names[i]) cylinder position")

        #standard repulsive spring-damper pair
        add_coordinate!(vms, CoordDifference(frame, "$(repulsed_frames_names[i]) cylinder position") ; id = "$(repulsed_frames_names[i]) cylinder diff" )
        add_coordinate!(vms, CoordSlice("$(repulsed_frames_names[i]) cylinder diff", SVector(2,3)); id="$(repulsed_frames_names[i]) planar error")
        add_coordinate!(vms, CoordNorm("$(repulsed_frames_names[i]) planar error") ; id = "$(repulsed_frames_names[i]) planar error norm")
        add_coordinate!(vms, CoordDifference("$(repulsed_frames_names[i]) planar error norm", "$(repulsed_frames_names[i]) cylinder radius"); id = "shifted $(repulsed_frames_names[i]) cylinder error" )
    
        add_component!(vms, ReLUSpring(5.0, "shifted $(repulsed_frames_names[i]) cylinder error", true); id="$(repulsed_frames_names[i]) cylinder repulsive spring")
        add_component!(vms, RectifiedDamper(5.0, "$(repulsed_frames_names[i]) planar error norm", (0.0, 1.05*cylinder_radius), true, false); id="$(repulsed_frames_names[i]) cylinder damper")
    end

    println("Virtual Mechanism Built !")

    print("Linking real robot and virtual robot ...")

    # feedback_stiffness = 0.1
    # feedback_damping = 0.000
    mismatch_deadzone = 0.05

    # START BY LINKING UNCOUPLED JOINTS
    joint_limits = shadow_cfg.joint_limits
    uncoupled_joints = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ3", "rh_MFJ4", "rh_RFJ3", "rh_RFJ4", 
                        "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4","rh_THJ5"]


    for joint_id in uncoupled_joints
        add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord");id="$(joint_id) coord diff")
        # use deadzone springs instead of linear springs to take the natural mismatch into account
        add_deadzone_springs!(vms, feedback_stiffness, (-mismatch_deadzone, mismatch_deadzone), "$(joint_id) coord diff") 

        # implement "deadzone" damping using two rectified dampers (transition is linear, not sharp)
        add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (0.9*mismatch_deadzone, 1.1*mismatch_deadzone), false, false); id = "$(joint_id) feedback damper 1")
        add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (-1.1*mismatch_deadzone, -0.9*mismatch_deadzone), true, false); id = "$(joint_id) feedback damper 2")
    end

    # LINKING COUPLED JOINTS

    coupled_joints = ["rh_FFJ0", "rh_MFJ0", "rh_RFJ0" ,"rh_LFJ0"]

    for joint_id in coupled_joints
        add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord");id="$(joint_id) coord diff")
        add_deadzone_springs!(vms, feedback_stiffness, (-2*mismatch_deadzone, 2*mismatch_deadzone), "$(joint_id) coord diff")
        add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (1.8*mismatch_deadzone, 2.2*mismatch_deadzone), false, false); id = "$(joint_id) feedback damper 1")
        add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (-2.2*mismatch_deadzone, -1.8*mismatch_deadzone), true, false); id = "$(joint_id) feedback damper 2")
    end

    println("Linked !")


    function f_setup(cache)

        radius_joints                = Dict{String, Any}() # joint IDs of the rigid joints controlling the attracting cylinder radius for each finger
        root_joints                  = Dict{String, Any}() # joint IDs of the root joints controlling the attracting cylinder position for each finger
        attraction_coordID           = Dict{String, Any}() # coord IDs of the attraction spring between each attach point and the corresponding cylinder center (used to detect virtual contact)
        cylinder_radius_coord_dict   = Dict{String, Any}() # coord IDs of the spring rest length controlling the repulsive cylinder radius f
        cylinder_position_coord_dict = Dict{String, Any}() # coord IDs of the fixed point controlling the repulsive cylinder position
        damper_component_dict        = Dict{String, Any}() # component ID of the damper of the repulsive cylinder (cannot be modulated with coord)
        feedback_coordID_uncoupled   = Dict{String, Any}() # coord IDs of the feedback springs for uncoupled joints (used to detect real contact)
        feedback_coordID_coupled     = Dict{String, Any}() # coord IDs of the feedback springs for coupled joints (used to detect real contact, higher deadzone)

        for (finger, cfg) in FINGER_CONFIGS

            for frame in cfg.attracted_frames_names
                radius_joints[frame]      = get_compiled_jointID(cache, ".virtual_mechanism.fixed_joint_$(frame)")
                root_joints[frame]        = get_compiled_jointID(cache, ".virtual_mechanism.root_joint_$(frame)")
                attraction_coordID[frame] = get_compiled_coordID(cache, "ee $(frame) diff")
            end

            for frame in cfg.repulsed_frames_names
                cylinder_radius_coord_dict[frame]   = get_compiled_coordID(cache, "$(frame) cylinder radius")
                cylinder_position_coord_dict[frame] = get_compiled_coordID(cache, "$(frame) cylinder position")
                damper_component_dict[frame]        = get_compiled_componentID(cache, "$(frame) cylinder damper")
            end

            for joint in cfg.uncoupled_joints
                feedback_coordID_uncoupled[joint] = get_compiled_coordID(cache, "$(joint) coord diff")
            end

            for joint in cfg.coupled_joints
                feedback_coordID_coupled[joint] = get_compiled_coordID(cache, "$(joint) coord diff")
            end
        end

        return (radius_joints, root_joints,
                cylinder_radius_coord_dict, cylinder_position_coord_dict, damper_component_dict,
                feedback_coordID_uncoupled, feedback_coordID_coupled, attraction_coordID)
    end


    finger_states = Dict(name => FingerModulationState(cylinder_radius) for name in keys(FINGER_CONFIGS))

    last_t = 0.0

    # function f_control(cache, t, args, extra)

    #     radius_joints, root_joints, cylinder_radius_coord_dict, cylinder_position_coord_dict, virtual_object_damper_component_dict, feedback_coordID_uncoupled, feedback_coordID_coupled, attraction_coordID = args

    #     # ── 1. VIRTUAL CONTACT: any attach point has reached virtual object ────────
    #     finger_states["ff"].equilibrium = all(FINGER_CONFIGS["ff"].attracted_frames_names) do point
    #         norm(configuration(cache, attraction_coordID[point])) < 0.002
    #     end

    #     # ── 2. REAL CONTACT: mismatch exceeds deadzone on any relevant joint ───────
    #     uncoupled_contact = any(FINGER_CONFIGS["ff"].uncoupled_joints) do joint
    #         abs(only(configuration(cache, feedback_coordID_uncoupled[joint]))) > mismatch_deadzone
    #     end

    #     coupled_contact = any(FINGER_CONFIGS["ff"].coupled_joints) do joint
    #         abs(only(configuration(cache, feedback_coordID_coupled[joint]))) > 2 * mismatch_deadzone
    #     end

    #     finger_states["ff"].contact_detected = uncoupled_contact || coupled_contact

    #     # ── 3. ACTIVATION: sustained virtual contact without real contact ──────────
    #     if !finger_states["ff"].modulation_activated && !finger_states["ff"].modulation_stopped
    #         if finger_states["ff"].equilibrium && !finger_states["ff"].contact_detected
    #             if finger_states["ff"].activation_time == 0.0
    #                 finger_states["ff"].activation_time = t          # start timing
    #             elseif t - finger_states["ff"].activation_time > 0.2 # sustained 0.2s
    #                 finger_states["ff"].modulation_activated = true
    #                 @info "ff radius modulation activated"
    #             end
    #         else
    #             finger_states["ff"].activation_time = 0.0            # reset if condition lost
    #         end
    #     end

    #     # ── 4. RADIUS MODULATION ───────────────────────────────────────────────────
    #     if finger_states["ff"].modulation_activated && !finger_states["ff"].modulation_stopped

    #         # Decrement radius at each control step
    #         finger_states["ff"].radius = max(finger_states["ff"].radius - 0.002 * (t - last_t), 0.005)    # rate: 2 mm/s, floor: 5mm
    #         @info "ff current radius: $(round(finger_states["ff"].radius*1000, digits=2)) mm"

    #         update_cylinder_radius("ff", cache, finger_states["ff"].radius, radius_joints, cylinder_radius_coord_dict, virtual_object_damper_component_dict)
    #         update_cylinder_position("ff", m, cache, kcache, finger_states["ff"].radius, root_joints, cylinder_position_coord_dict)

    #         # ── 5. STOPPING: sustained real contact detected ───────────────────────
    #         if finger_states["ff"].contact_detected
    #             if finger_states["ff"].stopping_time == 0.0
    #                 finger_states["ff"].stopping_time = t                      # start stop timer
    #             elseif t - finger_states["ff"].stopping_time > 0.2            # sustained 0.2s
    #                 finger_states["ff"].modulation_activated = false
    #                 finger_states["ff"].modulation_stopped = true               # lock: do not re-activate
    #                 @info "ff radius modulation stopped at r = $(round(finger_states["ff"].radius*1000, digits=1)) mm"
    #             end
    #         else
    #             finger_states["ff"].stopping_time = 0.0                        # reset if contact lost
    #         end

    #     end

    #     last_t = t

    # end

    function f_control(cache, t, args, extra)

        (radius_joints, root_joints,
        cylinder_radius_coord_dict, cylinder_position_coord_dict, damper_component_dict,
        feedback_coordID_uncoupled, feedback_coordID_coupled, attraction_coordID) = args

        for (finger, cfg) in FINGER_CONFIGS
            state = finger_states[finger]

            update_finger_state!(state, finger, cache, t,
                                attraction_coordID,
                                feedback_coordID_uncoupled,
                                feedback_coordID_coupled)

            apply_radius_modulation!(finger, state, cache, t, m, kcache,
                                    radius_joints, root_joints,
                                    cylinder_radius_coord_dict,
                                    cylinder_position_coord_dict,
                                    damper_component_dict, last_t)
        end

        last_t = t
    end

    


    # Compile the virtual mechanism system, and run the controller via ROS
    # Make sure rospy_client.py is running first.
    println("Connecting to ROS client...")
    cvms = compile(vms)
    qᵛ = medium_wrap_preshape
    # qᵛ = zeros(24)

    joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                    "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                    "rh_THJ3", "rh_THJ4", "rh_THJ5"]


    with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; f_control, f_setup, E_max=10.0)
    end

end

function update_cylinder_position(finger, m, cache, kcache, new_radius, root_jointID, cylinder_position_coord_dict)
    medium_wrap_preshape = zeros(24)
    medium_wrap_preshape[21] = 1.2 # thumb extended
    kinematics!(kcache, 0.0, medium_wrap_preshape)

    if new_radius < 0.015
        # add one centimeter to the radius to avoid intersection with the fingers 
        rh_ffknuckle_frame_id = get_compiled_frameID(m, "rh_ffknuckle")
        ffknuckle_transform = get_transform(kcache, rh_ffknuckle_frame_id)
    
        cylinder_pos = SVector(0.0, -0.03, ffknuckle_transform.origin[3] - new_radius - 0.007)
    else
        # Get the positions of the finger tips
        rh_fftip_frame_id = get_compiled_frameID(m, "rh_fftip")
        fftip_transform = get_transform(kcache, rh_fftip_frame_id)
        p11 = [fftip_transform.origin[2], fftip_transform.origin[3]]  

        rh_ffmiddle_frame_id = get_compiled_frameID(m, "rh_ffmiddle")
        ffmiddle_transform = get_transform(kcache, rh_ffmiddle_frame_id)
        p12 = [ffmiddle_transform.origin[2], ffmiddle_transform.origin[3]]

        rh_thtip_frame_id = get_compiled_frameID(m, "rh_thtip")
        thtip_transform = get_transform(kcache, rh_thtip_frame_id)
        p21 = [thtip_transform.origin[2], thtip_transform.origin[3]]

        rh_thmiddle_frame_id = get_compiled_frameID(m, "rh_thmiddle")
        thmiddle_transform = get_transform(kcache, rh_thmiddle_frame_id)
        p22 = [thmiddle_transform.origin[2], thmiddle_transform.origin[3]]

        # add one centimeter to the radius to avoid intersection with the fingers
        cylinder_pos = circle_center_tangent_to_lines(p11, p12, p21, p22, new_radius + 0.01)
        cylinder_pos = SVector(0.0, cylinder_pos[1], cylinder_pos[2])  # Convert to SVector
    end


    for i in 1:length(FINGER_CONFIGS[finger].attracted_frames)
        frame_pos = configuration(kcache, get_compiled_coordID(kcache, FINGER_CONFIGS[finger].attracted_frames[i]))
        cache[root_jointID[FINGER_CONFIGS[finger].attracted_frames_names[i]]] = remake(
            cache[root_jointID[FINGER_CONFIGS[finger].attracted_frames_names[i]]];
            jointData = Rigid(Transform(SVector(frame_pos[1], cylinder_pos[2], cylinder_pos[3])))
        )
    end

    for frame in FINGER_CONFIGS[finger].repulsed_frames_names
        cache[cylinder_position_coord_dict[frame]] = remake(
            cache[cylinder_position_coord_dict[frame]];
            coord_data = ConstCoord(cylinder_pos)
        )
    end
end

function update_cylinder_radius(finger, cache, new_radius, radius_joints, cylinder_radius_coord_dict, virtual_object_damper_component_dict)

    for frame in FINGER_CONFIGS[finger].attracted_frames_names 
        cache[radius_joints[frame]] = remake(cache[radius_joints[frame]];
            jointData = Rigid(Transform(SVector(0.0, 0.0, new_radius))))
    end

    for frame in FINGER_CONFIGS[finger].repulsed_frames_names
        cache[cylinder_radius_coord_dict[frame]] = remake(cache[cylinder_radius_coord_dict[frame]];
            coord_data = ConstCoord(new_radius))
        cache[virtual_object_damper_component_dict[frame]] = remake(cache[virtual_object_damper_component_dict[frame]];
            bounds = (0.0, 1.05*new_radius))
    end
end

function update_finger_state!(state, finger, cache, t, attraction_coordID, feedback_coordID_uncoupled, feedback_coordID_coupled)

    cfg = FINGER_CONFIGS[finger]

    # ── 1. Virtual contact ────────────────────────────────────────────────────
    state.equilibrium = any(cfg.attracted_frames_names) do point
        norm(configuration(cache, attraction_coordID[point])) < 0.005
    end

    # ── 2. Real contact ───────────────────────────────────────────────────────
    uncoupled_contact = any(cfg.uncoupled_joints) do joint
        abs(only(configuration(cache, feedback_coordID_uncoupled[joint]))) > MISMATCH_DEADZONE
    end
    coupled_contact = any(cfg.coupled_joints) do joint
        abs(only(configuration(cache, feedback_coordID_coupled[joint]))) > 2 * MISMATCH_DEADZONE
    end
    state.contact_detected = uncoupled_contact || coupled_contact

    # ── 3. Activation: sustained virtual contact without real contact ─────────
    if !state.modulation_activated && !state.modulation_stopped
        if state.equilibrium && !state.contact_detected
            if state.activation_time == 0.0
                state.activation_time = t
            elseif t - state.activation_time > 0.2
                state.modulation_activated = true
                @info "Radius modulation activated for finger $(finger)"
            end
        else
            state.activation_time = 0.0
        end
    end
end

function apply_radius_modulation!(finger, state, cache, t, m, kcache,
                                   radius_joints, root_joints,
                                   cylinder_radius_coord_dict,
                                   cylinder_position_coord_dict,
                                   damper_component_dict, last_t)

    (!state.modulation_activated || state.modulation_stopped) && return

    # ── 4. Decrement radius ───────────────────────────────────────────────────
    dt = t - last_t
    state.radius = max(state.radius - 0.002 * dt, 0.005)

    update_cylinder_radius(finger, cache, state.radius, radius_joints, cylinder_radius_coord_dict, damper_component_dict)

    update_cylinder_position(finger, m, cache, kcache, state.radius,root_joints, cylinder_position_coord_dict)

    # ── 5. Stopping: sustained real contact ───────────────────────────────────
    if state.contact_detected
        if state.stopping_time == 0.0
            state.stopping_time = t
        elseif t - state.stopping_time > 0.2
            state.modulation_activated = false
            state.modulation_stopped   = true
            @info "$(finger) modulation stopped at r = $(round(state.radius*1000, digits=1)) mm"
        end
    else
        state.stopping_time = 0.0
    end
end

