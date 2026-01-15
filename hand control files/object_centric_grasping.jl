using Revise
using GeometryBasics: Vec3f, Point3f
using LinearAlgebra
using StaticArrays
using VMRobotControl
using VMRobotControl.Splines: CubicSpline
using DifferentialEquations
include("functions.jl")

module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
include(joinpath(module_path, "ros/ROS.jl"))

using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end

function object_centric_medium_wrap(cylinder_radius)

    module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])

    shadow_cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
    shadow_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), shadow_cfg)

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


    ##### COMPLEMENTING THE VIRTUAL ROBOT #####

    # Gravity Compensation and joint limits/damping
    

    joint_limits = vm_cfg.joint_limits

    for joint_id in keys(joints(vm_robot))
        limits = joint_limits[joint_id]
        isnothing(limits) && continue
        add_coordinate!(vm_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
        @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
        add_deadzone_springs!(vm_robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
        add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
    end

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
    "rh_thdistal_mass_coord", "rh_thmiddle_mass_coord")

    attracted_frames_names = ("lfdistal", "lfmiddle", "lfprox", "rfdistal", "rfmiddle", "rfprox", "mfdistal", "mfmiddle", "mfprox", "ffdistal", "ffmiddle", 
    "ffprox", "thdistal", "thmiddle")
    
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
    phalanx_scaling_factor = 0.5
    finger_scaling_factor = 1.5
    
    stiffnesses = generate_stiffnesses_geometric_scaling(base_stiffness, phalanx_scaling_factor, finger_scaling_factor)
    
    damping_decay_rate = 161 # 20% of damping at |z| = 0.01
    exponential_damping_coeff = 0.1
    exponential_damping_matrix = SMatrix{3, 3}(exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff)
    
    for i in 1:length(attracted_frames)
        K = SMatrix{3, 3}(stiffnesses[i], 0., 0., 0., stiffnesses[i], 0., 0., 0., stiffnesses[i])
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(attracted_frames_names[i]) ee position", ".virtual_mechanism.$(attracted_frames[i])"); id = "ee $(attracted_frames_names[i]) diff")
        add_component!(vms, LinearSpring(K, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) spring")
        add_component!(vms, LinearDamper(D, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) damper")
        add_component!(vms, ExponentialDamper(exponential_damping_matrix, "ee $(attracted_frames_names[i]) diff", damping_decay_rate); id = "ee $(attracted_frames_names[i]) exp damper")
    end

    add_component!(vms, LinearDamper(SMatrix{3, 3}(10.0, 0., 0., 0., 10.0, 0., 0., 0., 10.0), "ee thmiddle diff"); id = "thmiddle massive damper")
    
    #lightly constraint some joints to avoid unwanted motions 
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ1_coord"); id = "wr j1 angular spring")
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ2_coord"); id = "wr j2 angular spring")

    # CYLINDER COLLISION MODEL  

    add_coordinate!(vms,  ConstCoord(cylinder_position);  id="cylinder position")
    add_coordinate!(vms, ConstCoord(cylinder_radius); id="cylinder radius")
    
    add_coordinate!(vms, FramePoint(".virtual_mechanism.rh_palm", SVector(0. , 0., 0.07)); id="second palm point")
    
    repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_mftip_mass_coord", ".virtual_mechanism.rh_rftip_mass_coord",".virtual_mechanism.rh_lftip_mass_coord" , 
                        ".virtual_mechanism.rh_thtip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord",".virtual_mechanism.rh_mfmiddle_mass_coord", ".virtual_mechanism.rh_rfmiddle_mass_coord",
                        ".virtual_mechanism.rh_lfmiddle_mass_coord",  ".virtual_mechanism.rh_thmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord", ".virtual_mechanism.rh_mfproximal_mass_coord",
                        ".virtual_mechanism.rh_rfproximal_mass_coord", ".virtual_mechanism.rh_lfproximal_mass_coord", ".virtual_mechanism.rh_thproximal_mass_coord", ".virtual_mechanism.rh_palm_mass_coord", "second palm point",
                        ".virtual_mechanism.rh_ffdistal", ".virtual_mechanism.rh_mfdistal", ".virtual_mechanism.rh_rfdistal", ".virtual_mechanism.rh_lfdistal", ".virtual_mechanism.rh_thdistal", ".virtual_mechanism.rh_thmiddle")
    repulsed_frames_names = ("fftip", "mftip", "rftip", "lftip", "thtip", "ffmiddle", "mfmiddle", "rfmiddle", "lfmiddle", "thmiddle", "ffprox", 
                    "mfprox", "rfprox", "lfprox", "thprox", "palm", "palm2", "ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal", "thmiddle2")
    
    for i in 1:length(repulsed_frames)
        frame = repulsed_frames[i]
        add_coordinate!(vms, CoordDifference(frame, "cylinder position") ; id = "$(repulsed_frames_names[i]) cylinder diff" )
        add_coordinate!(vms, CoordSlice("$(repulsed_frames_names[i]) cylinder diff", SVector(2,3)); id="$(repulsed_frames_names[i]) planar error")
        add_coordinate!(vms, CoordNorm("$(repulsed_frames_names[i]) planar error") ; id = "$(repulsed_frames_names[i]) planar error norm")
        add_coordinate!(vms, CoordDifference("$(repulsed_frames_names[i]) planar error norm", "cylinder radius"); id = "shifted $(repulsed_frames_names[i]) cylinder error" )
    
        add_component!(vms, ReLUSpring(5.0, "shifted $(repulsed_frames_names[i]) cylinder error", true); id="$(repulsed_frames_names[i]) cylinder repulsive spring")
        add_component!(vms, RectifiedDamper(5.0, "$(repulsed_frames_names[i]) planar error norm", (0.0, 1.05*cylinder_radius), true, false); id="$(repulsed_frames_names[i]) cylinder damper")
    end



    println("Virtual controller built. Connecting to ROS client...")
    cvms = compile(vms)
    qᵛ = medium_wrap_preshape

    joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                    "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                    "rh_THJ3", "rh_THJ4", "rh_THJ5"]


    with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; E_max=10.0, steady_state_check=true)
    end

end

function object_centric_power_sphere(ball_radius)

    module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])

    shadow_cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
    shadow_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), shadow_cfg)



    vm_cfg = URDFParserConfig(;suppress_warnings=true) 
    # For the moment the urdfs are the same but we might want to change the properties of the virtual robot
    vm_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), vm_cfg) 

    add_coordinate!(vm_robot, FrameOrigin("rh_ffdistal"); id="rh_ffdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_mfdistal"); id="rh_mfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_rfdistal"); id="rh_rfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_lfdistal"); id="rh_lfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thdistal"); id="rh_thdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thmiddle"); id="rh_thmiddle")



    # Gravity Compensation and joint limits/damping
    

    joint_limits = vm_cfg.joint_limits

    for joint_id in keys(joints(vm_robot))
        limits = joint_limits[joint_id]
        isnothing(limits) && continue
        add_coordinate!(vm_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
        @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
        add_deadzone_springs!(vm_robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
        add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
    end

    # remove parametric positioning of the ball for strucured experiments
    # if ball_radius <= 0.025
    #     ball_position = SVector(0.0, -0.11, 0.33)
    # else
    #     ball_position = SVector(0.0, -ball_radius - 0.021, 0.33)
    # end
    ball_position = SVector(0.0, -ball_radius - 0.021, 0.33)

    attracted_frames = ("rh_lfdistal_mass_coord", "rh_lfmiddle_mass_coord", "rh_lfproximal_mass_coord", "rh_rfdistal_mass_coord", 
    "rh_rfmiddle_mass_coord", "rh_rfproximal_mass_coord", "rh_mfdistal_mass_coord", "rh_mfmiddle_mass_coord",
    "rh_mfproximal_mass_coord", "rh_ffdistal_mass_coord", "rh_ffmiddle_mass_coord", "rh_ffproximal_mass_coord",
    "rh_thdistal_mass_coord", "rh_thmiddle_mass_coord")
    
    attracted_frames_names = ("lfdistal", "lfmiddle", "lfprox", "rfdistal", "rfmiddle", "rfprox", "mfdistal", "mfmiddle", "mfprox", "ffdistal", "ffmiddle", 
    "ffprox", "thdistal", "thmiddle")


    for i in 1:length(attracted_frames)
        add_frame!(vm_robot; id = "revo_frame_1_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Revolute(SVector(0.0,0.0,1.0),Transform(SVector(ball_position[1], ball_position[2], ball_position[3]))); parent=root_frame(vm_robot), child="revo_frame_1_$(attracted_frames_names[i])", id="revo_joint_1_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="revo_frame_2_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Revolute(SVector(1.0,0.0,0.0)); parent="revo_frame_1_$(attracted_frames_names[i])", child="revo_frame_2_$(attracted_frames_names[i])", id="revo_joint_2_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="ee_frame_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Rigid(Transform(SVector(0.0,0.0,ball_radius))); parent ="revo_frame_2_$(attracted_frames_names[i])", child ="ee_frame_$(attracted_frames_names[i])", id = "fixed_joint_$(attracted_frames_names[i])")

        add_coordinate!(vm_robot, FrameOrigin("ee_frame_$(attracted_frames_names[i])"); id="$(attracted_frames_names[i]) ee position")
        add_component!(vm_robot, PointMass(0.01, "$(attracted_frames_names[i]) ee position"); id="$(attracted_frames_names[i]) ee mass")
        inertia = 0.001*(ball_radius^2)/5
        I_mat = @SMatrix [inertia  0.    0.  ;0.    inertia  0.  ;0.    0.    inertia]
        add_inertia!(vm_robot, "ee_frame_$(attracted_frames_names[i])", I_mat; id="$(attracted_frames_names[i]) ee inertia")

    end

    add_gravity_compensation!(vm_robot, VMRobotControl.DEFAULT_GRAVITY)

    
    vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)

    # HAND MOTION

    D = SMatrix{3, 3}(0.15, 0., 0., 0., 0.15, 0., 0., 0., 0.15)

    base_stiffness = 0.05
    phalanx_scaling_factor = 0.1
    finger_scaling_factor = 1.5

    stiffnesses = generate_stiffnesses_geometric_scaling(base_stiffness, phalanx_scaling_factor, finger_scaling_factor)

    damping_decay_rate = 161 # 20% of damping at |z| = 0.01
    exponential_damping_coeff = 0.2
    exponential_damping_matrix = SMatrix{3, 3}(exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff)
    # hand converging to the ball 

    for i in 1:length(attracted_frames)
        K = SMatrix{3, 3}(stiffnesses[i], 0., 0., 0., stiffnesses[i], 0., 0., 0., stiffnesses[i])
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(attracted_frames_names[i]) ee position", ".virtual_mechanism.$(attracted_frames[i])"); id = "ee $(attracted_frames_names[i]) diff")
        add_component!(vms, LinearSpring(K, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) spring")
        add_component!(vms, LinearDamper(D, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) damper")
        add_component!(vms, ExponentialDamper(exponential_damping_matrix, "ee $(attracted_frames_names[i]) diff", damping_decay_rate); id = "ee $(attracted_frames_names[i]) exp damper")
    end

    add_component!(vms, LinearDamper(SMatrix{3, 3}(5.0, 0., 0., 0., 5.0, 0., 0., 0., 5.0), "ee thmiddle diff"); id = "thmiddle massive damper")

    # fingers spacing : Joint level

    add_coordinate!(vms, ConstCoord(0.6); id = "angular spring length")

    #ff mf spacing
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_MFJ4_coord", ".virtual_mechanism.rh_FFJ4_coord"); id="ff mf j4 angular diff")
    add_coordinate!(vms, CoordDifference("ff mf j4 angular diff", "angular spring length") ; id="ff mf j4 angular error")
    add_component!(vms, LinearSpring(0.001, "ff mf j4 angular error"); id="ff mf angular spring")
    add_component!(vms, LinearDamper(0.001, "ff mf j4 angular error"); id="ff mf angular damper")

    #mf rf spacing
    add_coordinate!(vms, CoordSum(".virtual_mechanism.rh_RFJ4_coord", ".virtual_mechanism.rh_MFJ4_coord"); id="mf rf j4 angular diff")
    add_coordinate!(vms, CoordSum("mf rf j4 angular diff", "angular spring length") ; id="mf rf j4 angular error")
    add_component!(vms, LinearSpring(0.001, "mf rf j4 angular error"); id="mf rf angular spring")
    add_component!(vms, LinearDamper(0.001, "mf rf j4 angular error"); id="mf rf angular damper")

    #rf lf spacing
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_RFJ4_coord", ".virtual_mechanism.rh_LFJ4_coord"); id="rf lf j4 angular diff")
    add_coordinate!(vms, CoordDifference("rf lf j4 angular diff", "angular spring length") ; id="rf lf j4 angular error")
    add_component!(vms, LinearSpring(0.001, "rf lf j4 angular error"); id="rf lf angular spring")
    add_component!(vms, LinearDamper(0.001, "rf lf j4 angular error"); id="rf lf angular damper")

    #th spacing
    add_coordinate!(vms, ConstCoord(1.22); id="th spring length")
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_THJ4_coord", "th spring length"); id="th j4 error")
    add_component!(vms, LinearSpring(0.001, "th j4 error"); id="th j4 spring")

    # BALL COLLISION MODEL  

    add_coordinate!(vms,  ConstCoord(ball_position);  id="ball position")
    add_coordinate!(vms, ConstCoord(ball_radius); id="ball radius")

    add_coordinate!(vms, FramePoint(".virtual_mechanism.rh_palm", SVector(0. , 0., 0.07)); id="second palm point")

    repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_mftip_mass_coord", ".virtual_mechanism.rh_rftip_mass_coord",".virtual_mechanism.rh_lftip_mass_coord" , 
                        ".virtual_mechanism.rh_thtip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord",".virtual_mechanism.rh_mfmiddle_mass_coord", ".virtual_mechanism.rh_rfmiddle_mass_coord",
                        ".virtual_mechanism.rh_lfmiddle_mass_coord",  ".virtual_mechanism.rh_thmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord", ".virtual_mechanism.rh_mfproximal_mass_coord",
                        ".virtual_mechanism.rh_rfproximal_mass_coord", ".virtual_mechanism.rh_lfproximal_mass_coord", ".virtual_mechanism.rh_thproximal_mass_coord", ".virtual_mechanism.rh_palm_mass_coord", "second palm point",
                        ".virtual_mechanism.rh_ffdistal", ".virtual_mechanism.rh_mfdistal", ".virtual_mechanism.rh_rfdistal", ".virtual_mechanism.rh_lfdistal", ".virtual_mechanism.rh_thdistal",
                        ".virtual_mechanism.rh_thmiddle")
    repulsed_frames_names = ("fftip", "mftip", "rftip", "lftip", "thtip", "ffmiddle", "mfmiddle", "rfmiddle", "lfmiddle", "thmiddle", "ffprox", 
                    "mfprox", "rfprox", "lfprox", "thprox", "palm", "palm2", "ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal", "thmiddle2")

    for i in 1:length(repulsed_frames)
        frame = repulsed_frames[i]
        add_coordinate!(vms, CoordDifference(frame, "ball position") ; id = "$(repulsed_frames_names[i]) ball error" )
        add_coordinate!(vms, CoordNorm("$(repulsed_frames_names[i]) ball error") ; id = "$(repulsed_frames_names[i]) ball error norm")
        add_coordinate!(vms, CoordDifference("$(repulsed_frames_names[i]) ball error norm", "ball radius"); id = "shifted $(repulsed_frames_names[i]) ball error" )

        add_component!(vms, ReLUSpring(5.0, "shifted $(repulsed_frames_names[i]) ball error", true); id="$(repulsed_frames_names[i]) ball repulsive spring")
        add_component!(vms, RectifiedDamper(5.0, "$(repulsed_frames_names[i]) ball error norm", (0.0, 1.1*ball_radius), true, false); id="$(repulsed_frames_names[i]) ball damper")
    end

    println("Virtual controller built. Connecting to ROS client...")

    cvms = compile(vms)
    qᵛ = zero_q(cvms.virtual_mechanism)
    qᵛ[21] = 1.2
    qᵛ[3] = -0.35
    qᵛ[7] = -0.12
    qᵛ[11] = -0.12
    qᵛ[16] = -0.35

    joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                    "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                    "rh_THJ3", "rh_THJ4", "rh_THJ5"]


    with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; E_max=10.0, steady_state_check=true)
    end

end

function object_centric_power_sphere_test(ball_radius)
        module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])

    shadow_cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
    shadow_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), shadow_cfg)



    vm_cfg = URDFParserConfig(;suppress_warnings=true) 
    # For the moment the urdfs are the same but we might want to change the properties of the virtual robot
    vm_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), vm_cfg) 

    add_coordinate!(vm_robot, FrameOrigin("rh_ffdistal"); id="rh_ffdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_mfdistal"); id="rh_mfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_rfdistal"); id="rh_rfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_lfdistal"); id="rh_lfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thdistal"); id="rh_thdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thmiddle"); id="rh_thmiddle")



    # Gravity Compensation and joint limits/damping
    

    joint_limits = vm_cfg.joint_limits

    for joint_id in keys(joints(vm_robot))
        limits = joint_limits[joint_id]
        isnothing(limits) && continue
        add_coordinate!(vm_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
        @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
        add_deadzone_springs!(vm_robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
        add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
    end

    # remove parametric positioning of the ball for strucured experiments
    # if ball_radius <= 0.025
    #     ball_position = SVector(0.0, -0.11, 0.33)
    # else
    #     ball_position = SVector(0.0, -ball_radius - 0.021, 0.33)
    # end
    ball_position = SVector(0.0, -ball_radius - 0.021, 0.33)

    attracted_frames = ("rh_lfdistal_mass_coord", "rh_lfmiddle_mass_coord", "rh_lfproximal_mass_coord", "rh_rfdistal_mass_coord", 
    "rh_rfmiddle_mass_coord", "rh_rfproximal_mass_coord", "rh_mfdistal_mass_coord", "rh_mfmiddle_mass_coord",
    "rh_mfproximal_mass_coord", "rh_ffdistal_mass_coord", "rh_ffmiddle_mass_coord", "rh_ffproximal_mass_coord",
    "rh_thdistal_mass_coord", "rh_thmiddle_mass_coord")
    
    attracted_frames_names = ("lfdistal", "lfmiddle", "lfprox", "rfdistal", "rfmiddle", "rfprox", "mfdistal", "mfmiddle", "mfprox", "ffdistal", "ffmiddle", 
    "ffprox", "thdistal", "thmiddle")


    for i in 1:length(attracted_frames)
        add_frame!(vm_robot; id = "revo_frame_1_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Revolute(SVector(0.0,0.0,1.0),Transform(SVector(ball_position[1], ball_position[2], ball_position[3]))); parent=root_frame(vm_robot), child="revo_frame_1_$(attracted_frames_names[i])", id="revo_joint_1_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="revo_frame_2_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Revolute(SVector(1.0,0.0,0.0)); parent="revo_frame_1_$(attracted_frames_names[i])", child="revo_frame_2_$(attracted_frames_names[i])", id="revo_joint_2_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="ee_frame_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Rigid(Transform(SVector(0.0,0.0,ball_radius))); parent ="revo_frame_2_$(attracted_frames_names[i])", child ="ee_frame_$(attracted_frames_names[i])", id = "fixed_joint_$(attracted_frames_names[i])")

        add_coordinate!(vm_robot, FrameOrigin("ee_frame_$(attracted_frames_names[i])"); id="$(attracted_frames_names[i]) ee position")
        add_component!(vm_robot, PointMass(0.01, "$(attracted_frames_names[i]) ee position"); id="$(attracted_frames_names[i]) ee mass")
        inertia = 0.001*(ball_radius^2)/5
        I_mat = @SMatrix [inertia  0.    0.  ;0.    inertia  0.  ;0.    0.    inertia]
        add_inertia!(vm_robot, "ee_frame_$(attracted_frames_names[i])", I_mat; id="$(attracted_frames_names[i]) ee inertia")

    end

    add_gravity_compensation!(vm_robot, VMRobotControl.DEFAULT_GRAVITY)

    
    vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)

    # HAND MOTION

    D = SMatrix{3, 3}(0.15, 0., 0., 0., 0.15, 0., 0., 0., 0.15)

    base_stiffness = 0.05
    phalanx_scaling_factor = 0.1
    finger_scaling_factor = 1.5

    stiffnesses = generate_stiffnesses_geometric_scaling(base_stiffness, phalanx_scaling_factor, finger_scaling_factor)

    damping_decay_rate = 161 # 20% of damping at |z| = 0.01
    exponential_damping_coeff = 0.2
    exponential_damping_matrix = SMatrix{3, 3}(exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff)
    # hand converging to the ball 

    for i in 1:length(attracted_frames)
        K = SMatrix{3, 3}(stiffnesses[i], 0., 0., 0., stiffnesses[i], 0., 0., 0., stiffnesses[i])
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(attracted_frames_names[i]) ee position", ".virtual_mechanism.$(attracted_frames[i])"); id = "ee $(attracted_frames_names[i]) diff")
        add_component!(vms, LinearSpring(K, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) spring")
        add_component!(vms, LinearDamper(D, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) damper")
        add_component!(vms, ExponentialDamper(exponential_damping_matrix, "ee $(attracted_frames_names[i]) diff", damping_decay_rate); id = "ee $(attracted_frames_names[i]) exp damper")
    end

    if ball_radius <= 0.033
        thumb_massive_damping = 0.7
        add_component!(vms, LinearDamper(SMatrix{3, 3}(thumb_massive_damping, 0., 0., 0., thumb_massive_damping, 0., 0., 0., thumb_massive_damping), "ee thmiddle diff"); id = "thmiddle massive damper")
        thumb_tip_damping = 0.15
        add_component!(vms, LinearDamper(SMatrix{3, 3}(thumb_tip_damping, 0., 0., 0., thumb_tip_damping, 0., 0., 0., thumb_tip_damping), "ee thdistal diff"); id = "thdistal massive damper")
    else 
        thumb_massive_damping = 1.0
        add_component!(vms, LinearDamper(SMatrix{3, 3}(thumb_massive_damping, 0., 0., 0., thumb_massive_damping, 0., 0., 0., thumb_massive_damping), "ee thmiddle diff"); id = "thmiddle massive damper")
    end
    
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ1_coord"); id = "wrj1 spring")
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ2_coord"); id = "wrj2 spring")

    # fingers spacing : Joint level

    add_coordinate!(vms, ConstCoord(0.6); id = "angular spring length")

    #ff mf spacing
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_MFJ4_coord", ".virtual_mechanism.rh_FFJ4_coord"); id="ff mf j4 angular diff")
    add_coordinate!(vms, CoordDifference("ff mf j4 angular diff", "angular spring length") ; id="ff mf j4 angular error")
    add_component!(vms, LinearSpring(0.001, "ff mf j4 angular error"); id="ff mf angular spring")
    add_component!(vms, LinearDamper(0.001, "ff mf j4 angular error"); id="ff mf angular damper")

    #mf rf spacing
    add_coordinate!(vms, CoordSum(".virtual_mechanism.rh_RFJ4_coord", ".virtual_mechanism.rh_MFJ4_coord"); id="mf rf j4 angular diff")
    add_coordinate!(vms, CoordSum("mf rf j4 angular diff", "angular spring length") ; id="mf rf j4 angular error")
    add_component!(vms, LinearSpring(0.001, "mf rf j4 angular error"); id="mf rf angular spring")
    add_component!(vms, LinearDamper(0.001, "mf rf j4 angular error"); id="mf rf angular damper")

    #rf lf spacing
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_RFJ4_coord", ".virtual_mechanism.rh_LFJ4_coord"); id="rf lf j4 angular diff")
    add_coordinate!(vms, CoordDifference("rf lf j4 angular diff", "angular spring length") ; id="rf lf j4 angular error")
    add_component!(vms, LinearSpring(0.001, "rf lf j4 angular error"); id="rf lf angular spring")
    add_component!(vms, LinearDamper(0.001, "rf lf j4 angular error"); id="rf lf angular damper")

    #th spacing
    add_coordinate!(vms, ConstCoord(1.22); id="th spring length")
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.rh_THJ4_coord", "th spring length"); id="th j4 error")
    add_component!(vms, LinearSpring(0.001, "th j4 error"); id="th j4 spring")

    # BALL COLLISION MODEL  

    add_coordinate!(vms,  ConstCoord(ball_position);  id="ball position")
    add_coordinate!(vms, ConstCoord(ball_radius); id="ball radius")

    add_coordinate!(vms, FramePoint(".virtual_mechanism.rh_palm", SVector(0. , 0., 0.07)); id="second palm point")

    repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_mftip_mass_coord", ".virtual_mechanism.rh_rftip_mass_coord",".virtual_mechanism.rh_lftip_mass_coord" , 
                        ".virtual_mechanism.rh_thtip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord",".virtual_mechanism.rh_mfmiddle_mass_coord", ".virtual_mechanism.rh_rfmiddle_mass_coord",
                        ".virtual_mechanism.rh_lfmiddle_mass_coord",  ".virtual_mechanism.rh_thmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord", ".virtual_mechanism.rh_mfproximal_mass_coord",
                        ".virtual_mechanism.rh_rfproximal_mass_coord", ".virtual_mechanism.rh_lfproximal_mass_coord", ".virtual_mechanism.rh_thproximal_mass_coord", ".virtual_mechanism.rh_palm_mass_coord", "second palm point",
                        ".virtual_mechanism.rh_ffdistal", ".virtual_mechanism.rh_mfdistal", ".virtual_mechanism.rh_rfdistal", ".virtual_mechanism.rh_lfdistal", ".virtual_mechanism.rh_thdistal",
                        ".virtual_mechanism.rh_thmiddle")
    repulsed_frames_names = ("fftip", "mftip", "rftip", "lftip", "thtip", "ffmiddle", "mfmiddle", "rfmiddle", "lfmiddle", "thmiddle", "ffprox", 
                    "mfprox", "rfprox", "lfprox", "thprox", "palm", "palm2", "ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal", "thmiddle2")

    for i in 1:length(repulsed_frames)
        frame = repulsed_frames[i]
        add_coordinate!(vms, CoordDifference(frame, "ball position") ; id = "$(repulsed_frames_names[i]) ball error" )
        add_coordinate!(vms, CoordNorm("$(repulsed_frames_names[i]) ball error") ; id = "$(repulsed_frames_names[i]) ball error norm")
        add_coordinate!(vms, CoordDifference("$(repulsed_frames_names[i]) ball error norm", "ball radius"); id = "shifted $(repulsed_frames_names[i]) ball error" )

        add_component!(vms, ReLUSpring(5.0, "shifted $(repulsed_frames_names[i]) ball error", true); id="$(repulsed_frames_names[i]) ball repulsive spring")
        add_component!(vms, RectifiedDamper(5.0, "$(repulsed_frames_names[i]) ball error norm", (0.0, 1.1*ball_radius), true, false); id="$(repulsed_frames_names[i]) ball damper")
    end

    # Thumb-index repulsion through shadow coordinate
    add_coordinate!(vms, ShadowCoord(".virtual_mechanism.rh_ffdistal"); id="shadow coord ffdistal")
    add_coordinate!(vms, CoordDifference("shadow coord ffdistal", ".virtual_mechanism.rh_thdistal"); id="thumb repulsive diff")
    add_coordinate!(vms, CoordNorm("thumb repulsive diff"); id="thumb repulsive diff norm")

    add_component!(vms, GaussianSpring("thumb repulsive diff norm"; stiffness = -1.0, width = 0.01); id="thumb repulsive gaussian spring")

    println("Virtual controller built. Connecting to ROS client...")

    cvms = compile(vms)
    qᵛ = zero_q(cvms.virtual_mechanism)
    qᵛ[21] = 1.2
    qᵛ[3] = -0.35
    qᵛ[7] = -0.12
    qᵛ[11] = -0.12
    qᵛ[16] = -0.35

    joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                    "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                    "rh_THJ3", "rh_THJ4", "rh_THJ5"]


    with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; E_max=10.0, steady_state_check=true)
    end
end

function object_centric_lateral_pinch(box_width, box_thickness)

    module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])

    shadow_cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
    shadow_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), shadow_cfg)

    vm_cfg = URDFParserConfig(;suppress_warnings=true) 
    # For the moment the urdfs are the same but we might want to change the properties of the virtual robot
    vm_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), vm_cfg) 

    add_coordinate!(vm_robot, FrameOrigin("rh_fftip"); id = "rh_fftip")
    add_coordinate!(vm_robot, FrameOrigin("rh_ffdistal"); id="rh_ffdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_ffmiddle"); id="rh_ffmiddle")
    add_coordinate!(vm_robot, FrameOrigin("rh_ffproximal"); id="rh_ffproximal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thtip"); id = "rh_thtip")
    add_coordinate!(vm_robot, FrameOrigin("rh_thdistal"); id="rh_thdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thmiddle"); id="rh_thmiddle")

    ##### COMPLEMENTING THE VIRTUAL ROBOT #####

    # Gravity Compensation and joint limits/damping
    

    joint_limits = vm_cfg.joint_limits

    for joint_id in keys(joints(vm_robot))
        limits = joint_limits[joint_id]
        isnothing(limits) && continue
        add_coordinate!(vm_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
        @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
        add_deadzone_springs!(vm_robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
        add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
    end

    box_dimensions = [box_thickness, box_width, 0.05]
    box_position = SVector(0.042 + box_dimensions[1], -0.03, 0.32+box_dimensions[3])
    
    attracted_frames = ("rh_fftip", "rh_ffdistal", "rh_ffmiddle","rh_ffproximal", "rh_thtip", "rh_thdistal")
    attracted_frames_names = ("fftip", "ffdistal", "ffmiddle", "ffprox", "thtip", "thdistal")
    orientation = (-1.0, -1.0, -1.0, -1.0, 1.0, 1.0)

    # BOX PRISMATIC JOINTS
    
    for i in 1:length(attracted_frames)
        add_frame!(vm_robot; id = "base_frame_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Rigid(Transform(box_position)); parent=root_frame(vm_robot), child="base_frame_$(attracted_frames_names[i])", id="base_joint_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id = "prism_frame_1_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Prismatic(SVector(0.0,1.0,0.0)); parent="base_frame_$(attracted_frames_names[i])", child="prism_frame_1_$(attracted_frames_names[i])", id="prism_joint_1_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="prism_frame_2_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Prismatic(SVector(0.0,0.0,1.0)); parent="prism_frame_1_$(attracted_frames_names[i])", child="prism_frame_2_$(attracted_frames_names[i])", id="prism_joint_2_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="ee_frame_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Rigid(Transform(SVector(orientation[i]*box_dimensions[1],0.0,0.0))); parent ="prism_frame_2_$(attracted_frames_names[i])", child ="ee_frame_$(attracted_frames_names[i])", id = "rigid_joint_$(attracted_frames_names[i])")
    
        add_coordinate!(vm_robot, FrameOrigin("ee_frame_$(attracted_frames_names[i])"); id="$(attracted_frames_names[i]) ee position")
        add_component!(vm_robot, PointMass(0.01, "$(attracted_frames_names[i]) ee position"); id="$(attracted_frames_names[i]) ee mass")

    
        joint_damping = 0.05
        add_coordinate!(vm_robot, JointSubspace("prism_joint_1_$(attracted_frames_names[i])"); id="prism_joint_1_$(attracted_frames_names[i])")
        add_component!(vm_robot, LinearDamper(joint_damping, "prism_joint_1_$(attracted_frames_names[i])"); id="prism_joint_1_$(attracted_frames_names[i])_damper")
        add_coordinate!(vm_robot, JointSubspace("prism_joint_2_$(attracted_frames_names[i])"); id="prism_joint_2_$(attracted_frames_names[i])")
        add_component!(vm_robot, LinearDamper(joint_damping, "prism_joint_2_$(attracted_frames_names[i])"); id="prism_joint_2_$(attracted_frames_names[i])_damper")   
    
        # DEADZONE SPRINGS : CONSTRAINT THE MOTION INSIDE THE BOX

        margin = 0.015
        deadzone_stiffness = 5.0
        add_deadzone_springs!(vm_robot, deadzone_stiffness, (-box_dimensions[2] + margin, box_dimensions[2] - margin), "prism_joint_1_$(attracted_frames_names[i])")
        add_deadzone_springs!(vm_robot, deadzone_stiffness, (-box_dimensions[3] + margin, box_dimensions[3] - margin), "prism_joint_2_$(attracted_frames_names[i])")
    end

    add_gravity_compensation!(vm_robot, VMRobotControl.DEFAULT_GRAVITY)


    vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)

    # HAND MOTION

    base_damping = 0.05
    D = SMatrix{3, 3}(base_damping , 0., 0., 0., base_damping, 0., 0., 0., base_damping)
    x_stiffnesses = [0.01, 0.01, 0.01, 0.01, 0.1, 0.1]
    yz_stiffness = 0.1

    damping_decay_rate = 460 # 20% of damping at |z| = 0.005
    exponential_damping_coeff = 0.1
    exponential_damping_matrix = SMatrix{3, 3}(exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff, 0., 0., 0., exponential_damping_coeff)

    # Establishing contact with the box 

    for i in 1:length(attracted_frames)
        K = SMatrix{3, 3}(x_stiffnesses[i], 0., 0., 0., yz_stiffness, 0., 0., 0., yz_stiffness)
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(attracted_frames_names[i]) ee position", ".virtual_mechanism.$(attracted_frames[i])"); id = "ee $(attracted_frames_names[i]) diff")
        add_component!(vms, LinearSpring(K, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) spring")
        add_component!(vms, LinearDamper(D, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) damper")
        add_component!(vms, ExponentialDamper(exponential_damping_matrix, "ee $(attracted_frames_names[i]) diff", damping_decay_rate); id = "ee $(attracted_frames_names[i]) exp damper")
    end

    add_component!(vms, LinearDamper(SMatrix{3, 3}(1.0, 0., 0., 0., 1.0, 0., 0., 0., 1.0),"ee thdistal diff"); id = "ee thdistal mass damper")

    # "Closing" the finger ---> connecting the two extremes to the corners of the box
    K = SMatrix{3, 3}(yz_stiffness, 0., 0., 0., yz_stiffness, 0., 0., 0., yz_stiffness)

    ext_corner = SVector(box_position[1] - box_dimensions[1], box_position[2] + box_dimensions[2], box_position[3] - box_dimensions[3])
    add_coordinate!(vms, ConstCoord(ext_corner); id = "ext corner")
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.ffprox ee position", "ext corner"); id = "ext corner diff")
    add_component!(vms, LinearSpring(K, "ext corner diff"); id = "ext corner spring")
    add_component!(vms, LinearDamper(D, "ext corner diff"); id = "ext corner damper")

    int_corner = SVector(box_position[1] - box_dimensions[1], box_position[2] - box_dimensions[2], box_position[3] - box_dimensions[3])
    add_coordinate!(vms, ConstCoord(int_corner); id = "int corner")
    add_coordinate!(vms, CoordDifference(".virtual_mechanism.fftip ee position", "int corner"); id = "int corner diff")
    add_component!(vms, LinearSpring(K, "int corner diff"); id = "int corner spring")
    add_component!(vms, LinearDamper(D, "int corner diff"); id = "int corner damper")

    # Thumb push 
    add_coordinate!(vms,  ConstCoord(box_position);  id="box position")
    add_coordinate!(vms, CoordDifference("box position", ".virtual_mechanism.rh_thdistal"); id = "th distal box diff")
    add_coordinate!(vms, CoordSlice("th distal box diff", SVector(2)); id = "th distal y diff")
    add_component!(vms, LinearSpring(yz_stiffness, "th distal y diff"); id = "th distal y spring")
    add_component!(vms, LinearDamper(base_damping, "th distal y diff"); id = "th distal y damper")

    # BOX COLLISION MODEL  

    add_coordinate!(vms, ConstCoord(box_dimensions[1]); id="box dimension 1")
    add_coordinate!(vms, ConstCoord(box_dimensions[2]); id="box dimension 2")
    add_coordinate!(vms, ConstCoord(box_dimensions[3]); id="box dimension 3")
    
    repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord", ".virtual_mechanism.rh_fftip", 
                        ".virtual_mechanism.rh_ffmiddle", ".virtual_mechanism.rh_ffproximal", ".virtual_mechanism.rh_thtip", ".virtual_mechanism.rh_thdistal", ".virtual_mechanism.rh_thdistal_mass_coord", 
                        ".virtual_mechanism.rh_thproximal_mass_coord", ".virtual_mechanism.rh_thmiddle")
    frames_names = ("fftip_mass", "ffmiddle_mass", "ffprox_mass", "fftip", "ffmiddle", "ffprox", "thtip", "thdistal", "thdistal_mass", "thproximal_mass", "thmiddle")
    
    for i in 1:length(repulsed_frames)
        frame = repulsed_frames[i]
        add_coordinate!(vms, CoordDifference(frame, "box position") ; id = "$(frames_names[i]) box diff" )
        for j in 1:3
            add_coordinate!(vms, CoordSlice("$(frames_names[i]) box diff", SVector(j)); id = "$(frames_names[i]) box diff dimension $(j)")
            add_coordinate!(vms, CoordNorm("$(frames_names[i]) box diff dimension $(j)"); id = "$(frames_names[i]) box norm dimension $(j)")
            add_coordinate!(vms, CoordDifference("$(frames_names[i]) box norm dimension $(j)","box dimension $(j)"); id = "shifted $(frames_names[i]) box norm dimension $(j)")
    
            add_component!(vms, ReLUSpring(0.0, "shifted $(frames_names[i]) box norm dimension $(j)", true); id="$(frames_names[i]) dimension $(j) repulsive spring")
            add_component!(vms, RectifiedDamper(0.0, "$(frames_names[i]) box norm dimension $(j)", (0.0, 1.1*box_dimensions[j]), true, false); id="$(frames_names[i]) dimension $(j) damper")
        end
    end


    function setup_box_collision_model(cache, repulsed_frames, frames_names)
        repulsed_frames_coord_ID = []
        repulsive_springs_damper_ID = []
        for i in 1:length(repulsed_frames)
            frame = repulsed_frames[i]
            push!(repulsed_frames_coord_ID, get_compiled_coordID(cache, frame))
            frame_springs_dampers_vec = []
            for j in 1:3
                push!(frame_springs_dampers_vec, get_compiled_componentID(cache, "$(frames_names[i]) dimension $(j) repulsive spring"))
                push!(frame_springs_dampers_vec, get_compiled_componentID(cache, "$(frames_names[i]) dimension $(j) damper"))
            end
            push!(repulsive_springs_damper_ID, frame_springs_dampers_vec)
        end
    
        return box_position, box_dimensions, repulsed_frames_coord_ID, repulsive_springs_damper_ID
    end
    
    function update_box_collision_model(cache, collision_args)
        box_position, box_dimensions, repulsed_frames_coord_ID, repulsive_springs_damper_ID = collision_args
        margin = 0.001
        for i in 1:length(repulsed_frames_coord_ID)
            frame_pos = configuration(cache, repulsed_frames_coord_ID[i])
            for j in 1:3
                # get the indices different from j
                others = filter(x -> x ≠ j, 1:3) 
                #Check if the position of the frame is inside "the field of action" of the spring
                if abs(frame_pos[others[1]] - box_position[others[1]]) < (box_dimensions[others[1]]-margin) && abs(frame_pos[others[2]] - box_position[others[2]]) < (box_dimensions[others[2]]-margin)
                    cache[repulsive_springs_damper_ID[i][2*j-1]] = remake(cache[repulsive_springs_damper_ID[i][2*j-1]] ; stiffness = 5.0)
                    cache[repulsive_springs_damper_ID[i][2*j]] = remake(cache[repulsive_springs_damper_ID[i][2*j]] ; damping = 5.0)          
                else
                    cache[repulsive_springs_damper_ID[i][2*j-1]] = remake(cache[repulsive_springs_damper_ID[i][2*j-1]] ; stiffness = 0.0)
                    cache[repulsive_springs_damper_ID[i][2*j]] = remake(cache[repulsive_springs_damper_ID[i][2*j]] ; damping = 0.0)
                end
            end
        end
    end
    
    function f_setup(cache)
    
        box_collision_args = setup_box_collision_model(cache, repulsed_frames, frames_names)
        return box_collision_args
        
    end
    
    function f_control(cache, t, args, extra)
        
        collision_args = args 
        update_box_collision_model(cache, collision_args)
    
    end


    println("Virtual controller built. Connecting to ROS client...")

    cvms = compile(vms)
    qᵛ = generate_q_init(cvms; mf=true, rf=true, lf=true)
    #thumb max extension
    qᵛ[21] = 0.2
    qᵛ[23] = -0.7
    qᵛ[24] = -0.26
    qᵛ[5] = 1.57
    qᵛ[4] = 1.0

    joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                    "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                    "rh_THJ3", "rh_THJ4", "rh_THJ5"]


    with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; f_control, f_setup, E_max=10.0, steady_state_check=true)
    end

end
