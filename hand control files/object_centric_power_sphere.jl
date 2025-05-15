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


function object_centric_power_sphere(ball_radius)
    print("Waiting 5 seconds...")
    sleep(5)
    print("parsing robot URDF... ")

    module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])

    shadow_cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
    shadow_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), shadow_cfg)

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

    println("URDF parsed !")

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
        add_deadzone_springs!(vm_robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
        add_component!(vm_robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
    end

    println("Robot built !")

    print("Building the virtual mechanisms...")
    ball_position = SVector(0.0, -0.035, 0.32)

    attracted_frames = ("rh_ffdistal_mass_coord", "rh_mfdistal_mass_coord", "rh_rfdistal_mass_coord","rh_lfdistal_mass_coord" , 
                        "rh_thdistal_mass_coord", "rh_ffmiddle_mass_coord","rh_mfmiddle_mass_coord", "rh_rfmiddle_mass_coord",
                        "rh_lfmiddle_mass_coord",  "rh_thmiddle_mass_coord", "rh_ffproximal_mass_coord", "rh_mfproximal_mass_coord",
                        "rh_rfproximal_mass_coord", "rh_lfproximal_mass_coord", "rh_thproximal_mass_coord", "rh_palm_mass_coord")
    attracted_frames_names = ("ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal", "ffmiddle", "mfmiddle", "rfmiddle", "lfmiddle", "thmiddle", "ffprox", 
                    "mfprox", "rfprox", "lfprox", "thprox", "palm")


    for i in 1:length(attracted_frames)
        add_frame!(vm_robot; id = "revo_frame_1_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Revolute(SVector(0.0,0.0,1.0),Transform(SVector(ball_position[1], ball_position[2], ball_position[3]))); parent=root_frame(vm_robot), child="revo_frame_1_$(attracted_frames_names[i])", id="revo_joint_1_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="revo_frame_2_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Revolute(SVector(1.0,0.0,0.0)); parent="revo_frame_1_$(attracted_frames_names[i])", child="revo_frame_2_$(attracted_frames_names[i])", id="revo_joint_2_$(attracted_frames_names[i])")
        add_frame!(vm_robot; id="ee_frame_$(attracted_frames_names[i])")
        add_joint!(vm_robot, Rigid(Transform(SVector(0.0,0.0,ball_radius))); parent ="revo_frame_2_$(attracted_frames_names[i])", child ="ee_frame_$(attracted_frames_names[i])", id = "fixed_joint_$(attracted_frames_names[i])")

        add_coordinate!(vm_robot, FrameOrigin("ee_frame_$(attracted_frames_names[i])"); id="$(attracted_frames_names[i]) ee position")
        add_component!(vm_robot, PointMass(0.01, "$(attracted_frames_names[i]) ee position"); id="$(attracted_frames_names[i]) ee mass")

        add_coordinate!(vm_robot, JointSubspace("revo_joint_1_$(attracted_frames_names[i])"); id="revo_joint_1_$(attracted_frames_names[i])")
        #add_component!(vm_robot, LinearDamper(10.0, "revo_joint_1_$(attracted_frames_names[i])"); id="revo_joint_1_$(attracted_frames_names[i])_damper")
        add_coordinate!(vm_robot, JointSubspace("revo_joint_2_$(attracted_frames_names[i])"); id="revo_joint_2_$(attracted_frames_names[i])")
        #add_component!(vm_robot, LinearDamper(0.5, "revo_joint_2_$(attracted_frames_names[i])"); id="revo_joint_2_$(attracted_frames_names[i])_damper")   
        
        I_mat = @SMatrix [0.1  0.    0.  ;0.    0.1  0.  ;0.    0.    0.1]
        add_inertia!(vm_robot, "ee_frame_$(attracted_frames_names[i])", I_mat; id="$(attracted_frames_names[i]) ee inertia")
    end

    
    vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)

    # HAND MOTION

    D = SMatrix{3, 3}(0.05, 0., 0., 0., 0.05, 0., 0., 0., 0.05)

    stiffnesses = [0.03, 0.03, 0.03, 0.03, 0.03, 0.05, 0.05, 0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1, 0.05]

    # hand converging to the ball 

    for i in 1:length(attracted_frames)
        K = SMatrix{3, 3}(stiffnesses[i], 0., 0., 0., stiffnesses[i], 0., 0., 0., stiffnesses[i])
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(attracted_frames_names[i]) ee position", ".virtual_mechanism.$(attracted_frames[i])"); id = "ee $(attracted_frames_names[i]) diff")
        add_component!(vms, LinearSpring(K, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) spring")
        add_component!(vms, LinearDamper(D, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) damper")
    end

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

    # finger spacing : finger level 

    add_coordinate!(vms, ConstCoord(0.05); id = "finger spring length")

    finger_spacing_stiffness = 0.07

    add_coordinate!(vms, RotatedCoord(".virtual_mechanism.rh_mfmiddle_mass_coord", ".virtual_mechanism.rh_ffmiddle"); id="ff mf middle diff")
    add_coordinate!(vms, CoordSlice("ff mf middle diff", SVector(1)); id="ff mf middle diff x")
    add_coordinate!(vms, CoordNorm("ff mf middle diff x"); id="ff mf middle norm")
    add_coordinate!(vms, CoordDifference("ff mf middle norm", "finger spring length"); id="ff mf middle error")
    add_component!(vms, LinearSpring(finger_spacing_stiffness, "ff mf middle error"); id="ff mf middle spring")
    add_component!(vms, LinearDamper(0.01, "ff mf middle error"); id="ff mf middle damper")

    add_coordinate!(vms, RotatedCoord(".virtual_mechanism.rh_rfmiddle_mass_coord", ".virtual_mechanism.rh_mfmiddle"); id="mf rf middle diff")
    add_coordinate!(vms, CoordSlice("mf rf middle diff", SVector(1)); id="mf rf middle diff x")
    add_coordinate!(vms, CoordNorm("mf rf middle diff x"); id="mf rf middle norm")
    add_coordinate!(vms, CoordDifference("mf rf middle norm", "finger spring length"); id="mf rf middle error")
    add_component!(vms, LinearSpring(finger_spacing_stiffness, "mf rf middle error"); id="mf rf middle spring")
    add_component!(vms, LinearDamper(0.01, "mf rf middle error"); id="mf rf middle damper")

    add_coordinate!(vms, RotatedCoord(".virtual_mechanism.rh_lfmiddle_mass_coord", ".virtual_mechanism.rh_rfmiddle"); id="rf lf middle diff")
    add_coordinate!(vms, CoordSlice("rf lf middle diff", SVector(1)); id="rf lf middle diff x")
    add_coordinate!(vms, CoordNorm("rf lf middle diff x"); id="rf lf middle norm")
    add_coordinate!(vms, CoordDifference("rf lf middle norm", "finger spring length"); id="rf lf middle error")
    add_component!(vms, LinearSpring(finger_spacing_stiffness, "rf lf middle error"); id="rf lf middle spring")
    add_component!(vms, LinearDamper(0.01, "rf lf middle error"); id="rf lf middle damper")

    # BALL COLLISION MODEL  

    add_coordinate!(vms,  ConstCoord(ball_position);  id="ball position")
    add_coordinate!(vms, ConstCoord(ball_radius); id="ball radius")

    add_coordinate!(vms, FramePoint(".virtual_mechanism.rh_palm", SVector(0. , 0., 0.07)); id="second palm point")

    repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_mftip_mass_coord", ".virtual_mechanism.rh_rftip_mass_coord",".virtual_mechanism.rh_lftip_mass_coord" , 
                        ".virtual_mechanism.rh_thtip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord",".virtual_mechanism.rh_mfmiddle_mass_coord", ".virtual_mechanism.rh_rfmiddle_mass_coord",
                        ".virtual_mechanism.rh_lfmiddle_mass_coord",  ".virtual_mechanism.rh_thmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord", ".virtual_mechanism.rh_mfproximal_mass_coord",
                        ".virtual_mechanism.rh_rfproximal_mass_coord", ".virtual_mechanism.rh_lfproximal_mass_coord", ".virtual_mechanism.rh_thproximal_mass_coord", ".virtual_mechanism.rh_palm_mass_coord", "second palm point",
                        ".virtual_mechanism.rh_ffdistal", ".virtual_mechanism.rh_mfdistal", ".virtual_mechanism.rh_rfdistal", ".virtual_mechanism.rh_lfdistal", ".virtual_mechanism.rh_thdistal")
    repulsed_frames_names = ("fftip", "mftip", "rftip", "lftip", "thtip", "ffmiddle", "mfmiddle", "rfmiddle", "lfmiddle", "thmiddle", "ffprox", 
                    "mfprox", "rfprox", "lfprox", "thprox", "palm", "palm2", "ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal")

    for i in 1:length(repulsed_frames)
        frame = repulsed_frames[i]
        add_coordinate!(vms, CoordDifference(frame, "ball position") ; id = "$(repulsed_frames_names[i]) ball error" )
        add_coordinate!(vms, CoordNorm("$(repulsed_frames_names[i]) ball error") ; id = "$(repulsed_frames_names[i]) ball error norm")
        add_coordinate!(vms, CoordDifference("$(repulsed_frames_names[i]) ball error norm", "ball radius"); id = "shifted $(repulsed_frames_names[i]) ball error" )

        add_component!(vms, ReLUSpring(1.0, "shifted $(repulsed_frames_names[i]) ball error", true); id="$(repulsed_frames_names[i]) ball repulsive spring")
        add_component!(vms, RectifiedDamper(0.1, "$(repulsed_frames_names[i]) ball error norm", (0.0, 1.1*ball_radius), true, false); id="$(repulsed_frames_names[i]) ball damper")
    end

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
    qᵛ[21] = 1.2

    joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                    "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                    "rh_THJ3", "rh_THJ4", "rh_THJ5"]


    with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; E_max=10.0)
    end

end