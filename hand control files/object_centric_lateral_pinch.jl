using Revise
using GeometryBasics: Vec3f, Point3f
using LinearAlgebra
using StaticArrays
using VMRobotControl
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


function object_centric_lateral_pinch(box_width, box_thickness)
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

    add_coordinate!(vm_robot, FrameOrigin("rh_fftip"); id = "rh_fftip")
    add_coordinate!(vm_robot, FrameOrigin("rh_ffdistal"); id="rh_ffdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_ffmiddle"); id="rh_ffmiddle")
    add_coordinate!(vm_robot, FrameOrigin("rh_ffproximal"); id="rh_ffproximal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thtip"); id = "rh_thtip")
    add_coordinate!(vm_robot, FrameOrigin("rh_thdistal"); id="rh_thdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thmiddle"); id="rh_thmiddle")

    println("URDF parsed !")

    ##### COMPLEMENTING THE VIRTUAL ROBOT #####

    print("Building the virtual robot...")

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

    println("Robot built !")

    print("Building the virtual mechanisms...")

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
    
        # margin_factor = 0.8
        # deadzone_stiffness = 5.0
        # add_deadzone_springs!(vm_robot, deadzone_stiffness, (-box_dimensions[2]*margin_factor, box_dimensions[2]*margin_factor), "prism_joint_1_$(attracted_frames_names[i])")
        # add_deadzone_springs!(vm_robot, deadzone_stiffness, (-box_dimensions[3]*margin_factor, box_dimensions[3]*margin_factor), "prism_joint_2_$(attracted_frames_names[i])")

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

    #damping_decay_rate = 161 # 20% of damping at |z| = 0.01
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

    # Compile the virtual mechanism system, and run the controller via ROS
    # Make sure rospy_client.py is running first.
    println("Connecting to ROS client...")
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
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; f_control, f_setup, E_max=10.0)
    end

end