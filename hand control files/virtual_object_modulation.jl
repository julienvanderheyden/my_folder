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
    
    # attracted_frames = ("rh_lfdistal_mass_coord", "rh_lfmiddle_mass_coord", "rh_lfproximal_mass_coord", "rh_rfdistal_mass_coord", 
    # "rh_rfmiddle_mass_coord", "rh_rfproximal_mass_coord", "rh_mfdistal_mass_coord", "rh_mfmiddle_mass_coord",
    # "rh_mfproximal_mass_coord", "rh_ffdistal_mass_coord", "rh_ffmiddle_mass_coord", "rh_ffproximal_mass_coord",
    # "rh_thdistal_mass_coord", "rh_thmiddle_mass_coord") #, "rh_thproximal_mass_coord" , "rh_palm_mass_coord")

    attracted_frames = ("rh_ffdistal_mass_coord", "rh_ffmiddle_mass_coord", "rh_ffproximal_mass_coord") #, "rh_thproximal_mass_coord" , "rh_palm_mass_coord")

    # attracted_frames_names = ("lfdistal", "lfmiddle", "lfprox", "rfdistal", "rfmiddle", "rfprox", "mfdistal", "mfmiddle", "mfprox", "ffdistal", "ffmiddle", 
    # "ffprox", "thdistal", "thmiddle") #, "thprox", "palm")

    attracted_frames_names = ("ffdistal", "ffmiddle", "ffprox") #, "thprox", "palm")
    
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

    # WHY NOT REDUCING THE LAST RIGID JOINT (FOR THE LITTLE FINGER) SUCH THAT THIS FINGER EXERTS MORE FORCE? (read in a paper that this finger exerts more force) 
    # ---> should also adapt the collision model then   

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
        #K = SMatrix{3, 3}(stiffnesses[i], 0., 0., 0., stiffnesses[i], 0., 0., 0., stiffnesses[i])
        K = SMatrix{3, 3}(base_stiffness, 0., 0., 0., base_stiffness, 0., 0., 0., base_stiffness)
        add_coordinate!(vms, CoordDifference(".virtual_mechanism.$(attracted_frames_names[i]) ee position", ".virtual_mechanism.$(attracted_frames[i])"); id = "ee $(attracted_frames_names[i]) diff")
        add_component!(vms, LinearSpring(K, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) spring")
        add_component!(vms, LinearDamper(D, "ee $(attracted_frames_names[i]) diff"); id = "ee $(attracted_frames_names[i]) damper")
        add_component!(vms, ExponentialDamper(exponential_damping_matrix, "ee $(attracted_frames_names[i]) diff", damping_decay_rate); id = "ee $(attracted_frames_names[i]) exp damper")
    end

    # add_component!(vms, LinearDamper(SMatrix{3, 3}(10.0, 0., 0., 0., 10.0, 0., 0., 0., 10.0), "ee thmiddle diff"); id = "thmiddle massive damper")
    
    #lightly constraint some joints to avoid unwanted motions 
    # add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_FFJ4_coord"); id = "ff j4 angular spring")
    # add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_MFJ4_coord"); id = "mf j4 angular spring")
    # add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_RFJ4_coord"); id = "rf j4 angular spring")
    # add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_LFJ4_coord"); id = "lf j4 angular spring")
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ1_coord"); id = "wr j1 angular spring")
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ2_coord"); id = "wr j2 angular spring")

    # CYLINDER COLLISION MODEL  

    add_coordinate!(vms,  ConstCoord(cylinder_position);  id="cylinder position")
    add_coordinate!(vms, ConstCoord(cylinder_radius); id="cylinder radius")
    
    add_coordinate!(vms, FramePoint(".virtual_mechanism.rh_palm", SVector(0. , 0., 0.07)); id="second palm point")
    
    # repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_mftip_mass_coord", ".virtual_mechanism.rh_rftip_mass_coord",".virtual_mechanism.rh_lftip_mass_coord" , 
    #                     ".virtual_mechanism.rh_thtip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord",".virtual_mechanism.rh_mfmiddle_mass_coord", ".virtual_mechanism.rh_rfmiddle_mass_coord",
    #                     ".virtual_mechanism.rh_lfmiddle_mass_coord",  ".virtual_mechanism.rh_thmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord", ".virtual_mechanism.rh_mfproximal_mass_coord",
    #                     ".virtual_mechanism.rh_rfproximal_mass_coord", ".virtual_mechanism.rh_lfproximal_mass_coord", ".virtual_mechanism.rh_thproximal_mass_coord", ".virtual_mechanism.rh_palm_mass_coord", "second palm point",
    #                     ".virtual_mechanism.rh_ffdistal", ".virtual_mechanism.rh_mfdistal", ".virtual_mechanism.rh_rfdistal", ".virtual_mechanism.rh_lfdistal", ".virtual_mechanism.rh_thdistal", ".virtual_mechanism.rh_thmiddle")

    repulsed_frames = (".virtual_mechanism.rh_fftip_mass_coord", ".virtual_mechanism.rh_ffmiddle_mass_coord", ".virtual_mechanism.rh_ffproximal_mass_coord",".virtual_mechanism.rh_ffdistal")

    # repulsed_frames_names = ("fftip", "mftip", "rftip", "lftip", "thtip", "ffmiddle", "mfmiddle", "rfmiddle", "lfmiddle", "thmiddle", "ffprox", 
    #                 "mfprox", "rfprox", "lfprox", "thprox", "palm", "palm2", "ffdistal", "mfdistal", "rfdistal", "lfdistal", "thdistal", "thmiddle2")

    repulsed_frames_names = ("fftip", "ffmiddle",  "ffprox", "ffdistal")
    
    for i in 1:length(repulsed_frames)
        frame = repulsed_frames[i]
        add_coordinate!(vms, CoordDifference(frame, "cylinder position") ; id = "$(repulsed_frames_names[i]) cylinder diff" )
        add_coordinate!(vms, CoordSlice("$(repulsed_frames_names[i]) cylinder diff", SVector(2,3)); id="$(repulsed_frames_names[i]) planar error")
        add_coordinate!(vms, CoordNorm("$(repulsed_frames_names[i]) planar error") ; id = "$(repulsed_frames_names[i]) planar error norm")
        add_coordinate!(vms, CoordDifference("$(repulsed_frames_names[i]) planar error norm", "cylinder radius"); id = "shifted $(repulsed_frames_names[i]) cylinder error" )
    
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
        add_deadzone_springs!(vms, feedback_stiffness, (-mismatch_deadzone, mismatch_deadzone), "$(joint_id) coord diff")
        add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (1.8*mismatch_deadzone, 2.2*mismatch_deadzone), false, false); id = "$(joint_id) feedback damper 1")
        add_component!(vms, RectifiedDamper(feedback_damping,"$(joint_id) coord diff", (-2.2*mismatch_deadzone, -1.8*mismatch_deadzone), true, false); id = "$(joint_id) feedback damper 2")
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
        
        rigid_joints = Dict{String, Any}()
        for frame in attracted_frames_names
            jointID = get_compiled_jointID(cache, ".virtual_mechanism.fixed_joint_$(frame)")
            rigid_joints[frame] = jointID
        end
        return rigid_joints
        
    end
    
    function f_control(cache, t, args, extra)
        
        rigid_joints = args 
        if t  > 10.0
            new_radius = 0.01
            cache[rigid_joints["ffdistal"]] = remake(cache[rigid_joints["ffdistal"]]; jointData = Rigid(Transform(SVector(0.0, 0.0, new_radius))))
            cache[rigid_joints["ffmiddle"]] = remake(cache[rigid_joints["ffmiddle"]]; jointData = Rigid(Transform(SVector(0.0, 0.0, new_radius))))
            cache[rigid_joints["ffprox"]] = remake(cache[rigid_joints["ffprox"]]; jointData = Rigid(Transform(SVector(0.0, 0.0, new_radius))))

        end
    end



    # Compile the virtual mechanism system, and run the controller via ROS
    # Make sure rospy_client.py is running first.
    println("Connecting to ROS client...")
    cvms = compile(vms)
    qᵛ = medium_wrap_preshape

    joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                    "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                    "rh_THJ3", "rh_THJ4", "rh_THJ5"]


    with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; f_control, f_setup, E_max=10.0)
    end

end
