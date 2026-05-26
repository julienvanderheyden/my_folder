using GeometryBasics: Vec3f, Point3f
using LinearAlgebra
using StaticArrays
using VMRobotControl
using VMRobotControl: remake
include("functions.jl")
include("force_modulated_medium_wrap_functions.jl")

module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
include(joinpath(module_path, "ros/ROS.jl"))
shadow_hand_urdf_path = joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf")

###### URDF PARSING #####
using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end

const MISMATCH_DEADZONE = 0.05

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
        ["rh_thtip_mass_coord", "rh_thmiddle_mass_coord","rh_thdistal", "rh_thmiddle"],
        ["thtip", "thmiddle", "thdistal", "thmiddle2"],
        [],                                          # thumb has no coupled J0
        ["rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"],
    ),
)



function force_modulation(cylinder_radius, penetration_depth, feedback_stiffness, feedback_damping)

    # ------------------ BUILD THE ROBOTS -------------------------
    shadow_robot = build_robot(shadow_hand_urdf_path)
    # For the moment the urdfs are the same but we might want to change the properties of the virtual robot
    vm_robot = build_robot(shadow_hand_urdf_path; joint_limiting=true) 

    # add frames for the points of interest (finger tips, knuckles, palm)
    add_coordinate!(vm_robot, FrameOrigin("rh_ffdistal"); id="rh_ffdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_mfdistal"); id="rh_mfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_rfdistal"); id="rh_rfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_lfdistal"); id="rh_lfdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thdistal"); id="rh_thdistal")
    add_coordinate!(vm_robot, FrameOrigin("rh_thmiddle"); id="rh_thmiddle")

    # ------------------ BUILD THE VIRTUAL MECHANISM ------------------
    cylinder_position = compute_cylinder_position(vm_robot, cylinder_radius)
    
    build_cylinder_virtual_object(vm_robot, FINGER_CONFIGS, cylinder_position, cylinder_radius)
    
    vms = VirtualMechanismSystem("myShadowVMS", shadow_robot, vm_robot)

    # -------------------- HAND MOTION --------------------------------

    # CYLINDER ATTRACTION CONNECTION
    base_stiffness         = 0.05
    phalanx_scaling_factor = 0.5
    finger_scaling_factor  = 1.5

    base_damping              = 0.05
    damping_decay_rate        = 161
    exponential_damping_coeff = 0.1

    connect_virtual_hand_object(vms, FINGER_CONFIGS, base_stiffness, phalanx_scaling_factor, finger_scaling_factor,
                                base_damping, damping_decay_rate, exponential_damping_coeff)

    # CYLINDER COLLISION MODEL  
    build_cylinder_collision_model(vms, FINGER_CONFIGS, cylinder_radius, cylinder_position)
    
    # --------------- REAL/VIRTUAL HAND INTERCONNECTION -----------------

    WRIST_JOINTS = ["rh_WRJ1", "rh_WRJ2"]

    for joint_id in WRIST_JOINTS
        add_joint_feedback!(vms, joint_id, MISMATCH_DEADZONE, feedback_stiffness, feedback_damping)
    end

    for config in values(FINGER_CONFIGS)
        for joint_id in config.uncoupled_joints
            add_joint_feedback!(vms, joint_id, MISMATCH_DEADZONE, feedback_stiffness, feedback_damping)
        end
        for joint_id in config.coupled_joints
            add_joint_feedback!(vms, joint_id, 2*MISMATCH_DEADZONE, feedback_stiffness, feedback_damping)
        end
    end



    # ---------------- TEST : CONTACT DETECTION COORDINATES ---------------------
    for (frame_id, name) in zip(FINGER_CONFIGS["ff"].attracted_frames, FINGER_CONFIGS["ff"].attracted_frames_names)
        add_coordinate!(vms, CoordDifference(".robot.$frame_id", "ff cylinder position");          id="robot $name cylinder diff")
        add_coordinate!(vms, CoordSlice("robot $name cylinder diff", SVector(2, 3));           id="robot $name planar error")
        add_coordinate!(vms, CoordNorm("robot $name planar error");                            id="robot $name planar error norm")
        add_coordinate!(vms, CoordDifference("$name planar error norm", "robot $name planar error norm"); id="$name radial penetration")
    end

    for (frame_id, name) in zip(FINGER_CONFIGS["mf"].attracted_frames, FINGER_CONFIGS["mf"].attracted_frames_names)
        add_coordinate!(vms, CoordDifference(".robot.$frame_id", "mf cylinder position");          id="robot $name cylinder diff")
        add_coordinate!(vms, CoordSlice("robot $name cylinder diff", SVector(2, 3));           id="robot $name planar error")
        add_coordinate!(vms, CoordNorm("robot $name planar error");                            id="robot $name planar error norm")
        add_coordinate!(vms, CoordDifference("$name planar error norm", "robot $name planar error norm"); id="$name radial penetration")
    end

    function f_setup(cache)
        penetration_dict = Dict{String, Any}()
        real_robot_radial_pos_dict = Dict{String, Any}()
        for name in FINGER_CONFIGS["ff"].attracted_frames_names
            penetration_ID = get_compiled_coordID(cache, "$name radial penetration")
            penetration_dict[name] = penetration_ID
            real_robot_radial_pos_ID = get_compiled_coordID(cache, "robot $name planar error norm")
            real_robot_radial_pos_dict[name] = real_robot_radial_pos_ID
        end
        return penetration_dict, real_robot_radial_pos_dict
    end

    # State outside f_control
    last_t = 0.0
    previous_radial_pos = Dict{String, Union{Nothing, Float64}}(
                            name => nothing 
                            for name in FINGER_CONFIGS["ff"].attracted_frames_names
                        )

    function f_control(cache, t, args, extra)
        penetration_dict, real_robot_radial_pos_dict = args
        dt = t - last_t

        for name in FINGER_CONFIGS["ff"].attracted_frames_names
            penetration        = only(configuration(cache, penetration_dict[name]))
            radial_pos         = only(configuration(cache, real_robot_radial_pos_dict[name]))
            prev               = previous_radial_pos[name]

            if !isnothing(prev) && dt > 0
                radial_velocity = only(velocity(cache, real_robot_radial_pos_dict[name]))
                radial_acceleration = only(acceleration(cache, real_robot_radial_pos_dict[name]))
                # @info "$name penetration : $(round(penetration*1000, digits=2)) mm | " *
                #     "radial velocity: $(round(radial_velocity*1000, digits=2)) mm/s" *
                #     "radial acceleration: $(round(radial_acceleration*1000, digits=5)) mm/s²"
                if radial_acceleration > 0.0000005 && abs(radial_velocity) < 0.005 && penetration < -0.003
                    @info "Contact detected — $name | " *
                        "penetration: $(round(penetration*1000, digits=1)) mm | " *
                        "radial velocity: $(round(radial_velocity*1000, digits=1)) mm/s"
                end
            end

            previous_radial_pos[name] = radial_pos
        end

        last_t = t
    end


    # finger_states = Dict(name => FingerModulationState(cylinder_radius) for name in keys(FINGER_CONFIGS))

    # last_t = 0.0


    println("Connecting to ROS client...")
    cvms = compile(vms)
    medium_wrap_preshape = zeros(24)
    medium_wrap_preshape[21] = 1.2 # thumb extended
    qᵛ = medium_wrap_preshape

    joint_names = ["rh_WRJ1", "rh_WRJ2", "rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", "rh_MFJ1",
                    "rh_MFJ2", "rh_MFJ3", "rh_MFJ4", "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4", 
                    "rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5", "rh_THJ1", "rh_THJ2", 
                    "rh_THJ3", "rh_THJ4", "rh_THJ5"]


    with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 24, 48) do connection
        ros_vm_position_controller(connection, cvms, qᵛ, joint_names; f_control, f_setup, E_max=10.0)
        # ros_vm_position_controller(connection, cvms, qᵛ, joint_names; E_max=10.0)
    end

end


# function f_control(cache, t, args, extra)

#     (radius_joints, root_joints,
#     cylinder_radius_coord_dict, cylinder_position_coord_dict, damper_component_dict,
#     feedback_coordID_uncoupled, feedback_coordID_coupled, attraction_coordID) = args

#     for (finger, cfg) in FINGER_CONFIGS
#         state = finger_states[finger]

#         update_finger_state!(state, finger, cache, t,
#                             attraction_coordID,
#                             feedback_coordID_uncoupled,
#                             feedback_coordID_coupled)

#         apply_radius_modulation!(finger, state, cache, t, m, kcache,
#                                 radius_joints, root_joints,
#                                 cylinder_radius_coord_dict,
#                                 cylinder_position_coord_dict,
#                                 damper_component_dict, last_t)
#     end

#     last_t = t
# end



# function f_setup(cache)

#     radius_joints                = Dict{String, Any}() # joint IDs of the rigid joints controlling the attracting cylinder radius for each finger
#     root_joints                  = Dict{String, Any}() # joint IDs of the root joints controlling the attracting cylinder position for each finger
#     attraction_coordID           = Dict{String, Any}() # coord IDs of the attraction spring between each attach point and the corresponding cylinder center (used to detect virtual contact)
#     cylinder_radius_coord_dict   = Dict{String, Any}() # coord IDs of the spring rest length controlling the repulsive cylinder radius f
#     cylinder_position_coord_dict = Dict{String, Any}() # coord IDs of the fixed point controlling the repulsive cylinder position
#     damper_component_dict        = Dict{String, Any}() # component ID of the damper of the repulsive cylinder (cannot be modulated with coord)
#     feedback_coordID_uncoupled   = Dict{String, Any}() # coord IDs of the feedback springs for uncoupled joints (used to detect real contact)
#     feedback_coordID_coupled     = Dict{String, Any}() # coord IDs of the feedback springs for coupled joints (used to detect real contact, higher deadzone)

#     for (finger, cfg) in FINGER_CONFIGS

#         for frame in cfg.attracted_frames_names
#             radius_joints[frame]      = get_compiled_jointID(cache, ".virtual_mechanism.fixed_joint_$(frame)")
#             root_joints[frame]        = get_compiled_jointID(cache, ".virtual_mechanism.root_joint_$(frame)")
#             attraction_coordID[frame] = get_compiled_coordID(cache, "ee $(frame) diff")
#         end

#         for frame in cfg.repulsed_frames_names
#             cylinder_radius_coord_dict[frame]   = get_compiled_coordID(cache, "$(frame) cylinder radius")
#             cylinder_position_coord_dict[frame] = get_compiled_coordID(cache, "$(frame) cylinder position")
#             damper_component_dict[frame]        = get_compiled_componentID(cache, "$(frame) cylinder damper")
#         end

#         for joint in cfg.uncoupled_joints
#             feedback_coordID_uncoupled[joint] = get_compiled_coordID(cache, "$(joint) coord diff")
#         end

#         for joint in cfg.coupled_joints
#             feedback_coordID_coupled[joint] = get_compiled_coordID(cache, "$(joint) coord diff")
#         end
#     end

#     return (radius_joints, root_joints,
#             cylinder_radius_coord_dict, cylinder_position_coord_dict, damper_component_dict,
#             feedback_coordID_uncoupled, feedback_coordID_coupled, attraction_coordID)
# end

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

