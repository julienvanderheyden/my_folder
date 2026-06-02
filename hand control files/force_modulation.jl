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
    contact_detected::Bool
    contact_detection_time::Float64
    accel_hysteresis::Vector{Int}
    real_object_radius::Float64
    virtual_object_radius::Float64
    radius_modulation::Bool
    equilibrium_detection_time::Float64
end

FingerModulationState(initial_radius::Float64, n_frames::Int) = FingerModulationState(
    false, 0.0, zeros(Int, n_frames),0.0, initial_radius, false, 0.0)

struct FingerConfig
    attracted_frames::Vector{String}       # mass coord IDs used for attraction springs
    attracted_frames_names::Vector{String} # short names for those frames
    repulsed_frames::Vector{String}        # mass coord IDs used for repulsion
    repulsed_frames_names::Vector{String}  # short names for those frames
    coupled_joints::Vector{String}         # joints where J0 = J1 + J2
    uncoupled_joints::Vector{String}       # standard individual joints
    joints::Vector{String}                 # all joints of the finger (for convenience)
    finger_width::Float64                  # used for real object dimension estimation
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
        ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4"],
        0.005,
    ),

    "mf" => FingerConfig(
        ["rh_mfdistal_mass_coord", "rh_mfmiddle_mass_coord", "rh_mfproximal_mass_coord"],
        ["mfdistal", "mfmiddle", "mfprox"],
        ["rh_mftip_mass_coord", "rh_mfmiddle_mass_coord",
         "rh_mfproximal_mass_coord", "rh_mfdistal"],
        ["mftip", "mfmiddle", "mfprox", "mfdistal"],
        ["rh_MFJ0"],
        ["rh_MFJ3", "rh_MFJ4"],
        ["rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4"],
        0.005,
    ),

    "rf" => FingerConfig(
        ["rh_rfdistal_mass_coord", "rh_rfmiddle_mass_coord", "rh_rfproximal_mass_coord"],
        ["rfdistal", "rfmiddle", "rfprox"],
        ["rh_rftip_mass_coord", "rh_rfmiddle_mass_coord",
         "rh_rfproximal_mass_coord", "rh_rfdistal"],
        ["rftip", "rfmiddle", "rfprox", "rfdistal"],
        ["rh_RFJ0"],
        ["rh_RFJ3", "rh_RFJ4"],
        ["rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4"],
        0.005,
    ),

    "lf" => FingerConfig(
        ["rh_lfdistal_mass_coord", "rh_lfmiddle_mass_coord", "rh_lfproximal_mass_coord"],
        ["lfdistal", "lfmiddle", "lfprox"],
        ["rh_lftip_mass_coord", "rh_lfmiddle_mass_coord",
         "rh_lfproximal_mass_coord", "rh_lfdistal"],
        ["lftip", "lfmiddle", "lfprox", "lfdistal"],
        ["rh_LFJ0"],
        ["rh_LFJ3", "rh_LFJ4", "rh_LFJ5"],
        ["rh_LFJ1", "rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5"],
        0.005,
    ),

    "th" => FingerConfig(
        ["rh_thdistal_mass_coord", "rh_thmiddle_mass_coord"],
        ["thdistal", "thmiddle"],
        ["rh_thtip_mass_coord", "rh_thmiddle_mass_coord","rh_thdistal", "rh_thmiddle"],
        ["thtip", "thmiddle", "thdistal", "thmiddle2"],
        [],                                          # thumb has no coupled J0
        ["rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"],
        ["rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5"],
        0.007
    ),
)



function force_modulation(cylinder_radius, penetration_depth, attraction_stiffness, feedback_stiffness, feedback_damping)

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
    cylinder_position, _ = compute_cylinder_position(vm_robot, cylinder_radius)
    
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

    # IF USING REAL HAND WITH COUPLED JOINTS 
    # for config in values(FINGER_CONFIGS)
    #     for joint_id in config.uncoupled_joints
    #         add_joint_feedback!(vms, joint_id, MISMATCH_DEADZONE, feedback_stiffness, feedback_damping)
    #     end
    #     for joint_id in config.coupled_joints
    #         add_joint_feedback!(vms, joint_id, 2*MISMATCH_DEADZONE, feedback_stiffness, feedback_damping)
    #     end
    # end

    # IF USING IDEAL HAND IN SIMULATION
    for cfg in values(FINGER_CONFIGS)
        for joint_id in cfg.joints
            add_joint_feedback!(vms, joint_id, MISMATCH_DEADZONE, feedback_stiffness, feedback_damping)
        end
    end



    # ---------------- TEST : CONTACT DETECTION COORDINATES ---------------------
    # Radial penetration
    for finger in keys(FINGER_CONFIGS)
        for (frame_id, name) in zip(FINGER_CONFIGS[finger].attracted_frames, FINGER_CONFIGS[finger].attracted_frames_names)
            add_coordinate!(vms, CoordDifference(".robot.$frame_id", "$finger cylinder position");          id="robot $name cylinder diff")
            add_coordinate!(vms, CoordSlice("robot $name cylinder diff", SVector(2, 3));           id="robot $name planar error")
            add_coordinate!(vms, CoordNorm("robot $name planar error");                            id="robot $name planar error norm")
            add_coordinate!(vms, CoordDifference("$name planar error norm", "robot $name planar error norm"); id="$name radial penetration")
        end
    end

    # virtual/real joint connection for contact detection in joint space (decoupling coupled joints)
    # already done in the feedback connection section but should be done even with the real hand for contact detection

    # for joint_id in ("rh_FFJ1", "rh_FFJ2", "rh_MFJ1", "rh_MFJ2", "rh_RFJ1", "rh_RFJ2", "rh_LFJ1", "rh_LFJ2")
    #      add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord"); id="$(joint_id) coord diff")
    # end

    function f_setup(cache)

        #CONTACT DETECTION COORDINATES
        penetration_dict = Dict{String, Any}()
        real_robot_radial_pos_dict = Dict{String, Any}()
        feedback_coordID_dict = Dict{String, Any}()
        for finger in keys(FINGER_CONFIGS)
            for name in FINGER_CONFIGS[finger].attracted_frames_names
                penetration_dict[name] = get_compiled_coordID(cache, "$name radial penetration")
                real_robot_radial_pos_dict[name] = get_compiled_coordID(cache, "robot $name planar error norm")
            end
            for joint in FINGER_CONFIGS[finger].joints
                feedback_coordID_dict[joint] = get_compiled_coordID(cache, "$(joint) coord diff")
            end
        end

        contact_detection_args = penetration_dict, real_robot_radial_pos_dict, feedback_coordID_dict

        #VIRTUAL CYLINDER COORDINATES : RADIUS AND POSITION
        radius_joints_dict           = Dict{String, Any}() # joint IDs of the rigid joints controlling the attracting cylinder radius 
        root_joints_dict             = Dict{String, Any}() # joint IDs of the root joints controlling the attracting cylinder position
        cylinder_position_coord_dict = Dict{String, Any}() # coord IDs of the fixed point controlling the repulsive cylinder position
        cylinder_radius_coord_dict   = Dict{String, Any}() # coord IDs of the spring rest length controlling the repulsive cylinder radius
        damper_component_dict        = Dict{String, Any}() # component ID of the damper of the repulsive cylinder (cannot be modulated with coord)

        for (finger, cfg) in FINGER_CONFIGS
            for frame in cfg.attracted_frames_names
                radius_joints_dict[frame]      = get_compiled_jointID(cache, ".virtual_mechanism.fixed_joint_$(frame)")
                root_joints_dict[frame]        = get_compiled_jointID(cache, ".virtual_mechanism.root_joint_$(frame)")
            end

            for frame in cfg.repulsed_frames_names
                damper_component_dict[frame]        = get_compiled_componentID(cache, "$(frame) cylinder damper")
            end

            cylinder_radius_coord_dict[finger]   = get_compiled_coordID(cache, "$(finger) cylinder radius")
            cylinder_position_coord_dict[finger] = get_compiled_coordID(cache, "$(finger) cylinder position")
        end

        virtual_object_args = radius_joints_dict, root_joints_dict, cylinder_radius_coord_dict, cylinder_position_coord_dict, damper_component_dict

        # ATTRACTION COORDINATES : SPRING STIFFNESSES
        attraction_spring_component_dict = Dict{String, Any}()
        attraction_coordID_dict = Dict{String, Any}()
        for cfg in values(FINGER_CONFIGS)
            for name in cfg.attracted_frames_names
                attraction_spring_component_dict[name] = get_compiled_componentID(cache, "ee $(name) spring")
                attraction_coordID_dict[name] = get_compiled_coordID(cache, "ee $(name) diff")
            end
        end

        attraction_spring_args = attraction_spring_component_dict, attraction_coordID_dict

        return contact_detection_args, virtual_object_args, attraction_spring_args
    end

    finger_states = Dict(
        name => FingerModulationState(cylinder_radius, 
                                    length(FINGER_CONFIGS[name].attracted_frames_names))
        for name in keys(FINGER_CONFIGS)
    )

    function detect_contact_task_space(finger, cache, penetration_dict, real_robot_radial_pos_dict)
        state = finger_states[finger]
        cfg   = FINGER_CONFIGS[finger]

        contact = false
        for (i, name) in enumerate(cfg.attracted_frames_names)
            penetration       = only(configuration(cache, penetration_dict[name]))
            radial_accel      = only(acceleration(cache, real_robot_radial_pos_dict[name]))
            radial_velocity   = only(velocity(cache, real_robot_radial_pos_dict[name]))

            # HYSTERESIS on acceleration : contact can be detected only if the hand is closing and gets stopped
            if state.accel_hysteresis[i] == 0 && radial_accel < -5e-5
                state.accel_hysteresis[i] = -1
            elseif state.accel_hysteresis[i] == -1 && radial_accel > 5e-7
                state.accel_hysteresis[i] = 1
            elseif state.accel_hysteresis[i] == 1 && radial_accel < -5e-5
                state.accel_hysteresis[i] = -1
            end

            if state.accel_hysteresis[i] == 1 && abs(radial_velocity) < 0.004 && penetration < -0.005
                contact = true
            end
        end

        return contact
    end

    function detect_contact_joint_space(finger, cache, feedback_coordID_dict, real_robot_radial_pos_dict)
        cfg = FINGER_CONFIGS[finger]

        #when using non-ideal hand, coupled and uncoupled joints should be treated separately with different deadzones
        contact = any(cfg.joints) do joint
            abs(only(configuration(cache, feedback_coordID_dict[joint]))) > MISMATCH_DEADZONE
        end

        return contact
    end

    last_t = 0.0

    function f_control(cache, t, args, extra)
        contact_detection_args, virtual_object_args, attraction_spring_args = args
        penetration_dict, real_robot_radial_pos_dict, feedback_coordID_dict = contact_detection_args
        radius_joints_dict, root_joints_dict, cylinder_radius_coord_dict, cylinder_position_coord_dict, damper_component_dict = virtual_object_args
        attraction_spring_component_dict, attraction_coordID_dict = attraction_spring_args

        for (finger, cfg) in FINGER_CONFIGS
            state = finger_states[finger]
            state.contact_detected && continue

            contact = detect_contact_task_space(finger, cache, penetration_dict, real_robot_radial_pos_dict)
            #contact = detect_contact_joint_space(finger, cache , feedback_coordID_dict, real_robot_radial_pos_dict)

            if contact
                if state.contact_detection_time == 0.0
                    state.contact_detection_time = t 

                elseif t - state.contact_detection_time > 0.2
                    state.real_object_radius = minimum((only(configuration(cache, real_robot_radial_pos_dict[n])) for n in cfg.attracted_frames_names)) - cfg.finger_width
                    state.contact_detected = true
                    state.radius_modulation = false
                    @info "Contact detected for $(finger) at r = $(round(state.real_object_radius*1000, digits=1)) mm"
                    # CONTACT IS DETECTED : place the virtual object within the real object and adapt stiffnesses accordingly

                    # the position of the virtual object should be centered on the one of the real object 
                    update_cylinder_position(finger, cache, shadow_robot, state.real_object_radius, root_joints_dict, cylinder_position_coord_dict)
                    # the radius of the virtual object should be within the one of the real object
                    state.virtual_object_radius = state.real_object_radius - penetration_depth
                    update_cylinder_radius(finger, cache, state.virtual_object_radius, radius_joints_dict, cylinder_radius_coord_dict, damper_component_dict)
                    # the attractive stiffness should be unified to a single value for all fingers 
                    for frame in FINGER_CONFIGS[finger].attracted_frames_names 
                        cache[attraction_spring_component_dict[frame]] = remake(cache[attraction_spring_component_dict[frame]];
                            stiffness = SMatrix{3,3}(attraction_stiffness * I))
                    end
                end
            else
                state.contact_detection_time = 0.0   # reset timer when contact lost

                if !state.radius_modulation 
                    #check if equilibrium is reached
                    velocity_equilibrium = all(cfg.attracted_frames_names) do point
                        abs(only(velocity(cache, real_robot_radial_pos_dict[point]))) < 0.001
                    end
                    position_equilibrium = all(cfg.attracted_frames_names) do point
                        norm(only(configuration(cache, penetration_dict[point]))) < 0.005
                    end
                    # latch for 0.5s
                    if velocity_equilibrium && position_equilibrium 
                        if state.equilibrium_detection_time == 0.0
                            state.equilibrium_detection_time = t
                        elseif t - state.equilibrium_detection_time > 0.5
                            state.radius_modulation = true
                            @info "Equilibrium reached for $(finger) without contact, starting radius modulation"
                        end
                    else 
                        state.equilibrium_detection_time = 0.0
                    end
                else 
                    state.virtual_object_radius = state.virtual_object_radius - 0.0 * (t - last_t)
                    update_cylinder_position(finger, cache, shadow_robot, state.virtual_object_radius, root_joints_dict, cylinder_position_coord_dict)
                    update_cylinder_radius(finger, cache, state.virtual_object_radius, radius_joints_dict, cylinder_radius_coord_dict, damper_component_dict)
                end
            end
        end

        last_t = t
    end

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
function update_cylinder_position(finger, cache, robot, new_radius, root_jointID, cylinder_position_coord_dict)

    cylinder_pos, kcache = compute_cylinder_position(robot, new_radius)

    #change the root joint position of each attracted frame
    for i in 1:length(FINGER_CONFIGS[finger].attracted_frames)
        frame_pos = configuration(kcache, get_compiled_coordID(kcache, FINGER_CONFIGS[finger].attracted_frames[i]))
        cache[root_jointID[FINGER_CONFIGS[finger].attracted_frames_names[i]]] = remake(
            cache[root_jointID[FINGER_CONFIGS[finger].attracted_frames_names[i]]];
            jointData = Rigid(Transform(SVector(frame_pos[1], cylinder_pos[2], cylinder_pos[3])))
        )
    end

    # update the global radius for the collision model
    cache[cylinder_position_coord_dict[finger]] = remake(
        cache[cylinder_position_coord_dict[finger]];
        coord_data = ConstCoord(cylinder_pos)
    )
end

function update_cylinder_radius(finger, cache, new_radius, radius_joints, cylinder_radius_coord_dict, virtual_object_damper_component_dict)

    for frame in FINGER_CONFIGS[finger].attracted_frames_names 
        cache[radius_joints[frame]] = remake(cache[radius_joints[frame]];
            jointData = Rigid(Transform(SVector(0.0, 0.0, new_radius))))
    end

    cache[cylinder_radius_coord_dict[finger]] = remake(cache[cylinder_radius_coord_dict[finger]];
            coord_data = ConstCoord(new_radius))

    for frame in FINGER_CONFIGS[finger].repulsed_frames_names
        cache[virtual_object_damper_component_dict[frame]] = remake(cache[virtual_object_damper_component_dict[frame]];
            bounds = (0.0, 1.05*new_radius))
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



# function update_finger_state!(state, finger, cache, t, attraction_coordID, feedback_coordID_uncoupled, feedback_coordID_coupled)

#     cfg = FINGER_CONFIGS[finger]

#     # ── 1. Virtual contact ────────────────────────────────────────────────────
#     state.equilibrium = any(cfg.attracted_frames_names) do point
#         norm(configuration(cache, attraction_coordID[point])) < 0.005
#     end

#     # ── 2. Real contact ───────────────────────────────────────────────────────
#     uncoupled_contact = any(cfg.uncoupled_joints) do joint
#         abs(only(configuration(cache, feedback_coordID_uncoupled[joint]))) > MISMATCH_DEADZONE
#     end
#     coupled_contact = any(cfg.coupled_joints) do joint
#         abs(only(configuration(cache, feedback_coordID_coupled[joint]))) > 2 * MISMATCH_DEADZONE
#     end
#     state.contact_detected = uncoupled_contact || coupled_contact

#     # ── 3. Activation: sustained virtual contact without real contact ─────────
#     if !state.modulation_activated && !state.modulation_stopped
#         if state.equilibrium && !state.contact_detected
#             if state.activation_time == 0.0
#                 state.activation_time = t
#             elseif t - state.activation_time > 0.2
#                 state.modulation_activated = true
#                 @info "Radius modulation activated for finger $(finger)"
#             end
#         else
#             state.activation_time = 0.0
#         end
#     end
# end

# function apply_radius_modulation!(finger, state, cache, t, m, kcache,
#                                    radius_joints, root_joints,
#                                    cylinder_radius_coord_dict,
#                                    cylinder_position_coord_dict,
#                                    damper_component_dict, last_t)

#     (!state.modulation_activated || state.modulation_stopped) && return

#     # ── 4. Decrement radius ───────────────────────────────────────────────────
#     dt = t - last_t
#     state.radius = max(state.radius - 0.002 * dt, 0.005)

#     update_cylinder_radius(finger, cache, state.radius, radius_joints, cylinder_radius_coord_dict, damper_component_dict)

#     update_cylinder_position(finger, m, cache, kcache, state.radius,root_joints, cylinder_position_coord_dict)

#     # ── 5. Stopping: sustained real contact ───────────────────────────────────
#     if state.contact_detected
#         if state.stopping_time == 0.0
#             state.stopping_time = t
#         elseif t - state.stopping_time > 0.2
#             state.modulation_activated = false
#             state.modulation_stopped   = true
#             @info "$(finger) modulation stopped at r = $(round(state.radius*1000, digits=1)) mm"
#         end
#     else
#         state.stopping_time = 0.0
#     end
# end

