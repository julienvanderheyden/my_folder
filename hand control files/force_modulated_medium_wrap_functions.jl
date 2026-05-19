function build_robot(urdf_path; joint_limiting = false)
    # PARSE URDF
    cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
    robot = parseURDF(urdf_path, cfg)

    # ADD "JOINT SUBSPACE" COORDINATES
    joint_limits = cfg.joint_limits
    for joint_id in keys(joints(robot))
        # "limits" are here used simply to identify the joints that actually move with respect to the fixed joints
        limits = joint_limits[joint_id]
        isnothing(limits) && continue
        add_coordinate!(robot, JointSubspace(joint_id);  id="$(joint_id)_coord")

        # JOINT LIMITS AND DAMPING
        if joint_limiting && !isnothing(limits.lower) && !isnothing(limits.upper)
            add_deadzone_springs!(robot, 0.01, (limits.lower+0.0, limits.upper-0.0), "$(joint_id)_coord")
            add_component!(robot, LinearDamper(0.0001, "$(joint_id)_coord"); id="$(joint_id)_damper")
        end
    end

    # ADD COUPLED JOINT COORDINATES
    add_coordinate!(robot, CoordSum("rh_FFJ1_coord", "rh_FFJ2_coord"); id="rh_FFJ0_coord")
    add_coordinate!(robot, CoordSum("rh_MFJ1_coord", "rh_MFJ2_coord"); id="rh_MFJ0_coord")
    add_coordinate!(robot, CoordSum("rh_RFJ1_coord", "rh_RFJ2_coord"); id="rh_RFJ0_coord")
    add_coordinate!(robot, CoordSum("rh_LFJ1_coord", "rh_LFJ2_coord"); id="rh_LFJ0_coord") 
    
    # ADD PALM COORDINATE
    add_coordinate!(robot, FramePoint("rh_palm", SVector(0. , 0., 0.07)); id="rh_palm2")

    return robot
end

function compute_cylinder_position(robot, cylinder_radius)
    m = compile(robot)
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
    return cylinder_position
end

function build_cylinder_virtual_object(robot, finger_cfg, cylinder_position, cylinder_radius)
    
    m = compile(robot)
    kcache = new_kinematics_cache(m)  
    medium_wrap_preshape = zeros(24)
    medium_wrap_preshape[21] = 1.2 # thumb extended
    kinematics!(kcache, 0.0, medium_wrap_preshape) 

    for (finger, config) in finger_cfg
        for (frame_id, name) in zip(config.attracted_frames, config.attracted_frames_names)

            # Build the cylinder surface joints mechanism
            frame_pos = configuration(kcache, get_compiled_coordID(kcache, frame_id))
            center_pos = SVector(frame_pos[1], cylinder_position[2], cylinder_position[3])

            add_frame!(robot; id="center_frame_$name")
            add_joint!(robot, Rigid(Transform(center_pos)); parent=root_frame(robot), child="center_frame_$name", id="root_joint_$name")

            add_frame!(robot; id="prism_frame_$name")
            add_joint!(robot, Prismatic(SVector(1.0, 0.0, 0.0)); parent="center_frame_$name", child="prism_frame_$name", id="prism_joint_$name")

            add_frame!(robot; id="revo_frame_$name")
            add_joint!(robot, Revolute(SVector(1.0, 0.0, 0.0)); parent="prism_frame_$name", child="revo_frame_$name", id="revo_joint_$name")

            add_frame!(robot; id="ee_frame_$name")
            add_joint!(robot, Rigid(Transform(SVector(0.0, 0.0, cylinder_radius))); parent="revo_frame_$name", child="ee_frame_$name", id="fixed_joint_$name")

            # Add components : mass, damping and comeback spring
            add_coordinate!(robot, FrameOrigin("ee_frame_$name"); id="$name ee position")
            add_component!(robot, PointMass(0.01, "$name ee position");  id="$name ee mass")

            add_coordinate!(robot, JointSubspace("prism_joint_$name"); id="prism_joint_$name")
            add_component!(robot, LinearDamper(0.1, "prism_joint_$name"); id="prism_joint_$(name)_damper")

            add_coordinate!(robot, FrameOrigin("center_frame_$name"); id="center_frame_$name")
            add_coordinate!(robot, FrameOrigin("prism_frame_$name"); id="prism_frame_$name")
            add_coordinate!(robot, CoordDifference("center_frame_$name", "prism_frame_$name"); id="$(name)_prismatic_error")
            add_component!(robot, LinearSpring(SMatrix{3, 3}(0.1 * I), "$(name)_prismatic_error"); id="$(name)_comeback_spring")
        end
    end
    add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
end

function frame_stiffness(base, finger_idx, n_fingers, phalanx_idx, n_phalanges, phalanx_factor, finger_factor)
    finger_scale  = (finger_idx  - 1) / (n_fingers   - 1) # from 0 (little) to 1 (thumb)
    phalanx_scale = (phalanx_idx - 1) / max(n_phalanges - 1, 1) # from 0 (distal) to 1 (proximal), avoid division by zero if n_phalanges=1
    return base * finger_factor^finger_scale * phalanx_factor^phalanx_scale
end

function connect_virtual_hand_object(vms, finger_cfg, base_stiffness, phalanx_scaling_factor, finger_scaling_factor, base_damping, damping_decay_rate, exponential_damping_coeff)
    
    finger_order = ["lf", "rf", "mf", "ff", "th"]
    
    for (finger_idx, finger) in enumerate(finger_order)
        config = FINGER_CONFIGS[finger]
        n_phalanges = length(config.attracted_frames)

        for (phalanx_idx, (frame_id, name)) in enumerate(zip(config.attracted_frames, config.attracted_frames_names))
            #geometric scaling stiffness
            stiffness = frame_stiffness(base_stiffness, finger_idx, length(finger_order), phalanx_idx, n_phalanges, phalanx_scaling_factor, finger_scaling_factor)

            add_coordinate!(vms, CoordDifference(".virtual_mechanism.$name ee position", ".virtual_mechanism.$frame_id"); id="ee $name diff")
            add_component!(vms, LinearSpring(SMatrix{3,3}(stiffness * I),                      "ee $name diff"); id="ee $name spring")
            add_component!(vms, LinearDamper(SMatrix{3,3}(base_damping * I),                   "ee $name diff"); id="ee $name damper")
            add_component!(vms, ExponentialDamper(SMatrix{3,3}(exponential_damping_coeff * I), "ee $name diff", damping_decay_rate); id="ee $name exp damper")
        end
    end

    # ADDITIONAL COMPONENTS 
    add_component!(vms, LinearDamper(SMatrix{3, 3}(10.0*I), "ee thmiddle diff"); id = "thmiddle massive damper")  
    #lightly constraint some joints to avoid unwanted motions 
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ1_coord"); id = "wr j1 angular spring")
    add_component!(vms, LinearSpring(0.01, ".virtual_mechanism.rh_WRJ2_coord"); id = "wr j2 angular spring")

end

function add_repulsion!(vms, frame, name, cylinder_radius, cylinder_position)
    add_coordinate!(vms, ConstCoord(cylinder_radius);   id="$name cylinder radius")
    add_coordinate!(vms, ConstCoord(cylinder_position); id="$name cylinder position")

    add_coordinate!(vms, CoordDifference(frame, "$name cylinder position");          id="$name cylinder diff")
    add_coordinate!(vms, CoordSlice("$name cylinder diff", SVector(2, 3));           id="$name planar error")
    add_coordinate!(vms, CoordNorm("$name planar error");                            id="$name planar error norm")
    add_coordinate!(vms, CoordDifference("$name planar error norm", "$name cylinder radius"); id="shifted $name cylinder error")

    add_component!(vms, ReLUSpring(5.0, "shifted $name cylinder error", true);                              id="$name cylinder repulsive spring")
    add_component!(vms, RectifiedDamper(5.0, "$name planar error norm", (0.0, 1.05*cylinder_radius), true, false); id="$name cylinder damper")
end

function build_cylinder_collision_model(vms, finger_cfg, cylinder_radius, cylinder_position)
    # Special frames not belonging to any finger
    EXTRA_REPULSED_FRAMES = [
        (".virtual_mechanism.rh_palm_mass_coord", "palm"),
        (".virtual_mechanism.rh_palm2",           "palm2"),
    ]

    for config in values(finger_cfg)
        for (frame_id, name) in zip(config.repulsed_frames, config.repulsed_frames_names)
            add_repulsion!(vms, ".virtual_mechanism.$frame_id", name, cylinder_radius, cylinder_position)
        end
    end

    for (frame_id, name) in EXTRA_REPULSED_FRAMES
        add_repulsion!(vms, frame_id, name, cylinder_radius, cylinder_position)
    end
end

function add_joint_feedback!(vms, joint_id, deadzone, feedback_stiffness, feedback_damping)
    add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord"); id="$joint_id coord diff")
    add_deadzone_springs!(vms, feedback_stiffness, (-deadzone, deadzone), "$joint_id coord diff")
    add_component!(vms, RectifiedDamper(feedback_damping, "$joint_id coord diff", ( 0.9*deadzone,  1.1*deadzone), false, false); id="$joint_id feedback damper 1")
    add_component!(vms, RectifiedDamper(feedback_damping, "$joint_id coord diff", (-1.1*deadzone, -0.9*deadzone), true,  false); id="$joint_id feedback damper 2")
end
