function rotor_to_matrix(R)
    s, (x, y, z) = R.scalar, R.bivector # Extract scalar and vector part
    
    return [
        1 - 2(y^2 + z^2)   2(x*y - s*z)   2(x*z + s*y)
        2(x*y + s*z)   1 - 2(x^2 + z^2)   2(y*z - s*x)
        2(x*z - s*y)   2(y*z + s*x)   1 - 2(x^2 + y^2)
    ]
end

function struct_fields_and_types(T::Type)
    for (name, type) in zip(fieldnames(T), fieldtypes(T))
        println(name, "::", type)
    end
end



function generate_q_init(vms_compiled; ff=false, mf=false, rf=false, lf=false)
    q_init = zero_q(vms_compiled.robot)

    if ff
        q_init[3] = 1.4
        q_init[4] = 1.57
        q_init[5] = 1.57
    end

    if mf
        q_init[8] = 1.4
        q_init[9] = 1.57
        q_init[10] = 1.57
    end

    if rf
        q_init[12] = 1.4
        q_init[13] = 1.57
        q_init[14] = 1.57
    end

    if lf
        q_init[17] = 1.4
        q_init[18] = 1.57
        q_init[19] = 1.57
    end

    return q_init
end

function generate_stiffnesses_linear_scaling(base::Float64, alpha::Float64, beta::Float64)
    num_fingers = 5  # little to thumb
    num_phalanges = 3  # distal to proximal

    stiffnesses = Float64[]

    for finger in 0:num_fingers-1
        for phalanx in 0:num_phalanges-1
            # Scale factors normalized in [0, 1]
            finger_scale = finger / (num_fingers - 1)   # from 0 (little) to 1 (thumb)
            phalanx_scale = phalanx / (num_phalanges - 1)  # from 0 (distal) to 1 (proximal)

            # Compute stiffness with uniform scaling
            stiffness = base * (1 + alpha * phalanx_scale) * (1 + beta * finger_scale)
            push!(stiffnesses, max(stiffness, 0.0))  # Ensure non-negative stiffness
        end
    end

    return stiffnesses
end

function generate_stiffnesses_geometric_scaling(base::Float64, alpha::Float64, beta::Float64)

    """
        alpha > 1 means that the proximal stiffness is higher than the distal stiffness
        alpha < 1 means that the distal stiffness is higher than the proximal stiffness

        beta > 1 means that the little stiffness is higher than the thumb stiffness
        beta < 1 means that the thumb stiffness is higher than the little stiffness
    """

    num_fingers = 5  # little to thumb
    num_phalanges = 3  # distal to proximal

    stiffnesses = Float64[]

    for finger in 0:num_fingers-1
        for phalanx in 0:num_phalanges-1
            # Scale factors normalized in [0, 1]
            finger_scale  = finger / (num_fingers - 1) # from 0 (little) to 1 (thumb)
            phalanx_scale = phalanx / (num_phalanges - 1) # from 0 (distal) to 1 (proximal)

            # Compute stiffness with geometric interpolation 
            phalanx_factor = alpha^phalanx_scale     # from 1 to alpha
            finger_factor  = beta^finger_scale      # from 1 to beta

            stiffness = base * phalanx_factor * finger_factor
            push!(stiffnesses, stiffness)
        end
    end

    return stiffnesses
end

function circle_center_tangent_to_lines(p11, p12, p21, p22, r)
    # Convert to Float64 vectors
    p11 = Vector{Float64}(p11)
    p12 = Vector{Float64}(p12)
    p21 = Vector{Float64}(p21)
    p22 = Vector{Float64}(p22)

    # Direction vectors (initially arbitrary)
    d1 = p12 - p11
    d2 = p22 - p21

    # Solve for intersection point of the lines: p11 + t1*d1 = p21 + t2*d2
    A = hcat(d1, -d2)
    b = p21 - p11
    if rank(A) < 2
        error("Lines are parallel or coincident")
    end
    ts = A \ b
    P = p11 + ts[1] * d1  # Intersection point

    # Redefine directions starting from P (to ensure correct orientation)
    d1 = normalize(p12 - P)
    d2 = normalize(p22 - P)

    # Internal angle bisector direction
    bisector = normalize(d1 + d2)

    # Angle between the two direction vectors
    cosθ = clamp(dot(d1, d2), -1.0, 1.0)
    θ = acos(cosθ)

    # Distance from intersection to circle center
    d = r / sin(θ / 2)

    # Circle center
    C = P + d * bisector

    return C
end

function build_robot(urdf_path; joint_limiting = false, gravity_compensation = false)
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

    if gravity_compensation
        add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
    end   

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

function add_joint_feedback!(vms, joint_id, deadzone, feedback_stiffness, feedback_damping)
    add_coordinate!(vms, CoordDifference(".robot.$(joint_id)_coord", ".virtual_mechanism.$(joint_id)_coord"); id="$joint_id coord diff")
    add_deadzone_springs!(vms, feedback_stiffness, (-deadzone, deadzone), "$joint_id coord diff")
    add_component!(vms, RectifiedDamper(feedback_damping, "$joint_id coord diff", ( 0.9*deadzone,  1.1*deadzone), false, false); id="$joint_id feedback damper 1")
    add_component!(vms, RectifiedDamper(feedback_damping, "$joint_id coord diff", (-1.1*deadzone, -0.9*deadzone), true,  false); id="$joint_id feedback damper 2")
end
