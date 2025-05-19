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

function display_frame(m::CompiledMechanism, ls::LScene, frame_name::String)
    # Get the compiled frame ID
    frame_id = get_compiled_frameID(m, frame_name)
    kcache = new_kinematics_cache(m)
    # Get the transformation of the frame
    transform = get_transform(kcache, frame_id)

    # Convert rotor to rotation matrix
    R_matrix = rotor_to_matrix(transform.rotor)

    # Extract position as a Point{3, Float32}
    p = Point{3, Float32}(transform.origin[1], transform.origin[2], transform.origin[3])

    # Define axis colors
    axis_colors = (:red, :green, :blue)

    # Extract frame axes and scale them
    scale = 0.1
    axis_vectors = scale .* [R_matrix[:, i] for i in 1:3]

    # Plot the frame axes
    for (i, color) in enumerate(axis_colors)
        a = Vec{3, Float64}(axis_vectors[i])
        arrows!(ls, [p], [a]; arrowsize=0.01, color=color)
    end
end

function display_transform(ls::LScene, transform::Matrix{Float64})
    @assert size(transform) == (4, 4) "Transform matrix must be 4×4"

    # Extract rotation matrix (top-left 3×3 submatrix)
    R_matrix = transform[1:3, 1:3]

    # Extract translation vector (last column, first 3 elements)
    p = Point{3, Float32}(transform[1, 4], transform[2, 4], transform[3, 4])

    # Define axis colors
    axis_colors = (:red, :green, :blue)

    # Scale axes for visualization
    scale = 0.1
    axis_vectors = scale .* [R_matrix[:, i] for i in 1:3]

    # Plot the frame axes
    for (i, color) in enumerate(axis_colors)
        a = Vec{3, Float64}(axis_vectors[i])
        arrows!(ls, [p], [a]; arrowsize=0.01, color=color)
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
