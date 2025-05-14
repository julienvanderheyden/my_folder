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