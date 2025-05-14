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