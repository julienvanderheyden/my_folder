# ON A LAISSE TOMBER CETTE IDEE D'AVOIR LE ROBOT EN VIRTUAL MECHANISM. J'AI PAS SUPER BIEN COMPRIS POURQUOI MAIS DANS TOUS LES CAS POUR 
# LA SUITE IL VA FALLOIR FAIRE LE TAFF DONC ON LAISSE TOMBER CE FICHIER NON TERMINE POUR L'INSTANT 


using Revise
using GeometryBasics: Vec3f, Point3f
using LinearAlgebra
using StaticArrays
using VMRobotControl
using VMRobotControl.Splines: CubicSpline
using DifferentialEquations
using MeshIO
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
include(joinpath(module_path, "ros/ROS.jl"))

###### URDF PARSING #####

using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end

println("parsing URDF...")

cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
shadow_robot = parseURDF(joinpath(module_path, "URDFs/sr_description/sr_hand_vm_compatible.urdf"), cfg)

println("URDF parsed !")

##### BUILDING A "FAKE" ROBOT #####

robot = Mechanism{Float64}("FakeRobot")

##### BUILDING SHADOW ROBOT #####

println("Building the robot (as a virtual mechanism ) ...")

# Gravity Compensation

add_gravity_compensation!(shadow_robot, VMRobotControl.DEFAULT_GRAVITY)

# Joint Damping and Limit Springs

joint_limits = cfg.joint_limits

for joint_id in keys(joints(shadow_robot))
    limits = joint_limits[joint_id]
    isnothing(limits) && continue
    add_coordinate!(shadow_robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
    @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
    add_deadzone_springs!(shadow_robot, 50.0, (limits.lower+0.1, limits.upper-0.1), "$(joint_id)_coord")
    add_component!(shadow_robot, LinearDamper(0.01, "$(joint_id)_coord"); id="$(joint_id)_damper")
end

# Deep copy of the robot 
vm = deepcopy(shadow_robot)

println("Robot built ! ")

##### VIRTUAL MECHANISM #####

println("Building virtual mechanisms ... ")

# Building the sphere
root_vm = root_frame(vm)
F1 = add_frame!(vm; id="sphere_L1_frame")
F2 = add_frame!(vm; id="sphere_L2_frame")
F_sphere = add_frame!(vm; id="sphere_frame")

J1 = Prismatic(SVector(1. ,0. ,0. ))
J2 = Prismatic(SVector(0. ,1. ,0. ))
J3 = Prismatic(SVector(0. ,0. ,1. ))

add_joint!(vm, J1; parent=root_vm, child=F1, id="J1")
add_joint!(vm, J2; parent=F1, child=F2, id="J2")
add_joint!(vm, J3; parent=F2, child=F_sphere, id="J3")

add_coordinate!(vm, FrameOrigin(F_sphere); id = "sphere position")
add_component!(vm, PointMass(1.0, "sphere position"); id="sphere mass")
add_gravity_compensation!(vm, VMRobotControl.DEFAULT_GRAVITY)

add_coordinate!(vm, JointSubspace("J1"); id="J1")
add_coordinate!(vm, JointSubspace("J2"); id="J2")
add_coordinate!(vm, JointSubspace("J3"); id="J3")

# Building the track 

start = SVector(0.0, -0.2, 0.32)

Nk = 6
L = 0.11
y0 = start[2]
y_vector = Vector(LinRange(y0, y0+L, Nk))
track_points = Matrix{Float64}(undef, 0, 3)
for i = 1:Nk
    global track_points
    track_points = vcat(
        track_points,
        hcat(start[1], y_vector[i], start[3])
    )
end

track_spline = CubicSpline(track_points)

cart_frame = add_frame!(vm, "Cart")
add_joint!(vm, Rail(track_spline, zero(Transform{Float64}));
        parent=root_frame(vm), child=cart_frame,        id="RailJoint")

add_coordinate!(vm, JointSubspace("RailJoint");         id="CartDistance")
add_coordinate!(vm, FrameOrigin(cart_frame);            id="Target position")
add_component!(vm, LinearInerter(1.0, "Target position");  id="CartInertance") # Cart mass
add_component!(vm, LinearDamper(100.0, "Target position"); id="CartDamper")

# Adding springs and dampers
vms = VirtualMechanismSystem("myShadowVMS", robot, vm)
root = root_frame(vms.robot)

# FINGER SPACING --> PRE-SHAPING
add_coordinate!(vms, CoordDifference(".robot.rh_ffmiddle_mass_coord", ".robot.rh_mfmiddle_mass_coord"); id="ff mf middle diff")
add_coordinate!(vms, CoordDifference(".robot.rh_mfmiddle_mass_coord", ".robot.rh_rfmiddle_mass_coord"); id="mf rf middle diff")
add_coordinate!(vms, CoordDifference(".robot.rh_rfmiddle_mass_coord", ".robot.rh_lfmiddle_mass_coord"); id="rf lf middle diff")
add_coordinate!(vms, CoordDifference(".robot.rh_ffmiddle_mass_coord", ".robot.rh_thmiddle_mass_coord"); id="ff th middle diff")

add_coordinate!(vms, CoordSlice("ff mf middle diff", SVector(1,3)); id = "ff mf middle diff xz")
add_coordinate!(vms, CoordSlice("mf rf middle diff", SVector(1,3)); id = "mf rf middle diff xz")
add_coordinate!(vms, CoordSlice("rf lf middle diff", SVector(1,3)); id = "rf lf middle diff xz")

add_coordinate!(vms, CoordNorm("ff mf middle diff xz"); id="ff mf middle planar norm")
add_coordinate!(vms, CoordNorm("mf rf middle diff xz"); id="mf rf middle planar norm")
add_coordinate!(vms, CoordNorm("rf lf middle diff xz"); id="rf lf middle planar norm")
add_coordinate!(vms, CoordNorm("ff th middle diff"); id="ff th middle norm")

fingers_spacing = 0.03
add_coordinate!(vms, ConstCoord(fingers_spacing); id = "fingers spacing")
thumb_spacing = 0.1
add_coordinate!(vms, ConstCoord(thumb_spacing); id = "thumb spacing")

add_coordinate!(vms, CoordDifference("ff mf middle planar norm", "fingers spacing"); id = "ff mf middle planar error")
add_coordinate!(vms, CoordDifference("mf rf middle planar norm", "fingers spacing"); id = "mf rf middle planar error")
add_coordinate!(vms, CoordDifference("rf lf middle planar norm", "fingers spacing"); id = "rf lf middle planar error")
add_coordinate!(vms, CoordDifference("ff th middle norm", "thumb spacing"); id = "ff th middle error")


K = 100.0
D = 10.0 

add_component!(vms, LinearSpring(K, "ff mf middle planar error");     id="ff mf middle spring")
add_component!(vms, LinearDamper(D, "ff mf middle planar error");     id="ff mf middle damper")
add_component!(vms, LinearSpring(K, "mf rf middle planar error");     id="mf rf middle spring")
add_component!(vms, LinearDamper(D, "mf rf middle planar error");     id="mf rf middle damper")
add_component!(vms, LinearSpring(K, "rf lf middle planar error");     id="rf lf middle spring")
add_component!(vms, LinearDamper(D, "rf lf middle planar error");     id="rf lf middle damper")
add_component!(vms, LinearSpring(K, "ff th middle error");     id="ff th middle spring")
add_component!(vms, LinearDamper(D, "ff th middle error");     id="ff th middle damper")

#add_component!(vms, LinearSpring(100.0, ".robot.rh_LFJ5_coord"); id = "lf j5 angular spring")

# TARGET REACHING -->
#add_coordinate!(vms,  ConstCoord(SVector(0.0, -0.075, 0.32));  id="Target position")

add_coordinate!(vms, CoordDifference(".robot.rh_fftip_mass_coord", ".virtual_mechanism.Target position"); id="ff position error")
add_coordinate!(vms, CoordDifference(".robot.rh_mftip_mass_coord", ".virtual_mechanism.Target position"); id="mf position error")
add_coordinate!(vms, CoordDifference(".robot.rh_rftip_mass_coord", ".virtual_mechanism.Target position"); id="rf position error")
add_coordinate!(vms, CoordDifference(".robot.rh_lftip_mass_coord", ".virtual_mechanism.Target position"); id="lf position error")
add_coordinate!(vms, CoordDifference(".robot.rh_thtip_mass_coord", ".virtual_mechanism.Target position"); id="th position error")

K_matrix = SMatrix{3, 3}(0., 0., 0., 0., 0., 0., 0., 0., 0.)
D_matrix = SMatrix{3, 3}(0., 0., 0., 0., 0.0, 0., 0., 0., 0.)

add_component!(vms, LinearSpring(K_matrix, "ff position error"); id="ff spring")
add_component!(vms, LinearDamper(D_matrix, "ff position error"); id="ff damper")
add_component!(vms, LinearSpring(K_matrix, "mf position error"); id="mf spring")
add_component!(vms, LinearDamper(D_matrix, "mf position error"); id="mf damper")
add_component!(vms, LinearSpring(K_matrix, "rf position error"); id="rf spring")
add_component!(vms, LinearDamper(D_matrix, "rf position error"); id="rf damper")
add_component!(vms, LinearSpring(K_matrix, "lf position error"); id="lf spring")
add_component!(vms, LinearDamper(D_matrix, "lf position error"); id="lf damper")
add_component!(vms, LinearSpring(K_matrix, "th position error"); id="th spring")
add_component!(vms, LinearDamper(D_matrix, "th position error"); id="th damper")

max_power = 10.0
force_source = ForceSource(SVector(0.0), max_power, ".virtual_mechanism.CartDistance")
add_component!(vms, force_source;   id="Cart force source");

# REPULSIVE BALL 

sphere_radius = 0.035

add_coordinate!(vms, ConstCoord(sphere_radius); id="sphere radius")

add_coordinate!(vms, FramePoint(".robot.rh_palm", SVector(0. , 0., 0.07)); id="second palm point")

repulsed_frames = (".robot.rh_fftip_mass_coord", ".robot.rh_mftip_mass_coord", ".robot.rh_rftip_mass_coord",".robot.rh_lftip_mass_coord" , 
                    ".robot.rh_thtip_mass_coord", ".robot.rh_ffmiddle_mass_coord",".robot.rh_mfmiddle_mass_coord", ".robot.rh_rfmiddle_mass_coord",
                    ".robot.rh_lfmiddle_mass_coord",  ".robot.rh_thmiddle_mass_coord", ".robot.rh_ffproximal_mass_coord", ".robot.rh_mfproximal_mass_coord",
                    ".robot.rh_rfproximal_mass_coord", ".robot.rh_lfproximal_mass_coord", ".robot.rh_thproximal_mass_coord", ".robot.rh_palm_mass_coord", "second palm point")
frames_names = ("fftip", "mftip", "rftip", "lftip", "thtip", "ffmiddle", "mfmiddle", "rfmiddle", "lfmiddle", "thmiddle", "ffprox", 
                "mfprox", "rfprox", "lfprox", "thprox", "palm", "palm2")

for i in 1:length(repulsed_frames)
    frame = repulsed_frames[i]
    add_coordinate!(vms, CoordDifference(frame, ".virtual_mechanism.sphere position") ; id = "$(frames_names[i]) sphere diff" )
    add_coordinate!(vms, CoordNorm("$(frames_names[i]) sphere diff") ; id = "$(frames_names[i]) sphere diff norm")
    add_coordinate!(vms, CoordDifference("$(frames_names[i]) sphere diff norm", "sphere radius"); id = "shifted $(frames_names[i]) sphere diff norm" )

    add_component!(vms, ReLUSpring(1000.0, "shifted $(frames_names[i]) sphere diff norm", true); id="$(frames_names[i]) sphere repulsive spring")
    add_component!(vms, RectifiedDamper(100.0, "$(frames_names[i]) sphere diff norm", (0.0, 1.1*sphere_radius), true, false); id="$(frames_names[i]) sphere damper")
end

println("Virtual Mechanism Built !")

##### F_CONTROL AND F_SETUP ##### 

preshaping_phase = true
grasping_phase = false
flags = (preshaping_phase, grasping_phase)

function f_setup(cache) 
    preshaping_springs = ("ff mf middle spring", "mf rf middle spring", "rf lf middle spring", "ff th middle spring")
    preshaping_dampers = ("ff mf middle damper", "mf rf middle damper", "rf lf middle damper", "ff th middle damper")
    preshaping_springs_ID = []
    preshaping_dampers_ID = []

    for (spring, damper) in zip(preshaping_springs, preshaping_dampers)
        spring_ID = get_compiled_componentID(dcache, spring)
        damper_ID = get_compiled_componentID(dcache, damper)
    
        push!(preshaping_springs_ID, spring_ID)
        push!(preshaping_dampers_ID, damper_ID)
    end

    grasping_springs = ("ff spring", "mf spring", "rf spring", "lf spring", "th spring")
    grasping_dampers = ("ff damper", "mf damper", "rf damper", "lf damper", "th damper")
    grasping_springs_ID = []
    grasping_dampers_ID = []

    for (spring, damper) in zip(grasping_springs, grasping_dampers)
        spring_ID = get_compiled_componentID(dcache, spring)
        damper_ID = get_compiled_componentID(dcache, damper)
    
        push!(grasping_springs_ID, spring_ID)
        push!(grasping_dampers_ID, damper_ID)
    end
    
    return (preshaping_springs_ID , preshaping_dampers_ID, grasping_springs_ID, grasping_dampers_ID, get_compiled_componentID(cache, "Cart force source"), flags)
end

function f_control(cache, t, args, extra)
    preshaping_springs_ID, preshaping_dampers_ID , grasping_springs_ID, grasping_dampers_ID, force_source_id, flags = args
    preshaping_phase, grasping_phase = flags
    
    # Move from pre-shaping to grasping phase
    if t > 3 && preshaping_phase

        #flags update
        preshaping_phase = false
        grasping_phase = true

        # for (spring_id, damper_id) in zip(preshaping_springs_ID, preshaping_dampers_ID)
        #     cache[spring_id] = remake(cache[spring_id]; stiffness = 0.0)
        #     cache[damper_id] = remake(cache[damper_id]; damping = 0.0)
        # end

        #Only removing the long spring between the thumb and the first finger
        cache[preshaping_springs_ID[4]] = remake(cache[preshaping_springs_ID[4]]; stiffness = 0.0)
        cache[preshaping_dampers_ID[4]] = remake(cache[preshaping_dampers_ID[4]]; damping = 0.0)

        #activating the target reaching springs
        new_K_matrix = SMatrix{3, 3}(100., 0., 0., 0., 100., 0., 0., 0., 100.)
        new_D_matrix = SMatrix{3, 3}(30., 0., 0., 0., 30., 0., 0., 0., 30.)

        for (spring_id, damper_id) in zip(grasping_springs_ID, grasping_dampers_ID)
            cache[spring_id] = remake(cache[spring_id]; stiffness = new_K_matrix)
            cache[damper_id] = remake(cache[damper_id]; damping = new_D_matrix)
        end

        #activating the force source
        cache[force_source_id] = remake(cache[force_source_id] ; force_max = SVector(1.0))
    end 

    nothing
end


display("file executed")