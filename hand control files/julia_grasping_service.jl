using RobotOS
@rosimport std_msgs.msg: Int32, String
rostypegen()
using .std_msgs.msg

#include("object_centric_grasping.jl")
include("object_centric_grasping_sensitivity_analysis.jl")

# Publisher to send completion status back
const status_pub = Ref{Publisher}()

function grasp_command_callback(msg)
    try
        # Parse the message - adjust based on your message format
        # Example: "3,0.025,0.001" for lateral pinch with two parameters
        parts = split(msg.data, ",")
        grasp_type = parse(Int, parts[1])

        stiffness_modifier = 0.25
        damping_modifier = 0.0
        
        println("Received grasp command: type=$grasp_type")
        
        # Execute the appropriate grasp
        if grasp_type == 1
            # Medium wrap - expects 1 parameter
            param = parse(Float64, parts[2])
            object_centric_medium_wrap(param, stiffness_modifier, damping_modifier)
        elseif grasp_type == 2
            # Power sphere - expects 1 parameter
            param = parse(Float64, parts[2])
            object_centric_power_sphere(param)
        elseif grasp_type == 3
            # Lateral pinch - expects 2 parameters
            param1 = parse(Float64, parts[2])
            param2 = parse(Float64, parts[3])
            object_centric_lateral_pinch(param1, param2, stiffness_modifier, damping_modifier)
        else
            println("Unknown grasp type: $grasp_type")
            # Send error status
            publish(status_pub[], Int32Msg(-1))
            return
        end
        
        println("Grasp execution completed successfully")
        # Send success status
        publish(status_pub[], Int32Msg(0))
        
    catch e
        println("Error executing grasp: $e")
        # Send error status
        publish(status_pub[], Int32Msg(-1))
    end
end

function main()
    init_node("julia_grasp_executor")
    
    # Publisher for status
    status_pub[] = Publisher("/grasp_status", Int32Msg; queue_size=10)
    
    # Subscriber for grasp commands
    sub = Subscriber{StringMsg}("/grasp_command", grasp_command_callback; queue_size=10)

    # println("Initialization : completing the three grasps")
    # object_centric_medium_wrap(0.03)
    # object_centric_power_sphere(0.03)
    # object_centric_lateral_pinch(0.025, 0.005)
    
    println("Julia grasp executor node ready and listening...")
    
    spin()
end

main()