using RobotOS
@rosimport std_msgs.msg: Int32
rostypegen()
using .std_msgs.msg

include("object_centric_grasping.jl")


# Callback function for the subscriber


function main()
    init_node("julia_ros_synchronizer")

    pub = Publisher("/ros_julia_synchronization", Int32Msg; queue_size=10)

    current_step = 1 # step 1 : going to home position

    grasp_type = 1 # 1 for medium wrap, 2 for power sphere, 3 for lateral pinch
    grasp_parameters = [0.03, 0.04, 0.03, 0.04, 0.03] 
    function callback(msg)
        if msg.data == 0
            
            
            if current_step !=1 
                println("Arm motion completed , executing hand motion")
                if grasp_type == 1
                    object_centric_medium_wrap(grasp_parameters[current_step - 1])
                elseif grasp_type == 2
                    object_centric_power_sphere(grasp_parameters[current_step - 1])
                elseif grasp_type == 3
                    object_centric_lateral_pinch(grasp_parameters[current_step - 1])
                end
            else 
                println("Arm to home position, sequence is starting")
            end 

            current_step += 1
            if current_step >= 7
                println("All steps completed, test is done. Arm back to home position")
                msg = Int32Msg(1)
                publish(pub, msg)
                return
            end
            println("Hand motion completed, sending arm to position ", current_step)
            msg = Int32Msg(current_step)
            publish(pub, msg)
        else 
            return
        end
    end
    
    sub = Subscriber{Int32Msg}("/ros_julia_synchronization", callback; queue_size=10)
    msg = Int32Msg(current_step)

    println("Starting the test, sending arm to home position ")
    publish(pub, msg)
    sleep(Rate(1.0))
    publish(pub, msg)

    spin()
end

main()


