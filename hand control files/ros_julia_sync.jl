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
    grasp_parameters = [0.006, 0.012, 0.015, 0.018, 0.02, 0.025, 0.0275, 0.03, 0.035]  # cylinders
    # grasp_parameters = [0.006, 0.012, 0.012, 0.015, 0.017, 0.02] #, 0.02, 0.03, 0.023, 0.025, 0.028, 0.032, 0.038]  # prisms? 

    #grasp_type = 2
    # grasp_parameters = [0.016, 0.019, 0.024, 0.028, 0.032] # spheres
    # grasp_parameters = [0.016, 0.019, 0.024, 0.028, 0.032]

    #grasp_type = 3
    #grasp_parameters = [[0.025, 0.001], [0.025, 0.006],[0.025, 0.013], [0.0375, 0.001], 
    #[0.0375, 0.006], [0.0375, 0.013], [0.05, 0.001], [0.05, 0.006], [0.05, 0.013]]

    dimension_noise = 0.45

    function callback(msg)
        if msg.data == 0 && current_step <= 10 
            
            if current_step !=1 
                println("Arm motion completed , executing hand motion")
                if grasp_type == 1
                    object_centric_medium_wrap((1+dimension_noise)*grasp_parameters[current_step - 1])
                elseif grasp_type == 2
                    object_centric_power_sphere((1+dimension_noise)*grasp_parameters[current_step - 1])
                elseif grasp_type == 3
                    object_centric_lateral_pinch((1+dimension_noise)*grasp_parameters[current_step - 1][1], (1+dimension_noise)*grasp_parameters[current_step - 1][2])
                end
            else 
                println("Arm to home position, sequence is starting")
            end 

            current_step += 1
            if current_step >= 11
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


