using RobotOS
@rosimport std_msgs.msg: Int32
rostypegen()
using .std_msgs.msg

include("object_centric_medium_wrap_2.jl")


# Callback function for the subscriber


function main()
    init_node("julia_ros_synchronizer")

    pub = Publisher("/ros_julia_synchronization", Int32Msg; queue_size=10)

    current_step = 1
    grasp_parameters = [0.03, 0.04, 0.03, 0.04]

    function callback(msg)
        if msg.data == 0
            println("Arm motion completed , executing hand motion")
            object_centric_medium_wrap(grasp_parameters[current_step])
            current_step += 1
            if current_step > 5
                current_step = 1
                println("All steps completed, test is done")
                return
            end
            println("Hand motion completed, sending arm to position", current_step +1)
            msg = Int32Msg(current_step + 1 )
            publish(pub, msg)
        else 
            println("Arm motion went wrong, stopping the test")
            return
        end
    end
    
    #sub = Subscriber{Int32Msg}("/ros_julia_synchronization", callback; queue_size=10)
    msg = Int32Msg(current_step + 1)
    println("Starting the test, sending arm to position ", current_step +1)
    publish(pub, msg)
    println("First message sent, waiting a bit and resending")
    sleep(Rate(1.0))
    publish(pub, msg)
    println("Waiting for arm to complete motion...")

    spin()
end

main()


