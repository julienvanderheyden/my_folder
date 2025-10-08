using RobotOS

# Import the ROS message definition
@rosimport std_msgs.msg: Float64
rostypegen()

function main()
    # Initialize ROS node
    init_node("julia_number_publisher")

    # Create publisher with queue_size
    pub = Publisher("/simple_number", std_msgs.msg.Float64Msg; queue_size=10)

    rate = Rate(1.0)  # 1 Hz
    x = 0.0

    while !is_shutdown()
        msg = std_msgs.msg.Float64Msg(data = x)
        publish(pub, msg)
        println("Published number: $x")
        x += 0.1
        sleep(rate)
    end
end

main()
