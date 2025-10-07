using RobotOS

@rosimport std_msgs.msg: Float64  # import the message definition
rostypegen()  # generate Julia bindings

# Initialize ROS node
init_node("julia_number_publisher")

# Create a publisher
pub = Publisher("/simple_number", std_msgs.msg.Float64)

rate = Rate(1.0)  # 1 Hz

x = 0.0
while !is_shutdown()
    msg = std_msgs.msg.Float64(data = x)
    publish(pub, msg)
    println("Published number: $x")
    x += 0.1
    sleep(rate)
end
