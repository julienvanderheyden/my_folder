using RobotOS
@rosimport std_msgs.msg: Int32
rostypegen()
using .std_msgs.msg


# Callback function for the subscriber
function callback(msg)
    println("Received integer: $(msg.data)")
end

function main()
    init_node("julia_int_publisher")

    pub = Publisher("/simple_number", Int32Msg; queue_size=10)
    rate = Rate(1.0)
    x = 0
    
    sub = Subscriber{Int32Msg}("/simple_number_2", callback; queue_size=10)


    while !is_shutdown()
	msg = Int32Msg(x)
	publish(pub, msg)
	println("Published integer: $x")
	x += 1
	sleep(rate)
    end

end

main()


