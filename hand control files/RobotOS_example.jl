#!/usr/bin/env julia

using RobotOS
@rosimport std_msgs.msg: Int32
rostypegen()
using .std_msgs.msg

function callback(msg::Int32, pub_obj::Publisher{Int32})
    pt_msg = Int32(msg.data, 0.0)
    publish(pub_obj, pt_msg)
end

function loop(pub_obj)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        npt = Int32(rand(), 0.0)
        publish(pub_obj, npt)
        rossleep(loop_rate)
    end
end

function main()
    init_node("rosjl_example")
    pub = Publisher{Int32}("pts", queue_size=10)
    sub = Subscriber{Int32}("pose", callback, (pub,), queue_size=10)
    loop(pub)
end

if ! isinteractive()
    main()
end