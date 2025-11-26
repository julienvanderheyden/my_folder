module ROSMessages
    using RobotOS
    @rosimport std_msgs.msg: Int32
    if !isdefined(Main, :__ros_types_generated)
        rostypegen()
        const __ros_types_generated = true
    end
    import .std_msgs.msg as Msg
end