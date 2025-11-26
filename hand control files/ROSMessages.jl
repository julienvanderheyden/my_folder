module ROSMessages

using RobotOS

# Import ROS message types
@rosimport std_msgs.msg: Int32

# Generate Julia wrappers only once per session
if !isdefined(@__MODULE__, :__ros_types_generated)
    rostypegen()
    const __ros_types_generated = true
end

# Keep all types inside the module namespace
import .std_msgs.msg as Msg

end # module
