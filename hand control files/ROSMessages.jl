using RobotOS
@rosimport std_msgs.msg: Int32
rostypegen()
using .std_msgs.msg
include("object_centric_grasping.jl")
init_node("julia_ros_synchronizer")
global pub = Publisher("/ros_julia_synchronization", Int32Msg; queue_size=10)
