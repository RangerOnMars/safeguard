from numpy import broadcast
import rospy
import time
from std_msgs.msg import String,Bool
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction
def generate_params_dict(key, params):
    for param in rospy.get_param_names():
        if param.split("/").count(key):
            param_key = param.split("/")[-1]
            params[param_key] = rospy.get_param(param)
    return params

def main():
    params = {}
    params = generate_params_dict("safeguard_main", params)
    rospy.init_node('safeguard_main', anonymous=True)
    client = SimpleActionClient("safeguard_main",MoveBaseAction)
    broadcast_info_publisher = rospy.Publisher(params["broadcast_info_topic"],String,queue_size=2)
    pub = rospy.Publisher("/fuck",Bool,queue_size=2)
    #Main Loop
    while (not rospy.is_shutdown()):
        broadcast_info = String()
        broadcast_info.data = "fence_warning"
        broadcast_info_publisher.publish(broadcast_info)
        client.cancel_all_goals()
        pub.publish(Bool())
        time.sleep(0.5)
        
        
    
if __name__ == "__main__":
    main()