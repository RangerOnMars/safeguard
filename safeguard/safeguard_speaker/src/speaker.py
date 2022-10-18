import rospy
import rospkg
import os
from playsound import playsound
from threading import Lock
from std_msgs.msg import String



def callBack(data, args):
    lock = args["lock"]
    lock.acquire()
    args["cmd_string"] = data.data
    lock.release()

def generate_params_dict(key, params):
    for param in rospy.get_param_names():
        if param.split("/").count(key):
            param_key = param.split("/")[-1]
            params[param_key] = rospy.get_param(param)
    return params

def main():
    lock = Lock()
    cmd_string = str()
    args = {"lock":lock,
            "cmd_string": cmd_string
            }
    params = {}
    params = generate_params_dict('safeguard_speaker', params)
    rospy.init_node('safeguard_speaker', anonymous=True)
    rospy.Subscriber("safeguard/broadcast_info", String, callBack, args)
    
    #Get all params of safeguard_speaker
    path_prefix = os.path.join(rospkg.RosPack().get_path('safeguard_speaker'), "misc")
    #Main Loop
    while True:
        lock.acquire()
        if (args["cmd_string"] is not None) and (args["cmd_string"] in params):
            for act in params[args["cmd_string"]]:
                path = os.path.join(path_prefix, act + ".mp3")
                print(path)
                playsound(path)
        args["cmd_string"] = None
        lock.release()
    
if __name__ == "__main__":
    main()