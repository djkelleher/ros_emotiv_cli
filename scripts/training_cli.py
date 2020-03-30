#!/usr/bin/env python

import rospy
import emotiv_msgs.srv
import emotiv_msgs.msg
import sys


mental_training_controls = ("mc_none", "mc_start", "mc_accept", "mc_reject", "mc_erase", "mc_reset")
facial_training_controls = ("fe_none", "fe_start", "fe_accept", "fe_reject", "fe_erase", "fe_reset")

training_actions = ['mc_neutral', 'fe_neutral']

def print_usage():
    commands = [
        "\n'q' -- Exit the command line program.",
        "'help' -- Print available commands.",
        "mental training controls: {}".format(mental_training_controls),
        "facial training controls: {}".format(facial_training_controls),
        "training actions: {}".format(training_actions),
        "'headset_status' -- Print the connection status of your headset.\n"
    ]
    print("\n\n".join(commands))

def start_ros():
    global training_actions
    # initialize ROS node.
    rospy.init_node('emotiv_cli', anonymous=True)

    # initialize 'training_status' subscriber.
    rospy.Subscriber("training_sub", emotiv_msgs.msg.TrainingStatus, callback=get_training_status)

    # wait for Emotiv services.
    for service in ('set_action', 'set_control', 'get_headset_status'):
        rospy.loginfo("Waiting for service '{}'.".format(service))
        rospy.wait_for_service(service, 10)
        rospy.loginfo("Found service '{}'".format(service))

    # get params from ROS param server.
    new_training_actions = rospy.get_param("selected_actions")
    if new_training_actions is None:
        rospy.logerr("COULD NOT GET 'training_actions' FROM PARAMETER SERVER. EXITING.")
        sys.exit(1)
    training_actions += new_training_actions

def set_action(action):
    try:
        set_action_srv = rospy.ServiceProxy('set_action', emotiv_msgs.srv.SetAction)
        result = set_action_srv(action)
        rospy.loginfo("'set_action_srv' SERVICE RESPONSE: {}".format(result.success))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def set_control(control):
    try:
        set_control_srv = rospy.ServiceProxy('set_control', emotiv_msgs.srv.SetControl)
        result = set_control_srv(control)
        rospy.loginfo("'set_control' SERVICE RESPONSE: {}".format(result.success))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def get_headset_status():
    try:
        headset_status_srv = rospy.ServiceProxy('get_headset_status', emotiv_msgs.srv.GetHeadsetStatus)
        result = headset_status_srv()
        rospy.loginfo("'get_headset_status' SERVICE RESPONSE: {}".format(result.success))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def get_training_status(training_status):
    command = None
    # check training status that requires user input.
    if training_status.status == "mental training succeeded":
        while command not in ('y','n','yes','no'):
            command = str(input("MENTAL TRAINING SUCCEEDED! UPDATE TRAINED MENTAL PROFILE? (y,n)")).lower()
            if command in ('y','yes'):
                set_action("mc_accept")
            elif command in ('n','no'):
                set_action("mc_reject")
    if training_status.status == "mental training failed":
        while command not in ('y','n','yes','no'):
            command = str(input("MENTAL TRAINING FAILED! RESTART TRAINING? (y,n)")).lower()
            if command in ('y','yes'):
                set_action("mc_start")
            elif command in ('n','no'):
                return
    if training_status.status == "facial expression training succeeded":
        while command not in ('y','n','yes','no'):
            command = str(input("FACIAL EXPRESSION TRAINING SUCCEEDED! UPDATE TRAINED FACIAL PROFILE? (y,n)")).lower()
            if command in ('y','yes'):
                set_action("fe_accept")
            elif command in ('n','no'):
                set_action("fe_reject")
    if training_status.status == "facial expression training failed":
        while command not in ('y','n','yes','no'):
            command = str(input("FACIAL EXPRESSION TRAINING FAILED! RESTART TRAINING? (y,n)")).lower()
            if command in ('y','yes'):
                set_action("fe_start")
            elif command in ('n','no'):
                return

def run_cli():
    command = None
    while command is not "q":
        command = str(input("Press 'q' to quit. Type 'help' to show commands.\nEnter a command: ")).split()
        num_words = len(command)
        if num_words != 1:
            rospy.logerr("INVALID COMMAND. COMMAND SHOULD BE 1 WORD.")
            print_usage()
        command = command[0]
        if command == "q":
            rospy.loginfo("QUITING")
            return
        elif command == "help":
            print_usage()
        elif command in mental_training_controls or facial_training_controls:
            set_control(command)
        elif command in training_actions:
            set_action(command)
        elif command == "headset_status":
            get_headset_status()
        else:
            rospy.logerr("INVALID COMMAND: {}".format(command))


if __name__ == '__main__':
    # initialize the subscriber and services.
    start_ros()
    # start command line prompt.
    run_cli()