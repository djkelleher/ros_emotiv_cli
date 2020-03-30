
### This package contains two ROS nodes: emotiv_training and training_cli.   

### Description:   
**emotiv_training** -- The emotiv training node provides a ROS interface to all functionality of the free Emotiv community-sdk.   
The free community-sdk supports training facial expressions and mental commands.     
After training facial expressions and/or mental commands, emotiv_training can run in an event loop where it detects trained   
facial and/or mental signatures.     

**emotiv_cli** -- This node provides a command line interface to the services provided by emotiv_training. It is intended to
make the training process easier.   

### Training process:   
1. Fill in parameters in the launch file 'train.launch'.   
The parameters that need to be filled in are 'profile_path' and 'selected_actions'.   
'profile_path' should be the path to your user profile. ex) '/home/name/myprofile.emu'. If you have not saved a profile yet,
enter a path to an empty .emu file.     
'selected_actions' are the mental and/or facial actions that you want to use. The sdk supports a max of 4 actions.   

All supported facial actions:     
"neutral", "blink", "wink_left", "windright", "horieye", "suprise", "frown", "smile", "clench".  
All supported mental actions:    
"push", "pull", "lift", "drop", "left", "right", "rotate_left", "rotate_right", "rotate_clockwise",   
"rotate_counter_clockwise", "rotate_forwards", "rotate_reverse", "disappear"  

Note: The name of the mental action does not have to accurately match the actual action you are performing in real life.   
For example, when training "push", you can think about closing a robotic hand, and when training "pull", you can think about opening the hand.   

1. Start the training node: roslaunch emotiv_training train.launch   
2. Start the command line interface: rosrun emotiv_training training_cli.py     
3. Start a mental training session with command: mc_start   
4. Train your neutral state (don't think about anything) with command: mc_neutral   
5. Wait about 8 seconds.  
6. Accept your neutral state with command: mc_accept   
7. Start a new training session: mc_start   
8. Train an action (ex: push)   
9. Test the action on your hardware (ex: robotic hand)
10. If the signal was good you will be prompted to accept the new data as an update to your profile.   