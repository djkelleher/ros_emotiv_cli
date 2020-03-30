#ifndef EMOTIV_TRAINING_HPP_
#define EMOTIV_TRAINING_HPP_

#include <iostream>
#include <unordered_map>
#include <functional>
#include <thread>

#include "emotiv_sdk/IEmoStateDLL.h"
#include "emotiv_sdk/Iedk.h"
#include "emotiv_sdk/IedkErrorCode.h"
#include "emotiv_sdk/MentalCommandDetection.h"
#include "emotiv_sdk/IEmoStatePerformanceMetric.h"

#include <emotiv_msgs/FacialState.h>
#include <emotiv_msgs/MentalState.h>
#include <emotiv_msgs/HeadsetStatus.h>
#include <emotiv_msgs/TrainingStatus.h>

#include <emotiv_msgs/GetHeadsetStatus.h>
#include <emotiv_msgs/SetAction.h>
#include <emotiv_msgs/SetControl.h>

#include <ros/ros.h>

class EmotivTraining
{
  private:
    ros::NodeHandle nh;
    ros::Publisher facial_state_pub, mental_state_pub, training_status_pub;
    ros::ServiceServer set_action_srv, set_control_srv, get_headset_status_srv;

    unsigned long selected_facial_actions, selected_mental_actions;

    EmoStateHandle eState;
    EmoEngineEventHandle eEvent;

    XmlRpc::XmlRpcValue selected_actions_param;

    std::unordered_map<IEE_EEG_ContactQuality_enum, std::string, std::hash<int>> connection_status;

    std::unordered_map<std::string, IEE_MentalCommandAction_enum> mental_actions;
    std::unordered_map<std::string, IEE_MentalCommandTrainingControl_enum> mental_controls;

    std::unordered_map<std::string, IEE_FacialExpressionAlgo_enum> facial_actions;
    std::unordered_map<std::string, IEE_FacialExpressionTrainingControl_enum> facial_controls;

    std::unordered_map<IEE_MentalCommandEvent_enum, std::string, std::hash<int>> mental_training_status;
    std::unordered_map<IEE_FacialExpressionEvent_enum, std::string, std::hash<int>> facial_training_status;

    struct EmotivUser
    {
        int id;
        std::string profile_path;
        emotiv_msgs::FacialState facial_state;
        emotiv_msgs::MentalState mental_state;
        emotiv_msgs::HeadsetStatus headset_state;
        emotiv_msgs::TrainingStatus training_status;
        bool trained_facial_signature;
        unsigned int trained_mental_actions;
    };
    EmotivUser current_user;
    std::vector<EmotivUser> emotiv_users;

  public:
    EmotivTraining();
    ~EmotivTraining();

    void update();
    
    bool add_user(int user_id, std::string profile_path = "", bool set_as_current_user = true);
    
    bool set_action(emotiv_msgs::SetAction::Request &req,
                    emotiv_msgs::SetAction::Response &res);

    bool set_control(emotiv_msgs::SetControl::Request &req,
                     emotiv_msgs::SetControl::Response &res);

    bool get_headset_status(emotiv_msgs::GetHeadsetStatus::Request &req,
                            emotiv_msgs::GetHeadsetStatus::Response &res);

    void get_mental_state();
                    
    void get_facial_state();

    void get_mental_training_status();
                    
    void get_facial_training_status();
};

#endif
