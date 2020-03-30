#include "emotiv_training/emotiv_training.hpp"

EmotivTraining::EmotivTraining() : selected_facial_actions(0), selected_mental_actions(0),
                                   mental_training_status{
                                       {IEE_MentalCommandTrainingStarted, "mental training started"},
                                       {IEE_MentalCommandTrainingSucceeded, "mental training succeeded"},
                                       {IEE_MentalCommandTrainingFailed, "mental training failed"},
                                       {IEE_MentalCommandTrainingCompleted, "mental training completed"},
                                       {IEE_MentalCommandTrainingDataErased, "mental training data erased"},
                                       {IEE_MentalCommandTrainingRejected, "mental training rejected"},
                                       {IEE_MentalCommandTrainingReset, "mental training reset"},
                                       {IEE_MentalCommandAutoSamplingNeutralCompleted, "mental command auto sampling neutral completed"},
                                       {IEE_MentalCommandSignatureUpdated, "mental command signature updated"},
                                       {IEE_MentalCommandNoEvent, "mental training no event"}},

                                   facial_training_status{{IEE_FacialExpressionTrainingStarted, "facial expression training started"}, {IEE_FacialExpressionTrainingSucceeded, "facial expression training succeeded"}, {IEE_FacialExpressionTrainingFailed, "facial expression training failed"}, {IEE_FacialExpressionTrainingCompleted, "facial expression training completed"}, {IEE_FacialExpressionTrainingDataErased, "facial expression training data erased"}, {IEE_FacialExpressionTrainingRejected, "facial expression training rejected"}, {IEE_FacialExpressionTrainingReset, "facial expression training reset"}, {IEE_FacialExpressionNoEvent, "facial expression no event"}},

                                   facial_actions{{"fe_neutral", FE_NEUTRAL}, {"blink", FE_BLINK}, {"wink_left", FE_WINK_LEFT}, {"windright", FE_WINK_RIGHT}, {"horieye", FE_HORIEYE}, {"suprise", FE_SURPRISE}, {"frown", FE_FROWN}, {"smile", FE_SMILE}, {"clench", FE_CLENCH}},

                                   facial_controls{{"fe_none", FE_NONE}, {"fe_start", FE_START}, {"fe_accept", FE_ACCEPT}, {"fe_reject", FE_REJECT}, {"fe_erase", FE_ERASE}, {"fe_reset", FE_RESET}},

                                   mental_actions{{"mc_neutral", MC_NEUTRAL}, {"push", MC_PUSH}, {"pull", MC_PULL}, {"lift", MC_LIFT}, {"drop", MC_DROP}, {"left", MC_LEFT}, {"right", MC_RIGHT}, {"rotate_left", MC_ROTATE_LEFT}, {"rotate_right", MC_ROTATE_RIGHT}, {"rotate_clockwise", MC_ROTATE_CLOCKWISE}, {"rotate_counter_clockwise", MC_ROTATE_COUNTER_CLOCKWISE}, {"rotate_forwards", MC_ROTATE_FORWARDS}, {"rotate_reverse", MC_ROTATE_REVERSE}, {"disappear", MC_DISAPPEAR}},

                                   mental_controls{{"mc_none", MC_NONE}, {"mc_start", MC_START}, {"mc_accept", MC_ACCEPT}, {"mc_reject", MC_REJECT}, {"mc_erase", MC_ERASE}, {"mc_reset", MC_RESET}},

                                   connection_status{{IEEG_CQ_NO_SIGNAL, "No Signal"}, {IEEG_CQ_VERY_BAD, "Very Bad"}, {IEEG_CQ_POOR, "Poor"}, {IEEG_CQ_FAIR, "Fair"}, {IEEG_CQ_GOOD, "Good"}}
{
    if (!nh.getParam("selected_actions", selected_actions_param))
    {
        ROS_ERROR("COULD NOT GET PARAMETER 'selected_actions'. CAN NOT DO TRAINING!");
        return;
    }
    if (IEE_EngineConnect() != EDK_OK)
    {
        ROS_ERROR("EMOTIV DRIVER START UP FAILED!");
        return;
    }
    eEvent = IEE_EmoEngineEventCreate();
    eState = IEE_EmoStateCreate();

    int num_actions = selected_actions_param.size();
    if (num_actions > 4)
    {
        ROS_ERROR("'community-sdk' API CAN ONLY HANDLE 4 ACTIVE CONNECTIONS! %i WERE REQUESTED!", num_actions);
        return;
    }
    for (size_t i = 0; i < num_actions; ++i)
    {
        // ROS parameters are in XmlRpc, so cast to a string.
        auto action_string = static_cast<std::string>(selected_actions_param[i]);

        auto ma_it = mental_actions.find(action_string);
        if (ma_it == mental_actions.end())
        {
            auto fa_it = facial_actions.find(action_string);
            if (fa_it == facial_actions.end())
            {
                ROS_ERROR("INVALID ACTION '%s'", action_string.c_str());
                continue;
            }
            else
            {
                selected_facial_actions |= fa_it->second;
            }
        }
        else
        {
            selected_mental_actions |= ma_it->second;
        }
    }
    // Start ROS service servers.
    set_action_srv = nh.advertiseService("set_action", &EmotivTraining::set_action, this);
    set_control_srv = nh.advertiseService("set_control", &EmotivTraining::set_control, this);
    get_headset_status_srv = nh.advertiseService("get_headset_status", &EmotivTraining::get_headset_status, this);

    // Start ROS publishers.
    mental_state_pub = nh.advertise<emotiv_msgs::MentalState>("mental_state", 1000, this);
    facial_state_pub = nh.advertise<emotiv_msgs::FacialState>("facial_state", 1000, this);
    training_status_pub = nh.advertise<emotiv_msgs::TrainingStatus>("training_status", 1000, this);
}

bool EmotivTraining::add_user(int user_id, std::string profile_path, bool set_as_current_user)
{
    // Check if user_id and profile_path pair are already stored.
    bool user_id_exists(false);
    auto it = emotiv_users.begin();
    for (it; it != emotiv_users.end(); ++it)
    {
        if (it->id == user_id)
        {
            user_id_exists = true;
            break;
        }
    }
    // Check if profile for existsing user has changed.
    if (user_id_exists)
    {
        if (it->profile_path == profile_path)
        {
            return true;
        }
    }
#ifdef PAID_SDK
    if (IEE_LoadUserProfile(user_id, profile_path.c_str()) != EDK_OK)
    {
        ROS_ERROR("COULD NOT REGISTERE USER '%i' PROFILE WITH EMOENGINE.", user_id);
        return false;
    }
    ROS_INFO("REGISTERED USER '%i' PROFILE WITH EMOENGINE.", user_id);
#endif

    unsigned long trained_mental_actions = 0;
    IEE_MentalCommandGetTrainedSignatureActions(user_id, &trained_mental_actions);

    if (selected_mental_actions)
    {
        auto ret_val = IEE_MentalCommandSetActiveActions(user_id, selected_mental_actions);
        if (ret_val != EDK_OK)
        {
            ROS_ERROR("COULD NOT REGISTER MENTAL TRAINING ACTIONS WITH THE SDK! ERROR CODE: %i", ret_val);
            return false;
        }
    }
    int sig_avail(0);
    if (selected_facial_actions)
    {
        // Check if the user has trained sufficient actions to activate a trained signature.
        if (IEE_FacialExpressionGetTrainedSignatureAvailable(user_id, &sig_avail) == EDK_OK)
        {
            if (sig_avail)
            {
                if (IEE_FacialExpressionSetSignatureType(user_id, FE_SIG_TRAINED) == EDK_OK)
                {
                    ROS_INFO("A TRAINED FACIAL SIGNATURE WILL BE USED FOR USER %i", user_id);
                }
                else
                {
                    ROS_WARN("COULD NOT SET TRAINED FACIAL SIGNATURE FOR USER %i", user_id);
                }
            }
            else
            {
                ROS_INFO("NO TRAINED FACIAL SIGNATURE WAS FOUND FOR USER %i", user_id);
            }
        }
    }
    if (set_as_current_user)
    {
        current_user.id = user_id;
        current_user.profile_path = profile_path;
        current_user.trained_mental_actions = trained_mental_actions;
        current_user.trained_facial_signature = sig_avail;
    }
    else
    {
        EmotivUser new_user;
        new_user.id = user_id;
        new_user.profile_path = profile_path;
        new_user.trained_mental_actions = trained_mental_actions;
        new_user.trained_facial_signature = sig_avail;
        emotiv_users.push_back(new_user);
    }
    return true;
}

bool EmotivTraining::get_headset_status(emotiv_msgs::GetHeadsetStatus::Request &req, emotiv_msgs::GetHeadsetStatus::Response &res)
{
    if (IS_GetHeadsetOn(eState))
    {
        ROS_INFO("HEADSET IT ON!");
    }
    else
    {
        ROS_WARN("HEADSET IS NOT ON!");
    }
    // Check if the signal is too noisy for MentalCommand detection to be active.
    // detection state (0: Not Active, 1: Active)
    auto status = IS_MentalCommandIsActive(eState);
    if (status == 0)
    {
        ROS_ERROR("SIGNAL IS TOO NOISY FOR MENTAL COMMAND DETECTION!");
    }
    res.headset_status.active = status;

    // Get the current wireless signal strength.
    // Returns [No Signal, Bad, Fair, Good, Excellent].
    auto sig_stren = IS_GetWirelessSignalStatus(eState);
    res.headset_status.signal_strength = sig_stren;
    ROS_INFO("WIRELESS SIGNAL STRENGTH: %i", sig_stren);

    /*Used to characterize the EEG signal reception or electrode contact for a sensor on the headset.
    Note that this differs from the wireless signal strength,
    which refers to the radio communication between the headset transmitter and USB dongle receiver.*/
    IEE_EEG_ContactQuality_t contactQuality[18];

    // Get the contact quality of all the electrodes in one single call.
    IS_GetContactQualityFromAllChannels(eState, &contactQuality[0], 18);

    auto get_status_string = [&](IEE_EEG_ContactQuality_t status) {
        auto it = connection_status.find(status);
        return it->second;
    };
    std::string status_string;
    IEE_EEG_ContactQuality_enum AF3_contact = contactQuality[IEE_CHAN_AF3];
    status_string = get_status_string(AF3_contact);
    res.headset_status.contact_quality_AF3 = status_string;
    ROS_INFO("IEE_CHAN_AF3 CONTACT QUALITY: %s", status_string.c_str());

    IEE_EEG_ContactQuality_enum AF4_contact = contactQuality[IEE_CHAN_AF4];
    status_string = get_status_string(AF4_contact);
    res.headset_status.contact_quality_AF4 = status_string;
    ROS_INFO("IEE_CHAN_AF4 CONTACT QUALITY: %s", status_string.c_str());

    IEE_EEG_ContactQuality_enum T7_contact = contactQuality[IEE_CHAN_T7];
    status_string = get_status_string(T7_contact);
    res.headset_status.contact_quality_T7 = status_string;
    ROS_INFO("IEE_CHAN_T7 CONTACT QUALITY: %s", status_string.c_str());

    IEE_EEG_ContactQuality_enum T8_contact = contactQuality[IEE_CHAN_T8];
    status_string = get_status_string(T8_contact);
    res.headset_status.contact_quality_T8 = status_string;
    ROS_INFO("IEE_CHAN_T8 CONTACT QUALITY: %s", status_string.c_str());

    IEE_EEG_ContactQuality_enum Pz_contact = contactQuality[IEE_CHAN_Pz];
    status_string = get_status_string(Pz_contact);
    res.headset_status.contact_quality_Pz = status_string;
    ROS_INFO("IEE_CHAN_Pz CONTACT QUALITY: %s", status_string.c_str());
}

bool EmotivTraining::set_action(emotiv_msgs::SetAction::Request &req, emotiv_msgs::SetAction::Response &res)
{
    auto action = std::string(req.action);
    int status(EDK_UNKNOWN_ERROR);

    auto ma_it = mental_actions.find(action);
    if (ma_it == mental_actions.end())
    {
        auto fa_it = facial_actions.find(action);
        if (fa_it == facial_actions.end())
        {
            ROS_ERROR("COULD NOT FIND ACTION ENUM FOR STRING '%s'!", action.c_str());
            res.success = false;
            return false;
        }
        else
        {
            status = IEE_FacialExpressionSetTrainingAction(current_user.id, fa_it->second);
        }
    }
    else
    {
        status = IEE_MentalCommandSetTrainingAction(current_user.id, ma_it->second);
    }
    if (status != EDK_OK)
    {
        ROS_ERROR("COULD NOT SET ACTION '%s'!", action.c_str());
        res.success = false;
        return false;
    }
    else
    {
        ROS_INFO("SET ACTION '%s'!", action.c_str());
        res.success = true;
        return true;
    }
}

bool EmotivTraining::set_control(emotiv_msgs::SetControl::Request &req, emotiv_msgs::SetControl::Response &res)
{
    std::string control = req.control;

    auto mc_it = mental_controls.find(control);
    if (mc_it == mental_controls.end())
    {
        auto fe_it = facial_controls.find(control);
        if (fe_it == facial_controls.end())
        {
            ROS_ERROR("COULD NOT FIND CONTROL ENUM FOR STRING '%s'", control.c_str());
            res.success = false;
            return false;
        }
        else
        {
            auto fe_control = fe_it->second;
            if (IEE_FacialExpressionSetTrainingControl(current_user.id, fe_control) != EDK_OK)
            {
                ROS_ERROR("COULD NOT SET CONTROL '%s'!", control.c_str());
                res.success = false;
                return false;
            }
#ifdef PAID_SDK
            if (fe_control == FE_ACCEPT)
            {
                if (IEE_SaveUserProfile(current_user.id, current_user.profile_path.c_str()) != EDK_OK)
                {
                    ROS_WARN("UNABLE TO SAVE TRAINING UPDATE TO: %s", current_user.profile_path.c_str());
                }
            }
#endif
        }
    }
    else
    {
        auto mc_control = mc_it->second;
        if (IEE_MentalCommandSetTrainingControl(current_user.id, mc_control) != EDK_OK)
        {
            ROS_ERROR("COULD NOT SET CONTROL '%s'!", control.c_str());
            res.success = false;
            return false;
        }
#ifdef PAID_SDK
        if (mc_control == MC_ACCEPT)
        {
            if (IEE_SaveUserProfile(current_user.id, current_user.profile_path.c_str()) != EDK_OK)
            {
                ROS_WARN("UNABLE TO SAVE TRAINING UPDATE TO: %s", current_user.profile_path.c_str());
            }
        }
#endif
    }
    ROS_INFO("SET CONTROL '%s'!", control.c_str());
    res.success = true;
    return true;
}

void EmotivTraining::get_mental_training_status()
{
    // Return the MentalCommand-specific event type.
    IEE_MentalCommandEvent_enum status_change = IEE_MentalCommandEventGetType(eEvent);
    auto it = mental_training_status.find(status_change);
    if (it == mental_training_status.end())
    {
        auto error_status = std::string("COULD NOT DETERMINE MENTAL TRAINING STATUS.");
        ROS_ERROR_STREAM(error_status);

        current_user.training_status.status = error_status;
    }
    else
    {
        current_user.training_status.status = it->second;
    }
    // publish the updated message.
    training_status_pub.publish(current_user.training_status);
}

void EmotivTraining::get_facial_training_status()
{
    IEE_FacialExpressionEvent_enum status_change = IEE_FacialExpressionEventGetType(eEvent);
    auto it = facial_training_status.find(status_change);
    if (it == facial_training_status.end())
    {
        auto error_status = std::string("COULD NOT DETERMINE FACIAL TRAINING STATUS.");
        ROS_ERROR_STREAM(error_status);
        current_user.training_status.status = error_status;
    }
    else
    {
        current_user.training_status.status = it->second;
    }
    training_status_pub.publish(current_user.training_status);
}

void EmotivTraining::get_mental_state()
{
    // Should return an action type that has registered signature with user.
    IEE_MentalCommandAction_t action_type = IS_MentalCommandGetCurrentAction(eState);

    // Get the current skill rating for particular MentalCommand actions of the user. [from 0.0 to 1.0]
    IEE_MentalCommandGetActionSkillRating(current_user.id, action_type, &current_user.mental_state.skill);

    // Returns the detected MentalCommand action power of the user. (0.0 to 1.0)
    current_user.mental_state.power = IS_MentalCommandGetCurrentActionPower(eState);

    //IEE_MentalCommandGetActivationLevel

#ifdef PAID_SDK
    // Post-process results of IS_PerformanceMetric.
    auto calc_scale = [&](double &raw_score, double &min_scale, double &max_scale) -> double {
        if (raw_score < min_scale)
        {
            return 0.0;
        }
        else if (raw_score > max_scale)
        {
            return 1.0;
        }
        else
        {
            return (raw_score - min_scale) / (max_scale - min_scale);
        }
    };
    // Arguments for IS_PerformanceMetric functions.
    double raw_score(0), min_scale(0), max_scale(0);

    // Returns Stress model parameters.
    IS_PerformanceMetricGetStressModelParams(eState, &raw_score, &min_scale, &max_scale);
    current_user.mental_state.stress_score = calc_scale(raw_score, min_scale, max_scale);
    // Returns EngagementBoredom model parameters.
    IS_PerformanceMetricGetEngagementBoredomModelParams(eState, &raw_score, &min_scale, &max_scale);
    current_user.mental_state.engagement_boredom_score = calc_scale(raw_score, min_scale, max_scale);
    // Returns Relaxation model parameters.
    IS_PerformanceMetricGetRelaxationModelParams(eState, &raw_score, &min_scale, &max_scale);
    current_user.mental_state.relaxation_score = calc_scale(raw_score, min_scale, max_scale);
    // Returns short term excitement model parameters.
    IS_PerformanceMetricGetInstantaneousExcitementModelParams(eState, &raw_score, &min_scale, &max_scale);
    current_user.mental_state.excitement_score = calc_scale(raw_score, min_scale, max_scale);
    // Returns Interest model parameters.
    IS_PerformanceMetricGetInterestModelParams(eState, &raw_score, &min_scale, &max_scale);
    current_user.mental_state.interest_score = calc_scale(raw_score, min_scale, max_scale);
#endif

    mental_state_pub.publish(current_user.mental_state);
}

void EmotivTraining::get_facial_state()
{
    IEE_FacialExpressionAlgo_t upper_face_type = IS_FacialExpressionGetUpperFaceAction(eState);
    IEE_FacialExpressionAlgo_t lower_face_type = IS_FacialExpressionGetLowerFaceAction(eState);

    float upper_face_amp = IS_FacialExpressionGetUpperFaceActionPower(eState);
    float lower_face_amp = IS_FacialExpressionGetLowerFaceActionPower(eState);

    current_user.facial_state.blink = IS_FacialExpressionIsBlink(eState);
    current_user.facial_state.left_wink = IS_FacialExpressionIsLeftWink(eState);
    current_user.facial_state.right_wink = IS_FacialExpressionIsRightWink(eState);

    // Initialize values to 0.0;
    current_user.facial_state.suprise = 0.0;
    current_user.facial_state.frown = 0.0;
    current_user.facial_state.clench = 0.0;
    current_user.facial_state.smile = 0.0;

    if (upper_face_amp > 0.0)
    {

        switch (upper_face_type)
        {

        case FE_SURPRISE:
            current_user.facial_state.suprise = upper_face_amp;
            break;

        case FE_FROWN:
            current_user.facial_state.frown = upper_face_amp;
            break;
        }
    }
    if (lower_face_amp > 0.0)
    {

        switch (lower_face_type)
        {

        case FE_CLENCH:
            current_user.facial_state.clench = lower_face_amp;
            break;

        case FE_SMILE:
            current_user.facial_state.smile = lower_face_amp;
            break;
        }
    }
    facial_state_pub.publish(current_user.facial_state);
}

void EmotivTraining::update()
{
    // Retrieve the next EmoEngine event. Non-blocking call.
    if (IEE_EngineGetNextEvent(eEvent) == EDK_OK)
    {
        ROS_INFO("RECIEVED NEW EVENT FROM EMOENGINE.");

        unsigned int UserID;
        if (IEE_EmoEngineEventGetUserId(eEvent, &UserID) != EDK_OK)
        {
            ROS_ERROR("COULD NOT GET USER ID!");
        }
        ROS_INFO_STREAM("GOT USER ID: " << UserID);
        bool added = add_user(UserID);
        
        ROS_INFO("USER ADDED");
        if (added)
        {
            // Get the event type for an event already retrieved using IEE_EngineGetNextEvent().
            IEE_Event_t eventType = IEE_EmoEngineEventGetType(eEvent);
            ROS_INFO("GOT EVENT TYPE");

            // Check for the two event types we are interesed in for mental command trainnig.
            switch (eventType)
            {
            // Detection results have been updated from EmoEngine.
            case IEE_EmoStateUpdated:
            {
                // Copy an EmoState returned with a IEE_EmoStateUpdate event to memory referenced by an EmoStateHandle.
                if (IEE_EmoEngineEventGetEmoState(eEvent, eState) == EDK_OK)
                {
                    if (selected_mental_actions)
                    {
                        get_mental_state();
                    }
                    if (selected_facial_actions)
                    {
                        get_facial_state();
                    }
                }
                else
                {
                    ROS_ERROR("COULD NOT COPY EMOSTATE TO MEMORY");
                }
                break;
            }
            // A IEE_MentalCommandEvent_t has been returned from EmoEngine.
            case IEE_MentalCommandEvent:
            {
                get_mental_training_status();
                break;
            }
            case IEE_FacialExpressionEvent:
            {
                get_facial_training_status();
                break;
            }
            }
        }
    }
}

EmotivTraining::~EmotivTraining()
{
    IEE_EngineDisconnect();
    IEE_EmoStateFree(eState);
    IEE_EmoEngineEventFree(eEvent);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "emotiv_training");

    EmotivTraining training_node;
    
    while (ros::ok)
    {
        training_node.update();
        ros::spinOnce();
    }
}
