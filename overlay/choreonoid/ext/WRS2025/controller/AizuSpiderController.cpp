#include <cnoid/SimpleController>
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <cmath>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {
    constexpr double STICK_THRESH = 0.1;
}

class AizuController : public SimpleController
{
public:
    AizuController() = default;
    virtual ~AizuController();

    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;

private:
    // --- Choreonoid members ---
    Body* body{nullptr};
    double dt{0.0};
    int mainActuationMode{Link::JointTorque};
    vector<Link*> tracks;
    int trackActuationMode{Link::JointVelocity};
    double trackVelocityRatio{1.0};
    double kd_mainTrack{8.0};
    double kd_subTrack{1.5};
    vector<double> qprev_track;
    enum TrackId { L_TRACK=0, R_TRACK, FL_SUB_TRACK, FR_SUB_TRACK, BL_SUB_TRACK, BR_SUB_TRACK, NUM_TRACKS };
    struct JointInfo { Link* link{nullptr}; double qref{0.0}; double qold{0.0}; double kp{0.0}; double kd{0.0}; };
    vector<JointInfo> jointInfos;
    struct JointSpec { string name; double kp_torque; double kd_torque; double kp_velocity; };

    // --- ROS2 related ---
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_sub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
    thread ros_thread;
    mutable mutex joy_mutex;
    sensor_msgs::msg::Joy::SharedPtr latest_joy_msg;
    atomic<bool> stop_ros_thread{false};
    atomic<bool> is_active_{true}; // Start active by default
    static atomic<bool> rclcpp_initialized_; // Track global ROS2 initialization state

    // --- Methods ---
    bool initializeTracks(SimpleControllerIO* io);
    bool initializeTracks(SimpleControllerIO* io, const vector<string>& names);
    bool initializeFlipperJoints(SimpleControllerIO* io);
    bool initializeJoints(SimpleControllerIO* io, const vector<JointSpec>& specs);
    void controlTracks();
    void setTrackTorque(int id, double dq_target, double kd);
    void updateFlipperTargetPositions();
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();
    bool initializeROS(SimpleControllerIO* io);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void rosSpinThread();
    double getJoyAxis(int axis, double threshold = 0.0) const;
    bool getJoyButton(int button) const;
};

// Initialize static member
atomic<bool> AizuController::rclcpp_initialized_{false};

AizuController::~AizuController()
{
    stop_ros_thread = true;
    
    if(executor){
        try {
            executor->cancel();
        } catch(...) {}
    }
    
    if(ros_thread.joinable()){
        ros_thread.join();
    }
    
    // Clean up node and subscriptions
    controller_sub_.reset();
    joy_sub.reset();
    
    if(executor){
        if(node){
            try {
                executor->remove_node(node);
            } catch(...) {}
        }
        executor.reset();
    }
    
    node.reset();
    
    // Note: Don't call rclcpp::shutdown() here as it affects all nodes globally
}

void AizuController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    lock_guard<mutex> lock(joy_mutex);
    latest_joy_msg = msg;
}

void AizuController::rosSpinThread()
{
    if(!executor || !node) return;
    while(!stop_ros_thread && rclcpp::ok()){
        try {
            executor->spin_some(std::chrono::milliseconds(10));
        } catch(const std::exception& e) {
            // Handle exceptions during spin
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

double AizuController::getJoyAxis(int axis, double threshold) const
{
    if(!latest_joy_msg) return 0.0;
    lock_guard<mutex> lock(joy_mutex);
    if(axis < 0 || axis >= static_cast<int>(latest_joy_msg->axes.size())) return 0.0;
    double v = latest_joy_msg->axes[axis];
    return (std::abs(v) < threshold) ? 0.0 : v;
}

bool AizuController::getJoyButton(int button) const
{
    if(!latest_joy_msg) return false;
    lock_guard<mutex> lock(joy_mutex);
    if(button < 0 || button >= static_cast<int>(latest_joy_msg->buttons.size())) return false;
    return latest_joy_msg->buttons[button] != 0;
}

bool AizuController::initializeROS(SimpleControllerIO* io)
{
    try {
        // Check if context is valid before initializing
        bool expected = false;
        if(rclcpp_initialized_.compare_exchange_strong(expected, true)) {
            if(!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }
        }
        
        // Wait a bit to ensure context is ready
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Verify ROS2 is properly initialized before creating node
        if(!rclcpp::ok()) {
            throw std::runtime_error("ROS2 context failed to initialize");
        }
        
        // Create node with proper context
        node = rclcpp::Node::make_shared("aizu_spider_controller");
        
        if(!node) {
            throw std::runtime_error("Failed to create ROS2 node");
        }
        
        // Create subscriptions
        joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&AizuController::joyCallback, this, std::placeholders::_1)
        );

        controller_sub_ = node->create_subscription<std_msgs::msg::String>(
            "/active_controller", 10,
            [this](const std_msgs::msg::String::SharedPtr msg){
                if(msg->data == "choreonoid"){
                    if (!this->is_active_) {
                        RCLCPP_INFO(this->node->get_logger(), "==> Choreonoid Base/Flipper Control Activated <==");
                    }
                    this->is_active_ = true;
                } else {
                    if (this->is_active_) {
                        RCLCPP_INFO(this->node->get_logger(), "==> Choreonoid Base/Flipper Control Deactivated <==");
                    }
                    this->is_active_ = false;
                }
            }
        );

        // Create executor and add node
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        
        // Start ROS spin thread
        stop_ros_thread = false;
        ros_thread = thread(&AizuController::rosSpinThread, this);
        
        RCLCPP_INFO(node->get_logger(), "AizuController ROS2 interface initialized successfully");
        return true;
        
    } catch(const std::exception& ex){
        io->os() << "Failed to initialize ROS2: " << ex.what() << std::endl;
        
        // Clean up on failure
        controller_sub_.reset();
        joy_sub.reset();
        
        if(executor && node){
            try {
                executor->remove_node(node);
            } catch(...) {}
        }
        
        node.reset();
        executor.reset();
        return false;
    }
}

bool AizuController::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();
    mainActuationMode = Link::JointTorque;
    
    for(const auto& opt : io->options()){
        if(opt == "velocity"){ 
            mainActuationMode = Link::JointVelocity;
        } else if(opt == "position"){ 
            mainActuationMode = Link::JointDisplacement;
        } else if(opt == "effort" || opt == "torque"){ 
            mainActuationMode = Link::JointTorque;
        }
    }
    
    io->os() << format("AizuController: actuation mode = {0}\n", mainActuationMode);
    
    if(!initializeTracks(io)) return false;
    if(!initializeFlipperJoints(io)) return false;
    
    // ROS2 initialization is now mandatory for joystick input.
    if(!initializeROS(io)){
        io->os() << "Fatal: Failed to initialize ROS2 interface. Controller cannot operate." << std::endl;
        return false;
    }
        
    return true;
}

bool AizuController::control()
{
    if(!is_active_){
        return true;
    }

    controlTracks();
    updateFlipperTargetPositions();
    
    if(mainActuationMode == Link::JointTorque){ 
        controlJointsWithTorque();
    } else if(mainActuationMode == Link::JointVelocity){ 
        controlJointsWithVelocity();
    } else if(mainActuationMode == Link::JointDisplacement){ 
        controlJointsWithPosition();
    }
    
    return true;
}

bool AizuController::initializeTracks(SimpleControllerIO* io)
{
    tracks.clear(); 
    qprev_track.clear();
    
    vector<string> wheelNames = {"L_TRACK_WHEEL1", "R_TRACK_WHEEL1", "FL_SUB_TRACK_WHEEL1", "FR_SUB_TRACK_WHEEL1", "BL_SUB_TRACK_WHEEL1", "BR_SUB_TRACK_WHEEL1"};
    vector<string> trackNames = {"L_TRACK", "R_TRACK", "FL_SUB_TRACK", "FR_SUB_TRACK", "BL_SUB_TRACK", "BR_SUB_TRACK"};
    
    if(body->link(wheelNames[0])){
        if(mainActuationMode == Link::JointTorque){
            trackActuationMode = Link::JointTorque; 
            trackVelocityRatio = 0.5; 
            kd_mainTrack = 8.0; 
            kd_subTrack = 1.5;
        } else {
            trackActuationMode = Link::JointVelocity; 
            trackVelocityRatio = 4.0;
        }
        return initializeTracks(io, wheelNames);
    } else {
        trackActuationMode = Link::JointVelocity; 
        trackVelocityRatio = 0.5; 
        return initializeTracks(io, trackNames);
    }
}

bool AizuController::initializeTracks(SimpleControllerIO* io, const vector<string>& names)
{
    for(const auto& name : names){
        Link* link = body->link(name); 
        if(!link){ 
            io->os() << format("Track link '{0}' not found\n", name); 
            return false;
        }
        link->setActuationMode(trackActuationMode); 
        io->enableOutput(link); 
        tracks.push_back(link); 
        qprev_track.push_back(link->q());
    }
    return true;
}

bool AizuController::initializeFlipperJoints(SimpleControllerIO* io)
{
    jointInfos.clear();
    
    const double FLIPPER_P_GAIN_TORQUE = 800.0;
    const double FLIPPER_D_GAIN_TORQUE = 20.0;
    const double FLIPPER_P_GAIN_VELOCITY = 0.5;
    
    vector<JointSpec> specs = {
        {"FR_FLIPPER", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY},
        {"FL_FLIPPER", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY},
        {"BR_FLIPPER", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY},
        {"BL_FLIPPER", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY}
    };
    
    return initializeJoints(io, specs);
}

bool AizuController::initializeJoints(SimpleControllerIO* io, const vector<JointSpec>& specs)
{
    for(const auto& s : specs){
        Link* joint = body->link(s.name); 
        if(!joint){ 
            io->os() << format("Joint '{0}' not found\n", s.name); 
            return false;
        }
        
        joint->setActuationMode(mainActuationMode); 
        io->enableIO(joint); 
        
        JointInfo info;
        info.link = joint; 
        info.qref = info.qold = joint->q();
        
        if(mainActuationMode == Link::JointVelocity){ 
            info.kp = s.kp_velocity; 
            info.kd = 0.0;
        } else if(mainActuationMode == Link::JointTorque){ 
            info.kp = s.kp_torque; 
            info.kd = s.kd_torque;
        } else { 
            info.kp = 1.0; 
            info.kd = 0.0;
        }
        
        jointInfos.push_back(info);
    }
    return true;
}

void AizuController::controlTracks()
{
    double hpos = 0.0, vpos = 0.0;
    
    if(latest_joy_msg){
        hpos = getJoyAxis(0, STICK_THRESH) + 0.8 * getJoyAxis(6);
        vpos = -getJoyAxis(1, STICK_THRESH) - 0.8 * getJoyAxis(7);
        
        bool R1 = getJoyButton(5); // RB on F310
        bool R2 = (getJoyAxis(5, STICK_THRESH) < -0.1) || getJoyButton(7); // RT is axis 5
        
        if(R1){ 
            vpos = 0.0; 
            hpos = getJoyAxis(3, STICK_THRESH);
        }
        if(R2){ 
            double boost = 1.5; 
            hpos *= boost; 
            vpos *= boost;
        }
    }
    
    vpos = -vpos; // Invert the joystick vertical axis for tracks

    double dq_L = trackVelocityRatio * (vpos + 0.4 * hpos);
    double dq_R = trackVelocityRatio * (vpos - 0.4 * hpos);
    
    if(trackActuationMode == Link::JointVelocity){
        for(size_t i=0; i<tracks.size(); i+=2){ 
            tracks[i]->dq_target() = dq_L; 
            if(i+1 < tracks.size()) tracks[i+1]->dq_target() = dq_R;
        }
    } else if(trackActuationMode == Link::JointTorque){
        setTrackTorque(L_TRACK, dq_L, kd_mainTrack); 
        setTrackTorque(R_TRACK, dq_R, kd_mainTrack);
        setTrackTorque(FL_SUB_TRACK, dq_L, kd_subTrack); 
        setTrackTorque(FR_SUB_TRACK, dq_R, kd_subTrack);
        setTrackTorque(BL_SUB_TRACK, dq_L, kd_subTrack); 
        setTrackTorque(BR_SUB_TRACK, dq_R, kd_subTrack);
    }
}

void AizuController::setTrackTorque(int id, double dq_target, double kd)
{
    if(id < 0 || id >= static_cast<int>(tracks.size())) return;
    
    Link* axis = tracks[id]; 
    double dq_current = (axis->q() - qprev_track[id]) / dt;
    axis->u() = kd * (dq_target - dq_current); 
    qprev_track[id] = axis->q();
}

void AizuController::updateFlipperTargetPositions()
{
    static const double FLIPPER_GAIN = 0.5;
    
    double collective_pos = 0.0;
    bool equalize_button = false;
    bool front_left_btn = false, front_right_btn = false, back_left_btn = false, back_right_btn = false;

    if(latest_joy_msg){
        equalize_button = getJoyButton(10); // Right stick click
        collective_pos = getJoyAxis(4, STICK_THRESH); // Right Stick Y-axis

        front_left_btn  = getJoyButton(2); // X button
        front_right_btn = getJoyButton(1); // B button
        back_left_btn   = getJoyButton(0); // A button
        back_right_btn  = getJoyButton(3); // Y button
    }
 
    collective_pos = -collective_pos; // Invert the joystick vertical axis for flippers

    if(equalize_button){
        double qa = 0.0;
        for(const auto& j : jointInfos) qa += j.qref;
        qa /= static_cast<double>(jointInfos.size());
        double dqmax = dt * 0.5;
        
        for(auto& j : jointInfos){
            double dq = qa - j.qref;
            dq = std::clamp(dq, -dqmax, dqmax);
            j.qref += dq;
        }
        return;
    }
 
    double dq = dt * FLIPPER_GAIN * collective_pos;
    bool individual_control = front_left_btn || front_right_btn || back_left_btn || back_right_btn;

    if(!individual_control){
        for(auto& j : jointInfos) j.qref += dq;
    } else {
        // Assumes joint order: [0]FR, [1]FL, [2]BR, [3]BL
        if(front_right_btn && jointInfos.size() > 0) jointInfos[0].qref += dq;
        if(front_left_btn  && jointInfos.size() > 1) jointInfos[1].qref += dq;
        if(back_right_btn  && jointInfos.size() > 2) jointInfos[2].qref += dq;
        if(back_left_btn   && jointInfos.size() > 3) jointInfos[3].qref += dq;
    }
}

void AizuController::controlJointsWithTorque()
{
    for(auto& info : jointInfos){ 
        Link* joint = info.link; 
        double q = joint->q(); 
        double dq = (q - info.qold) / dt;
        joint->u() = info.kp * (info.qref - q) + info.kd * (0.0 - dq); 
        info.qold = q;
    }
}

void AizuController::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){ 
        Link* joint = info.link; 
        joint->dq_target() = info.kp * (info.qref - joint->q()) / dt;
    }
}

void AizuController::controlJointsWithPosition()
{
    for(auto& info : jointInfos){ 
        info.link->q_target() = info.qref;
    }
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AizuController)