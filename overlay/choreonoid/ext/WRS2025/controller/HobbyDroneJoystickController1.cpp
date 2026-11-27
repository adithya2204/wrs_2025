/**
   HobbyDrone Controller with ROS2 Joy3 Support (ROS2 Only)
   @author Kenta Suzuki (modified for ROS2)
   If no joy3 messages, drone stays still
*/

#include <cnoid/EigenUtil>
#include <cnoid/Rotor>
#include <cnoid/SimpleController>
#include <cnoid/SimplePilot>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace cnoid;

namespace {
const double pgain[] = { 10.0, 1.0, 1.0, 0.01 };
const double dgain[] = { 5.0, 1.0, 1.0, 0.002 };
} // namespace

class HobbyDroneController1 : public SimpleController
{
    SimplePilot pilot;
    DeviceList<Rotor> rotors;
    Vector4 zref;
    Vector4 dzref, dzold;
    Vector2 dxref;
    bool manualMode;
    double timeStep;

    struct ActionInfo {
        int actionId;
        int buttonId;
        bool prevButtonState;
        bool stateChanged;
        ActionInfo(int actionId, int buttonId)
            : actionId(actionId), buttonId(buttonId), prevButtonState(false), stateChanged(false) { }
    };
    std::vector<ActionInfo> actions;

    // ROS2 members
    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
    sensor_msgs::msg::Joy::SharedPtr currentJoyMsg;
    std::mutex joy_mutex;
    bool hasReceivedJoyMsg;
    int messageCount;
    SimpleControllerIO* ioPtr;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioPtr = io;
        rotors = io->body()->devices();
        manualMode = false;
        hasReceivedJoyMsg = false;
        messageCount = 0;
        
        pilot.setStickMode(StickMode::MODE1);
        if(pilot.initialize(io)) {
            if(!pilot.findRateGyroSensor("GyroSensor")) { 
                io->os() << "✗ Failed to find GyroSensor" << std::endl;
                return false; 
            }
        }
        
        for(auto opt : io->options()) {
            if(opt == "manual") { manualMode = true; }
        }
        
        for(auto& rotor : rotors) { io->enableInput(rotor); }
        
        zref = pilot.zrpy();
        dzref = dzold = Vector4::Zero();
        dxref = Vector2::Zero();
        timeStep = io->timeStep();
        actions = { {0, 0}, {1, 1} };
        
        io->os() << "=== HobbyDroneController1 (ROS2 Only Mode) ===" << std::endl;
        
        if(!initializeROS(io)) {
            io->os() << "✗ Failed to initialize ROS2" << std::endl;
            return false;
        }
        
        io->os() << "✓ Initialization complete" << std::endl;
        io->os() << "✓ Waiting for /joy3 messages (drone will stay still until messages arrive)" << std::endl;
        
        return true;
    }

    bool initializeROS(SimpleControllerIO* io)
    {
        try {
            if(!rclcpp::ok()) { 
                io->os() << "Initializing ROS2 context..." << std::endl;
                rclcpp::init(0, nullptr); 
            }
            
            rosNode = rclcpp::Node::make_shared("hobby_drone_controller1");
            
            joySubscriber = rosNode->create_subscription<sensor_msgs::msg::Joy>(
                "/joy3", 10, 
                std::bind(&HobbyDroneController1::joyCallback, this, std::placeholders::_1)
            );
            
            io->os() << "✓ ROS2 node 'hobby_drone_controller1' created" << std::endl;
            io->os() << "✓ Subscribed to /joy3" << std::endl;
            
            return true;
        } catch (const std::exception& e) {
            io->os() << "✗ ROS2 initialization failed: " << e.what() << std::endl;
            return false;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joy_mutex);
        currentJoyMsg = msg;
        
        if(!hasReceivedJoyMsg) {
            hasReceivedJoyMsg = true;
            if(ioPtr) {
                ioPtr->os() << "✓ First /joy3 message received!" << std::endl;
                ioPtr->os() << "   Axes: " << msg->axes.size() 
                           << ", Buttons: " << msg->buttons.size() << std::endl;
            }
        }
        
        messageCount++;
        
        // Debug: Print every 50th message
        if(messageCount % 50 == 0 && ioPtr) {
            ioPtr->os() << "Joy3 msg #" << messageCount 
                       << " | Throttle: " << (msg->axes.size() > 0 ? msg->axes[0] : 0.0)
                       << " | Pilot ON: " << (pilot.on() ? "YES" : "NO") << std::endl;
        }
    }

    double getJoystickPosition(int axisId)
    {
        std::lock_guard<std::mutex> lock(joy_mutex);
        
        // If no message received yet, return 0.0 (neutral/stay still)
        if(!currentJoyMsg) { return 0.0; }
        
        if(axisId >= static_cast<int>(currentJoyMsg->axes.size())) { 
            return 0.0; 
        }
        
        return currentJoyMsg->axes[axisId];
    }

    bool getJoystickButtonState(int buttonId)
    {
        std::lock_guard<std::mutex> lock(joy_mutex);
        
        // If no message received yet, return false
        if(!currentJoyMsg) { return false; }
        
        if(buttonId >= static_cast<int>(currentJoyMsg->buttons.size())) { 
            return false; 
        }
        
        return currentJoyMsg->buttons[buttonId] > 0;
    }

    virtual bool control() override
    {
        // Spin ROS2 to process callbacks
        if(rosNode) { 
            rclcpp::spin_some(rosNode); 
        }
        
        // Warning if no messages received after some time
        static int controlCount = 0;
        controlCount++;
        if(controlCount == 200 && !hasReceivedJoyMsg && ioPtr) {
            ioPtr->os() << "⚠ WARNING: No /joy3 messages received yet. Drone staying still." << std::endl;
            ioPtr->os() << "   Check: ros2 topic list | grep joy3" << std::endl;
            ioPtr->os() << "   Check: ros2 topic echo /joy3" << std::endl;
        }
        
        // Read pilot state
        pilot.readCurrentState();
        
        // Process button actions (only if we have joy messages)
        if(hasReceivedJoyMsg) {
            for(auto& info : actions) {
                bool buttonState = getJoystickButtonState(info.buttonId);
                if(buttonState && !info.prevButtonState) {
                    if(info.actionId == 0) { 
                        pilot.on(!pilot.on());
                        if(ioPtr) {
                            ioPtr->os() << "Button 0: Pilot " 
                                       << (pilot.on() ? "ENABLED" : "DISABLED") << std::endl;
                        }
                    } 
                    else if(info.actionId == 1) { 
                        pilot.switchMode();
                        if(ioPtr) {
                            ioPtr->os() << "Button 1: Switched mode" << std::endl;
                        }
                    }
                }
                info.prevButtonState = buttonState;
            }
        }

        // Get joystick positions (will be 0.0 if no messages)
        static const int controlID[] = { 
            FlightControl::THROTTLE, 
            FlightControl::AILERON, 
            FlightControl::ELEVATOR, 
            FlightControl::RUDDER 
        };
        
        double pos[4];
        for(int i = 0; i < 4; i++) {
            pos[i] = getJoystickPosition(pilot.axisId(controlID[i]));
            if(fabs(pos[i]) < 0.2) { pos[i] = 0.0; }
        }

        // Flight control calculations
        Vector4 fz = Vector4::Zero();
        Vector4 z = pilot.zrpy();
        Vector4 dz = pilot.dzrpy();
        Vector4 ddz = (dz - dzold) / timeStep;
        Vector2 dx_local = pilot.dxy_local();
        Vector2 ddx_local = pilot.ddxy_local();
        double gc = pilot.gravityCompensation(4);

        // Safety: Disable if tilted too much
        if((fabs(degree(z[1])) > 45.0) || (fabs(degree(z[2])) > 45.0)) { 
            pilot.on(false); 
        }
        
        if(!pilot.on()) { 
            zref[0] = 0.0; 
            dzref[0] = 0.0; 
        }

        // Control loop
        for(int i = 0; i < 4; i++) {
            if(i == 3) {
                dzref[i] = -1.0 * pos[i];
                fz[i] = (dzref[i] - dz[i]) * pgain[i] + (0.0 - ddz[i]) * dgain[i];
            } else {
                if(i == 0) { 
                    zref[i] += -0.001 * pos[i]; 
                } 
                else {
                    if(manualMode) { 
                        zref[i] = (i == 1 ? 1.0 : -1.0) * pos[i]; 
                    } 
                    else {
                        int j = i - 1;
                        dxref[j] = -1.0 * pos[i];
                        zref[i] = 1.0 * (dxref[j] - dx_local[1 - j]) + 1.0 * (0.0 - ddx_local[1 - j]);
                    }
                }
                if(i == 1) { zref[i] *= -1.0; }
                fz[i] = (zref[i] - z[i]) * pgain[i] + (0.0 - dz[i]) * dgain[i];
            }
        }
        dzold = dz;

        // Apply forces to rotors
        static const double TD[4][4] = {
            {1, -1, -1, -1}, 
            {1, 1, -1, 1}, 
            {1, 1, 1, -1}, 
            {1, -1, 1, 1}
        };
        static const double ATD[] = { -1.0, 1.0, -1.0, 1.0 };

        for(size_t i = 0; i < rotors.size(); i++) {
            Rotor* rotor = rotors[i];
            double ft = pilot.on() ? 
                gc + TD[i][0]*fz[0] + TD[i][1]*fz[1] + TD[i][2]*fz[2] + TD[i][3]*fz[3] : 
                0.0;
            rotor->force() = ft;
            rotor->torque() = ATD[i] * ft;
            rotor->notifyStateChange();
        }
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HobbyDroneController1)