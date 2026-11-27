/**
   HobbyDrone Controller with ROS2 Joy2 Support (ROS2-only, no fallback)
   Author: Kenta Suzuki (modified)
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

class HobbyDroneController : public SimpleController
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

    // ROS2-only members
    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
    sensor_msgs::msg::Joy::SharedPtr currentJoyMsg;
    std::mutex joy_mutex;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        // Setup rotors and pilot
        rotors = io->body()->devices();
        manualMode = false;
        pilot.setStickMode(StickMode::MODE1);
        if(pilot.initialize(io)) {
            if(!pilot.findRateGyroSensor("GyroSensor")) { return false; }
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
        pilot.on(true); 

        // Initialize ROS2 node + subscription
        if(!initializeROS(io)) {
            io->os() << "Failed to initialize ROS2" << std::endl;
            return false;
        }

        return true;
    }

    bool initializeROS(SimpleControllerIO* io)
    {
        try {
            // Initialize rclcpp only if not already initialized
            if(!rclcpp::ok()) {
                // We use argc=0, argv=nullptr to avoid interfering with parent's args
                rclcpp::init(0, nullptr);
            }

            rosNode = rclcpp::Node::make_shared("hobby_drone_controller");
            // Create subscription to /joy2
            joySubscriber = rosNode->create_subscription<sensor_msgs::msg::Joy>(
                "/joy2", rclcpp::QoS(3),
                std::bind(&HobbyDroneController::joyCallback, this, std::placeholders::_1)
            );

            RCLCPP_INFO(rosNode->get_logger(), "HobbyDrone controller initialized with ROS2 joy2 subscriber");
            return true;
        } catch (const std::exception& e) {
            // Use global logger if rosNode isn't available
            RCLCPP_ERROR(rclcpp::get_logger("hobby_drone"), "Failed to initialize ROS: %s", e.what());
            return false;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joy_mutex);
        currentJoyMsg = msg;
    }

    double getJoystickPosition(int axisId)
    {
        std::lock_guard<std::mutex> lock(joy_mutex);
        if(!currentJoyMsg) return 0.0;
        if(axisId < 0 || axisId >= static_cast<int>(currentJoyMsg->axes.size())) return 0.0;
        return currentJoyMsg->axes[axisId];
    }

    bool getJoystickButtonState(int buttonId)
    {
        std::lock_guard<std::mutex> lock(joy_mutex);
        if(!currentJoyMsg) return false;
        if(buttonId < 0 || buttonId >= static_cast<int>(currentJoyMsg->buttons.size())) return false;
        return currentJoyMsg->buttons[buttonId] > 0;
    }

    virtual bool control() override
    {
        // Pump ROS callbacks for the node (non-blocking)
        if(rosNode) {
            rclcpp::spin_some(rosNode);
        }

        pilot.readCurrentState();

        for(auto& info : actions) {
            bool buttonState = getJoystickButtonState(info.buttonId);
            if(buttonState && !info.prevButtonState) {
                if(info.actionId == 0) { pilot.on(!pilot.on()); }
                else if(info.actionId == 1) { pilot.switchMode(); }
            }
            info.prevButtonState = buttonState;
        }

        static const int controlID[] = { FlightControl::THROTTLE, FlightControl::AILERON, FlightControl::ELEVATOR, FlightControl::RUDDER };
        double pos[4];
        for(int i = 0; i < 4; i++) {
            pos[i] = getJoystickPosition(pilot.axisId(controlID[i]));
            if(fabs(pos[i]) < 0.2) { pos[i] = 0.0; }
        }

        Vector4 fz = Vector4::Zero();
        Vector4 z = pilot.zrpy();
        Vector4 dz = pilot.dzrpy();
        Vector4 ddz = (dz - dzold) / timeStep;
        Vector2 dx_local = pilot.dxy_local();
        Vector2 ddx_local = pilot.ddxy_local();
        double gc = pilot.gravityCompensation(4);

        if((fabs(degree(z[1])) > 45.0) || (fabs(degree(z[2])) > 45.0)) { pilot.on(false); }
        if(!pilot.on()) { zref[0] = 0.0; dzref[0] = 0.0; }

        for(int i = 0; i < 4; i++) {
            if(i == 3) {
                dzref[i] = -1.0 * pos[i];
                fz[i] = (dzref[i] - dz[i]) * pgain[i] + (0.0 - ddz[i]) * dgain[i];
            } else {
                if(i == 0) { zref[i] += -0.001 * pos[i]; }
                else {
                    if(manualMode) { zref[i] = (i == 1 ? 1.0 : -1.0) * pos[i]; }
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

        static const double TD[4][4] = {{1, -1, -1, -1}, {1, 1, -1, 1}, {1, 1, 1, -1}, {1, -1, 1, 1}};
        static const double ATD[] = { -1.0, 1.0, -1.0, 1.0 };

        for(size_t i = 0; i < rotors.size(); i++) {
            Rotor* rotor = rotors[i];
            double ft = pilot.on() ? gc + TD[i][0]*fz[0] + TD[i][1]*fz[1] + TD[i][2]*fz[2] + TD[i][3]*fz[3] : 0.0;
            rotor->force() = ft;
            rotor->torque() = ATD[i] * ft;
            rotor->notifyStateChange();
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HobbyDroneController)
