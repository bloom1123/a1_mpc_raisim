#include "swing_controller.hpp"

class LocomotionController {
    public:
    A1* robot;
    GaitGenerator* gait_generator;
    SwingController* swing_controller;
    StanceController* stance_controller;
    double reset_time = robot->getTimeSinceReset();
    double time_since_reset = 0;

    LocomotionController(){};
    LocomotionController(
        A1* _robot,
        GaitGenerator* _gait_generator,
        SwingController* _swing_controller,
        StanceController* _stance_controller
    )
    :robot(_robot),
    gait_generator(_gait_generator),
    swing_controller(_swing_controller),
    stance_controller(_stance_controller) {};

    void reset();
    void update(VectorXd lin_speed, double ang_speed);
    VectorXd getAction(
        bool mpc_step, std::vector<double> mpc_weights, double mass, vector<double> inertia);
};