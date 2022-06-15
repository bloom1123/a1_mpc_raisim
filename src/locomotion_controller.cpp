#include "locomotion_controller.hpp"

void LocomotionController::reset() {
    reset_time = robot->getTimeSinceReset();
    time_since_reset = 0;
    gait_generator->reset();
    swing_controller->reset();
}

void LocomotionController::update(VectorXd lin_speed, double ang_speed) {
    swing_controller->desired_speed = lin_speed;
    swing_controller->desired_twisting_speed = ang_speed;
    stance_controller->desired_speed = lin_speed;
    stance_controller->desired_twisting_speed = ang_speed;
    
    time_since_reset = robot->getTimeSinceReset() - reset_time;
    gait_generator->update(time_since_reset);
    swing_controller->update();
}

VectorXd LocomotionController::getAction(bool mpc_step, std::vector<double> mpc_weights, double mass, vector<double> inertia) {
    
    auto swing_action = swing_controller->getAction();

    //Only recompute stance action on MPC step
    map<int,double> stance_action;
    if (mpc_step) {
        stance_action = stance_controller->getAction(mpc_weights, mass, inertia);
    }
    else {
        stance_action = stance_controller->last_action;
    }
    
    VectorXd action(12);

    for (int j_id = 0; j_id < 12; j_id++) {
        if (stance_action.find(j_id) == stance_action.end()) {
            //swing action
            action[j_id] = swing_action[j_id];
        }
        else {
            //stance action
            action[j_id] = stance_action[j_id];
        }
    }

    return action;
}