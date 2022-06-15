#include "gait_generator.hpp"
#include <iostream>

GaitGenerator::GaitGenerator(
    VectorXd _stance_duration, 
    VectorXd _duty_factor,
    VectorXi _initial_leg_state,
    VectorXd _initial_leg_phase)
    
    :stance_duration(_stance_duration),
    duty_factor(_duty_factor),
    initial_leg_phase(_initial_leg_phase),
    initial_leg_state(_initial_leg_state) {

    swing_duration = stance_duration.array() / 
        duty_factor.array() - stance_duration.array();
    
    for (int i = 0; i < 4; i++) {
        if (initial_leg_state[i] == 0) {
            initial_state_ratio_in_cycle[i] = 1 - duty_factor[i];
            next_leg_state[i] = 1;
        }
        else {
            initial_state_ratio_in_cycle[i] = duty_factor[i];
            next_leg_state[i] = 0;
        };
    };
    
    reset(); 
};

void GaitGenerator::reset() {
    normalized_phase = VectorXd::Zero(4);
    leg_state = initial_leg_state;
    desired_leg_state = initial_leg_state;
};

void GaitGenerator::update(double current_time) {
    for (int leg_id = 0; leg_id < 4; leg_id++) {

        double full_cycle_period = 
            stance_duration[leg_id] / duty_factor[leg_id];
        double augmented_time = current_time + 
            initial_leg_phase[leg_id] * full_cycle_period;
        double phase_in_full_cycle = fmod(
            augmented_time, full_cycle_period) / full_cycle_period;
        double ratio = initial_state_ratio_in_cycle[leg_id];

        if (phase_in_full_cycle < ratio) {
            desired_leg_state[leg_id] = initial_leg_state[leg_id];
            normalized_phase[leg_id] = phase_in_full_cycle / ratio;
        }
        else {
            desired_leg_state[leg_id] = next_leg_state[leg_id];
            normalized_phase[leg_id] = (phase_in_full_cycle - ratio)
                / (1 - ratio);
        };

        leg_state[leg_id] = desired_leg_state[leg_id];
    };
}