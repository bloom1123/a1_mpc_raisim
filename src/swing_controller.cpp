#include "swing_controller.hpp"

double generate_parabola(
    double phase, double start, double mid, double end) {

    double mid_phase = 0.5;
    double delta_1 = mid - start;
    double delta_2 = end - start;
    double delta_3 = pow(mid_phase,2) - mid_phase;
    double coef_a = (delta_1 - delta_2 * mid_phase) / delta_3;
    double coef_b = (delta_2 * pow(mid_phase,2) - delta_1) / delta_3;
    double coef_c = start;

    return coef_a * pow(phase,2) + coef_b * phase + coef_c;
};

VectorXd generate_swing_trajectory(
    double input_phase, VectorXd start_pos, VectorXd end_pos) {
    
    auto phase = input_phase;
    if (input_phase <= 0.5) {
        phase = 0.8 * sin(input_phase * M_PI);
    }
    else {
        phase = 0.8 + (input_phase - 0.5) * 0.4;
    };

    auto x = (1 - phase) * start_pos[0] + phase * end_pos[0];
    auto y = (1 - phase) * start_pos[1] + phase * end_pos[1];
    auto max_clearance = 0.15;
    auto mid = std::max(end_pos[2], start_pos[2]) + max_clearance;
    auto z = generate_parabola(phase, start_pos[2], mid, end_pos[2]);

    VectorXd trajectory {{x, y, z}};
    return trajectory; 
};

SwingController::SwingController(
    A1* _robot,
    GaitGenerator* _gait_generator,
    Vector3d _desired_speed,
    double _desired_twisting_speed,
    double _desired_height,
    double _foot_clearance)

    :robot(_robot),
    gait_generator(_gait_generator),
    last_leg_state(gait_generator->desired_leg_state),
    desired_twisting_speed(_desired_twisting_speed) {

    desired_speed << _desired_speed[0], _desired_speed[1], 0;
    desired_height << 0, 0, (_desired_height - _foot_clearance);

    reset();
    }

void SwingController::reset() {
    last_leg_state = gait_generator->desired_leg_state;
    phase_switch_local_foot_pos = robot->getFootPositionsInBaseFrame();
    joint_angles_dict.clear();
}

void SwingController::update() {
    auto new_leg_state = gait_generator->desired_leg_state;

    //Detects phase switch for each leg so we can remember the feet position at
    //the beginning of the swing phase.
    for (int leg_id = 0; leg_id < 4; leg_id++) {
        if((new_leg_state[leg_id] == 0) && 
            (new_leg_state[leg_id] != last_leg_state[leg_id])) {
            
            phase_switch_local_foot_pos.row(leg_id) = 
            robot->getFootPositionsInBaseFrame().row(leg_id) - robot->hip_offsets.row(leg_id);
        };
    };

    last_leg_state = new_leg_state;
};

map<int,double> SwingController::getAction() {
    auto com_velocity = robot->getComVelocity();
    com_velocity[2] = 0;
    auto rpy_rate = robot->getBaseRollPitchYawRate();
    auto hip_positions = robot->hip_offsets;
    
    //update the stored joint angles for swing legs
    for (int leg_id = 0; leg_id < 4; leg_id++) {
        if (gait_generator->leg_state[leg_id] == 1) {continue;};
        
        VectorXd hip_offset = hip_positions.row(leg_id);
        VectorXd twisting_vector {{-hip_offset[1], hip_offset[0], 0}};
        VectorXd hip_horizontal_velocity = 
            com_velocity + rpy_rate[2] * twisting_vector;
        
        VectorXd target_hip_horizontal_velocity = (
            desired_speed + desired_twisting_speed * twisting_vector);
        
        VectorXd hip_l_vec {{0, hip_l[leg_id], 0}};
        VectorXd foot_target_position = (
            hip_horizontal_velocity *
            gait_generator->stance_duration[leg_id] / 2 + kp *
            (target_hip_horizontal_velocity - hip_horizontal_velocity)
            ) - desired_height + hip_l_vec;

        auto foot_position = generate_swing_trajectory(
            gait_generator->normalized_phase[leg_id],
            phase_switch_local_foot_pos.row(leg_id),
            foot_target_position);

        auto [joint_ids, joint_angles] = 
        robot->getJointAnglesFromLocalFootPosition(leg_id, foot_position);

        //Update joint angles
        for (int j_id = 0; j_id < 3; j_id++) {
            tuple<double,int> angle_and_l_id {joint_angles[j_id], leg_id};
            joint_angles_dict[joint_ids[j_id]] = angle_and_l_id;
        }
    }
    
    map<int,double> action;
    auto kps = robot->motor_kp;
    auto kds = robot->motor_kd;

    //Build a dictionary of true joint states
    map<int,Vector2d> joint_states_dict;
    auto true_joint_angles = robot->getJointAngles();
    auto true_joint_vels = robot->getJointVelocities();
    
    for (int j_id = 0; j_id < 12; j_id++) {
        Vector2d state {true_joint_angles[j_id], true_joint_vels[j_id]};
        joint_states_dict[j_id] = state;
    }
    
    //Calculate forces using a PD controller
    for (auto &[j_id, angle_and_l_id] : joint_angles_dict) {
        auto leg_id = std::get<1>(angle_and_l_id);

        if (gait_generator->desired_leg_state[leg_id] == 0) {
            auto torque = -(kps[j_id]*(joint_states_dict[j_id][0]-
                std::get<0>(angle_and_l_id)))-kds[j_id] * (joint_states_dict[j_id][1]);
            
            action[j_id] = torque;
        } 
    }

    return action;
};