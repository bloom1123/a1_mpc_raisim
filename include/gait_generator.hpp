#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::VectorXi;
using Eigen::Vector4i;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

class GaitGenerator {
    public:
    VectorXd normalized_phase;
    VectorXi leg_state;     //0 = swing, 1 = stance
    VectorXi desired_leg_state;
    Vector4i next_leg_state;

    VectorXd stance_duration;
    VectorXd duty_factor;
    Vector4d swing_duration;
    VectorXd initial_leg_phase;
    VectorXi initial_leg_state;

    Vector4d initial_state_ratio_in_cycle;

    GaitGenerator(){};
    GaitGenerator(
        VectorXd _stance_duration, 
        VectorXd _duty_factor,
        VectorXi _initial_leg_state,
        VectorXd _initial_leg_phase
    );

    void reset();
    void update(double current_time);
};