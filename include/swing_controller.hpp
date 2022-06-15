#include "stance_controller.hpp"
#include <algorithm>

using Eigen::Vector2d;
using std::tuple;

double generate_parabola(
    double phase, double start, double mid, double end);

VectorXd generate_swing_trajectory(
    double input_phase, VectorXd start_pos, VectorXd end_pos);

class SwingController {
    public:
    A1* robot;
    GaitGenerator* gait_generator;
    VectorXi last_leg_state;
    Vector3d desired_speed, desired_height;
    double desired_twisting_speed;
    map<int,tuple<double,int>> joint_angles_dict;
    MatrixXd phase_switch_local_foot_pos;

    double kp = 0.03;
    Vector4d hip_l {-0.08505, 0.08505, -0.08505, 0.08505};

    SwingController(){};
    SwingController(
        A1* _robot,
        GaitGenerator* _gait_generator,
        Vector3d _desired_speed,
        double _desired_twisting_speed,
        double _desired_height,
        double _foot_clearance
    );

    void reset();
    void update();
    map<int,double> getAction();
};