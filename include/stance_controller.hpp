#include "a1.hpp"
#include "gait_generator.hpp"
#include "mpc_osqp.hpp"

using std::vector;

class StanceController {
    public:
    A1* robot;
    GaitGenerator* gait_generator;
    VectorXd desired_speed;
    double desired_twisting_speed;
    double desired_body_height;
    double body_mass;
    int num_legs = 4;
    vector<double> friction_coeffs{0.45, 0.45, 0.45, 0.45};
    vector<double> body_inertia {0.07335, 0, 0, 0, 0.25068,
        0, 0, 0, 0.25447};
    QPSolverName qp_solver = OSQP;
    map<int,double> last_action;

    StanceController(){};
    StanceController(
        A1* _robot,
        GaitGenerator* _gait_generator,
        VectorXd _desired_speed,
        double _desired_twisting_speed,
        double _desired_body_height,
        double _body_mass);
    
    std::map<int,double> getAction(std::vector<double> mpc_weights, double mass, vector<double> inertia);
};