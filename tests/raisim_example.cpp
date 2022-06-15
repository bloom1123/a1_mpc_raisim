#include <string>
#include <list>
#include <vector>

#include <Eigen/Dense>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

using namespace std;

int main() {

  //Construct simulator
  raisim::World world;
  double time_step = 0.001;
  world.setTimeStep(time_step);
  raisim::RaisimServer server(&world);
  world.addGround();
  server.launchServer();

  string urdf_path = "/home/user/raisim_workspace/raisim_mpc/a1_data/urdf/a1.urdf";
  auto a1 = world.addArticulatedSystem(urdf_path);
  a1->setName("a1");

  Eigen::VectorXd jointConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointVel(18), jointPgain(18), jointDgain(18);

  jointPgain.setZero();
  jointPgain.tail(12).setConstant(200.0);

  jointDgain.setZero();
  jointDgain.tail(12).setConstant(10.0);

  jointVelocityTarget.setZero();
  jointConfig << 0.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0, 0.0, 0.8, -1.6, 
    0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6;
  jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  
  a1->setState(jointConfig, jointVel);
  a1->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  a1->setPdGains(jointPgain, jointDgain);
  a1->setPdTarget(jointConfig, jointVelocityTarget);
  a1->setGeneralizedForce(Eigen::VectorXd::Zero(a1->getDOF()));
  a1->setGeneralizedCoordinate(jointConfig);

  for (int i=0; i<10000000; i++) {
    world.integrate();
    std::this_thread::sleep_for(std::chrono::seconds(
      size_t(time_step)));
  }
  
  server.killServer();
}
