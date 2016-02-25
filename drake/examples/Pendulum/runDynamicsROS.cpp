// Copyright 2016 The Drake Authors

#include <iostream>

#include "drake/util/Polynomial.h"
#include "drake/examples/Pendulum/Pendulum.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/BotVisualizerROS.h"
#include "drake/systems/ROSSystem.h"

#include "ros/ros.h"

int main(int argc, char* argv[]) {
  std::cout << "runPendulumDynamicsROS: Simulation started!" << std::endl;

  ros::init(argc, argv, "runPendulumDynamicsROS");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  // if(!lcm->good())
  //  return 1;

  auto pp = std::make_shared<Pendulum>();
  auto vv = std::make_shared<Drake::BotVisualizerROS<PendulumState>>(
    Drake::getDrakePath() + "/examples/Pendulum/Pendulum.urdf",
    DrakeJoint::FIXED);

  PendulumState<double> x0;
  x0.theta = 1;
  x0.thetadot = 0;
//  cout << "PendulumState::x0 = " << x0 << endl;

  auto sys = cascade(pp, vv);

  cout << "coords:" << getCoordinateName(x0, 0) << ", "
       << getCoordinateName(x0, 1) << endl;

  Drake::SimulationOptions options;
  options.realtime_factor = 1.0;

//  simulate(*sys,0,10,x0,options);
  Drake::runROS(sys, 0, 10, x0, options);

  // ros::waitForShutdown();
}
