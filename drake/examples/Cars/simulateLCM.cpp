#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_control_cmd_t.hpp"

using namespace std;
using namespace Eigen;
using namespace Drake;

#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

template <typename ScalarType = double>
class DrivingCommand {
 public:
  typedef drake::lcmt_driving_control_cmd_t LCMMessageType;
  static std::string channel() { return "DRIVING_COMMAND"; };

  DrivingCommand(void) : throttle(0), brake(0), steering_angle(0){};
  template <typename Derived>
  DrivingCommand(const Eigen::MatrixBase<Derived>& x)
      : steering_angle(x(0)), throttle(x(1)), brake(x(2)){};

  template <typename Derived>
  DrivingCommand& operator=(const Eigen::MatrixBase<Derived>& x) {
    steering_angle = x(0);
    throttle = x(1);
    brake = x(2);
    return *this;
  }

  friend Eigen::Vector3d toEigen(const DrivingCommand<ScalarType>& vec) {
    Eigen::Vector3d x;
    x << vec.steering_angle, vec.throttle, vec.brake;
    return x;
  }

  friend std::string getCoordinateName(const DrivingCommand<ScalarType>& vec,
                                       unsigned int index) {
    switch (index) {
      case 0:
        return "steering_angle";
      case 1:
        return "throttle";
      case 2:
        return "brake";
    }
    return "error";
  }
  const static int RowsAtCompileTime = 3;

  ScalarType steering_angle;
  ScalarType throttle;
  ScalarType brake;
};

/**
 * A toString method for DrivingCommand.
 */
template <typename ScalarType = double>
std::ostream& operator<<(std::ostream &os, const DrivingCommand<ScalarType> &dc) {
  return os << "[steering_angle = " << dc.steering_angle << ", throttle = " << dc.throttle << ", brake = " << dc.brake << "]";
}

bool decode(const drake::lcmt_driving_control_cmd_t& msg, double& t,
            DrivingCommand<double>& x) {
  t = double(msg.timestamp) / 1000.0;
  x.steering_angle = msg.steering_angle;
  x.throttle = msg.throttle_value;
  x.brake = msg.brake_value;
  return true;
}

/** Driving Simulator
 * Usage:  simulateLCM vehicle_urdf [world_urdf files ...]
 */

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " vehicle_urdf [world sdf files ...]"
              << std::endl;
    return 1;
  }

  // todo: consider moving this logic into the RigidBodySystem class so it can
  // be reused
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;
  // DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::FIXED;


  auto rigid_body_sys = make_shared<RigidBodySystem>();

  // The following variable, weld_to_frame, is only needed if the model is a
  // URDF file. It is needed since URDF does not specify the location and
  // orientation of the car's root node in the world. If the model is an SDF,
  // weld_to_frame is ignored by the parser.
  auto weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(),
      "world",
      nullptr,  // not used since the robot is attached to the world
      Eigen::Vector3d(0, 0, 0.378326),  // xyz of the car's root link
      Eigen::Vector3d(0, 0, 0));       // rpy of the car's root link

  rigid_body_sys->addRobotFromFile(argv[1], floating_base_type, weld_to_frame);

  auto const & tree = rigid_body_sys->getRigidBodyTree();
  for (int i=2; i<argc; i++)
    tree->addRobotFromSDF(argv[i], DrakeJoint::FIXED);  // add environment

  if (argc < 3) {  // add flat terrain
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -box_depth / 2;  // top of the box is at z=0
    auto& world = tree->bodies[0];
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world->addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, world), *world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

  PRINT_VAR(tree->num_positions)
  PRINT_VAR(tree->num_velocities)
  PRINT_VAR(tree->actuators.size())
  for (auto actuator: tree->actuators) {
    PRINT_VAR(actuator.name)
    PRINT_VAR(actuator.body->linkname)
    PRINT_VAR(actuator.reduction)
    PRINT_VAR(actuator.effort_limit_min)
    PRINT_VAR(actuator.effort_limit_max)
  }
  PRINT_VAR(tree->a_grav.transpose())
  PRINT_VAR(tree->B)
  PRINT_VAR(tree->bodies.size());
  for(auto body: tree->bodies) {
    PRINT_VAR(body->linkname);
    PRINT_VAR(body->mass);
    PRINT_VAR(body->hasParent())
    PRINT_VAR(body->I)
    if(body->hasParent()){
      const DrakeJoint& joint = body->getJoint();
      PRINT_VAR(joint.getName())
      PRINT_VAR(joint.getTransformToParentBody().matrix())
      PRINT_VAR(joint.getNumPositions())
      PRINT_VAR(joint.getNumVelocities())
      PRINT_VAR(joint.isFloating())
      PRINT_VAR(joint.zeroConfiguration())
      PRINT_VAR(joint.getJointLimitMin().transpose())
      PRINT_VAR(joint.getJointLimitMax().transpose())
    } else
      std::cout << "link has no parent joint!" << std::endl;
    std::cout << "===============================" << std::endl;
  }
  PRINT_VAR(tree->frames.size());
  for (auto frame: tree->frames) {
    PRINT_VAR(frame->name)
    PRINT_VAR(frame->frame_index)
    PRINT_VAR(frame->body->linkname)
    PRINT_VAR(frame->transform_to_body.matrix())
  }
  std::cout << "collision elements:\n" << *tree.get() << std::endl;

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->num_positions),
      Kd(getNumInputs(*rigid_body_sys), tree->num_velocities);
  Matrix<double, Eigen::Dynamic, 3> map_driving_cmd_to_x_d(
      tree->num_positions + tree->num_velocities, 3);

  {  // setup PD controller for throttle and steering
    double kpSteering = 400, kdSteering = 80, kThrottle = 100;
    Kp.setZero();
    Kd.setZero();
    map_driving_cmd_to_x_d.setZero();

    for (int actuator_idx = 0; actuator_idx < tree->actuators.size();
         actuator_idx++) {

      const std::string & actuatorName = tree->actuators[actuator_idx].name;

      if (strcmp(actuatorName.c_str(), "steering_angle") == 0 ||
          strcmp(actuatorName.c_str(), "steering") == 0) {
        auto const& b = tree->actuators[actuator_idx].body;
        Kp(actuator_idx, b->position_num_start) = kpSteering;  // steering
        Kd(actuator_idx, b->velocity_num_start) = kdSteering;  // steeringdot
        map_driving_cmd_to_x_d(b->position_num_start, 0) = 1;  // steering command

      } else if (strncmp(actuatorName.c_str(), "throttle", 8) == 0 ||
                 strcmp(actuatorName.c_str(), "right_wheel_joint") == 0 ||
                 strcmp(actuatorName.c_str(), "left_wheel_joint") == 0) {  // intentionally match all throttle_ inputs

        auto const& b = tree->actuators[actuator_idx].body;
        Kd(actuator_idx, b->velocity_num_start) = kThrottle;  // throttle
        map_driving_cmd_to_x_d(tree->num_positions + b->velocity_num_start, 1) =
            20;  // throttle (velocity) command
        map_driving_cmd_to_x_d(tree->num_positions + b->velocity_num_start, 2) =
            -20;  // braking (velocity) command
      }
    }
  }
  auto vehicle_with_pd =
      make_shared<PDControlSystem<RigidBodySystem>>(rigid_body_sys, Kp, Kd);
  auto vehicle_sys = cascade(
      make_shared<
          Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>>(
          map_driving_cmd_to_x_d),
      vehicle_with_pd);
  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);
  auto sys = cascade(vehicle_sys, visualizer);

  SimulationOptions options = default_simulation_options;
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping =
      rigid_body_sys->penetration_stiffness / 10.0;
  rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite friction
  options.initial_step_size = 5e-3;
  options.timeout_seconds = numeric_limits<double>::infinity();
  options.realtime_factor = 0; // run as fast as possible
  // options.realtime_factor = 1; // run at real-time
  // options.realtime_factor = 10; // run at 10X slower than real-time

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  // todo:  call getInitialState instead?  (but currently, that would require
  // snopt).  needs #1627
  // I'm getting away without it, but might be generating large internal forces
  // initially as the ackerman constraint (hopefully) gets enforced by the
  // stabilization terms.

  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), x0, options);
  //  simulate(*sys, 0, std::numeric_limits<double>::infinity(), x0, options);

  return 0;
}
