#ifndef DRAKE_ROSSYSTEM_H
#define DRAKE_ROSSYSTEM_H

#include <unordered_map>
#include <mutex>
#include <thread>
#include <stdexcept>

// #include <lcm/lcm-cpp.hpp>
// #include "lcmtypes/drake/lcmt_drake_signal.hpp"
#include "drake/systems/System.h"
#include "drake/systems/Simulation.h"
// #include "drake/drakeROSSystem_export.h"

// #include "ros/ros.h"

namespace Drake {

  /** @defgroup ros_vector_concept ROSVector<ScalarType> Concept
   * @ingroup vector_concept
   * @brief A specialization of the Vector concept adding the ability to read and publish ROS messages
   *
   * | Valid Expressions (which must be implemented) |  |
   * ------------------|-------------------------------------------------------------|
   * | ROSMessageType  | defined with a typedef                                      |
   * | static std::string channel() const         | return the name of the channel to subscribe to/ publish on      |
   * | bool encode(const double& t, const Vector<double>& x, ROSMessageType& msg) | define the mapping from your ROS type to your Vector type |
   * | bool decode(const ROSMessageType& msg, double& t, Vector<double>& x)  | define the mapping from your Vector type to your ROS type |
   */


  // template <class Vector>
  // bool encode(const double& t, const Vector& x, drake::lcmt_drake_signal& msg) {
  //   msg.timestamp = static_cast<int64_t>(t*1000);
  //   msg.dim = size(x);
  //   auto xvec = toEigen(x);
  //   for (int i=0; i<msg.dim; i++) {
  //     msg.coord.push_back(getCoordinateName(x,i));
  //     msg.val.push_back(xvec(i));
  //   }
  //   return true;
  // }

  // template <class Vector>
  // bool decode(const drake::lcmt_drake_signal& msg, double& t, Vector& x) {
  //   t = double(msg.timestamp)/1000.0;
  //   std::unordered_map<std::string,double> m;
  //   for (int i=0; i<msg.dim; i++) {
  //     m[msg.coord[i]] = msg.val[i];
  //   }
  //   Eigen::Matrix<double,Vector::RowsAtCompileTime,1> xvec(msg.dim);
  //   for (int i=0; i<msg.dim; i++) {
  //     xvec(i) = m[getCoordinateName(x,i)];
  //   }
  //   x = xvec;
  //   return true;
  // }

  namespace internal {

    template<template<typename> class Vector, typename Enable = void>
    class ROSInputSystem {
    public:
      template<typename ScalarType> using StateVector = NullVector<ScalarType>;
      template<typename ScalarType> using InputVector = NullVector<ScalarType>;
      template<typename ScalarType> using OutputVector = Vector<ScalarType>;
      const static bool has_ros_input = false;

      template <typename System>
      ROSInputSystem(const System & wrapped_sys) :
        all_zeros(Eigen::VectorXd::Zero(getNumInputs(wrapped_sys))) { };

      StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                                   const InputVector<double> &u) const {
        return StateVector<double>();
      }

      OutputVector<double> output(const double &t, const StateVector<double> &x,
                                  const InputVector<double> &u) const {
        return all_zeros;
      }

    private:
      OutputVector<double> all_zeros;
    };

    template<template<typename> class Vector>
    class ROSInputSystem<Vector, typename std::enable_if<!std::is_void<typename Vector<double>::ROSMessageType>::value>::type> {
    public:
      template<typename ScalarType> using StateVector = NullVector<ScalarType>;
      template<typename ScalarType> using InputVector = NullVector<ScalarType>;
      template<typename ScalarType> using OutputVector = Vector<ScalarType>;
      const static bool has_ros_input = true;

      template <typename System>
      ROSInputSystem(const System & sys) {

        // TODO: Setup ROS subscriber here

        // lcm::Subscription* sub = lcm->subscribe(Vector<double>::channel(),
        //   &ROSInputSystem<Vector>::handleMessage,this);
        // sub->setQueueCapacity(1);
      };

      virtual ~ROSInputSystem() {};

      void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                         const typename Vector<double>::ROSMessageType* msg) {
        data_mutex.lock();
        decode(*msg, timestamp, data);
        data_mutex.unlock();
      }

      StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                                   const InputVector<double> &u) const {
        return StateVector<double>();
      }

      OutputVector<double> output(const double &t, const StateVector<double> &x,
                                  const InputVector<double> &u) const {
        data_mutex.lock();
        OutputVector<double> y = data;  // make a copy of the data
        data_mutex.unlock();
        return y;
      }

    private:
      mutable std::mutex data_mutex;
      double timestamp;
      OutputVector<double> data;
    };

    template<template<typename> class Vector, typename Enable = void>
    class ROSOutputSystem {
    public:
      template<typename ScalarType> using StateVector = NullVector<ScalarType>;
      template<typename ScalarType> using InputVector = Vector<ScalarType>;
      template<typename ScalarType> using OutputVector = NullVector<ScalarType>;

      ROSOutputSystem(/*const std::shared_ptr<lcm::ROS> &lcm*/) { };

      StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                                   const InputVector<double> &u) const { return StateVector<double>(); }

      OutputVector<double> output(const double &t, const StateVector<double> &x, const InputVector<double> &u) const {
        return OutputVector<double>();
      }
    };

    template<template<typename> class Vector>
    class ROSOutputSystem<Vector, typename std::enable_if<!std::is_void<typename Vector<double>::ROSMessageType>::value>::type> {
    public:
      template<typename ScalarType> using StateVector = NullVector<ScalarType>;
      template<typename ScalarType> using InputVector = Vector<ScalarType>;
      template<typename ScalarType> using OutputVector = NullVector<ScalarType>;

      // ROSOutputSystem(const std::shared_ptr<lcm::ROS> &lcm) : lcm(lcm) { };
      ROSOutputSystem(/*const std::shared_ptr<lcm::ROS> &lcm*/) {};

      StateVector<double> dynamics(const double &t, const StateVector<double> &x,
                                   const InputVector<double> &u) const { return StateVector<double>(); }

      OutputVector<double> output(const double &t, const StateVector<double> &x, const InputVector<double> &u) const {
        typename Vector<double>::ROSMessageType msg;
        if (!encode(t, u, msg))
          throw std::runtime_error(std::string("failed to encode") + msg.getTypeName());
        
        // TODO: Replace with ROS node publish
        // lcm->publish(u.channel(), &msg);
        return OutputVector<double>();
      }

    // private:
      // std::shared_ptr<lcm::ROS> lcm;
    };

    // todo: template specialization for the CombinedVector case

    // class DRAKEROSSYSTEM_EXPORT ROSLoop {
    // public:
    //   bool stop;
    //   lcm::ROS& lcm;

    //   ROSLoop(lcm::ROS& _lcm) : lcm(_lcm), stop(false) {}

    //   void loopWithSelect();
    // };


  } //  end namespace internal

  /** runROS
   * @brief Simulates the system with the (exposed) inputs being read from ROS and the output being published to ROS.
   * @ingroup simulation
   *
   * The input and output vector types must overload a publishROS namespace method; the default for new vectors is to not publish anything.
   */

  template <typename System>
  void runROS(const std::shared_ptr<System>& sys, double t0, double tf,
              const typename System::template StateVector<double>& x0,
              const SimulationOptions& options = default_simulation_options) {

    // if(!lcm->good())
    //   throw std::runtime_error("bad ROS reference");

//    typename System::template OutputVector<double> x = 1;  // useful for debugging
    auto ros_input = std::make_shared<internal::ROSInputSystem<System::template InputVector> >(*sys);
    auto ros_output = std::make_shared<internal::ROSOutputSystem<System::template OutputVector> >();
    auto ros_sys = cascade(ros_input, cascade(sys,ros_output));

    bool has_ros_input = internal::ROSInputSystem<System::template InputVector>::has_ros_input;

    if (has_ros_input && size(x0) == 0 && !sys->isTimeVarying()) {
      // then this is really a static function, not a dynamical system.
      // block on receiving lcm input and process the output exactly when a new input message is received.
      // note: this will never return (unless there is an lcm error)

//      std::cout << "ROS output will be triggered on receipt of an ROS Input" << std::endl;

      double t = 0.0;
      typename System::template StateVector<double> x;
      NullVector<double> u;

      while (1) {
        // if (lcm->handle() != 0) { throw std::runtime_error("something went wrong in lcm.handle"); }
        ros_sys->output(t, x, u);
      }
    } else {
      // internal::ROSLoop ros_loop(*lcm);
      // std::thread ros_thread;
      // if (has_ros_input) {
      //   // only start up the listener thread if I actually have inputs to listen to
      //   ros_thread = std::thread(&internal::ROSLoop::loopWithSelect, &ros_loop);
      // }

      SimulationOptions ros_options = options;
      if (ros_options.realtime_factor < 0.0) ros_options.realtime_factor = 1.0;
      simulate(*ros_sys, t0, tf, x0, ros_options);

      // if (has_ros_input) {
      //   // shutdown the lcm thread
      //   ros_loop.stop = true;
      //   ros_thread.join();
      // }
    }
  }

  /*!
   * Run a simulation of system with a random but feasible initial state.
   *
   * \param sys The system being simulated.
   * \param t0 The initial time.
   * \param tf The final time.
   */
  template <typename System>
  void runROS(const std::shared_ptr<System>& sys, double t0, double tf) {
    runROS(sys, t0, tf, getInitialState(*sys));
  }

  } // end namespace Drake

#endif //DRAKE_ROSSYSTEM_H
