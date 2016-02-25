if (LCM_FOUND)
  add_executable(runPendulumDynamics runDynamics.cpp)
  target_link_libraries(runPendulumDynamics drakeRBM drakeLCMSystem)
  add_test(NAME runPendulumDynamics COMMAND runPendulumDynamics WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Pendulum)

  add_executable(runPendulumEnergyShaping runEnergyShaping.cpp)
  target_link_libraries(runPendulumEnergyShaping drakeRBM drakeLCMSystem)
#  add_test(NAME runPendulumEnergyShaping COMMAND runPendulumEnergyShaping)
#  set_tests_properties(runPendulumEnergyShaping PROPERTIES TIMEOUT 10)
    # disabled for now since it's not a good test, since it will always timeout

  add_executable(runPendulumLQR runLQR.cpp)
  target_link_libraries(runPendulumLQR drakeRBM drakeLCMSystem)
  add_test(NAME runPendulumLQR COMMAND runPendulumLQR WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Pendulum)

#  write_procman(Pendulum.pmd
#      GROUP "Simulate"
#        NAME "dynamics (cpp)" COMMAND "bash -c 'cd ${CMAKE_CURRENT_SOURCE_DIR} && ${CMAKE_BINARY_DIR}/bin/runPendulumDynamics'"
#      ENDGROUP
#      GROUP "LCM Controllers"
#        NAME "energy shaping (cpp)" COMMAND "${CMAKE_BINARY_DIR}/bin/runPendulumEnergyShaping"
#      ENDGROUP
#      GROUP "Controllers+Simulation In One"
#        NAME "LQR (cpp)" COMMAND "bash -c 'cd ${CMAKE_CURRENT_SOURCE_DIR} && ${CMAKE_BINARY_DIR}/bin/runPendulumLQR'"
#      ENDGROUP
#      NAME "viewer" COMMAND "${CMAKE_INSTALL_PREFIX}/bin/drake-visualizer"
#  )
endif()

# If ROS is installed on the local system, include support for it.
if(DEFINED ENV{ROS_ROOT})
  find_package(roscpp REQUIRED)
  find_package(std_msgs REQUIRED)

  include_directories(${roscpp_INCLUDE_DIRS})
  include_directories(${std_msgs_INCLUDE_DIRS})

  add_executable(runPendulumDynamicsROS runDynamicsROS.cpp)
  target_link_libraries(runPendulumDynamicsROS drakeRBM ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
  # add_test(NAME runPendulumDynamicsROS COMMAND runPendulumDynamicsROS WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Pendulum)
endif()

add_subdirectory(test)
