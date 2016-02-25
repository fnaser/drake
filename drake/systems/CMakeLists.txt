
if (simulink_FOUND)
  add_mex(DCSFunction DCSFunction.cpp)
endif()

if (LCM_FOUND)
  add_library_with_exports(LIB_NAME drakeLCMSystem SOURCE_FILES LCMSystem.cpp)
  add_dependencies(drakeLCMSystem drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(drakeLCMSystem lcm)

  pods_install_libraries(drakeLCMSystem)
  drake_install_headers(LCMSystem.h)
endif()

# If ROS is installed on the local system, include support for it.
if(DEFINED ENV{ROS_ROOT})

  # add_library_with_exports(LIB_NAME drakeROSSystem SOURCE_FILES ROSSystem.cpp)
  # add_dependencies(drakeROSSystem drake_lcmtypes lcmtype_agg_hpp)
  # pods_use_pkg_config_packages(drakeROSSystem lcm)

  # pods_install_libraries(drakeROSSystem)
  drake_install_headers(ROSSystem.h)
endif()

add_subdirectory(frames)
add_subdirectory(plants)
add_subdirectory(trajectories)
add_subdirectory(controllers)
add_subdirectory(robotInterfaces)

drake_install_headers(System.h LinearSystem.h Simulation.h)

add_subdirectory(test)
