execute_process(COMMAND "/home/wmrm/kinova-ros/build/kinova_demo/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/wmrm/kinova-ros/build/kinova_demo/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
