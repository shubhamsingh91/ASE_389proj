# Install script for directory: /home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated" TYPE PROGRAM FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated" TYPE PROGRAM FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/setup.bash;/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated" TYPE FILE FILES
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/setup.bash"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/setup.sh;/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated" TYPE FILE FILES
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/setup.sh"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/setup.zsh;/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated" TYPE FILE FILES
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/setup.zsh"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/install_isolated" TYPE FILE FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/xpp_msgs/msg" TYPE FILE FILES
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/msg/StateLin3d.msg"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/msg/State6d.msg"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/msg/RobotStateCartesianTrajectory.msg"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/msg/RobotStateCartesian.msg"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/msg/RobotStateJoint.msg"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/msg/RobotParameters.msg"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/msg/TerrainInfo.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/xpp_msgs/cmake" TYPE FILE FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/xpp_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/devel_isolated/xpp_msgs/include/xpp_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/devel_isolated/xpp_msgs/share/roseus/ros/xpp_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/devel_isolated/xpp_msgs/share/common-lisp/ros/xpp_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/devel_isolated/xpp_msgs/share/gennodejs/ros/xpp_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/devel_isolated/xpp_msgs/lib/python2.7/dist-packages/xpp_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/devel_isolated/xpp_msgs/lib/python2.7/dist-packages/xpp_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/xpp_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/xpp_msgs/cmake" TYPE FILE FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/xpp_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/xpp_msgs/cmake" TYPE FILE FILES
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/xpp_msgsConfig.cmake"
    "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/catkin_generated/installspace/xpp_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/xpp_msgs" TYPE FILE FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/xpp_msgs" TYPE DIRECTORY FILES "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/src/xpp/xpp_msgs/include/xpp_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/shubham/ASE389_proj/ASE389_proj/catkin_ws/build_isolated/xpp_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
