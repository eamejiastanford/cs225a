# Install script for directory: /home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/lecture2-demo/cmake_install.cmake")
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0/cmake_install.cmake")
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw0_solution/cmake_install.cmake")
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw1/cmake_install.cmake")
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw1_solution/cmake_install.cmake")
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw2/cmake_install.cmake")
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/hw3/cmake_install.cmake")
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/panda_starter_code/cmake_install.cmake")
  include("/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/kuka_starter_code/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/cs225a/Codes/sai2.0/apps/penalty_kicker/cs225a/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
