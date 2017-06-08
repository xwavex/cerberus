cmake_minimum_required(VERSION 2.8.3)
project(cogimon-urdf-to-kdl)

set(CMAKE_BUILD_TYPE Debug)#Release

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# -fno-omit-frame-pointer -g -finstrument-functions")

find_package(gazebo 7.0 REQUIRED)
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()
message(STATUS "Found GAZEBO, version: ${GAZEBO_VERSION}")
#message(STATUS "GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")
#message(STATUS "GAZEBO_LIBRARY_DIRS: ${GAZEBO_LIBRARY_DIRS}")
#message(STATUS "GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
find_package(orocos_kdl REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

find_package(cogimon-kdl-parser REQUIRED)
message(STATUS "cogimon-kdl-parser version: ${cogimon-kdl-parser_VERSION}")
include_directories(BEFORE SYSTEM ${cogimon-kdl-parser_INCLUDE_DIRS})
add_definitions(${cogimon-kdl-parser_CFLAGS})
link_directories(${cogimon-kdl-parser_LIBRARY_DIR} ${cogimon-kdl-parser_LIBRARY_DIRS})
message(STATUS "cogimon-kdl-parser_INCLUDE_DIRS: ${cogimon-kdl-parser_INCLUDE_DIRS}")
message(STATUS "cogimon-kdl-parser_CFLAGS: ${cogimon-kdl-parser_CFLAGS}")
message(STATUS "cogimon-kdl-parser_LIBRARY_DIR: ${cogimon-kdl-parser_LIBRARY_DIR}")
message(STATUS "cogimon-kdl-parser_LIBRARY_DIRS: ${cogimon-kdl-parser_LIBRARY_DIRS}")
message(STATUS "cogimon-kdl-parser_LIBRARIES: ${cogimon-kdl-parser_LIBRARIES}")

find_package(fcl REQUIRED)

find_package(XBotCoreModel REQUIRED)


find_package(assimp REQUIRED)
message(STATUS "assimp version: ${assimp_VERSION}")
include_directories(BEFORE SYSTEM ${ASSIMP_INCLUDE_DIRS})
add_definitions(${ASSIMP_CFLAGS})
link_directories(${ASSIMP_LIBRARY_DIR} ${ASSIMP_LIBRARY_DIRS})
message(STATUS "ASSIMP_INCLUDE_DIRS: ${ASSIMP_INCLUDE_DIRS}")
message(STATUS "ASSIMP_CFLAGS: ${ASSIMP_CFLAGS}")
message(STATUS "ASSIMP_LIBRARY_DIR: ${ASSIMP_LIBRARY_DIR}")
message(STATUS "ASSIMP_LIBRARY_DIRS: ${ASSIMP_LIBRARY_DIRS}")
message(STATUS "ASSIMP_LIBRARIES: ${ASSIMP_LIBRARIES}")

find_package(glfw3 REQUIRED)
message(STATUS "GLFW3 version: ${GLFW3_VERSION}")
include_directories(BEFORE SYSTEM ${GLFW3_INCLUDE_DIRS})
add_definitions(${GLFW3_CFLAGS})
link_directories(${GLFW3_LIBRARY_DIR})
message(STATUS "GLFW3_INCLUDE_DIRS: ${GLFW3_INCLUDE_DIRS}")
message(STATUS "GLFW3_CFLAGS: ${GLFW3_CFLAGS}")
message(STATUS "GLFW3_LIBRARY_DIR: ${GLFW3_LIBRARY_DIR}")
message(STATUS "GLFW3_LIBRARY_DIRS: ${GLFW3_LIBRARY_DIRS}")
message(STATUS "GLFW3_LIBRARIES: ${GLFW3_LIBRARIES}")
message(STATUS "GLFW3_LIBRARY: ${GLFW3_LIBRARY}")

FIND_PACKAGE(OpenGL REQUIRED)
message(STATUS "OPENGL version: ${OPENGL_VERSION}")
include_directories(BEFORE SYSTEM ${OPENGL_INCLUDE_DIRS})
add_definitions(${OPENGL_CFLAGS})
link_directories(${OPENGL_LIBRARY_DIR} ${OPENGL_LIBRARY_DIRS})
message(STATUS "OPENGL_INCLUDE_DIRS: ${OPENGL_INCLUDE_DIRS}")
message(STATUS "OPENGL_CFLAGS: ${OPENGL_CFLAGS}")
message(STATUS "OPENGL_LIBRARY_DIR: ${OPENGL_LIBRARY_DIR}")
message(STATUS "OPENGL_LIBRARY_DIRS: ${OPENGL_LIBRARY_DIRS}")
message(STATUS "OPENGL_LIBRARIES: ${OPENGL_LIBRARIES}")


#find_package(PkgConfig REQUIRED)
#pkg_search_module(TINYXML REQUIRED tinyxml)
#if(NOT TINYXML_INCLUDE_DIR)
#        find_path("/usr/include" tinyxml.h)
#        find_library(TINYXML_LIBRARY NAMES tinyxml)
#        set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
#    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
#    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
#endif()
#message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
#include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
#link_directories(${TINYXML_LIBRARY_DIR})
#message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
#message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
#message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
#message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

find_package(Boost COMPONENTS thread filesystem system REQUIRED)

include_directories(
    include
    src
    ${Boost_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${RST-RT_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIRS}
    ${srdfdom_advr_INCLUDE_DIRS}
	${XBotCoreModel_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    /usr/include/GL
)

orocos_component(${PROJECT_NAME} src/TemplateComponent.cpp src/RobotDescription.cpp src/conversions.cpp)

add_executable(check_kdl_parser src/check_kdl_parser.cpp src/RobotDescription.cpp src/conversions.cpp)

set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
   ${USE_OROCOS_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${RST-RT_LIBRARIES}
   ${cogimon-kdl-parser_LIBRARIES}
   ${TINYXML_LIBRARIES}
   ${srdfdom_advr_LIBRARIES}
   ${XBotCoreModel_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${ASSIMP_LIBRARIES}
   /usr/lib/x86_64-linux-gnu/libGLEW.so
   ${OPENGL_LIBRARIES}
   ${GLFW3_LIBRARY}
   ${GAZEBO_LIBRARIES}
   fcl
)

target_link_libraries(check_kdl_parser
   ${Boost_LIBRARIES}
   ${RST-RT_LIBRARIES}
   ${cogimon-kdl-parser_LIBRARIES}
   ${TINYXML_LIBRARIES}
   ${srdfdom_advr_LIBRARIES}
   ${XBotCoreModel_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${ASSIMP_LIBRARIES}
   /usr/lib/x86_64-linux-gnu/libGLEW.so
   ${OPENGL_LIBRARIES}
   ${GLFW3_LIBRARY}
   ${GAZEBO_LIBRARIES}
   fcl
)

orocos_generate_package()
