cmake_minimum_required(VERSION 2.8.3)

project (emotiv_training)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            rospy
            emotiv_msgs
)
     
catkin_package(CATKIN_DEPENDS
              roscpp
              rospy
              emotiv_msgs
              INCLUDE_DIRS include      
)
catkin_python_setup()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PROJECT_SOURCE_DIR}/lib)

find_library(EMOTIV_LIBS NAMES edk HINTS ${PROJECT_SOURCE_DIR}/lib)


add_executable(emotiv_training_node src/emotiv_training.cpp)
target_link_libraries(emotiv_training_node ${catkin_LIBRARIES} ${EMOTIV_LIBS})

## Mark executable for installation
install(TARGETS emotiv_training_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/emotiv_training/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY include/emotiv_sdk/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
