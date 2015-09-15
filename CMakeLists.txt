cmake_minimum_required(VERSION 2.8.3)
project(mtracker_gui)

find_package(catkin REQUIRED geometry_msgs message_generation)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy geometry_msgs message_runtime
    DEPENDS systemlib QtCore QtGui )

qt4_wrap_cpp(MOC_FILES
  include/controls_panel.h
  include/debug_panel.h
)

set(SOURCE_FILES
  src/controls_panel.cpp
  src/debug_panel.cpp
  include/controls_panel.h
  include/debug_panel.h
  ${MOC_FILES}
)

include_directories( include )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} ${SOURCE_FILES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
