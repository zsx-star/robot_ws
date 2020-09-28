cmake_minimum_required(VERSION 2.8.3)
project(autolabor_pro1_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  message_generation
  tf
)

#eigen 
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


add_message_files(
  FILES
  Encode.msg
)

add_service_files(
	FILES
	UpdateOdom.srv
)

generate_messages()
generate_dynamic_reconfigure_options(cfg/SimAutolaborDriver.cfg)

catkin_package(CATKIN_DEPENDS roscpp tf message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(autolabor_pro1_driver src/autolabor_driver.cpp)
add_dependencies(autolabor_pro1_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(autolabor_pro1_driver ${catkin_LIBRARIES})

add_executable(sim_autolabor_pro1_driver src/sim_autolabor_driver.cpp)
add_dependencies(sim_autolabor_pro1_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(sim_autolabor_pro1_driver ${catkin_LIBRARIES})
