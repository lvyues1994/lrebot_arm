cmake_minimum_required(VERSION 3.25...3.31)
project(rebot_b601 NONE)

find_package(ament_cmake REQUIRED)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/generated/SOURCE")
    message(FATAL_ERROR "The description has not been generated; run scripts/generate_description.py first.")
endif()

# The profile refers to the description relative to itself, so both stay in this package.
install(DIRECTORY config generated launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)

ament_package()
