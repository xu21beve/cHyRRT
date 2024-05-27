find_package(ompl REQUIRED)

# Add executable for multicopter.cpp
add_executable(multicopter multicopter.cpp)
target_link_libraries(multicopter HyRRT ${OMPL_LIBRARIES})

# Add executable for bouncing_ball.cpp
add_executable(bouncing_ball bouncing_ball.cpp)
target_link_libraries(bouncing_ball HyRRT ${OMPL_LIBRARIES})

# Include necessary directories
include_directories(${OMPL_INCLUDE_DIRS})
