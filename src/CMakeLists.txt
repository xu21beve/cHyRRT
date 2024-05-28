find_package(ompl REQUIRED)

add_library(HyRRT HyRRT.cpp)
target_link_libraries(HyRRT ${OMPL_LIBRARIES})

set(CMAKE_TOOLCHAIN_FILE ${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)

find_package(Boost REQUIRED
             COMPONENTS serialization filesystem system program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})


# add_library(RapidQuadcopterTrajectories RapidTrajectoryGenerator.cpp SingleAxisTrajectory.cpp)
add_library(Quartic quartic.cpp)

include_directories(${HyRRT_SOURCE_DIR}) # Include the directory where HyRRT.h is located

# Include necessary directories
include_directories(${OMPL_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")