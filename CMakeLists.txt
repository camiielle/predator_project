cmake_minimum_required(VERSION 3.16)
project(boids VERSION 0.1.0)

# enables support for testing, including the BUILD_TESTING option used below
include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address -fno-omit-frame-pointer")

add_executable(boids source/main.cpp source/flock.cpp source/boids.cpp source/stats.cpp)
add_subdirectory(Lyra)
target_link_libraries(boids PRIVATE bfg::lyra)

# to disable testing, pass -DBUILD_TESTING=OFF to cmake during the configuration phase
if (BUILD_TESTING)

 add_executable(parameters.t source/parameters.test.cpp)
 add_executable(boids.t source/boids.test.cpp source/boids.cpp)
 add_executable(flock.t source/flock.test.cpp source/flock.cpp source/boids.cpp)

 add_test(NAME parameters.t COMMAND parameters.t)
 add_test(NAME boids.t COMMAND boids.t)
 add_test(NAME flock.t COMMAND flock.t)

endif()
