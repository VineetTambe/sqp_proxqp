cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(sqp_proxqp VERSION 0.0.1
    LANGUAGES CXX
    DESCRIPTION "An SQP solver using the ProxQP solver and Eigen"
    HOMEPAGE_URL "https://github.com/"
    )

# Eigen library
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(proxsuite REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")

set(LIBS ${LIBS} ${PROXSUITE_LIBRARIES} Eigen3::Eigen)

add_library(${PROJECT_NAME} STATIC
    src/sqp_proxqp.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC include/)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

# Build the executable
add_executable(brick_sqp example/brick_sqp.cpp)
add_executable(brick_falling_sqp example/brick_falling_sqp.cpp)

target_link_libraries(brick_sqp ${LIBS})
target_link_libraries(brick_falling_sqp ${LIBS} ${PROJECT_NAME})
