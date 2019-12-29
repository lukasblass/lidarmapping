project(main)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    LidarSensor
    types
    Plotting
    Robot
)

