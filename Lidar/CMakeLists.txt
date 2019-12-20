project(LidarSensor)

add_library(${PROJECT_NAME} src/Lidar.cpp)
add_library(sub::lidar ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)