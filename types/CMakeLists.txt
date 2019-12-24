project(types)

add_library(${PROJECT_NAME} INTERFACE)

include_directories("${EIGEN3_INCLUDE_DIR}")

target_include_directories(${PROJECT_NAME}
  INTERFACE
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)