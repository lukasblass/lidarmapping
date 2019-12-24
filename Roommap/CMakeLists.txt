project(Roommap)

add_library(${PROJECT_NAME} src/Room2D.cpp)
add_library(sub::room ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    types
)