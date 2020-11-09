cmake_minimum_required(VERSION 2.8.3)

project(optimization)

find_package(OpenCV REQUIRED)

add_executable(gaussionNewton gaussionNewton.cpp)

target_link_libraries(gaussionNewton
                        ${OpenCV_LIBS})