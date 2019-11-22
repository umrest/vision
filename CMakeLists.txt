cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( vision )

file(GLOB vision_SRC
    "include/*.h"
    "src/*.cpp"
)

include_directories( include )

find_package( OpenCV REQUIRED )

add_executable( vision ${vision_SRC} )

target_link_libraries( vision ${OpenCV_LIBS} apriltag )