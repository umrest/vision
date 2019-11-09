cmake_minimum_required(VERSION 2.8)


project( vision )

file(GLOB vision_SRC
    "*.h"
    "*.cpp"
)

find_package( OpenCV REQUIRED )

add_executable( vision  )

target_link_libraries( vision ${OpenCV_LIBS} ${vision_SRC} )