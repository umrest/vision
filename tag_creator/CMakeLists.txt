cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( tag_creator )

file(GLOB tag_creator_SRC
    "main.cpp"
)

include_directories( include 
 ${BOOST_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )

add_executable( tag_creator ${tag_creator_SRC} )

target_link_libraries( tag_creator ${OpenCV_LIBS} )