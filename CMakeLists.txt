cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( vision )

file(GLOB vision_SRC
    "include/*.h"
    "src/*.cpp"
)

include_directories( include 
 ${BOOST_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )
find_package( apriltag REQUIRED apriltag)
find_package( Boost COMPONENTS filesystem system REQUIRED )
find_package (Threads REQUIRED)

link_directories( /usr/local/lib/comm )


add_executable( vision ${vision_SRC} )

target_link_libraries( vision ${OpenCV_LIBS} apriltag  comm 
${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})