project(rob4)
cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 17)

LINK_DIRECTORIES(lib/Vimba/VimbaCPP/DynamicLib/x86_64bit)

add_executable(rob4 main.cpp src/Vimba.cpp src/Vimba.h src/URSocket.cpp src/URSocket.h)

target_include_directories(rob4 PUBLIC lib/Vimba)
target_link_libraries(rob4 VimbaC)
target_link_libraries(rob4 VimbaCPP)

find_package(OpenCV)
target_link_libraries(rob4 ${OpenCV_LIBS})