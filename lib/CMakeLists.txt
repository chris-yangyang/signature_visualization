find_package(OpenCV REQUIRED)
include_directories(
include
${OpenCV_INCLUDE_DIRS}
)
add_library (vis_lib math_helper.cpp string_convertor.cpp transformation2D.cpp)

target_link_libraries(vis_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
