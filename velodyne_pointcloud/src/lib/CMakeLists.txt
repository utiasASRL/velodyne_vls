find_package(OpenCV REQUIRED)

find_package(ZLIB)
include_directories(${ZLIB_INCLUDE_DIRS})

add_library(applanix_time_conv time_conv.cpp)
target_link_libraries(applanix_time_conv
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        std_msgs
        cv_bridge
        sensor_msgs
        image_transport
        )

add_library(velodyne_rawdata rawdata.cc calibration.cc)
target_link_libraries(velodyne_rawdata
                      applanix_time_conv
                      ${ZLIB_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${YAML_CPP_LIBRARIES})
install(TARGETS velodyne_rawdata
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
