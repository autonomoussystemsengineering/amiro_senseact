cmake_minimum_required(VERSION 2.6)

ADD_EXECUTABLE(${NAME} RobocupKinectFloorCalibration
                       Viewer
                       Analyzer
)
TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES}
                              ${VTK_LIBRARIES}
                              ${OPENNI_LIBRARIES}
                              ${PCL_LIBRARIES}
)

INSTALL(TARGETS ${NAME} RUNTIME DESTINATION bin)
