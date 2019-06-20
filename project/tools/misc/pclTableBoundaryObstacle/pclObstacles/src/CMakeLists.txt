cmake_minimum_required(VERSION 2.6)

ADD_EXECUTABLE(${NAME} readKinect
		       			Driver
                       )
TARGET_LINK_LIBRARIES(${NAME} ${LOG4CXX_LIBRARIES} 
                              ${Boost_LIBRARIES}
                              ${VTK_LIBRARIES}
                              ${OPENNI_LIBRARIES}
                              ${PCL_LIBRARIES}
                              
)
