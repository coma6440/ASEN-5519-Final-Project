cmake_minimum_required(VERSION 3.0.0)
project(Test VERSION 0.1.0)

find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
# message(STATUS "OMPL_FOUND: ${OMPL_FOUND}")
# message(STATUS "OMPL_INCLUDE_DIRS: ${OMPL_INCLUDE_DIRS}")
# message(STATUS "OMPL_LIBRARIES: ${OMPL_LIBRARIES}")
# message(STATUS "OMPLAPP_LIBRARIES: ${OMPLAPP_LIBRARIES}")
# message(STATUS "OMPL_VERSION: ${OMPL_VERSION}")

# find_package(Boost REQUIRED COMPONENTS serialization)
# message(STATUS "BOOST_LIBRARIES: ${Boost_LIBRARIES}")

include_directories(
	${OMPL_INCLUDE_DIRS}
	${FCL_INCLUDE_DIRS}
)

include(CTest)
enable_testing()

add_executable(Test main.cpp)

target_link_libraries(Test
	${OMPL_LIBRARIES}
	${FCL_LIBRARIES}
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
