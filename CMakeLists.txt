cmake_minimum_required(VERSION 3.0.0)
project(FinalProject VERSION 0.1.0)

find_package(ompl REQUIRED)
find_package(fcl REQUIRED)

include_directories(
	${OMPL_INCLUDE_DIRS}
	${FCL_INCLUDE_DIRS}
)

include(CTest)
enable_testing()

add_executable(FinalProject main.cpp)

target_link_libraries(FinalProject
	${OMPL_LIBRARIES}
	${FCL_LIBRARIES}
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
