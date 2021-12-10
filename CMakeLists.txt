cmake_minimum_required(VERSION 3.0.0)
project(FinalProject VERSION 0.1.0)

find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
	${OMPL_INCLUDE_DIRS}
	${FCL_INCLUDE_DIRS}
)

file(GLOB SOURCES src/*.cpp)
file(GLOB LIB_SOURCES lib/*.cpp)
file(GLOB LIB_HEADERS lib/*.h)

add_library(mylib ${LIB_SOURCES} ${LIB_HEADERS})

add_executable(main src/main.cpp)
add_executable(kinodynamic src/kinodynamic.cpp)
add_executable(geometric src/geometric.cpp)

target_link_libraries(main
	${OMPL_LIBRARIES}
	${FCL_LIBRARIES}
	${mylib}
	yaml-cpp
)

target_link_libraries(kinodynamic
	${OMPL_LIBRARIES}
	${FCL_LIBRARIES}
	${mylib}
	yaml-cpp
)

target_link_libraries(geometric
	${OMPL_LIBRARIES}
	${FCL_LIBRARIES}
	${mylib}
	yaml-cpp
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
