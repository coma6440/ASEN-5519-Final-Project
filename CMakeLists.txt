cmake_minimum_required(VERSION 3.0.0)
project(FinalProject VERSION 0.1.0)

find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
	${OMPL_INCLUDE_DIRS}
	${FCL_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/src
)

set(FUNC_LIBS include/MyFunctions.h include/MyPlanner.h include/MySampler.h)
set(FUNC_SRCS src/MyFunctions.cpp src/MyPlanner.cpp src/MySampler.cpp)

add_library(MyLib ${FUNC_SRCS} ${FUNC_LIBS})

add_executable(main src/main.cpp)
add_executable(kinodynamic src/kinodynamic.cpp)
add_executable(geometric src/geometric.cpp)

target_link_libraries(MyLib yaml-cpp)

# Could loop this out?
target_link_libraries(main
	${OMPL_LIBRARIES}
	${FCL_LIBRARIES}
	MyLib
)

target_link_libraries(kinodynamic
	${OMPL_LIBRARIES}
	${FCL_LIBRARIES}
	MyLib
)

target_link_libraries(geometric
	${OMPL_LIBRARIES}
	${FCL_LIBRARIES}
	MyLib
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
