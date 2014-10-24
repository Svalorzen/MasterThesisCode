cmake_minimum_required (VERSION 2.6)

find_package(Boost 1.53 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
find_package(LpSolve REQUIRED)
include_directories(${LPSOLVE_INCLUDE_DIR})

find_library(AIMDP AIToolboxMDP ${LOCAL_LIBS})
find_library(AIPOMDP AIToolboxPOMDP ${LOCAL_LIBS})

add_executable(cameras mainCameras.cpp cameraProblem.cpp ./BeliefNode.cpp)
add_executable(camerasMB mainCameras.cpp cameraProblem.cpp ./BeliefNode.cpp)

add_executable(multiCameras mainMultiCameras.cpp cameraProblem.cpp ./BeliefNode.cpp)
add_executable(multiCamerasMB mainMultiCameras.cpp cameraProblem.cpp ./BeliefNode.cpp)

add_executable(myo mainMyopic.cpp myopicProblem.cpp ./BeliefNode.cpp)
add_executable(myoMB mainMyopic.cpp myopicProblem.cpp ./BeliefNode.cpp)

add_executable(fromFile usePremade.cpp ./BeliefNode.cpp)

#add_executable(tiger mainTiger.cpp)
#add_executable(policyReader usePolicy.cpp)

set_target_properties(cameras PROPERTIES COMPILE_DEFINITIONS "ENTROPY")
set_target_properties(multiCameras PROPERTIES COMPILE_DEFINITIONS "ENTROPY")
set_target_properties(myo     PROPERTIES COMPILE_DEFINITIONS "ENTROPY")

target_link_libraries(cameras      ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP})
target_link_libraries(camerasMB    ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP})

target_link_libraries(multiCameras      ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP}) # ${Boost_SYSTEM_LIBRARY})
target_link_libraries(multiCamerasMB    ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP})

target_link_libraries(myo          ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP})
target_link_libraries(myoMB        ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP})

target_link_libraries(fromFile        ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP})

#target_link_libraries(tiger        ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP})
#target_link_libraries(policyReader ${AIPOMDP} ${LPSOLVE_LIBRARIES} ${AIMDP})