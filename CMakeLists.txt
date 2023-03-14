cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0048 NEW)
project(MORB_SLAM VERSION 1.0 LANGUAGES CXX)

# IF(NOT CMAKE_BUILD_TYPE)
#   SET(CMAKE_BUILD_TYPE Release)
# ENDIF()

MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Set compiler standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7) # required for eigen
    message(FATAL_ERROR "Insufficient gcc version must be at least 7")
  endif()
endif()

set(include_dest "include/MORB_SLAM")
set(main_lib_dest "lib/MORB_SLAM")
set(lib_dest "${main_lib_dest}/${CMAKE_BUILD_TYPE}")

file(GLOB_RECURSE CPP_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB_RECURSE CPP_CAMERA_MODELS_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/src/CameraModels/*.cpp)
list(APPEND CPP_FILES ${CPP_CAMERA_MODELS_FILES})

file(GLOB_RECURSE INCLUDE_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/${PROJECT_SOURCE_DIR}/*.hpp ${PROJECT_SOURCE_DIR}/include/${PROJECT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE INCLUDE_CAMERA_MODELS_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/${PROJECT_SOURCE_DIR}/CameraModels/*.h)
list(APPEND INCLUDE_FILES ${INCLUDE_CAMERA_MODELS_FILES})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 4.2 HINTS "/usr/local/" )
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE(STATUS "OPENCV VERSION: \n ${OpenCV_VERSION}")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

function(set_fpic target)
    # https://stackoverflow.com/questions/38296756/what-is-the-idiomatic-way-in-cmake-to-add-the-fpic-compiler-option
    set_property(TARGET ${target} PROPERTY POSITION_INDEPENDENT_CODE ON) # add_definitions(-fPIC)
endfunction()

function (attach_general target)
        target_link_libraries(${target}
                ${OpenCV_LIBS}
                ${EIGEN3_LIBS}
                ${Pangolin_LIBRARIES}
                ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
                ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
                -lboost_serialization
                -lcrypto
        )
        target_link_libraries(${target} tbb)

endfunction()

add_library(${PROJECT_NAME} SHARED ${CPP_FILES} ${INCLUDE_FILES})
target_include_directories(${PROJECT_NAME}
    PUBLIC
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/DBoW2>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o>
)
attach_general(${PROJECT_NAME})

add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
        ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
        ${realsense2_LIBRARY}
    )
endif()

include(GNUInstallDirs)
message(status "---------- ${CMAKE_INSTALL_DATAROOTDIR}")
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
   EXPORT ${PROJECT_NAME}-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2
${Pangolin_INCLUDE_DIRS}
 DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${PROJECT_NAME}-targets
  FILE
      	${PROJECT_NAME}Targets.cmake
  NAMESPACE
        ${PROJECT_NAME}::
  DESTINATION
        ${INSTALL_CONFIGDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
	DESTINATION ${INSTALL_CONFIGDIR})

message(STATUS "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake to ${INSTALL_CONFIGDIR}")


install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME} DESTINATION "${lib_dest}")
install(FILES ${INCLUDE_FILES} DESTINATION "${include_dest}")

install(EXPORT ${PROJECT_NAME} NAMESPACE ${PROJECT_NAME}:: DESTINATION "${lib_dest}")

message("project name: ${PROJECT_NAME}")

# temp for building examples 

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)
add_executable(stereo_inertial_realsense_D435i
                Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})

message("added executable")