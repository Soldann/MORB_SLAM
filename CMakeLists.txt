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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE(STATUS "OPENCV VERSION:")
MESSAGE(STATUS ${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

# include_directories(
# ${PROJECT_SOURCE_DIR}
# ${PROJECT_SOURCE_DIR}/include
# ${PROJECT_SOURCE_DIR}/include/CameraModels
# ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
# ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2
# ${EIGEN3_INCLUDE_DIR}
# ${Pangolin_INCLUDE_DIRS}
# )

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Settings.h)


add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)

target_include_directories(${PROJECT_NAME}
    PUBLIC
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/DBoW2>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o>
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
)


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
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
   EXPORT ${PROJECT_NAME}-targets
   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
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

export(EXPORT ${PROJECT_NAME}-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::)

export(PACKAGE ${PROJECT_NAME})

# Build examples
if (CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
        # RGB-D examples
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

        add_executable(rgbd_tum
                Examples/RGB-D/rgbd_tum.cc)
        target_link_libraries(rgbd_tum ${PROJECT_NAME})

        if(realsense2_FOUND)
        add_executable(rgbd_realsense_D435i
                Examples/RGB-D/rgbd_realsense_D435i.cc)
        target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
        endif()


        # RGB-D inertial examples
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

        if(realsense2_FOUND)
        add_executable(rgbd_inertial_realsense_D435i
                Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
        target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
        endif()

        # #Stereo examples
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

        add_executable(stereo_kitti
                Examples/Stereo/stereo_kitti.cc)
        target_link_libraries(stereo_kitti ${PROJECT_NAME})

        add_executable(stereo_euroc
                Examples/Stereo/stereo_euroc.cc)
        target_link_libraries(stereo_euroc ${PROJECT_NAME})

        add_executable(stereo_tum_vi
                Examples/Stereo/stereo_tum_vi.cc)
        target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

        if(realsense2_FOUND)
        add_executable(stereo_realsense_t265
                Examples/Stereo/stereo_realsense_t265.cc)
        target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})

        add_executable(stereo_realsense_D435i
                Examples/Stereo/stereo_realsense_D435i.cc)
        target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
        endif()

        #Monocular examples
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

        add_executable(mono_tum
                Examples/Monocular/mono_tum.cc)
        target_link_libraries(mono_tum ${PROJECT_NAME})

        add_executable(mono_kitti
                Examples/Monocular/mono_kitti.cc)
        target_link_libraries(mono_kitti ${PROJECT_NAME})

        add_executable(mono_euroc
                Examples/Monocular/mono_euroc.cc)
        target_link_libraries(mono_euroc ${PROJECT_NAME})

        add_executable(mono_tum_vi
                Examples/Monocular/mono_tum_vi.cc)
        target_link_libraries(mono_tum_vi ${PROJECT_NAME})

        if(realsense2_FOUND)
        add_executable(mono_realsense_t265
                Examples/Monocular/mono_realsense_t265.cc)
        target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})

        add_executable(mono_realsense_D435i
                Examples/Monocular/mono_realsense_D435i.cc)
        target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
        endif()

        #Monocular inertial examples
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

        add_executable(mono_inertial_euroc
                Examples/Monocular-Inertial/mono_inertial_euroc.cc)
        target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

        add_executable(mono_inertial_tum_vi
                Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
        target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

        if(realsense2_FOUND)
        add_executable(mono_inertial_realsense_t265
                Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
        target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})

        add_executable(mono_inertial_realsense_D435i
                Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
        target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
        endif()

        #Stereo Inertial examples
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

        add_executable(stereo_inertial_euroc
                Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
        target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

        add_executable(stereo_inertial_tum_vi
                Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
        target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

        if(realsense2_FOUND)
        add_executable(stereo_inertial_realsense_t265
                Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
        target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})

        add_executable(stereo_inertial_realsense_D435i
                Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
        target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
        endif()

        # Calibration examples
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)
        if(realsense2_FOUND)
        add_executable(recorder_realsense_D435i
                Examples/Calibration/recorder_realsense_D435i.cc)
        target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})

        add_executable(recorder_realsense_T265
                Examples/Calibration/recorder_realsense_T265.cc)
        target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
        endif()
endif()