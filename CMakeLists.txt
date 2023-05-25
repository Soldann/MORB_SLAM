cmake_minimum_required(VERSION 3.24.0)

set(MORB_VERSION_MAJOR 0 CACHE STRING "major version" FORCE) 
set(MORB_VERSION_MINOR 0 CACHE STRING "major version" FORCE) 
set(MORB_VERSION_PATCH 1 CACHE STRING "major version" FORCE) 
set(MORB_VERSION_SHORT ${MORB_VERSION_MAJOR}.${MORB_VERSION_MINOR} CACHE STRING "short-version" FORCE)
set(MORB_VERSION ${MORB_VERSION_SHORT}.${MORB_VERSION_PATCH} CACHE STRING "version" FORCE)


project(MORB_SLAM 
    VERSION ${MORB_VERSION}
    DESCRIPTION "A library for visual simultanious localization and mapping (vSLAM) for autonomous vehicles"
    HOMEPAGE_URL "https://github.com/DavidPetkovsek/MORB_SLAM"
    LANGUAGES CXX
)


include(GNUInstallDirs)
# Set compiler standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
if( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release)
ENDIF()

# https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7) # required for eigen
        message(FATAL_ERROR "Insufficient gcc version must be at least 7, required for eigen")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5) # required for eigen
        message(FATAL_ERROR "Insufficient clang version must be at least 5, required for eigen")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.12) # required for eigen
        message(FATAL_ERROR "Insufficient msvc version must be at least 19.12, required for eigen")
    endif()
endif()


set(CMAKE_PROJECT_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(THREADS_PREFER_PTHREAD_FLAG ON)

include("${PROJECT_SOURCE_DIR}/cmake/FuncLib.cmake")


######################################################################
#                                                                    #
#           Subdirectories, Components, and targets                  #
#                                                                    #
######################################################################


add_subdirectory(Thirdparty/g2o)
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/Sophus)

set(TARGET_NAME ${PROJECT_NAME})

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2) # realsense only needed for example
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization regex)
find_package(Boost REQUIRED)

file(GLOB CPP_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
file(GLOB CPP_CAMERA_MODELS_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/CameraModels/*.cpp)
list(APPEND CPP_FILES ${CPP_CAMERA_MODELS_FILES})

file(GLOB headers CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h)
file(GLOB camera_model_headers CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/CameraModels/*.h ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/CameraModels/*.hpp)


add_library(${TARGET_NAME} SHARED ${CPP_FILES})
add_library(${PROJECT_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})
target_include_directories(${TARGET_NAME} PUBLIC
                       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # for headers when building
                       $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> # for client in install mode
                       )
target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::DBoW2 )
target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::g2o )
target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::sophus )
target_link_libraries( ${TARGET_NAME} PUBLIC Threads::Threads )
target_link_libraries( ${TARGET_NAME} PUBLIC Eigen3::Eigen )
target_link_libraries( ${TARGET_NAME} PUBLIC Boost::serialization Boost::regex )
target_link_libraries( ${TARGET_NAME} PUBLIC ${OpenCV_LIBS} )
target_link_libraries( ${TARGET_NAME} PRIVATE ${Pangolin_LIBRARIES} )


# https://cmake.org/cmake/help/latest/guide/importing-exporting/index.html#creating-a-package-version-file
# https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-packages
# https://mail.kde.org/pipermail/kde-buildsystem/2008-April/004544.html
set_property(TARGET ${TARGET_NAME} PROPERTY VERSION ${MORB_VERSION}) # x.x.x used for symlinks
set_property(TARGET ${TARGET_NAME} PROPERTY SOVERSION ${MORB_VERSION_MAJOR}) # ABI/API compatibility at so level must be integer
set_property(TARGET ${TARGET_NAME} PROPERTY "INTERFACE_${TARGET_NAME}_MAJOR_VERSION" ${MORB_VERSION_MAJOR})
set_property(TARGET ${TARGET_NAME} APPEND PROPERTY COMPATIBLE_INTERFACE_STRING "${TARGET_NAME}_MAJOR_VERSION")



set(EXPORT_TARGET_NAME "${TARGET_NAME}Targets")
set(EXPORT_TARGET_FILE_NAME "${PROJECT_NAME}${EXPORT_TARGET_NAME}.cmake")


# install the target and create export-set
install(TARGETS ${TARGET_NAME}
        EXPORT ${EXPORT_TARGET_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# install header files
install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
install(FILES ${camera_model_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/CameraModels)


# generate and install export file
install(EXPORT ${EXPORT_TARGET_NAME}
        FILE ${EXPORT_TARGET_FILE_NAME}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_PROJECT_CONFIG_INSTALL_DIR}
)

# Example stereo inertial realsense
if(realsense2_FOUND)
  set(TARGET_NAME stereo_inertial_realsense_D435i)
  add_executable(${TARGET_NAME} Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
  target_link_libraries( ${TARGET_NAME} PUBLIC ${realsense2_LIBRARY})
  target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::${PROJECT_NAME} )
  target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::DBoW2 )
  target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::g2o )
  target_link_libraries( ${TARGET_NAME} PUBLIC ${PROJECT_NAME}::sophus )
  target_link_libraries( ${TARGET_NAME} PUBLIC Threads::Threads )
  target_link_libraries( ${TARGET_NAME} PUBLIC Eigen3::Eigen )
  target_link_libraries( ${TARGET_NAME} PUBLIC Boost::serialization Boost::regex )
  target_link_libraries( ${TARGET_NAME} PUBLIC ${OpenCV_LIBS} )
  target_link_libraries( ${TARGET_NAME} PUBLIC ${Pangolin_LIBRARIES} )
  target_link_libraries(stereo_inertial_realsense_D435i PRIVATE ${PROJECT_NAME})
endif()


######################################################################
#                                                                    #
#                CMAKE INSTALL, CONFIG, AND PACKAGING                #
#                                                                    #
######################################################################


set(CONFIG_FILE_NAME_WITHOUT_EXT "${PROJECT_NAME}Config")
set(CMAKE_CONFIG_FILE_BASENAME "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_FILE_NAME_WITHOUT_EXT}")
set(CMAKE_CONFIG_VERSION_FILE_NAME "${CMAKE_CONFIG_FILE_BASENAME}-version.cmake")
set(CMAKE_CONFIG_FILE_NAME "${CMAKE_CONFIG_FILE_BASENAME}.cmake")

include(CMakePackageConfigHelpers)


# generate the version file for the config file
write_basic_package_version_file(
  "${CMAKE_CONFIG_VERSION_FILE_NAME}"
  COMPATIBILITY SameMajorVersion
)

# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CONFIG_FILE_NAME}"
  INSTALL_DESTINATION ${CMAKE_PROJECT_CONFIG_INSTALL_DIR}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# install config files
install(FILES
        "${CMAKE_CONFIG_FILE_NAME}"
        "${CMAKE_CONFIG_VERSION_FILE_NAME}"
        DESTINATION ${CMAKE_PROJECT_CONFIG_INSTALL_DIR}
)

include("${PROJECT_SOURCE_DIR}/cmake/Packing.cmake")
