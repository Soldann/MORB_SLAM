# create library

set(TARGET_NAME g2o)

find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )

file(GLOB_RECURSE CPP_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/*.c ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/*.cc)
file(GLOB headers_core CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/core/*.h ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/core/*.hpp)
file(GLOB headers_solvers CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/solvers/*.h ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/solvers/*.hpp)
file(GLOB headers_stuff CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/stuff/*.h ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/stuff/*.hpp)
file(GLOB headers_types CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/types/*.h ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/types/*.hpp)


add_library(${TARGET_NAME} SHARED ${CPP_FILES})
add_library(${PROJECT_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})
target_include_directories(${TARGET_NAME} PUBLIC
                       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # for headers when building
		       $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty> # for client in install mode
                       )

target_link_libraries( ${TARGET_NAME} PUBLIC ${OpenCV_LIBS} )
target_link_libraries( ${TARGET_NAME} PUBLIC Eigen3::Eigen )

# the following is from orb-sam-3
if(MSVC90)
  target_compile_definitions(${TARGET_NAME} PUBLIC -DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)
IF(UNIX)
  target_compile_definitions(${TARGET_NAME} PUBLIC -DUNIX)
  message(STATUS "Compiling on Unix")
ENDIF(UNIX)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
option(G2O_USE_OPENMP "Build g2o with OpenMP support (EXPERIMENTAL)" OFF) # Disabled by default
IF(G2O_USE_OPENMP)
  find_package(OpenMP REQUIRED)
  SET (G2O_OPENMP 1)
  target_compile_definitions(${TARGET_NAME} PUBLIC -DG2O_OPENMP=1)
  target_compile_definitions(${TARGET_NAME} PUBLIC -DEIGEN_DONT_PARALLELIZE)
  target_link_libraries(${TARGET_NAME} PUBLIC OpenMP::OpenMP_CXX)
  message(STATUS "Compiling with OpenMP support")
ENDIF(G2O_USE_OPENMP)

set(GENERATED_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}/config.h)
configure_file(config.h.in ${GENERATED_CONFIG})



#################################################
#                 boiler plate                  #
#################################################


# https://cmake.org/cmake/help/latest/guide/importing-exporting/index.html#creating-a-package-version-file
# https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-packages
# https://mail.kde.org/pipermail/kde-buildsystem/2008-April/004544.html
set_property(TARGET ${TARGET_NAME} PROPERTY VERSION 1.0) # x.x.x used for symlinks
set_property(TARGET ${TARGET_NAME} PROPERTY SOVERSION 1) # ABI/API compatibility at so level must be integer
set_property(TARGET ${TARGET_NAME} PROPERTY "INTERFACE_${TARGET_NAME}_MAJOR_VERSION" 1)
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

# install header file
install(FILES ${GENERATED_CONFIG} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME})
install(FILES ${headers_core} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME}/core)
install(FILES ${headers_solvers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME}/solvers)
install(FILES ${headers_stuff} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME}/stuff)
install(FILES ${headers_types} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME}/types)

# generate and install export file
install(EXPORT ${EXPORT_TARGET_NAME}
        FILE ${EXPORT_TARGET_FILE_NAME}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_PROJECT_CONFIG_INSTALL_DIR}
)
