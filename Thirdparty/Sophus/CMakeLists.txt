# create library

set(TARGET_NAME sophus)

find_package( Eigen3 REQUIRED )

set(SOPHUS_HEADER_FILES
  sophus/average.hpp
  sophus/common.hpp
  sophus/geometry.hpp
  sophus/interpolate.hpp
  sophus/interpolate_details.hpp
  sophus/num_diff.hpp
  sophus/rotation_matrix.hpp
  sophus/rxso2.hpp
  sophus/rxso3.hpp
  sophus/se2.hpp
  sophus/se3.hpp
  sophus/sim2.hpp
  sophus/sim3.hpp
  sophus/sim_details.hpp
  sophus/so2.hpp
  sophus/so3.hpp
  sophus/types.hpp
  sophus/velocities.hpp
  sophus/formatstring.hpp
)

set(SOPHUS_OTHER_FILES
  sophus/test_macros.hpp
  sophus/example_ensure_handler.cpp
)

add_library(${TARGET_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})
target_include_directories(${TARGET_NAME} INTERFACE
                       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # for headers when building
		       $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty> # for client in install mode
                       )

target_link_libraries( ${TARGET_NAME} INTERFACE Eigen3::Eigen )

# the following is from orb-sam-3
# Set compiler specific settings (FixMe: Should not cmake do this for us automatically?)
IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_CXX_FLAGS_RELEASE "-O3")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra  -Wno-deprecated-register -Qunused-arguments -fcolor-diagnostics")
ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_CXX_FLAGS_RELEASE "-O3")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -std=c++11 -Wno-deprecated-declarations -ftemplate-backtrace-limit=0")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")
  set_target_properties(${TARGET_NAME} PROPERTIES CMAKE_SHARED_LINKER_FLAGS_COVERAGE "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} --coverage")
ELSEIF(CMAKE_CXX_COMPILER_ID MATCHES "^MSVC$")
  target_compile_definitions(${TARGET_NAME} "-D _USE_MATH_DEFINES /bigobj /wd4305 /wd4244 /MP")
ENDIF()
if(MSVC)
  # Define common math constants if we compile with MSVC
  target_compile_definitions (${TARGET_NAME} INTERFACE _USE_MATH_DEFINES)
endif (MSVC)
# Declare all used C++11 features
target_compile_features (${TARGET_NAME} INTERFACE
  cxx_auto_type
  cxx_decltype
  cxx_nullptr
  cxx_right_angle_brackets
  cxx_variadic_macros
  cxx_variadic_templates
)
# Add sources as custom target so that they are shown in IDE's
add_custom_target(${TARGET_NAME}_other SOURCES ${SOPHUS_OTHER_FILES})

# Create 'test' make target using ctest
option(BUILD_TESTS "Build tests." OFF)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# Create examples make targets using ctest
option(BUILD_EXAMPLES "Build examples." OFF)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()


#################################################
#                 boiler plate                  #
#################################################


# https://cmake.org/cmake/help/latest/guide/importing-exporting/index.html#creating-a-package-version-file
# https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-packages
# https://mail.kde.org/pipermail/kde-buildsystem/2008-April/004544.html
set_property(TARGET ${TARGET_NAME} PROPERTY VERSION 1.1.0) # x.x.x used for symlinks
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
install(FILES ${SOPHUS_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME})

# generate and install export file
install(EXPORT ${EXPORT_TARGET_NAME}
        FILE ${EXPORT_TARGET_FILE_NAME}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_PROJECT_CONFIG_INSTALL_DIR}
)
