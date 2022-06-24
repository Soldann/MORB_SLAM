#FindDBoW2.cmake
#
# Finds the DBoW2 library
#
# from http://DBoW2.org/
#
# This will define the following variables
#
#    DBoW2_FOUND
#    DBoW2_VERSION
#    DBoW2_INCLUDE_DIRS
#
# and the following imported targets
#
#     DBoW2::DBoW2
#
# Author: Pablo Arias - pabloariasal@gmail.com
#

find_package(PkgConfig)
pkg_check_modules(PC_DBoW2 QUIET DBoW2)

find_path(DBoW2_INCLUDE_DIR
    NAMES BowVector.h FeatureVector.h
    PATHS ${PC_DBoW2_INCLUDE_DIRS}
    PATH_SUFFIXES DBoW2
)

set(DBoW2_VERSION ${PC_DBoW2_VERSION})

mark_as_advanced(DBoW2_FOUND DBoW2_INCLUDE_DIR DBoW2_VERSION)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DBoW2
    REQUIRED_VARS DBoW2_INCLUDE_DIR
    VERSION_VAR DBoW2_VERSION
)

if(DBoW2_FOUND)
    #Set include dirs to parent, to enable includes like #include <DBoW2/document.h>
    get_filename_component(DBoW2_INCLUDE_DIRS ${DBoW2_INCLUDE_DIR} DIRECTORY)
endif()

if(DBoW2_FOUND AND NOT TARGET DBoW2::DBoW2)
    add_library(DBoW2::DBoW2 INTERFACE IMPORTED)
    set_target_properties(DBoW2::DBoW2 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${DBoW2_INCLUDE_DIRS}"
    )
endif()