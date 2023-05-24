# create library

set(TARGET_NAME DBoW2)

find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS serialization regex)
find_package(Boost REQUIRED)

file(GLOB_RECURSE CPP_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.c ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
file(GLOB_RECURSE headers_dbow2 CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/*.h ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/*.hpp)
file(GLOB_RECURSE headers_dutils CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/*.h ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/*.hpp)


add_library(${TARGET_NAME} SHARED ${CPP_FILES})
add_library(${PROJECT_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})
target_include_directories(${TARGET_NAME} PUBLIC
                       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # for headers when building
		       $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME}> # for client in install mode
                       )

target_link_libraries( ${TARGET_NAME} PUBLIC ${OpenCV_LIBS} )
target_link_libraries( ${TARGET_NAME} PUBLIC Boost::serialization Boost::regex )



#################################################
#                 boiler plate                  #
#################################################


# https://cmake.org/cmake/help/latest/guide/importing-exporting/index.html#creating-a-package-version-file
# https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-packages
# https://mail.kde.org/pipermail/kde-buildsystem/2008-April/004544.html
set_property(TARGET ${TARGET_NAME} PROPERTY VERSION 3.16) # x.x.x used for symlinks
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
install(FILES ${headers_dbow2} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME}/DBoW2)
install(FILES ${headers_dutils} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/Thirdparty/${TARGET_NAME}/DUtils)

# generate and install export file
install(EXPORT ${EXPORT_TARGET_NAME}
        FILE ${EXPORT_TARGET_FILE_NAME}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_PROJECT_CONFIG_INSTALL_DIR}
)
