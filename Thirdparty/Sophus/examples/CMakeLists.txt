# Tests to run
SET( EXAMPLE_SOURCES HelloSO3)
find_package( Ceres 1.6.0 QUIET )

# Set compiler standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7) # required for eigen
    message(FATAL_ERROR "Insufficient gcc version must be at least 7")
  endif()
endif()

FOREACH(example_src ${EXAMPLE_SOURCES})
  ADD_EXECUTABLE( ${example_src} ${example_src}.cpp)
  TARGET_LINK_LIBRARIES( ${example_src} sophus )
ENDFOREACH(example_src)
