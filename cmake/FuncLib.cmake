
# # Useful CMake Cuda Links
# # https://cliutils.gitlab.io/modern-cmake/chapters/packages/CUDA.html
# # https://cmake.org/cmake/help/git-stage/module/FindCUDAToolkit.html
# # https://gist.github.com/ax3l/9489132#device-side-c-standard-support
# include(CheckLanguage)
# check_language(CUDA)
# if(CMAKE_CUDA_COMPILER)
#   enable_language(CUDA)
#   if(NOT DEFINED CMAKE_CUDA_STANDARD)
#       set(CMAKE_CUDA_STANDARD 17)
#       set(CMAKE_CUDA_STANDARD_REQUIRED ON)
#       set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
#       #set_target_properties(${target} PROPERTIES CUDA_ARCHITECTURES "72")
#       #target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
#       #                    --extended-lambda
#       #                    -Wno-subobject-linkage
#       #                    >)
#   endif()
# endif()





function(set_fpic target)
    # https://stackoverflow.com/questions/38296756/what-is-the-idiomatic-way-in-cmake-to-add-the-fpic-compiler-option
    set_property(TARGET ${target} PROPERTY POSITION_INDEPENDENT_CODE ON) # add_definitions(-fPIC)
endfunction()

function(attach_jni target)
    # https://github.com/wang-bin/JMI/blob/master/CMakeLists.txt
    # https://stackoverflow.com/questions/3570355/c-fvisibility-hidden-fvisibility-inlines-hidden

    # set(CMAKE_CXX_VISIBILITY_PRESET hidden) #use with -fdata-sections -ffunction-sections to reduce target size
    # set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

    set_fpic(${target})
    find_package(Java COMPONENTS Development REQUIRED)
    include(UseJava)

    find_package(JNI REQUIRED)
    target_include_directories(${target} PUBLIC ${JNI_INCLUDE_DIRS})

    target_compile_definitions(${target} PUBLIC -DUSING_JNI)
    
    get_target_property(target_type ${target} TYPE)
    if (NOT target_type STREQUAL "SHARED_LIBRARY")
        message( FATAL_ERROR "${target} build target must be a SHARED_LIBRARY not ${target_type}" )
    endif()
endfunction()