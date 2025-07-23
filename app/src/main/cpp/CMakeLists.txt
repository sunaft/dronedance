cmake_minimum_required(VERSION 3.4.1)

project(NativeBridge)

# Path to OpenCV SDK — adjust this relative path if needed
#set(OpenCV_SDK ${CMAKE_SOURCE_DIR}/../openCVLibrary/sdk)
set(OpenCV_SDK ${CMAKE_SOURCE_DIR}/../../../opencv/sdk/native/jni)


# Include OpenCV headers
include_directories(${OpenCV_SDK}/include)

# Declare OpenCV prebuilt shared lib
add_library(opencv_java4 SHARED IMPORTED)
set_target_properties(opencv_java4 PROPERTIES
    IMPORTED_LOCATION ${OpenCV_SDK}/../libs/${ANDROID_ABI}/libopencv_java4.so
)

# Define your native library
add_library(fastimglib SHARED fastimglib.cpp)

# Link to OpenCV + Android logging
find_library(log-lib log)

target_link_libraries(
    fastimglib
    opencv_java4
    ${log-lib}
)










### cmake_minimum_required(VERSION 3.4.1)

### project(NativeBridge)

# Include OpenCV from your installed OpenCV Android SDK
# Set this to your OpenCV SDK path
### set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/opencv/sdk/native/jni)

### include_directories(${CMAKE_SOURCE_DIR}/opencv/sdk/native/jni/include)

### add_library(opencv_java4 SHARED IMPORTED)
### set_target_properties(opencv_java4 PROPERTIES
###     IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/opencv/sdk/native/libs/${ANDROID_ABI}/libopencv_java4.so
### )

# Link against OpenCV and log
# find_library(log-lib log)

# find_package(OpenCV REQUIRED)

### add_library( # Sets the name of the library.
###              fastimglib

             # Sets the library as a shared library.
###              SHARED

             # Provides a relative path to your source file(s).
###              fastimglib.cpp)

# Link OpenCV + log + JNI
### target_link_libraries(fastimglib
###                       opencv_java4
#                      ${OpenCV_LIBS}
###                       log)