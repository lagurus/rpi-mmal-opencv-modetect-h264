cmake_minimum_required(VERSION 2.8)

SET(COMPILE_DEFINITIONS -Werror)

include_directories(/opt/vc/include)
include_directories(/opt/vc/include/interface/vcos/pthreads)
include_directories(/opt/vc/include/interface/vmcs_host)
include_directories(/opt/vc/include/interface/vmcs_host/linux)

#include_directories(/home/pi/src/raspberrypi/userland)
#include_directories(/opt/vc/src/hello_pi/libs/vgfont)

link_directories(/opt/vc/lib)
#link_directories(/opt/vc/src/hello_pi/libs/vgfont)

add_executable(mmal_opencv_modect opencv_modect.c)

#find_package( OpenCV REQUIRED )

#target_link_libraries(mmal_opencv_modect mmal_core mmal_util mmal_vc_client vcos bcm_host cairo ${OpenCV_LIBS} vgfont openmaxil EGL)
target_link_libraries(mmal_opencv_modect mmal_core mmal_util mmal_vc_client vcos bcm_host openmaxil EGL)

