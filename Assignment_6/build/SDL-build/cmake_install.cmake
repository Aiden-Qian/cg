# Install script for directory: /Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Library/Developer/CommandLineTools/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/libSDL2.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2.a")
    execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2.a")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/libSDL2-2.0.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2-2.0.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2-2.0.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2-2.0.dylib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/libSDL2main.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2main.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2main.a")
    execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSDL2main.a")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/SDL2/SDL2Targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/SDL2/SDL2Targets.cmake"
         "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/CMakeFiles/Export/lib/cmake/SDL2/SDL2Targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/SDL2/SDL2Targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/SDL2/SDL2Targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/SDL2" TYPE FILE FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/CMakeFiles/Export/lib/cmake/SDL2/SDL2Targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/SDL2" TYPE FILE FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/CMakeFiles/Export/lib/cmake/SDL2/SDL2Targets-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/SDL2" TYPE FILE FILES
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/SDL2Config.cmake"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL2ConfigVersion.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/SDL2" TYPE FILE FILES
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_assert.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_atomic.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_audio.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_bits.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_blendmode.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_clipboard.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_android.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_iphoneos.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_macosx.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_minimal.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_os2.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_pandora.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_psp.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_windows.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_winrt.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_config_wiz.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_copying.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_cpuinfo.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_egl.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_endian.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_error.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_events.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_filesystem.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_gamecontroller.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_gesture.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_haptic.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_hints.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_joystick.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_keyboard.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_keycode.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_loadso.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_log.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_main.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_messagebox.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_metal.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_mouse.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_mutex.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_name.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_opengl.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_opengl_glext.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_opengles.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_opengles2.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_opengles2_gl2.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_opengles2_gl2ext.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_opengles2_gl2platform.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_opengles2_khrplatform.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_pixels.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_platform.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_power.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_quit.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_rect.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_render.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_revision.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_rwops.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_scancode.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_sensor.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_shape.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_stdinc.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_surface.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_system.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_syswm.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_assert.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_common.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_compare.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_crc32.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_font.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_fuzzer.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_harness.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_images.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_log.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_md5.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_memory.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_test_random.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_thread.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_timer.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_touch.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_types.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_version.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_video.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/SDL_vulkan.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/begin_code.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/include/close_code.h"
    "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/include/SDL_config.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
          execute_process(COMMAND /usr/local/Cellar/cmake/3.21.2/bin/cmake -E create_symlink
            "libSDL2-2.0.dylib" "libSDL2.dylib"
            WORKING_DIRECTORY "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/libSDL2.dylib")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/sdl2.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE PROGRAM FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/Assignment_6/build/SDL-build/sdl2-config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/share/aclocal/sdl2.m4")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/share/aclocal" TYPE FILE FILES "/Users/qc/Desktop/NYU_MSIS/21fall/Computer_graphic/cg/ext/SDL/sdl2.m4")
endif()

