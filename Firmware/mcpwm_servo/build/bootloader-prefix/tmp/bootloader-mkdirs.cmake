# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Aiano/esp/esp-idf/components/bootloader/subproject"
  "D:/Projects/ServoLess/Firmware/mcpwm_servo/build/bootloader"
  "D:/Projects/ServoLess/Firmware/mcpwm_servo/build/bootloader-prefix"
  "D:/Projects/ServoLess/Firmware/mcpwm_servo/build/bootloader-prefix/tmp"
  "D:/Projects/ServoLess/Firmware/mcpwm_servo/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Projects/ServoLess/Firmware/mcpwm_servo/build/bootloader-prefix/src"
  "D:/Projects/ServoLess/Firmware/mcpwm_servo/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Projects/ServoLess/Firmware/mcpwm_servo/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Projects/ServoLess/Firmware/mcpwm_servo/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
