#===============================================================================
#
#   Cmake for Arduino
#
#===============================================================================

#-------------------------------------------------------------------------------
#
#   Arduino Toolchain for cross-compilation
#
Set( CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake )


#-------------------------------------------------------------------------------
#
#	CMake compatibility issues
#
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
MARK_AS_ADVANCED( CMAKE_BACKWARDS_COMPATIBILITY )


#-------------------------------------------------------------------------------
#
#   Source & Include directories
#
Set( INCLUDE_DIR "include" )
INCLUDE_DIRECTORIES( "include" )
Set( SOURCE_DIR "src" )


#-------------------------------------------------------------------------------
#
#  Setup Project
#
Project( Arduino C CXX )

Print_board_list()
Print_programmer_list()
Print_board_settings( diecimila )

#-------------------------------------------------------------------------------
#
#   Default Port, when not specified
#
Set( ARDUINO_DEFAULT_PORT /dev/ttyACM0 )

#-------------------------------------------------------------------------------
#
#   Library build process
#
Set( I2Cdev_HDRS   ${INCLUDE_DIR}/I2Cdev.h )
Set( I2Cdev_SRCS   ${SOURCE_DIR}/I2Cdev.cpp )
Set( I2Cdev_BOARD  diecimila )
Set( I2Cdev_PORT   /dev/ttyUSB0 )

Generate_Arduino_Library( I2Cdev )


#-------------------------------------------------------------------------------
#
#   Executable build process
#
Set( ${PROJECT_NAME}_HDRS
        ${INCLUDE_DIR}/I2Cdev.h
        ${INCLUDE_DIR}/helper_3dmath.h
        ${INCLUDE_DIR}/MPU6050.h
        ${INCLUDE_DIR}/MPU6050_6Axis_MotionApps20.h
        ${INCLUDE_DIR}/MPU6050_9Axis_MotionApps41.h )

Set( ${PROJECT_NAME}_SRCS
        ${SOURCE_DIR}/main.cpp
        ${SOURCE_DIR}/MPU6050.cpp )

Set( ${PROJECT_NAME}_LIBS    I2Cdev )
Set( ${PROJECT_NAME}_BOARD   diecimila )
Set( ${PROJECT_NAME}_PORT    /dev/ttyUSB0 )

Generate_Arduino_Firmware( ${PROJECT_NAME} )
