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

#Print_board_list()
#Print_programmer_list()
Print_board_settings( mega2560 )

#-------------------------------------------------------------------------------
#
#   Default Port, when not specified
#
Set( ARDUINO_DEFAULT_PORT /dev/ttyACM1 )
Set( ARDUINO_DEFAULT_SERIAL 9600 )


#-------------------------------------------------------------------------------
#
#   Library build process
#
Set( MMA7361_HDRS   ${INCLUDE_DIR}/MMA7361.h )
Set( MMA7361_SRCS   ${SOURCE_DIR}/MMA7361.cpp )
Set( MMA7361_BOARD  mega2560 )

Generate_Arduino_Library( MMA7361 )

#-------------------------------------------------------------------------------
#
#   Executable build process
#
Set( ${PROJECT_NAME}_HDRS
        ${INCLUDE_DIR}/MMA7361.h
        ${INCLUDE_DIR}/MPU6050.h
        ${INCLUDE_DIR}/I2Cdev.h
        ${INCLUDE_DIR}/helper_3dmath.h
        ${INCLUDE_DIR}/MPU6050_6Axis_MotionApps20.h
        ${INCLUDE_DIR}/Actuator.h
        ${INCLUDE_DIR}/KalmanFilter.h )

Set( ${PROJECT_NAME}_SRCS
        ${SOURCE_DIR}/Actuator.cpp
        ${SOURCE_DIR}/MPU6050.cpp
        ${SOURCE_DIR}/I2Cdev.cpp
        ${SOURCE_DIR}/main.cpp )

Set( ${PROJECT_NAME}_LIBS    )
Set( ${PROJECT_NAME}_BOARD   mega2560 )

Generate_Arduino_Firmware( ${PROJECT_NAME} )
