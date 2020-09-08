# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "src/main_task.cpp src/accelerometer_task.cpp lib/I2Cdev/I2Cdev.cpp lib/MPU6050/MPU6050.cpp lib/libfort/fort.c" )
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
