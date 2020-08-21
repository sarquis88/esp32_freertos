# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "src/main_task.cpp src/int_receiver_task.cpp src/wifi_scanner_task.cpp src/blinker_task.cpp src/wifi_connection_task.cpp src/tasks_list_task.cpp src/i2c_task.cpp" )
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
