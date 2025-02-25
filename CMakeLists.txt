cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(app)

pico_sdk_init()

add_compile_options(-Wall -Wextra)

add_executable(app
	main.c
	driver/bmi160.c
)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(app
	pico_stdlib
	hardware_flash
	hardware_i2c
	pico_multicore
)

pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 1)
pico_add_extra_outputs(app)
pico_set_binary_type(app copy_to_ram)

add_custom_target(flash
		cp ${PROJECT_NAME}.uf2 /media/mike/RPI-RP2
)
