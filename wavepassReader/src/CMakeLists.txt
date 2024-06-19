include_directories(${CMAKE_CURRENT_LIST_DIR}
                    ${CMAKE_CURRENT_LIST_DIR}/../include)

link_libraries(pico_multicore pico_stdlib pico_multicore tinyusb_device tinyusb_board)
# Add executable. Default name is the project name, version 0.1
add_executable(wavepass_pico wavepass_pico.cpp usb_descriptors.cpp ACIO.cpp ICCx.cpp Cipher.cpp)

pico_set_program_name(wavepass_pico "wavepass_pico")
pico_set_program_version(wavepass_pico "0.1")

pico_enable_stdio_uart(wavepass_pico 0)
pico_enable_stdio_usb(wavepass_pico 1)

# Add the standard library to the build
target_link_libraries(wavepass_pico
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(wavepass_pico PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(wavepass_pico 
        )

pico_add_extra_outputs(wavepass_pico)

