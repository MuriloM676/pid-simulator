cmake_minimum_required(VERSION 3.10)
project(altitude_sim C)

set(CMAKE_C_STANDARD 99)

add_executable(altitude_sim
    main.c
    pid.c
    aircraft.c
)

target_compile_options(altitude_sim PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-result   # popen/pclose já tratados explicitamente
)

target_link_libraries(altitude_sim m)

