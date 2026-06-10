cmake_minimum_required(VERSION 3.26)
project(remote C)

set(CMAKE_C_STANDARD 11)

include_directories(.)

add_executable(remote
        i6x.c
        i6x.h
        i6x_receive.c)
