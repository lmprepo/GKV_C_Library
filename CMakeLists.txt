﻿# CMakeList.txt: файл проекта CMake верхнего уровня; проведите глобальную конфигурацию
# и включите сюда подпроекты.
#
cmake_minimum_required (VERSION 3.8)

project (Examples)

# Включите подпроекты.
add_subdirectory (GKV_Communication_Library)

add_subdirectory (Receive_Data_in_Console)

add_subdirectory (Receive_Data_in_Console_linux)

add_subdirectory (Receive_ADC_Data_Example)

add_subdirectory (Receive_ADC_Data_Example_linux)

add_subdirectory (Choose_Algorithm_Example)

add_subdirectory (Choose_Algorithm_Example_linux)


