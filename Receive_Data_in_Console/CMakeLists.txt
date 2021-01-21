# CMakeList.txt: проект CMake для Receive_ADC_Data_Example; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)


add_executable (Receive_Data_in_Console Receive_Data_in_Console.c)

target_link_libraries(Receive_Data_in_Console PRIVATE GKV_Communication_Library)