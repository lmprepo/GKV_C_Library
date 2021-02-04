cmake_minimum_required(VERSION 3.11)

add_library(GKV_Communication_Library 
    src/GKV_CommunicationLibrary.c     
    src/GKV_AlgParamPacket.h
    src/GKV_AlorithmPackets.h
    src/GKV_BasePacket.h
    src/GKV_CommunicationLibrary.h
    src/GKV_CustomPacket.h
    src/GKV_FilterPacket.h
    src/GKV_GyroOffsetPacket.h
    src/GKV_SettingsPacket.h
    src/GKV_TestPacket.h)

    target_include_directories(GKV_Communication_Library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")