cmake_minimum_required(VERSION 3.16)
project(QJoysticks VERSION 1.0 LANGUAGES CXX)

# Options
option(DISABLE_SDL "Disable SDL support" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#-------------------------------
# Find Qt
#-------------------------------
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui)

#-------------------------------
# Sources, headers, resources
#-------------------------------
set(QJOYSTICKS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(QJOYSTICKS_SOURCES
    src/QJoysticks.cpp
    src/QJoysticks/SDL_Joysticks.cpp
    src/QJoysticks/VirtualJoystick.cpp
    src/QJoysticks/Android_Joystick.cpp
)

set(QJOYSTICKS_HEADERS
    src/QJoysticks.h
    src/QJoysticks/JoysticksCommon.h
    src/QJoysticks/SDL_Joysticks.h
    src/QJoysticks/VirtualJoystick.h
    src/QJoysticks/Android_Joystick.h
)

set(QJOYSTICKS_RESOURCES
    etc/resources/qjoysticks-res.qrc
)

#-------------------------------
# Build library
#-------------------------------
qt_add_library(QJoysticksLib STATIC
    ${QJOYSTICKS_SOURCES}
    ${QJOYSTICKS_HEADERS}
    ${QJOYSTICKS_RESOURCES}
)

target_include_directories(QJoysticksLib PUBLIC ${QJOYSTICKS_INCLUDE_DIR})
target_link_libraries(QJoysticksLib PUBLIC Qt6::Core Qt6::Gui Qt6::Widgets)

#-------------------------------
# SDL: optional cross-platform
#-------------------------------
if(NOT DISABLE_SDL)
    if(WIN32)
        set(SDL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/SDL/include)
        set(SDL_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/SDL/bin/windows/x64/SDL2.lib)

        add_library(SDL STATIC IMPORTED)
        set_target_properties(SDL PROPERTIES
            IMPORTED_LOCATION ${SDL_LIB}
            INTERFACE_INCLUDE_DIRECTORIES ${SDL_INCLUDE_DIR}
        )

        message(STATUS "Using prebuilt SDL2: ${SDL_LIB}")

    else()
        find_package(SDL2 REQUIRED)
        add_library(SDL STATIC IMPORTED GLOBAL)
        set_target_properties(SDL PROPERTIES
            IMPORTED_LOCATION ${SDL2_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${SDL2_INCLUDE_DIRS}
        )

        message(STATUS "Using system SDL2: ${SDL2_LIBRARY}")
    endif()

    target_link_libraries(QJoysticksLib PRIVATE SDL)
endif()

#-------------------------------
# Export library for parent project
#-------------------------------
add_library(QJoysticks::QJoysticksLib ALIAS QJoysticksLib)
