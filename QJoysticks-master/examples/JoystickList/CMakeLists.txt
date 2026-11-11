qt_add_executable(JoystickListExample
    main.cpp
    main.qml
)

# Qt modules
target_link_libraries(JoystickListExample PRIVATE QJoysticksLib Qt6::Core Qt6::Widgets Qt6::Qml Qt6::Quick)
