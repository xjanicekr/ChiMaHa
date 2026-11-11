qt_add_executable(joystickApp
    main.cpp
    mainwindow.cpp
    mainwindow.ui
)

target_link_libraries(joystickApp PRIVATE QJoysticksLib Qt6::Core Qt6::Gui Qt6::Widgets)
