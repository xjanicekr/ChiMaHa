# Create QJoysticks tests
qt_add_executable(QJoysticks_Test
    main.cpp
    Test_QJoysticks.h
)

target_link_libraries(QJoysticks_Test PRIVATE QJoysticksLib Qt6::Test)
