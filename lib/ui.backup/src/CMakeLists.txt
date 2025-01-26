SET(SOURCES ui_comp_limbcontrol.c
    ui_comp.c
    ui_HomeScreen.c
    ui_LCScreen.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c)

add_library(ui ${SOURCES})
