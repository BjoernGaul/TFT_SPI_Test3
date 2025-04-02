// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_comp.h"
#include "ui_comp_hook.h"
#include "ui_events.h"

//Battery
void create_battery_status(lv_obj_t *parent);
void update_battery_status(int percentage);
void delete_battery_status();
void showBatteryWarning(const char* message);

//waring
void show_warning_label(const char *message);
extern lv_obj_t * warning_container;

// SCREEN: ui_HomeScreen
void ui_HomeScreen_screen_init(void);
extern lv_obj_t * ui_HomeScreen;
void ui_event_LimbControl1(lv_event_t * e);
extern lv_obj_t * ui_LimbControl1;
extern lv_obj_t * ui_Label1;
void ui_event_Walk(lv_event_t * e);
extern lv_obj_t * ui_Walk;
extern lv_obj_t * ui_Lable2;
void ui_event_Placeholder2(lv_event_t * e);
extern lv_obj_t * ui_sitDown;
extern lv_obj_t * ui_Lable3;
void ui_event_Placeholder1(lv_event_t * e);
extern lv_obj_t * ui_standup;
extern lv_obj_t * ui_Lable4;
void ui_event_Hump(lv_event_t * e);
extern lv_obj_t * ui_HumpBut;
extern lv_obj_t * ui_HumpLable;
// CUSTOM VARIABLES
extern lv_obj_t * uic_HomeScreen;
extern lv_obj_t * uic_LimbControl;
extern lv_obj_t * uic_LimbControl;
extern lv_obj_t * uic_LimbControl;
extern lv_obj_t * uic_LimbControl;
extern lv_obj_t * uic_LimbControl;

// SCREEN: ui_LCScreen
void ui_LCScreen_screen_init(void);
extern lv_obj_t * ui_LCScreen;
void ui_event_Button2(lv_event_t * e);
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Label11;
void ui_event_Button1(lv_event_t * e);
extern lv_obj_t * ui_Button1;
extern lv_obj_t * ui_Label12;
void ui_event_Button5(lv_event_t * e);
extern lv_obj_t * ui_Button5;
extern lv_obj_t * ui_Label13;
void ui_event_Button6(lv_event_t * e);
extern lv_obj_t * ui_Button6;
extern lv_obj_t * ui_Label14;
void ui_event_HomeButton(lv_event_t * e);
extern lv_obj_t * ui_HomeButton;
extern lv_obj_t * ui_Label16;
// CUSTOM VARIABLES
extern lv_obj_t * uic_LCScreen;

// SCREEN: ui_WalkScreen
void ui_WalkScreen_screen_init(void);
extern lv_obj_t * ui_WalkScreen;
extern lv_obj_t * ui_DistanceBar;
extern lv_obj_t * ui_Label22;
extern lv_obj_t * ui_Panel2;
extern lv_obj_t * ui_Label24;
extern lv_obj_t * ui_Label25;
extern lv_obj_t * ui_Label26;
extern lv_obj_t * ui_Label27;
extern lv_obj_t * ui_Panel1;
extern lv_obj_t * ui_Label28;
extern lv_obj_t * ui_Label29;
extern lv_obj_t * ui_Label30;
extern lv_obj_t * ui_Label31;
extern lv_obj_t * ui_Label32;
extern lv_obj_t * ui_Label33;
void ui_event_HomeButton2(lv_event_t * e);
extern lv_obj_t * ui_HomeButton2;
extern lv_obj_t * ui_Label35;
// CUSTOM VARIABLES

// SCREEN: ui_FLScreen
void ui_FLScreen_screen_init(void);
extern lv_obj_t * ui_FLScreen;
extern lv_obj_t * ui_FLLabel5;
extern lv_obj_t * ui_Side;
extern lv_obj_t * ui_Top;
extern lv_obj_t * ui_Bottom;
void ui_event_flSideSlider(lv_event_t * e);
extern lv_obj_t * ui_flSideSlider;
void ui_event_flTopSlider(lv_event_t * e);
extern lv_obj_t * ui_flTopSlider;
void ui_event_flBotSlider(lv_event_t * e);
extern lv_obj_t * ui_flBotSlider;
void ui_event_backButton(lv_event_t * e);
extern lv_obj_t * ui_backButton;
extern lv_obj_t * ui_Label18;
// CUSTOM VARIABLES

// SCREEN: ui_FRScreen
void ui_FRScreen_screen_init(void);
extern lv_obj_t * ui_FRScreen;
void ui_event_Button12(lv_event_t * e);
extern lv_obj_t * ui_Button12;
extern lv_obj_t * ui_Label21;
extern lv_obj_t * ui_FLLabel2;
extern lv_obj_t * ui_Side2;
extern lv_obj_t * ui_Top2;
extern lv_obj_t * ui_Bottom2;
void ui_event_flSideSlider2(lv_event_t * e);
extern lv_obj_t * ui_flSideSlider2;
void ui_event_flTopSlider2(lv_event_t * e);
extern lv_obj_t * ui_flTopSlider2;
void ui_event_flBotSlider2(lv_event_t * e);
extern lv_obj_t * ui_flBotSlider2;
// CUSTOM VARIABLES

// SCREEN: ui_BLScreen
void ui_BLScreen_screen_init(void);
extern lv_obj_t * ui_BLScreen;
extern lv_obj_t * ui_FLLabel1;
extern lv_obj_t * ui_Side1;
extern lv_obj_t * ui_Top1;
extern lv_obj_t * ui_Bottom1;
void ui_event_flSideSlider1(lv_event_t * e);
extern lv_obj_t * ui_flSideSlider1;
void ui_event_flTopSlider1(lv_event_t * e);
extern lv_obj_t * ui_flTopSlider1;
void ui_event_flBotSlider1(lv_event_t * e);
extern lv_obj_t * ui_flBotSlider1;
void ui_event_backButton1(lv_event_t * e);
extern lv_obj_t * ui_backButton1;
extern lv_obj_t * ui_Label2;
// CUSTOM VARIABLES

// SCREEN: ui_BRScreen
void ui_BRScreen_screen_init(void);
extern lv_obj_t * ui_BRScreen;
extern lv_obj_t * ui_FLLabel3;
extern lv_obj_t * ui_Side3;
extern lv_obj_t * ui_Top3;
extern lv_obj_t * ui_Bottom3;
void ui_event_flSideSlider3(lv_event_t * e);
extern lv_obj_t * ui_flSideSlider3;
void ui_event_flTopSlider3(lv_event_t * e);
extern lv_obj_t * ui_flTopSlider3;
void ui_event_flBotSlider3(lv_event_t * e);
extern lv_obj_t * ui_flBotSlider3;
void ui_event_backButton2(lv_event_t * e);
extern lv_obj_t * ui_backButton2;
extern lv_obj_t * ui_Label3;
// CUSTOM VARIABLES

// EVENTS

extern lv_obj_t * ui____initial_actions0;

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
