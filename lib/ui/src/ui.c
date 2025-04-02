// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"
#include <stdio.h>
#include <Arduino.h>

///////////////////// VARIABLES ////////////////////

//Battery

lv_obj_t *ui_BatteryIcon;
lv_obj_t *ui_BatteryLabel;

void create_battery_status(lv_obj_t *parent) {
    // Create the battery icon
    ui_BatteryIcon = lv_obj_create(parent);
    lv_obj_set_width(ui_BatteryIcon, 40);
    lv_obj_set_height(ui_BatteryIcon, 20);
    lv_obj_set_x(ui_BatteryIcon, 0);
    lv_obj_set_y(ui_BatteryIcon, 0);
    lv_obj_set_align(ui_BatteryIcon, LV_ALIGN_TOP_RIGHT);
    lv_obj_set_style_bg_color(ui_BatteryIcon, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BatteryIcon, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Create the battery percentage label
    ui_BatteryLabel = lv_label_create(parent);
    lv_obj_set_width(ui_BatteryLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_BatteryLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_BatteryLabel, -10);
    lv_obj_set_y(ui_BatteryLabel, 2);
    lv_obj_set_align(ui_BatteryLabel, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_BatteryLabel, "100");
    lv_obj_set_style_text_color(ui_BatteryLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_BatteryLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void delete_battery_status() {
    if (ui_BatteryIcon != NULL) {
        lv_obj_del(ui_BatteryIcon);
        ui_BatteryIcon = NULL;
    }
    if (ui_BatteryLabel != NULL) {
        lv_obj_del(ui_BatteryLabel);
        ui_BatteryLabel = NULL;
    }
}

void update_battery_status(int percentage) {
    if (ui_BatteryLabel == NULL || ui_BatteryIcon == NULL) {
        return;
    }
    // Update the battery label with the current percentage
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%d", percentage);
    lv_label_set_text(ui_BatteryLabel, buffer);

    // Update the battery icon color based on the percentage
    if (percentage > 75) {
        lv_obj_set_style_bg_color(ui_BatteryIcon, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT); // Green
    } else if (percentage > 50) {
        lv_obj_set_style_bg_color(ui_BatteryIcon, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT); // Yellow
    } else if (percentage > 25) {
        lv_obj_set_style_bg_color(ui_BatteryIcon, lv_color_hex(0xFFA500), LV_PART_MAIN | LV_STATE_DEFAULT); // Orange
    } else {
        lv_obj_set_style_bg_color(ui_BatteryIcon, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT); // Red
    }
}

void showBatteryWarning(const char* message) {
    // Create a label on the active screen
    lv_obj_t* warningLabel = lv_label_create(lv_scr_act());

    // Set the text of the label
    lv_label_set_text(warningLabel, message);

    // Align the label to the center of the screen
    lv_obj_align(warningLabel, LV_ALIGN_CENTER, 0, 0);

    // Optionally, set a style for the label (e.g., red text for warnings)
    lv_obj_set_style_text_color(warningLabel, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_obj_set_style_text_font(warningLabel, LV_FONT_DEFAULT, LV_PART_MAIN);
}

// SCREEN: ui_HomeScreen
void ui_HomeScreen_screen_init(void);
lv_obj_t * ui_HomeScreen;
void ui_event_LimbControl1(lv_event_t * e);
lv_obj_t * ui_LimbControl1;
lv_obj_t * ui_Label1;
void ui_event_Walk(lv_event_t * e);
lv_obj_t * ui_Walk;
lv_obj_t * ui_Lable2;
void ui_event_Placeholder2(lv_event_t * e);
lv_obj_t * ui_sitDown;
lv_obj_t * ui_Lable3;
void ui_event_Placeholder1(lv_event_t * e);
lv_obj_t * ui_standup;
lv_obj_t * ui_Lable4;
void ui_event_Hump(lv_event_t * e);
lv_obj_t * ui_HumpBut;
lv_obj_t * ui_HumpLable;
// CUSTOM VARIABLES
lv_obj_t * uic_HomeScreen;
lv_obj_t * uic_LimbControl;
lv_obj_t * uic_LimbControl;
lv_obj_t * uic_LimbControl;
lv_obj_t * uic_LimbControl;
lv_obj_t * uic_LimbControl;
lv_obj_t * warning_container;


// SCREEN: ui_LCScreen
void ui_LCScreen_screen_init(void);
lv_obj_t * ui_LCScreen;
void ui_event_Button2(lv_event_t * e);
lv_obj_t * ui_Button2;
lv_obj_t * ui_Label11;
void ui_event_Button1(lv_event_t * e);
lv_obj_t * ui_Button1;
lv_obj_t * ui_Label12;
void ui_event_Button5(lv_event_t * e);
lv_obj_t * ui_Button5;
lv_obj_t * ui_Label13;
void ui_event_Button6(lv_event_t * e);
lv_obj_t * ui_Button6;
lv_obj_t * ui_Label14;
void ui_event_HomeButton(lv_event_t * e);
lv_obj_t * ui_HomeButton;
lv_obj_t * ui_Label16;
// CUSTOM VARIABLES
lv_obj_t * uic_LCScreen;


// SCREEN: ui_WalkScreen
void ui_WalkScreen_screen_init(void);
lv_obj_t * ui_WalkScreen;
lv_obj_t * ui_DistanceBar;
lv_obj_t * ui_Label22;
lv_obj_t * ui_Panel2;
lv_obj_t * ui_Label24;
lv_obj_t * ui_Label25;
lv_obj_t * ui_Label26;
lv_obj_t * ui_Label27;
lv_obj_t * ui_Panel1;
lv_obj_t * ui_Label28;
lv_obj_t * ui_Label29;
lv_obj_t * ui_Label30;
lv_obj_t * ui_Label31;
lv_obj_t * ui_Label32;
lv_obj_t * ui_Label33;
void ui_event_HomeButton2(lv_event_t * e);
lv_obj_t * ui_HomeButton2;
lv_obj_t * ui_Label35;
// CUSTOM VARIABLES


// SCREEN: ui_FLScreen
void ui_FLScreen_screen_init(void);
lv_obj_t * ui_FLScreen;
lv_obj_t * ui_FLLabel5;
lv_obj_t * ui_Side;
lv_obj_t * ui_Top;
lv_obj_t * ui_Bottom;
void ui_event_flSideSlider(lv_event_t * e);
lv_obj_t * ui_flSideSlider;
void ui_event_flTopSlider(lv_event_t * e);
lv_obj_t * ui_flTopSlider;
void ui_event_flBotSlider(lv_event_t * e);
lv_obj_t * ui_flBotSlider;
void ui_event_backButton(lv_event_t * e);
lv_obj_t * ui_backButton;
lv_obj_t * ui_Label18;
// CUSTOM VARIABLES


// SCREEN: ui_FRScreen
void ui_FRScreen_screen_init(void);
lv_obj_t * ui_FRScreen;
void ui_event_Button12(lv_event_t * e);
lv_obj_t * ui_Button12;
lv_obj_t * ui_Label21;
lv_obj_t * ui_FLLabel2;
lv_obj_t * ui_Side2;
lv_obj_t * ui_Top2;
lv_obj_t * ui_Bottom2;
void ui_event_flSideSlider2(lv_event_t * e);
lv_obj_t * ui_flSideSlider2;
void ui_event_flTopSlider2(lv_event_t * e);
lv_obj_t * ui_flTopSlider2;
void ui_event_flBotSlider2(lv_event_t * e);
lv_obj_t * ui_flBotSlider2;
// CUSTOM VARIABLES


// SCREEN: ui_BLScreen
void ui_BLScreen_screen_init(void);
lv_obj_t * ui_BLScreen;
lv_obj_t * ui_FLLabel1;
lv_obj_t * ui_Side1;
lv_obj_t * ui_Top1;
lv_obj_t * ui_Bottom1;
void ui_event_flSideSlider1(lv_event_t * e);
lv_obj_t * ui_flSideSlider1;
void ui_event_flTopSlider1(lv_event_t * e);
lv_obj_t * ui_flTopSlider1;
void ui_event_flBotSlider1(lv_event_t * e);
lv_obj_t * ui_flBotSlider1;
void ui_event_backButton1(lv_event_t * e);
lv_obj_t * ui_backButton1;
lv_obj_t * ui_Label2;
// CUSTOM VARIABLES


// SCREEN: ui_BRScreen
void ui_BRScreen_screen_init(void);
lv_obj_t * ui_BRScreen;
lv_obj_t * ui_FLLabel3;
lv_obj_t * ui_Side3;
lv_obj_t * ui_Top3;
lv_obj_t * ui_Bottom3;
void ui_event_flSideSlider3(lv_event_t * e);
lv_obj_t * ui_flSideSlider3;
void ui_event_flTopSlider3(lv_event_t * e);
lv_obj_t * ui_flTopSlider3;
void ui_event_flBotSlider3(lv_event_t * e);
lv_obj_t * ui_flBotSlider3;
void ui_event_backButton2(lv_event_t * e);
lv_obj_t * ui_backButton2;
lv_obj_t * ui_Label3;
// CUSTOM VARIABLES

// EVENTS
lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_LimbControl1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_LCScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_LCScreen_screen_init);
    }
    if(event_code == LV_EVENT_CLICKED) {
        getPositionLegs(e);
    }
}

void ui_event_Walk(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_WalkScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_WalkScreen_screen_init);
    }
}

void ui_event_Placeholder2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        sendSit1(e);
    }
}

void ui_event_Placeholder1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        standSend1(e);
    }
}

void ui_event_Hump(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        sendHump(e);
    }
}

void ui_event_Button2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_FLScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_FLScreen_screen_init);
    }
}

void ui_event_Button1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_FRScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_FRScreen_screen_init);
    }
}

void ui_event_Button5(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_BLScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_BLScreen_screen_init);
    }
}

void ui_event_Button6(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_BRScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_BRScreen_screen_init);
    }
}

void ui_event_HomeButton(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_HomeScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_HomeScreen_screen_init);
    }
    if(event_code == LV_EVENT_CLICKED) {
        resetPositionDog(e);
    }
}

void ui_event_HomeButton2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {

        _ui_screen_change(&ui_HomeScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_HomeScreen_screen_init);
    }
}

void ui_event_flSideSlider(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        flSideChangeVal(e);
    }
}

void ui_event_flTopSlider(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        flTopChangeVal(e);
    }
}

void ui_event_flBotSlider(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        flBotChangeVal(e);
    }
}

void ui_event_backButton(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_LCScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_LCScreen_screen_init);
    }
}

void ui_event_Button12(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_LCScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_LCScreen_screen_init);
    }
}

void ui_event_flSideSlider2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        frSideChangeVal(e);
    }
}

void ui_event_flTopSlider2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        frTopChangeVal(e);
    }
}

void ui_event_flBotSlider2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        frBotChangeVal(e);
    }
}

void ui_event_flSideSlider1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        blSideChangeVal(e);
    }
}

void ui_event_flTopSlider1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        blTopChangeVal(e);
    }
}

void ui_event_flBotSlider1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        blBotChangeVal(e);
    }
}

void ui_event_backButton1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_LCScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_LCScreen_screen_init);
    }
}

void ui_event_flSideSlider3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        brSideChangeVal(e);
    }
}

void ui_event_flTopSlider3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        brTopChangeVal(e);
    }
}

void ui_event_flBotSlider3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        brBotChangeVal(e);
    }
}

void ui_event_backButton2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_LCScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_LCScreen_screen_init);
    }
}

void show_warning_label(const char *message) {
    // Create a full-screen container for the warning
    warning_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(warning_container, lv_obj_get_width(lv_scr_act()), lv_obj_get_height(lv_scr_act()));
    lv_obj_set_style_bg_color(warning_container, lv_color_hex(0xFF0000), 0); // Red background
    lv_obj_set_style_bg_opa(warning_container, LV_OPA_50, 0);                // Semi-transparent
    lv_obj_add_flag(warning_container, LV_OBJ_FLAG_CLICKABLE);              // Block interactions with underlying objects

    // Create a label for the warning message
    lv_obj_t *warning_label = lv_label_create(warning_container);
    lv_label_set_text(warning_label, message);
    lv_obj_set_style_text_color(warning_label, lv_color_hex(0xFFFFFF), 0);  // White text
    lv_obj_set_style_text_font(warning_label, &lv_font_montserrat_24, 0);  // Font size
    lv_obj_align(warning_label, LV_ALIGN_CENTER, 0, 0);                    // Center the label

}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    LV_EVENT_GET_COMP_CHILD = lv_event_register_id();

    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_HomeScreen_screen_init();
    ui_LCScreen_screen_init();
    ui_WalkScreen_screen_init();
    ui_FLScreen_screen_init();
    ui_FRScreen_screen_init();
    ui_BLScreen_screen_init();
    ui_BRScreen_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_HomeScreen);
}
