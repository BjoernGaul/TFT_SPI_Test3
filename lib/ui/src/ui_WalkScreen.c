// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_WalkScreen_screen_init(void)
{
    ui_WalkScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_WalkScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_DistanceBar = lv_bar_create(ui_WalkScreen);
    lv_bar_set_value(ui_DistanceBar, 25, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_DistanceBar, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_DistanceBar, 300);
    lv_obj_set_height(ui_DistanceBar, 40);
    lv_obj_set_x(ui_DistanceBar, 0);
    lv_obj_set_y(ui_DistanceBar, 90);
    lv_obj_set_align(ui_DistanceBar, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_DistanceBar, lv_color_hex(0x7E0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_DistanceBar, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_DistanceBar, lv_color_hex(0xFF0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_DistanceBar, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Label22 = lv_label_create(ui_DistanceBar);
    lv_obj_set_width(ui_Label22, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label22, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label22, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label22, "Distance");
    lv_obj_set_style_text_color(ui_Label22, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label22, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel2 = lv_obj_create(ui_WalkScreen);
    lv_obj_set_width(ui_Panel2, 150);
    lv_obj_set_height(ui_Panel2, 150);
    lv_obj_set_x(ui_Panel2, -120);
    lv_obj_set_y(ui_Panel2, -30);
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel2, 90, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel2, lv_color_hex(0x7E0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label24 = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_Label24, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label24, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label24, 0);
    lv_obj_set_y(ui_Label24, -10);
    lv_obj_set_align(ui_Label24, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Label24, "Forward");

    ui_Label25 = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_Label25, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label25, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label25, 0);
    lv_obj_set_y(ui_Label25, 10);
    lv_obj_set_align(ui_Label25, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_Label25, "Back");

    ui_Label26 = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_Label26, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label26, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label26, -10);
    lv_obj_set_y(ui_Label26, 0);
    lv_obj_set_align(ui_Label26, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_Label26, "Left");

    ui_Label27 = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_Label27, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label27, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label27, 10);
    lv_obj_set_y(ui_Label27, 0);
    lv_obj_set_align(ui_Label27, LV_ALIGN_RIGHT_MID);
    lv_label_set_text(ui_Label27, "Right");

    ui_Panel1 = lv_obj_create(ui_WalkScreen);
    lv_obj_set_width(ui_Panel1, 150);
    lv_obj_set_height(ui_Panel1, 150);
    lv_obj_set_x(ui_Panel1, 120);
    lv_obj_set_y(ui_Panel1, -30);
    lv_obj_set_align(ui_Panel1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel1, 90, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0x7E0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label28 = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_Label28, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label28, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label28, 0);
    lv_obj_set_y(ui_Label28, -10);
    lv_obj_set_align(ui_Label28, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Label28, "Up");

    ui_Label29 = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_Label29, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label29, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label29, 0);
    lv_obj_set_y(ui_Label29, 10);
    lv_obj_set_align(ui_Label29, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_Label29, "Down");

    ui_Label30 = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_Label30, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label30, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label30, -10);
    lv_obj_set_y(ui_Label30, 0);
    lv_obj_set_align(ui_Label30, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_Label30, "Left");

    ui_Label31 = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_Label31, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label31, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label31, 10);
    lv_obj_set_y(ui_Label31, 0);
    lv_obj_set_align(ui_Label31, LV_ALIGN_RIGHT_MID);
    lv_label_set_text(ui_Label31, "Right");

    ui_Label32 = lv_label_create(ui_WalkScreen);
    lv_obj_set_width(ui_Label32, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label32, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label32, -120);
    lv_obj_set_y(ui_Label32, -130);
    lv_obj_set_align(ui_Label32, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label32, "Walk");
    lv_obj_set_style_text_font(ui_Label32, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label33 = lv_label_create(ui_WalkScreen);
    lv_obj_set_width(ui_Label33, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label33, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label33, 120);
    lv_obj_set_y(ui_Label33, -130);
    lv_obj_set_align(ui_Label33, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label33, "Turn");
    lv_obj_set_style_text_font(ui_Label33, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_HomeButton2 = lv_btn_create(ui_WalkScreen);
    lv_obj_set_width(ui_HomeButton2, 100);
    lv_obj_set_height(ui_HomeButton2, 50);
    lv_obj_set_x(ui_HomeButton2, 0);
    lv_obj_set_y(ui_HomeButton2, -150);
    lv_obj_set_align(ui_HomeButton2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_HomeButton2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_HomeButton2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_HomeButton2, lv_color_hex(0x7E0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_HomeButton2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label35 = lv_label_create(ui_HomeButton2);
    lv_obj_set_width(ui_Label35, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label35, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label35, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label35, "Home");

    lv_obj_add_event_cb(ui_HomeButton2, ui_event_HomeButton2, LV_EVENT_ALL, NULL);

}
