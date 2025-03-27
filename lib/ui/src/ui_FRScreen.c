// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_FRScreen_screen_init(void)
{
    ui_FRScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_FRScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags


    ui_Button12 = lv_btn_create(ui_FRScreen);
    lv_obj_set_width(ui_Button12, 100);
    lv_obj_set_height(ui_Button12, 50);
    lv_obj_set_x(ui_Button12, 0);
    lv_obj_set_y(ui_Button12, -10);
    lv_obj_set_align(ui_Button12, LV_ALIGN_TOP_MID);
    lv_obj_add_flag(ui_Button12, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button12, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button12, lv_color_hex(0x7E0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label21 = lv_label_create(ui_Button12);
    lv_obj_set_width(ui_Label21, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label21, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label21, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Label21, "Back");

    ui_FLLabel2 = lv_label_create(ui_FRScreen);
    lv_obj_set_width(ui_FLLabel2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_FLLabel2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_FLLabel2, 10);
    lv_obj_set_y(ui_FLLabel2, 10);
    lv_label_set_text(ui_FLLabel2, "Front Right");
    lv_obj_set_style_text_font(ui_FLLabel2, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Side2 = lv_label_create(ui_FRScreen);
    lv_obj_set_width(ui_Side2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Side2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Side2, 10);
    lv_obj_set_y(ui_Side2, 50);
    lv_label_set_text(ui_Side2, "Side");
    lv_obj_set_style_text_font(ui_Side2, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Top2 = lv_label_create(ui_FRScreen);
    lv_obj_set_width(ui_Top2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Top2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Top2, 10);
    lv_obj_set_y(ui_Top2, 140);
    lv_label_set_text(ui_Top2, "Top");
    lv_obj_set_style_text_font(ui_Top2, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Bottom2 = lv_label_create(ui_FRScreen);
    lv_obj_set_width(ui_Bottom2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Bottom2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Bottom2, 10);
    lv_obj_set_y(ui_Bottom2, 230);
    lv_label_set_text(ui_Bottom2, "Bottom");
    lv_obj_set_style_text_font(ui_Bottom2, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_flSideSlider2 = lv_slider_create(ui_FRScreen);
    lv_slider_set_range(ui_flSideSlider2, -90, 90);
    lv_slider_set_value(ui_flSideSlider2, 0, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_flSideSlider2) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_flSideSlider2, 0,
                                                                                                  LV_ANIM_OFF);
    lv_obj_set_width(ui_flSideSlider2, 400);
    lv_obj_set_height(ui_flSideSlider2, 30);
    lv_obj_set_x(ui_flSideSlider2, 0);
    lv_obj_set_y(ui_flSideSlider2, -60);
    lv_obj_set_align(ui_flSideSlider2, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_flSideSlider2, lv_color_hex(0x390000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flSideSlider2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_flSideSlider2, lv_color_hex(0x7E0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flSideSlider2, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_flSideSlider2, lv_color_hex(0x7E0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flSideSlider2, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_flSideSlider2, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_flSideSlider2, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_flSideSlider2, 1, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_flSideSlider2, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_flTopSlider2 = lv_slider_create(ui_FRScreen);
    lv_slider_set_range(ui_flTopSlider2, -90, 90);
    lv_slider_set_value(ui_flTopSlider2, 0, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_flTopSlider2) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_flTopSlider2, 0,
                                                                                                 LV_ANIM_OFF);
    lv_obj_set_width(ui_flTopSlider2, 400);
    lv_obj_set_height(ui_flTopSlider2, 30);
    lv_obj_set_x(ui_flTopSlider2, 0);
    lv_obj_set_y(ui_flTopSlider2, 30);
    lv_obj_set_align(ui_flTopSlider2, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_flTopSlider2, lv_color_hex(0x390000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flTopSlider2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_flTopSlider2, lv_color_hex(0x7E0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flTopSlider2, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_flTopSlider2, lv_color_hex(0x7E0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flTopSlider2, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_flTopSlider2, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_flTopSlider2, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_flTopSlider2, 1, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_flTopSlider2, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_flBotSlider2 = lv_slider_create(ui_FRScreen);
    lv_slider_set_range(ui_flBotSlider2, -90, 90);
    lv_slider_set_value(ui_flBotSlider2, 0, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_flBotSlider2) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_flBotSlider2, 0,
                                                                                                 LV_ANIM_OFF);
    lv_obj_set_width(ui_flBotSlider2, 400);
    lv_obj_set_height(ui_flBotSlider2, 30);
    lv_obj_set_x(ui_flBotSlider2, 0);
    lv_obj_set_y(ui_flBotSlider2, 120);
    lv_obj_set_align(ui_flBotSlider2, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_flBotSlider2, lv_color_hex(0x390000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flBotSlider2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_flBotSlider2, lv_color_hex(0x7E0000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flBotSlider2, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_flBotSlider2, lv_color_hex(0x7E0000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_flBotSlider2, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(ui_flBotSlider2, lv_color_hex(0x000000), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(ui_flBotSlider2, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui_flBotSlider2, 1, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(ui_flBotSlider2, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button12, ui_event_Button12, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_flSideSlider2, ui_event_flSideSlider2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_flTopSlider2, ui_event_flTopSlider2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_flBotSlider2, ui_event_flBotSlider2, LV_EVENT_ALL, NULL);

}
