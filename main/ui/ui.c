// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_ContainerPageM1;
lv_obj_t * ui_Chart1;
lv_obj_t * ui_ButtonChartRestart;
lv_obj_t * ui_ContainerPage0;
lv_obj_t * ui_Container1c1;
lv_obj_t * ui_Container1c1c1;
lv_obj_t * ui_LabelRealVoltage;
lv_obj_t * ui_Container1c1c2;
lv_obj_t * ui_LabelRealCurrent;
lv_obj_t * ui_Container1c1c3;
lv_obj_t * ui_LabelRealPower;
lv_obj_t * ui_Container1c2;
lv_obj_t * ui_Container1c2c1;
lv_obj_t * ui_LabelRealTemp;
lv_obj_t * ui_Container1c2c2;
lv_obj_t * ui_LabelRealDuty;
lv_obj_t * ui_Container1c2c3;
lv_obj_t * ui_ButtonHeatingToggle;
lv_obj_t * ui_LabelHeatingToggle;
lv_obj_t * ui_ContainerPage1;
lv_obj_t * ui_Container8;
lv_obj_t * ui_Label9;
lv_obj_t * ui_SliderSetKp;
lv_obj_t * ui_LabelSetKp;
lv_obj_t * ui_Container9;
lv_obj_t * ui_Label10;
lv_obj_t * ui_SliderSetKi;
lv_obj_t * ui_LabelSetKi;
lv_obj_t * ui_Container10;
lv_obj_t * ui_Label11;
lv_obj_t * ui_SliderSetKd;
lv_obj_t * ui_LabelSetKd;
lv_obj_t * ui_LabelDebug;
lv_obj_t * ui_ContainerPage2;
lv_obj_t * ui_Container11;
lv_obj_t * ui_Label17;
lv_obj_t * ui_SliderSetTemp;
lv_obj_t * ui_LabelSetTemp;
lv_obj_t * ui_Container2;
lv_obj_t * ui_Label3;
lv_obj_t * ui_SliderSetMaxPower;
lv_obj_t * ui_LabelSetMaxPower;
lv_obj_t * ui_Container3;
lv_obj_t * ui_Label4;
lv_obj_t * ui_SliderSetRPcb;
lv_obj_t * ui_LabelSetRPcb;
lv_obj_t * ui_Container1;
lv_obj_t * ui_Label2;
lv_obj_t * ui_SliderSetRDiv;
lv_obj_t * ui_LabelSetRDiv;
lv_obj_t * ui_Container6;
lv_obj_t * ui_Label7;
lv_obj_t * ui_SliderSetADCOffset;
lv_obj_t * ui_LabelSetADCOffset;
lv_obj_t * ui_ContainerPage3;
lv_obj_t * ui_Container16;
lv_obj_t * ui_Label1;
lv_obj_t * ui_SliderSetBL;
lv_obj_t * ui_LabelSetBL;
lv_obj_t * ui_Container4;
lv_obj_t * ui_Label5;
lv_obj_t * ui_SliderSetVol;
lv_obj_t * ui_LabelSetVol;
lv_obj_t * ui_Container5;
lv_obj_t * ui_Label6;
lv_obj_t * ui_SliderSetNote;
lv_obj_t * ui_LabelSetNote;
lv_obj_t * ui_ContainerPage4;
lv_obj_t * ui_Roller1;
lv_obj_t * ui_ContainerPage5;
lv_obj_t * ui_Panel1;
lv_obj_t * ui_LabelPDInfo;
// CUSTOM VARIABLES

// EVENTS
lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Screen1);
}
