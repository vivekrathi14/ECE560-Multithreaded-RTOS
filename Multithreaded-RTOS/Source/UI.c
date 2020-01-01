#include "UI.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "LCD.h"
#include "colors.h"
#include "ST7789.h"
#include "T6963.h"
#include "font.h"
#include "control.h"
#include "FX.h"
#include "timers.h"

// Green fields = read/write
// Orange fields = read-only
UI_FIELD_T Fields[] = {
	{"Duty Cycle  ", "ct", "", (volatile int *)&g_duty_cycle, NULL, {0,6}, 
	&red, &black, 1, 0, 0, 0,Control_DutyCycle_Handler},
	{"Enable Ctlr ", "", "", (volatile int *)&g_enable_control, NULL, {0,7}, 
	&green, &black, 1, 0, 0, 0, Control_OnOff_Handler},	
	{"Enable Flash", "", "", (volatile int *)&g_enable_flash, NULL, {0,8}, 
	&red, &black, 1, 0, 0, 0, Control_OnOff_Handler},	
	{"T_flash_prd ", "ms", "", (volatile int *)&g_flash_period, NULL, {0,9}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},		
	{"T_flash_on  ", "ms", "", (volatile int *)&g_flash_duration, NULL, {0,10}, 
	&red, &black,  1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_set       ", "mA", "", (volatile int *)&g_set_current, NULL, {0,11}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_set_peak  ", "mA", "", (volatile int *)&g_peak_set_current, NULL, {0,12}, 
	&red, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_measured  ", "mA", "", (volatile int *)&g_measured_current, NULL, {0,13}, 
	&green, &black, 1, 0, 1, 1, NULL},
	{"Control Mode", " ", "", (volatile int *)&control_mode, NULL, {0,14}, 
	&green, &black, 1, 0, 0, 0, Control_Mode},
	
};

UI_SLIDER_T Slider = {
	0, {0,LCD_HEIGHT-UI_SLIDER_HEIGHT}, {UI_SLIDER_WIDTH-1,LCD_HEIGHT-1}, 
	{119,LCD_HEIGHT-UI_SLIDER_HEIGHT}, {119,LCD_HEIGHT-1}, &yellow, &dark_gray, &light_gray
};

int UI_sel_field = -1;

void UI_Update_Field_Values (UI_FIELD_T * f, int num) {
	int i;
	for (i=0; i < num; i++) {
		snprintf(f[i].Buffer, sizeof(f[i].Buffer), "%s%4d %s", f[i].Label, f[i].Val? *(f[i].Val) : 0, f[i].Units);
		f[i].Updated = 1;
	}	
}

void UI_Update_Volatile_Field_Values(UI_FIELD_T * f) {
	int i;
	for (i=0; i < UI_NUM_FIELDS; i++) {
		if (f[i].Volatile) {
			snprintf(f[i].Buffer, sizeof(f[i].Buffer), "%s%4d %s", f[i].Label, f[i].Val? *(f[i].Val) : 0, f[i].Units);
			f[i].Updated = 1;
		}
	}
}

void UI_Draw_Fields(UI_FIELD_T * f, int num){
	int i;
	COLOR_T * bg_color;
	for (i=0; i < num; i++) {
		if ((f[i].Updated) || (f[i].Volatile)) { // redraw updated or volatile fields
			f[i].Updated = 0;
			if ((f[i].Selected) && (!f[i].ReadOnly)) {
				bg_color = &light_gray;
			} else {
				bg_color = f[i].ColorBG;
			}
			LCD_Text_Set_Colors(f[i].ColorFG, bg_color);
			LCD_Text_PrintStr_RC(f[i].RC.Y, f[i].RC.X, f[i].Buffer);
		}
	}
}

void UI_Draw_Slider(UI_SLIDER_T * s) {
	static int initialized=0;
	
	if (!initialized) {
		LCD_Fill_Rectangle(&s->UL, &s->LR, s->ColorBG);
		initialized = 1;
	}
	LCD_Fill_Rectangle(&s->BarUL, &s->BarLR, s->ColorBG); // Erase old bar
	
	s->BarUL.Y = s->UL.Y;
	s->BarLR.Y = s->LR.Y;
	s->BarUL.X = (s->LR.X - s->UL.X)/2 + s->Val;
	s->BarLR.X = s->BarUL.X + UI_SLIDER_BAR_WIDTH/2;
	s->BarUL.X -= UI_SLIDER_BAR_WIDTH/2;
	LCD_Fill_Rectangle(&s->BarUL, &s->BarLR, s->ColorFG); // Draw new bar
}

int UI_Identify_Field(PT_T * p) {
	int i, t, b, l, r;

	if ((p->X >= LCD_WIDTH) || (p->Y >= LCD_HEIGHT)) {
		return -1;
	}

	if ((p->X >= Slider.UL.X) && (p->X <= Slider.LR.X) 
		&& (p->Y >= Slider.UL.Y) && (p->Y <= Slider.LR.Y)) {
		return UI_SLIDER;
	}
  for (i=0; i<UI_NUM_FIELDS; i++) {
		l = COL_TO_X(Fields[i].RC.X);
		r = l + strlen(Fields[i].Buffer)*CHAR_WIDTH;
		t = ROW_TO_Y(Fields[i].RC.Y);
		b = t + CHAR_HEIGHT-1;
		
		if ((p->X >= l) && (p->X <= r) 
			&& (p->Y >= t) && (p->Y <= b) ) {
			return i;
		}
	}
	return -1;
}

void UI_Update_Field_Selects(int sel) {
	int i;
	for (i=0; i < UI_NUM_FIELDS; i++) {
		Fields[i].Selected = (i == sel)? 1 : 0;
	}
}

void UI_Process_Touch(PT_T * p) {  // Called by Thread_Read_TS
	int i;
	
	i = UI_Identify_Field(p);
	if (i == UI_SLIDER) {
		Slider.Val = p->X - (Slider.LR.X - Slider.UL.X)/2; // Determine slider position (value)
		if (UI_sel_field >= 0) {  // If a field is selected...
			if (Fields[UI_sel_field].Val != NULL) {
				if (Fields[UI_sel_field].Handler != NULL) {
					(*Fields[UI_sel_field].Handler)(&Fields[UI_sel_field], Slider.Val); // Have the field handle the new slider value
				}
				UI_Update_Field_Values(&Fields[UI_sel_field], 1);
			}
		}
	} else if (i>=0) {
		if (!Fields[i].ReadOnly) { // Can't select (and modify) a ReadOnly field
			UI_sel_field = i;
			UI_Update_Field_Selects(UI_sel_field);
			UI_Update_Field_Values(Fields, UI_NUM_FIELDS);
			Slider.Val = 0; // return to slider to zero if a different field is selected
		}
	} 
}

void UI_Draw_Screen(int first_time) { // Called by Thread_Update_Screen
	static uint32_t counter=0;
	char buffer[32];
	uint16_t i;
	static PT_T a,b,c,d,pi,pi1_M,pi2_M,pi1_S,pi2_S,r1,r2;
	a.X = 0, a.Y = 0, b.X=239, b.Y= 109;
	c.X = 0, c.Y = 109, d.X = 239, d.Y = 0;
	r1.X = 1, r1.Y = 1, r2.X = 238, r2.Y = 108;
	
	if (first_time) {
		UI_Update_Field_Values(Fields, UI_NUM_FIELDS);
		
	}
	
	UI_Update_Volatile_Field_Values(Fields);
	UI_Draw_Fields(Fields, UI_NUM_FIELDS);
	UI_Draw_Slider(&Slider);
	LCD_Text_Set_Colors(&white, &black);
	
	// Fill Background
	LCD_Fill_Rectangle(&r1,&r2,&black);
	LCD_Draw_Line(&a,&d,&white);
	LCD_Draw_Line(&b,&d,&white);
	LCD_Draw_Line(&b,&c,&white);
	LCD_Draw_Line(&c,&a,&white);

	
	
	//Measured Current - half pulse, pixel, dynamic
	for(i = 0; i<239;i++)
	{
		pi1_M.Y = 100 - arrM[i];
		pi1_S.Y = 100 - arrS[i];
		pi1_M.X = i;
		pi1_S.X = i;
		pi2_M.Y = 100 - arrM[i+1];
		pi2_S.Y = 100 - arrS[i+1];
		pi2_M.X = i+1;
		pi2_S.X = i+1;
		LCD_Draw_Line(&pi1_S,&pi2_S,&cyan);
		LCD_Draw_Line(&pi1_M,&pi2_M,&magenta);
		
	}
	
	draw = 0;
	
	
	
//	LCD_Text_PrintStr_RC(0, 6, "screen redraws");
//   
//	sprintf(buffer, "%5d", counter++);
//	LCD_Text_PrintStr_RC(0, 0, buffer);
}
