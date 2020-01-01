#include <MKL25Z4.H>
#include <stdio.h>
#include <stdint.h>

#include "gpio_defs.h"
#include "debug.h"

#include "control.h"
#include "threads.h"
#include "timers.h"
#include "delay.h"
#include "LEDs.h"
#include "UI.h"

#include "FX.h"

volatile int16_t g_duty_cycle=5;  // global to give debugger access

volatile int g_enable_flash=1;
volatile int g_peak_set_current=FLASH_CURRENT_MA; // Peak flash current
volatile int g_flash_duration=FLASH_DURATION_MS;
volatile int g_flash_period=FLASH_PERIOD_MS; 

volatile int g_enable_control=1;
volatile int g_set_current=0; // Default starting LED current

volatile int g_measured_current;
volatile int error;
volatile int arrM[250], arrS[250];
volatile int sample = 0;
volatile int draw = 0;




int32_t pGain_8 = PGAIN_8; // proportional gain numerator scaled by 2^8

SPid plantPID = {0, // dState
	0, // iState
	LIM_DUTY_CYCLE, // iMax
	-LIM_DUTY_CYCLE, // iMin
	P_GAIN_FL, // pGain
	I_GAIN_FL, // iGain
	D_GAIN_FL  // dGain
};

SPidFX plantPID_FX = {FL_TO_FX(0), // dState
	FL_TO_FX(0), // iState
	FL_TO_FX(LIM_DUTY_CYCLE), // iMax
	FL_TO_FX(-LIM_DUTY_CYCLE), // iMin
	P_GAIN_FX, // pGain
	I_GAIN_FX, // iGain
	D_GAIN_FX  // dGain
};

float UpdatePID(SPid * pid, float error, float position){
	float pTerm, dTerm, iTerm;

	// calculate the proportional term
	pTerm = pid->pGain * error;
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState; // calculate the integral term
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;

	return pTerm + iTerm - dTerm;
}

FX16_16 UpdatePID_FX(SPidFX * pid, FX16_16 error_FX, FX16_16 position_FX){
	FX16_16 pTerm, dTerm, iTerm, diff, ret_val;

	// calculate the proportional term
	pTerm = Multiply_FX(pid->pGain, error_FX);

	// calculate the integral state with appropriate limiting
	pid->iState = Add_FX(pid->iState, error_FX);
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	
	iTerm = Multiply_FX(pid->iGain, pid->iState); // calculate the integral term
	diff = Subtract_FX(position_FX, pid->dState);
	dTerm = Multiply_FX(pid->dGain, diff);
	pid->dState = position_FX;

	ret_val = Add_FX(pTerm, iTerm);
	ret_val = Subtract_FX(ret_val, dTerm);
	return ret_val;
}

void Control_HBLED(void) {
	uint16_t res;
	static uint16_t i=0,j=0;
	FX16_16 change_FX, error_FX;
	
	FPTB->PSOR = MASK(DBG_CONTROLLER);
	
#if USE_ADC_INTERRUPT
	// already completed conversion, so don't wait
#else
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
		; // wait until end of conversion
#endif
	res = ADC0->R[0];
	
	g_measured_current = (res*1500)>>16; // Extra Credit: Make this code work: V_REF_MV*MA_SCALING_FACTOR)/(ADC_FULL_SCALE*R_SENSE)
	
	//Sampling
	
	if(i<960 && sample)
	{
		if(i++%4 == 0)
		{
			arrS[j] = g_set_current;
			arrM[j++] = g_measured_current;
			
			
		}
	}
	if(j == 240)
		{
			j=0;
			i=0;
			sample = 0;
			draw = 1;
		}
	
	
	
	
	if (g_enable_control) {
		switch (control_mode) {
			case OpenLoop:
					// don't do anything!
				break;
			case BangBang:
				if (g_measured_current < g_set_current)
					g_duty_cycle = LIM_DUTY_CYCLE;
				else
					g_duty_cycle = 0;
				break;
			case Incremental:
				if (g_measured_current < g_set_current)
					g_duty_cycle += INC_STEP;
				else
					g_duty_cycle -= INC_STEP;
				break;
			case Proportional:
				g_duty_cycle += (pGain_8*(g_set_current - g_measured_current))/256; //  - 1;
			break;
			case PID:
				g_duty_cycle += UpdatePID(&plantPID, g_set_current - g_measured_current, g_measured_current);
				break;
			case PID_FX:
				error_FX = INT_TO_FX(g_set_current - g_measured_current);
				change_FX = UpdatePID_FX(&plantPID_FX, error_FX, INT_TO_FX(g_measured_current));
				g_duty_cycle += FX_TO_INT(change_FX);
			break;
			default:
				break;
		}
	
	
		// Update PWM controller with duty cycle
		if (g_duty_cycle < 0)
			g_duty_cycle = 0;
		else if (g_duty_cycle > LIM_DUTY_CYCLE)
			g_duty_cycle = LIM_DUTY_CYCLE;
		PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, g_duty_cycle);
	} // if g_enable_control
	FPTB->PCOR = MASK(DBG_CONTROLLER);
}

#if USE_ADC_INTERRUPT
// Update this 
void ADC0_IRQHandler() {
	static osStatus_t result;
	static MSG_REQ req_msg_in_ADC_IRQ;
	static MSG_RES res_msg;
	//enum ADC_CONV {HBLED_PRIORITY, TS_QUEUE} ;
	
//	typedef enum {HBLED_PRIORITY, TS_QUEUE} ADC_CONV;
	//static PRIORITY ADC_PRIORITY;
	//static enum ADC_CONV PRIORITY;
	static int PRIORITY = 0;
	FPTB->PSOR = MASK(DBG_IRQ_ADC);
	//Control_HBLED();

	switch(PRIORITY)
	{
		case 0: Control_HBLED();
				if (osMessageQueueGetCount(ADC_REQ_ID)!=0)
				{
				PRIORITY = 1;
				result = osMessageQueueGet(ADC_REQ_ID,&req_msg_in_ADC_IRQ,NULL,0);
				if (result==osOK)
				{
					FPTB->PSOR = MASK(DBG_TS_ADC);
					ADC0->SC2 = 0; // SW Trigger
					ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK; // clear channel
					ADC0->SC1[0] |= req_msg_in_ADC_IRQ.cnum; // write channel
					ADC0->SC1[0] |= ADC_SC1_AIEN(1); // enable interrupt
					res_msg.cnum = req_msg_in_ADC_IRQ.cnum;
				}
				}
				break;
				
		case 1: res_msg.adc_conv = ADC0->R[0];
				res_msg.cnum = req_msg_in_ADC_IRQ.cnum;
				osMessageQueuePut(ADC_RES_ID,&res_msg,NULL,0);
				FPTB->PCOR = MASK(DBG_TS_ADC);
				PRIORITY = 0;
				ADC0->SC2 = ADC_SC2_REFSEL(0);
				ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;
				ADC0->SC2 |= ADC_SC2_ADTRG(1);
				ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
				ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
				SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
				ADC0->SC1[0] |= ADC_SC1_AIEN(1);
				break;
			
				
	}
//	if (PRIORITY == 0)
//	{
//		Control_HBLED();
//		if (osMessageQueueGetCount(ADC_REQ_ID)!=0)
//		{
//		PRIORITY = 1;
//		result = osMessageQueueGet(ADC_REQ_ID,&req_msg_in_ADC_IRQ,NULL,0);
//		if (result==osOK)
//		{
//			FPTB->PSOR = MASK(DBG_TS_ADC);
//			ADC0->SC2 = 0; // SW Trigger
//			ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK; // clear channel
//			ADC0->SC1[0] |= req_msg_in_ADC_IRQ.cnum; // write channel
//			ADC0->SC1[0] |= ADC_SC1_AIEN(1); // enable interrupt
//			res_msg.cnum = req_msg_in_ADC_IRQ.cnum;
//		}
//	}
//	}
//	else if (PRIORITY == 1)
//	{
//		res_msg.adc_conv = ADC0->R[0];
//		res_msg.cnum = req_msg_in_ADC_IRQ.cnum;
//		osMessageQueuePut(ADC_RES_ID,&res_msg,NULL,0);
//		FPTB->PSOR = MASK(DBG_TS_ADC);
//		PRIORITY = 0;
//		ADC0->SC2 = ADC_SC2_REFSEL(0);
//		ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;
//		ADC0->SC2 |= ADC_SC2_ADTRG(1);
//		ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
//		ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
//		SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
//		ADC0->SC1[0] |= ADC_SC1_AIEN(1);
//	}

//	if (osMessageQueueGetCount(ADC_REQ_ID)!=0)
//	{
//		PRIORITY = 1;
//		result = osMessageQueueGet(ADC_REQ_ID,&req_msg_in_ADC_IRQ,NULL,0);
//		if (result==osOK)
//		{
//			ADC0->SC2 = 0; // SW Trigger
//			ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK; // clear channel
//			ADC0->SC1[0] |= req_msg_in_ADC_IRQ.cnum; // write channel
//			ADC0->SC1[0] |= ADC_SC1_AIEN(1); // enable interrupt
//			res_msg.cnum = req_msg_in_ADC_IRQ.cnum;
//		}
//	}
//	else
//	{
//		PRIORITY = 0;
//		ADC0->SC2 = ADC_SC2_REFSEL(0);
//		ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;
//		ADC0->SC2 |= ADC_SC2_ADTRG(1);
//		ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
//		ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
//		SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
//		ADC0->SC1[0] |= ADC_SC1_AIEN(1);
//	}
//	
	FPTB->PCOR = MASK(DBG_IRQ_ADC);
}
#endif

void Set_DAC(unsigned int code) {
	// Force 16-bit write to DAC
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Set_DAC_mA(unsigned int current) {
	unsigned int code = MA_TO_DAC_CODE(current);
	// Force 16-bit write to DAC
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Init_DAC_HBLED(void) {
  // Enable clock to DAC and Port E
	SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Select analog for pin
	PORTE->PCR[DAC_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[DAC_POS] |= PORT_PCR_MUX(0);	
		
	// Disable buffer mode
	DAC0->C1 = 0;
	DAC0->C2 = 0;
	
	// Enable DAC, select VDDA as reference voltage
	DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
	Set_DAC(0);
}

void Init_ADC_HBLED(void) {
#if USE_ADC_FOR_BUCK
	// Configure ADC to read Ch 8 (FPTB 0)
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
	ADC0->CFG1 = 0x0C; // 16 bit
	//	ADC0->CFG2 = ADC_CFG2_ADLSTS(3);
	ADC0->SC2 = ADC_SC2_REFSEL(0);

#if USE_ADC_HW_TRIGGER
	// Enable hardware triggering of ADC
	ADC0->SC2 |= ADC_SC2_ADTRG(1);
	// Select triggering by TPM0 Overflow
	SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
	// Select input channel 
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
#endif // USE_ADC_HW_TRIGGER

#if USE_ADC_INTERRUPT 
	// enable ADC interrupt
	ADC0->SC1[0] |= ADC_SC1_AIEN(1);

	// Configure NVIC for ADC interrupt
	NVIC_SetPriority(ADC0_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(ADC0_IRQn); 
	NVIC_EnableIRQ(ADC0_IRQn);	
#endif // USE_ADC_INTERRUPT
#endif // USE_ADC_FOR_BUCK
}

void Update_Set_Current(void) {
	static int delay=0;

	if (delay == 0){			// Just for initialization from global
		delay = g_flash_period;
	}
	if (g_enable_flash){
		delay--;
		if(delay == (g_flash_duration + 1 )) // everyone change this
		{
			sample = 1;
		}
		
		if (delay == g_flash_duration) { // assumes runs every 1 ms
			//sample = 1; // trigger to start sampling
			g_set_current = g_peak_set_current;
			Set_DAC_mA(g_set_current);
			
		} else  if (delay == 0) {
			delay = g_flash_period;
			g_set_current = 0;
			Set_DAC_mA(g_set_current);
		}
	}
}

void Init_Buck_HBLED(void) {
	Init_DAC_HBLED();
	Init_ADC_HBLED();
	
	// Configure driver for buck converter
	// Set up PTE31 to use for SMPS with TPM0 Ch 4
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[31]  &= PORT_PCR_MUX(7);
	PORTE->PCR[31]  |= PORT_PCR_MUX(3);
	PWM_Init(TPM0, PWM_HBLED_CHANNEL, PWM_PERIOD, g_duty_cycle, 0, 0);
	
}

// Handler functions (callbacks)
// Default handlers
void Control_OnOff_Handler (UI_FIELD_T * fld, int v) {
	if (fld->Val != NULL) {
		if (v > 0) {
			*fld->Val = 1;
		} else {
			*fld->Val = 0;
		}
	}
}

void Control_IntNonNegative_Handler (UI_FIELD_T * fld, int v) {
	int n;
	if (fld->Val != NULL) {
		n = *fld->Val + v/16;
		if (n < 0) {
			n = 0;
		}
		*fld->Val = n;
	}
}

void Control_DutyCycle_Handler(UI_FIELD_T * fld, int v) {
	int dc;
	if (fld->Val != NULL) {
		dc = g_duty_cycle + v/16;
		if (dc < 0)
			dc = 0;
		else if (dc > LIM_DUTY_CYCLE)
			dc = LIM_DUTY_CYCLE;
		*(fld->Val) = dc;
		PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, g_duty_cycle);
	}
}

void Control_Mode(UI_FIELD_T * fld, int v)
{
if (fld->Val != NULL) {
	if(v<-100)
	{
		
		*(fld->Val) = OpenLoop;
		
	}
	else if(v>=-100 &&v<-60)
	{
		
		*(fld->Val) = BangBang;
	}
	else if(v>=-60 &&v<0)
	{
		
		*(fld->Val) = Incremental;
	}
	else if(v>=0 &&v<60)
	{
		
		*(fld->Val) = Proportional;
	}
	else if(v>=60 &&v<100)
	{
		
		*(fld->Val) = PID_FX;
	}
	
}	
}
