#ifndef THREADS_H
#define THREADS_H

#include <cmsis_os2.h>
#include "LCD_driver.h"
#include <MKL25Z4.H>

#define THREAD_READ_TS_PERIOD_MS (50)  // 1 tick/ms
#define THREAD_UPDATE_SCREEN_PERIOD_MS (100)
#define THREAD_BUS_PERIOD_MS (1)

#define USE_LCD_MUTEX (1)

// Custom stack sizes for larger threads

void Init_Debug_Signals(void);

// Events for sound generation and control
#define EV_PLAYSOUND (1) 
#define EV_SOUND_ON (2)
#define EV_SOUND_OFF (4)

// Define message data structure for queue
typedef struct {
	uint32_t cnum;
	uint32_t *result_id;
} MSG_REQ;

typedef struct {
	uint32_t cnum;
	uint32_t adc_conv;
} MSG_RES;


// Define os Message Queue ID
extern osMessageQueueId_t ADC_REQ_ID;
extern osMessageQueueId_t ADC_RES_ID;

// result to check status
//osStatus_t result, result_TS;

#define EV_REFILL_SOUND_BUFFER  (1)

void Create_OS_Objects(void);
 
extern osThreadId_t t_Read_TS, t_Read_Accelerometer, t_Sound_Manager, t_US, t_Refill_Sound_Buffer;
extern osMutexId_t LCD_mutex;

#endif // THREADS_H

