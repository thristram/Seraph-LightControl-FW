
#ifndef __MAIN_H
#define __MAIN_H

#include "stm8s_gpio.h"
#include "stm8s_adc1.h"

/*light1 enable gpio definition*/
#define L1_EN_PIN_PORT          GPIOC
#define L1_EN_PIN               GPIO_PIN_4
#define L1_EN_ON 				GPIO_WriteLow(L1_EN_PIN_PORT, L1_EN_PIN);\
                                sys.light1.triacTriggeTime = TRIAC_TRIGGE_TIME
#define L1_EN_OFF 				GPIO_WriteHigh(L1_EN_PIN_PORT, L1_EN_PIN)
#define L1_EN_REVERSE			GPIO_WriteReverse(L1_EN_PIN_PORT, L1_EN_PIN)
#define L1_EN_STATUS      	    (GPIO_ReadOutputData(L1_EN_PIN_PORT) & L1_EN_PIN)

/*light2 enable gpio definition*/
#define L2_EN_PIN_PORT          GPIOC
#define L2_EN_PIN               GPIO_PIN_3
#define L2_EN_ON 				GPIO_WriteLow(L2_EN_PIN_PORT, L2_EN_PIN);\
                                sys.light2.triacTriggeTime = TRIAC_TRIGGE_TIME
#define L2_EN_OFF 				GPIO_WriteHigh(L2_EN_PIN_PORT, L2_EN_PIN)
#define L2_EN_REVERSE			GPIO_WriteReverse(L2_EN_PIN_PORT, L2_EN_PIN)
#define L2_EN_STATUS      	    (GPIO_ReadOutputData(L2_EN_PIN_PORT) & L2_EN_PIN)


/*zero detect input gpio definition*/
#define ZD_PIN_PORT          	GPIOD
#define ZD_PIN               	GPIO_PIN_3
#define ZD_STATUS      	    	(GPIO_ReadOutputData(ZD_PIN_PORT) & ZD_PIN)


/*wakeup input gpio definition*/
#define WAKEUP_PIN_PORT          	GPIOD
#define WAKEUP_PIN               	GPIO_PIN_4
#define WAKEUP_PIN_REVERSE			GPIO_WriteReverse(GPIOD, GPIO_PIN_4)
#define WAKEUP_STATUS      	    	(GPIO_ReadOutputData(WAKEUP_PIN_PORT) & WAKEUP_PIN)

/*UART1 gpio definition*/
#define TXD_PIN_PORT          		GPIOD
#define TXD_PIN               		GPIO_PIN_5
#define RXD_PIN_PORT          		GPIOD
#define RXD_PIN               		GPIO_PIN_6


/*the maximume brightness definition*/
#define MAX_BRIGHTNESS				(250) 

/*the light default brightness definition*/
#define DEFAULT_BRIGHTNESS			(40)


/*timer interrupt time is 40us@50Hz, 33us@60Hz, min is approximate 1us with dividing 16*/
#define TIMER2_INT_TIME_50HZ				(40) 
#define TIMER2_INT_TIME_60HZ				(33) 


/*timer interrupt time is approximate 50us, min is approximate 1us with dividing 16*/
#define HZ_COUNT					(180) //if hzCnt >=180 is 50Hz.
#define TIMER4_INT_TIME				(50) 

/*total detectiong 10 times zero crossing signal for getting the AC freqency*/
#define GET_AC_FRE_CNT				(10)

/*the count of checking 100ms if AC input*/
#define CHECK_AC_INPUT_CNT			(2000)  //2000 = 100ms/50us

/*traic trigge time definition as 2ms*/
#define TRIAC_TRIGGE_TIME			(40)  //40 = 2ms/50us

/*1s count definition*/
#define CNT_1S						(20000)  //20000 = 1s/50us


/*UART baud rate definition*/
#define	COMM_BAUD_RATE  			(115200)

/*communication Head definition*/
#define COMM_HEAD					(0x55)

/*communication package maximume length definition*/
#define COMM_MAX_LEN				(6)  // 1 length + 1 cmd + data + 2bytes crc

enum CMD
{
	CMD_BRIGHTNESS			= 1,	
	
};


enum RSP_CMD
{
	RESET_DONE,
	AC_OK,	
	AC_ERR,
	RSP_RX_CMD_OK,
	UNKNOWN_CMD, 
	IS_RUNNING,
	CRC_ERR,	
	VAL_OVERFLOW,
};

typedef struct
{  
    volatile uint8_t briCnt;    
    volatile uint8_t triacTriggeTime;    
	uint8_t briVal;	   
	volatile bool onFlag;
	
}Bri_TypeDef;


typedef struct
{
    Bri_TypeDef light1;
    Bri_TypeDef light2;
    volatile bool reqCalHzFlag;
    volatile bool gotHzFlag;
    volatile bool hz50Flag;
    volatile uint8_t calHzIntCnt;
    volatile uint16_t hzCnt;
    volatile uint16_t checkAcCnt;
    volatile bool acErrFlag;
    bool acOkFlag;
	volatile bool startCtrlLightFlag;
	volatile uint8_t rxIdx;
	volatile uint8_t rxBuf[COMM_MAX_LEN];
	volatile bool rxDone;
	volatile uint16_t cnt1s;
}Sys_TypeDef;


extern Sys_TypeDef sys;
/*functions declaration*/
void Sys_Init(void);
void GPIO_Config(void);
void ExtInterrupt_Config(void);
#ifdef ENABLE_WDT  
/*watchdog count period is 250ms*/
void Wdt_Init(void);
void FeedDog(void);
#endif
void TIMER2_Init(void);
void TIMER4_Init(void);
void UART_Init(void);
uint16_t CrcCal(uint8_t *buffer, uint16_t bufferLen);
void SendOnePkg(uint8_t cmd);
#endif /* __MAIN_H*/



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
