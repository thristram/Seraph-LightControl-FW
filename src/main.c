/* MAIN.C file
* 
* Copyright (c) 2002-2005 STMicroelectronics
*/
#include "stm8s_conf.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "stm8s.h"
#include "stm8s_flash.h"
#include "stm8s_gpio.h"
#include "stm8s_adc1.h"
#include "stm8s_tim1.h"
#include "stm8s_tim2.h"
#include "stm8s_tim4.h"
#include "stm8s_i2c.h"
#include "stm8s_clk.h"
#include "stm8s_iwdg.h"
#include "stm8s_exti.h"
#include "stm8s_itc.h"
#include "stm8s_uart1.h"

Sys_TypeDef sys = {0};
const uint16_t crc16_table[256] =
{
    0x0000L, 0x1021L, 0x2042L, 0x3063L, 0x4084L, 0x50A5L, 0x60C6L, 0x70E7L,
    0x8108L, 0x9129L, 0xA14AL, 0xB16BL, 0xC18CL, 0xD1ADL, 0xE1CEL, 0xF1EFL,
    0x1231L, 0x0210L, 0x3273L, 0x2252L, 0x52B5L, 0x4294L, 0x72F7L, 0x62D6L,
    0x9339L, 0x8318L, 0xB37BL, 0xA35AL, 0xD3BDL, 0xC39CL, 0xF3FFL, 0xE3DEL,
    0x2462L, 0x3443L, 0x0420L, 0x1401L, 0x64E6L, 0x74C7L, 0x44A4L, 0x5485L,
    0xA56AL, 0xB54BL, 0x8528L, 0x9509L, 0xE5EEL, 0xF5CFL, 0xC5ACL, 0xD58DL,
    0x3653L, 0x2672L, 0x1611L, 0x0630L, 0x76D7L, 0x66F6L, 0x5695L, 0x46B4L,
    0xB75BL, 0xA77AL, 0x9719L, 0x8738L, 0xF7DFL, 0xE7FEL, 0xD79DL, 0xC7BCL,
    0x48C4L, 0x58E5L, 0x6886L, 0x78A7L, 0x0840L, 0x1861L, 0x2802L, 0x3823L,
    0xC9CCL, 0xD9EDL, 0xE98EL, 0xF9AFL, 0x8948L, 0x9969L, 0xA90AL, 0xB92BL,
    0x5AF5L, 0x4AD4L, 0x7AB7L, 0x6A96L, 0x1A71L, 0x0A50L, 0x3A33L, 0x2A12L,
    0xDBFDL, 0xCBDCL, 0xFBBFL, 0xEB9EL, 0x9B79L, 0x8B58L, 0xBB3BL, 0xAB1AL,
    0x6CA6L, 0x7C87L, 0x4CE4L, 0x5CC5L, 0x2C22L, 0x3C03L, 0x0C60L, 0x1C41L,
    0xEDAEL, 0xFD8FL, 0xCDECL, 0xDDCDL, 0xAD2AL, 0xBD0BL, 0x8D68L, 0x9D49L,
    0x7E97L, 0x6EB6L, 0x5ED5L, 0x4EF4L, 0x3E13L, 0x2E32L, 0x1E51L, 0x0E70L,
    0xFF9FL, 0xEFBEL, 0xDFDDL, 0xCFFCL, 0xBF1BL, 0xAF3AL, 0x9F59L, 0x8F78L,
    0x9188L, 0x81A9L, 0xB1CAL, 0xA1EBL, 0xD10CL, 0xC12DL, 0xF14EL, 0xE16FL,
    0x1080L, 0x00A1L, 0x30C2L, 0x20E3L, 0x5004L, 0x4025L, 0x7046L, 0x6067L,
    0x83B9L, 0x9398L, 0xA3FBL, 0xB3DAL, 0xC33DL, 0xD31CL, 0xE37FL, 0xF35EL,
    0x02B1L, 0x1290L, 0x22F3L, 0x32D2L, 0x4235L, 0x5214L, 0x6277L, 0x7256L,
    0xB5EAL, 0xA5CBL, 0x95A8L, 0x8589L, 0xF56EL, 0xE54FL, 0xD52CL, 0xC50DL,
    0x34E2L, 0x24C3L, 0x14A0L, 0x0481L, 0x7466L, 0x6447L, 0x5424L, 0x4405L,
    0xA7DBL, 0xB7FAL, 0x8799L, 0x97B8L, 0xE75FL, 0xF77EL, 0xC71DL, 0xD73CL,
    0x26D3L, 0x36F2L, 0x0691L, 0x16B0L, 0x6657L, 0x7676L, 0x4615L, 0x5634L,
    0xD94CL, 0xC96DL, 0xF90EL, 0xE92FL, 0x99C8L, 0x89E9L, 0xB98AL, 0xA9ABL,
    0x5844L, 0x4865L, 0x7806L, 0x6827L, 0x18C0L, 0x08E1L, 0x3882L, 0x28A3L,
    0xCB7DL, 0xDB5CL, 0xEB3FL, 0xFB1EL, 0x8BF9L, 0x9BD8L, 0xABBBL, 0xBB9AL,
    0x4A75L, 0x5A54L, 0x6A37L, 0x7A16L, 0x0AF1L, 0x1AD0L, 0x2AB3L, 0x3A92L,
    0xFD2EL, 0xED0FL, 0xDD6CL, 0xCD4DL, 0xBDAAL, 0xAD8BL, 0x9DE8L, 0x8DC9L,
    0x7C26L, 0x6C07L, 0x5C64L, 0x4C45L, 0x3CA2L, 0x2C83L, 0x1CE0L, 0x0CC1L,
    0xEF1FL, 0xFF3EL, 0xCF5DL, 0xDF7CL, 0xAF9BL, 0xBFBAL, 0x8FD9L, 0x9FF8L,
    0x6E17L, 0x7E36L, 0x4E55L, 0x5E74L, 0x2E93L, 0x3EB2L, 0x0ED1L, 0x1EF0L,
};




#ifdef USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {
    }
}
#endif


void main(void)
{
    uint8_t buf[COMM_MAX_LEN];
    
    GPIO_Config();
    Sys_Init();
    ExtInterrupt_Config();
    TIMER4_Init();
    UART_Init();
     
 	/* Enable general interrupts */ 
	 enableInterrupts();
	 SendOnePkg(RESET_DONE);
	 /*waiting for the main power input and calculate the main AC Hz*/
	 while(!sys.gotHzFlag)
	 {
		 if (sys.checkAcCnt == 0)
		 {
			 /*no main AC input or fuse has been fused and respone a message to
			 master*/
			 SendOnePkg(AC_ERR);			 
			 // while (1);
			 /*remonitor the main AC input and calculate the Hz*/
			sys.gotHzFlag = FALSE;    
			sys.reqCalHzFlag = FALSE;
			sys.calHzIntCnt = GET_AC_FRE_CNT;
			sys.hzCnt = 0;
			sys.checkAcCnt = CHECK_AC_INPUT_CNT;
			break;
		 }
	 }

	 /*waiting for the main power input and calculate tha main AC Hz*/
	 while(!sys.gotHzFlag)
	 {
		 if (sys.checkAcCnt == 0)
		 {		
			 /*remonitor the main AC input and calculate the Hz*/
			sys.gotHzFlag = FALSE;    
			sys.reqCalHzFlag = FALSE;
			sys.calHzIntCnt = GET_AC_FRE_CNT;
			sys.hzCnt = 0;
			sys.checkAcCnt = CHECK_AC_INPUT_CNT;
		 }
	 }
	 sys.acOkFlag = TRUE;
	 TIMER2_Init();   
   
    
#ifdef ENABLE_WDT    
    Wdt_Init();
#endif   
    
#ifdef ENABLE_WDT   
    FeedDog();
#endif   
	/*notification maser that system reset is ready and start working*/
	SendOnePkg(AC_OK);
	while (1)
	{
		/*monitor main AC*/
		if (sys.checkAcCnt == 0 && sys.acErrFlag == FALSE)
		{
			 /*notification master when main AC lost or fuse has been fused*/
			 SendOnePkg(AC_ERR);
			 sys.acErrFlag = TRUE;
			 sys.acOkFlag = FALSE;
		}

		if (sys.acErrFlag == FALSE && sys.acOkFlag == FALSE)
		{
			/*notification maser that AC is restored and start working gain*/
			SendOnePkg(AC_OK);
			sys.acOkFlag = TRUE;
			sys.cnt1s = CNT_1S;
		}
		
		/*monitor uart command*/
		if(sys.rxDone)
		{
			//sys.cnt1s = CNT_1S;
			/*uart has received a package and save it to buffer*/
			memcpy(buf, (uint8_t *)sys.rxBuf, COMM_MAX_LEN);
			sys.rxDone = FALSE;
			sys.rxIdx = 0;	
			if  (CrcCal(buf, COMM_MAX_LEN) == 0)
			{				
				
				if (buf[1] == CMD_BRIGHTNESS)
				{
					if (buf[2] == 1 && buf[3] <= MAX_BRIGHTNESS)
					{
						/*master controls the first light*/
						sys.light1.briVal = buf[3];
					}
					else if (buf[2] == 2 && buf[3] <= MAX_BRIGHTNESS)
					{
						/*maser control the second light*/
						sys.light2.briVal = buf[3];
					}
					else
					{
						/*value overflows*/
						SendOnePkg(VAL_OVERFLOW);
					}
				}
				else
				{
					SendOnePkg(UNKNOWN_CMD);
				}
				/*response a command to maser that device is received a 
				successful command*/
				SendOnePkg(RSP_RX_CMD_OK);
			}
			else
			{
				/*response crc error to master*/
				SendOnePkg(CRC_ERR);
			}
		}

		if (sys.acOkFlag && sys.cnt1s == 0)
		{
			/*it indicates the system is working via UART sends a 
			IS_RUNNING to master every 1s */
			SendOnePkg(IS_RUNNING);
			sys.cnt1s = CNT_1S;
		}
#ifdef ENABLE_WDT   
		FeedDog();
#endif 
	}
  
}

static void Sys_Init(void)
{
    /*reset the all periphals clock to their default value */
    CLK_DeInit();
    /*enable the external high speed ocillator*/
    CLK_HSECmd(ENABLE);
    /*check the external ocillator is ready*/
    while (!CLK_GetFlagStatus(CLK_FLAG_HSERDY));
    /*HES selected as master clock source*/
    CLK_ClockSwitchCmd(ENABLE);						  
	CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO,CLK_SOURCE_HSE,DISABLE,CLK_CURRENTCLOCKSTATE_ENABLE);
    /*set CPU clock prescaler as 1, so fCPU as same as fMASTER that is 11.0592M*/
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);	
    
	/*output the master clock via pin14*/
	//CLK_CCOConfig(CLK_OUTPUT_MASTER);

    /*disable all peripherals and enable the clock of using peripherral*/
    CLK->PCKENR1 = 0;
    CLK->PCKENR2 = 0;
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, ENABLE);

	
    sys.gotHzFlag = FALSE;    
    sys.reqCalHzFlag = FALSE;
    sys.light1.briVal = DEFAULT_BRIGHTNESS;
    sys.light2.briVal = DEFAULT_BRIGHTNESS;    
    sys.calHzIntCnt = GET_AC_FRE_CNT;
    sys.hzCnt = 0;
    sys.checkAcCnt = CHECK_AC_INPUT_CNT;
}

static void GPIO_Config(void)
{
    /* Initialize I/Os in Output Mode and disable lights*/
    GPIO_Init(L1_EN_PIN_PORT, (GPIO_Pin_TypeDef)L1_EN_PIN, GPIO_MODE_OUT_PP_HIGH_SLOW);
    GPIO_Init(L2_EN_PIN_PORT, (GPIO_Pin_TypeDef)L2_EN_PIN, GPIO_MODE_OUT_PP_HIGH_SLOW);   
    L1_EN_OFF;
    L2_EN_OFF;
    GPIO_Init(ZD_PIN_PORT, (GPIO_Pin_TypeDef)ZD_PIN, GPIO_MODE_IN_FL_IT);  
    //GPIO_Init(WAKEUP_PIN_PORT, (GPIO_Pin_TypeDef)WAKEUP_PIN, GPIO_MODE_OUT_PP_HIGH_SLOW);   
}

static void ExtInterrupt_Config(void)
{
	EXTI_DeInit();
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);
	//ITC_SetSoftwarePriority(ITC_IRQ_PORTD, ITC_PRIORITYLEVEL_2);
}

#ifdef ENABLE_WDT  
/*watchdog count period is 250ms*/
static void Wdt_Init(void)
{
    IWDG_Enable();
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_128);
    IWDG_SetReload(127);
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
}

static void FeedDog(void)
{
    IWDG_ReloadCounter();
}
#endif

static void TIMER4_Init(void)
{    
    TIM4_TimeBaseInit(TIM4_PRESCALER_16, TIMER4_INT_TIME);
    TIM4_ClearFlag(TIM4_FLAG_UPDATE);
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);	
    TIM4_Cmd(ENABLE);
}

static void TIMER2_Init(void)
{    
	TIM2_TimeBaseInit(TIM2_PRESCALER_16, sys.hz50Flag ? TIMER2_INT_TIME_50HZ: TIMER2_INT_TIME_60HZ);
   	TIM2_ClearFlag(TIM2_FLAG_UPDATE);
   	TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);	   
}


static void UART_Init(void)
{
   	UART1_Init(COMM_BAUD_RATE, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
   	UART1_SYNCMODE_CLOCK_DISABLE,UART1_MODE_TXRX_ENABLE);
    UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
    UART1_Cmd(ENABLE);
}


static void SendOnePkg(uint8_t cmd)
{
	UART1_SendData8(cmd);
	while(!UART1_GetFlagStatus(UART1_FLAG_TC));
}

/*******************************************************************************
**
** Function         crcCalc
**
** Description      Compute a checksum on a buffer of data.
**
** Returns          2-byte checksum.
**
*******************************************************************************/
uint16_t CrcCal(uint8_t *buffer, uint16_t bufferLen)
{
    uint16_t i;
    uint16_t crc16_value = 0;

    for(i=0; i<bufferLen; ++i)
        crc16_value = ( crc16_value << 8 ) ^ (uint16_t)crc16_table[( crc16_value >> 8 ) ^ *buffer++];

    return crc16_value;
}




