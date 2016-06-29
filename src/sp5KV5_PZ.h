/*
 * sp5K.h
 *
 * Created on: 27/12/2013
 *      Author: root
 */

#ifndef SP5K_H_
#define SP5K_H_

#include <avr/io.h>			/* include I/O definitions (port names, pin names, etc) */
//#include <avr/signal.h>		/* include "signal" names (interrupt names) */
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <string.h>
#include <compat/deprecated.h>
#include <util/twi.h>
#include <util/delay.h>
#include <ctype.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

#include "sp5Klibs/avrlibdefs.h"
#include "sp5Klibs/avrlibtypes.h"
#include "sp5Klibs/global.h"			// include our global settings
#include "file_sp5K.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"

#include "cmdline.h"
#include "sp5K_i2c.h"
#include "sp5K_uart.h"

#include "mcp_sp5K.h"
#include "ads7828_sp5K.h"
#include "rtc_sp5K.h"

// DEFINICION DEL TIPO DE SISTEMA
//----------------------------------------------------------------------------
#define SP5K_REV "5.0.0"
#define SP5K_DATE "@ 20160629"

#define SP5K_MODELO "sp5KV5_PZ HW:avr1284P R5.0"
#define SP5K_VERSION "FW:FRTOS8"

#define CHAR64		64
#define CHAR128	 	128
#define CHAR256	 	256

//----------------------------------------------------------------------------
// TASKS
/* Stack de las tareas */
#define tkCmd_STACK_SIZE		512
#define tkControl_STACK_SIZE	512
#define tkGprsTx_STACK_SIZE		512
#define tkGprsRx_STACK_SIZE		512
#define tkRange_STACK_SIZE		512

/* Prioridades de las tareas */
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkControl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkGprsTx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprsRx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkRange_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )

/* Prototipos de tareas */
void tkCmd(void * pvParameters);
void tkControl(void * pvParameters);
void tkControlInit(void);
void tkGprsTx(void * pvParameters);
void tkGprsRx(void * pvParameters);
void tkGprsInit(void);
void tkRange(void * pvParameters);
void tkRangeInit( void );

TaskHandle_t xHandle_tkCmd, xHandle_tkControl, xHandle_tkGprsTx, xHandle_tkGprsRx, xHandle_tkRange ;

s08 startTask;
typedef struct {
	u08 resetCause;
	u08 mcusr;
} wdgStatus_t;

wdgStatus_t wdgStatus;

// Mensajes entre tareas
#define TK_PARAM_RELOAD			0x01	// to tkRange/tkGPRS: reload
#define TKR_READ_FRAME			0x02	// to tRange: (mode service) read a frame
//------------------------------------------------------------------------------------

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )
typedef enum { WK_IDLE = 0, WK_NORMAL, WK_SERVICE, WK_MONITOR_FRAME, WK_MONITOR_SQE  } t_wrkMode;
typedef enum { OFF = 0, ON = 1 } t_onOff;
typedef enum { D_NONE = 0, D_BASIC = 1, D_DATA = 2, D_GPRS = 4, D_MEM = 8, D_DIGITAL = 16,  D_DEBUG = 32 } t_debug;
typedef enum { MDM_PRENDIDO = 0, MDM_APAGADO } t_modemStatus;
typedef enum { modoPWRSAVE_OFF = 0, modoPWRSAVE_ON } t_pwrSave;
typedef enum { RUN = 0, STOP } t_rangeAction;
typedef enum { GOOD = 0, BAD } t_rangeQuality;
//------------------------------------------------------------------------------------

#define NRO_CHANNELS		3

#define DLGID_LENGTH		12
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define IP_LENGTH			24
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

typedef struct {
	// size = 7+3*4+4 = 23 bytes
	RtcTimeType_t rtc;				// 7
	u16 inputs[NRO_CHANNELS];		// 6
} frameData_t;	//  23 bytes


typedef struct {
	// Variables de trabajo.

	u08 dummyBytes;
	u08 initByte;

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char serverPort[PORT_LENGTH];
	char serverAddress[IP_LENGTH];
	char serverIp[IP_LENGTH];
	char dlgIp[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	u08 csq;
	u08 dbm;

	u16 timerPoll;

	t_wrkMode wrkMode;

	u08 debugLevel;		// Indica que funciones debugear.
	u08 gsmBand;

	// Nombre de los canales
	char chName[NRO_CHANNELS][PARAMNAME_LENGTH];

	s08 roaming;

} systemVarsType;	// 315 bytes

systemVarsType systemVars,tmpSV;

#define EEADDR_SV 32		// Direccion inicio de la EE de escritura del systemVars.

//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL.
//------------------------------------------------------------------------------------
void u_panic( u08 panicCode );
s08 u_configAnalogCh( u08 channel, char *chName );
s08 u_configTimerPoll(char *s_tPoll);
s08 u_saveSystemParams(void);
s08 u_loadSystemParams(void);
void u_loadDefaults(void);
char *u_now(void);
void u_reset(void);
void u_rangeSignal(t_rangeAction action);

char nowStr[32];

s16 u_readTimeToNextPoll(void);
void u_readDataFrame (frameData_t *dFrame);

s08 u_modemPwrStatus(void);

s08 u_wrRtc(char *s);

void u_debugPrint(u08 debugCode, char *msg, u16 size);
void pvMCP_init_MCP1(u08 modo);

//------------------------------------------------------------------------------------
// LED
#define LED_KA_PORT		PORTD
#define LED_KA_PIN		PIND
#define LED_KA_BIT		6
#define LED_KA_DDR		DDRD

#define LED_MODEM_PORT		PORTC
#define LED_MODEM_PIN		PINC
#define LED_MODEM_BIT		3
#define LED_MODEM_DDR		DDRC

//------------------------------------------------------------------------------------
// PANIC CODES
#define P_OUT_TIMERSTART	1
#define P_OUT_TIMERCREATE	2
#define P_RANGE_TIMERSTART	3
#define P_GPRS_TIMERSTART	4
#define P_CTL_TIMERCREATE	5
#define P_CTL_TIMERSTART	6
#define P_RANGE_TIMERCREATE	7

//------------------------------------------------------------------------------------
// WATCHDOG
u08 systemWdg;

#define WDG_CTL			0x01
#define WDG_CMD			0x02
#define WDG_GPRSTX		0x04
#define WDG_GPRSRX		0x08
#define WDG_RANGE		0x10

void u_clearWdg( u08 wdgId );

//------------------------------------------------------------------------------------
// DCD
// Como el MCP23018 a veces no detecta el nivel del modem, cableamos
// el DCD a PB3
// Pin de control de fuente de la terminal ( PB3)
#define DCD_PORT		PORTB
#define DCD_PIN			PINB
#define DCD_BIT			3
#define DCD_DDR			DDRB

//------------------------------------------------------------------------------------
// RANGEMETER
// Control
#define RM_RUN_PORT			PORTB
#define RM_RUN_PIN			PINB
#define RM_RUN_BIT			1
#define RM_RUN_DDR			DDRB

// Pulse Width
#define RM_PW_PORT			PORTB
#define RM_PW_PIN			PINB
#define RM_PW_BIT			2
#define RM_PW_DDR			DDRB

// Digital DIN0
#define RM_DIN0_PORT		PORTD
#define RM_DIN0_PIN			PIND
#define RM_DIN0_BIT			7
#define RM_DIN0_DDR			DDRD

// Digital DIN1
#define RM_DIN1_PORT		PORTC
#define RM_DIN1_PIN			PINC
#define RM_DIN1_BIT			4
#define RM_DIN1_DDR			DDRD

//------------------------------------------------------------------------------------
char debug_printfBuff[CHAR128];

#endif /* SP5K_H_ */
