/*
 * sp5KV5_PZ_tkRange.c
 *
 *  Created on: 22 de jun. de 2016
 *      Author: pablo
 */


#include "sp5KV5_PZ.h"

static char range_printfBuff[CHAR128];

TimerHandle_t pollingTimer;

// Estados
typedef enum {	rgST_R00 = 0,
				rgST_R01,
				rgST_R02,

} t_rangeState;

// Eventos
typedef enum {
	rg_ev_RELOADCONFIG = 0,		// EV_MSGreload
	rg_ev_START2POLL,			// EV_f_start2poll
	rg_ev_POLL_NOW,
	rg_ev_DIN_CHANGE,
	rg_ev_POLL,
} t_rangeEventos;

#define rgEVENT_COUNT		5

static s08 rgEventos[rgEVENT_COUNT];

// transiciones
static int rgTR_00(void);
static int rgTR_01(void);
static int rgTR_02(void);
static int rgTR_03(void);
static int rgTR_04(void);
static int rgTR_05(void);
static int rgTR_06(void);

static struct {
	s08 msgReload;			// flags de los mensajes recibidos.
	s08 start2poll;			// flag que habilita a polear.
	s08 msgPollNow;			// mensaje de POLL_FRAME
	s08 saveFrameInBD;		//
	s08 dInChange;
	s08 poll;
} RANGE_flags;

static struct {
	u16 secs4poll;
	u08 totalPollCounts;
	u08 goodPollCounts;
} RANGE_counters;

#define MAX_TOTAL_POLL_COUNTS	10
#define MAX_GOOD_POLL_COUNTS	3

static u08 tkRANGE_state = rgST_R00;				// Estado
static frameData_t Aframe;

struct {
	u16 distancia[MAX_GOOD_POLL_COUNTS];
	s08 quality[MAX_GOOD_POLL_COUNTS];
} rangeDataStruct;

static u08 din0,din1;

// Funciones generales
void  pv_RANGEtimerCallback( TimerHandle_t pxTimer );
static void pv_RANGEgetNextEvent(void);
static void pv_RANGEfsm(void);
static void pv_RANGEprintExitMsg(u08 code);
static s08 pv_awaitLineHIGH(void);
static s08 pv_awaitLineLOW(void);
static void pv_rangeMeter_Fire(u16 *distancia, s08 *status );
static s08  pv_checkDIN4Change(void);
static void pv_pollInit(void);

#define TIMER1_START ( TCCR1B |= ( 1 << CS11 ))	// Prescaler x8: cuento de a uS
#define TIMER1_STOP ( TCCR1B &= ~( 1 << CS11 ))

//--------------------------------------------------------------------------------------
 void tkRange(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("starting tkRange..\r\n\0"));
	FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );

	tkRANGE_state = rgST_R00;		// Estado inicial.
	RANGE_flags.msgReload = FALSE;		// No tengo ningun mensaje de reload pendiente.
	RANGE_flags.msgPollNow = FALSE;

	// Inicializo las entradas digitales.
	din0 = ( ( RM_DIN0_PIN & _BV(RM_DIN0_BIT) ) >> RM_DIN0_BIT );
	din1 = ( ( RM_DIN1_PIN & _BV(RM_DIN1_BIT) ) >> RM_DIN1_BIT );

	// Arranco el timer de poleo.
	// Interrumpe c/1s.
	if ( xTimerStart( pollingTimer, 0 ) != pdPASS )
		u_panic(P_RANGE_TIMERSTART);

	//
	for( ;; )
	{

		u_clearWdg(WDG_RANGE);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TK_PARAM_RELOAD ) != 0 ) {
				// Mensaje de reload configuration.
				RANGE_flags.msgReload = TRUE;
			}

			if ( ( ulNotifiedValue & TKR_READ_FRAME ) != 0 ) {
				// Mensaje de polear un frame ( estando en modo servicio )
				if ( systemVars.wrkMode == WK_SERVICE )
					RANGE_flags.msgPollNow = TRUE;
			}
		}

		// Analizo los eventos.
		pv_RANGEgetNextEvent();
		// Corro la maquina de estados.
		pv_RANGEfsm();
	}

}
/*------------------------------------------------------------------------------------*/
void tkRangeInit(void)
{
	// Esta funcion se utiliza  antes de arrancar el FRTOS de modo que cree
	// el timer que necesitamos en este modulo
	// Expira c/1sec

	pollingTimer = xTimerCreate (  "POLL_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 1000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdTRUE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_RANGEtimerCallback
	                   );

	if ( pollingTimer == NULL )
		u_panic(P_RANGE_TIMERCREATE);
}
//------------------------------------------------------------------------------------
void pv_RANGEtimerCallback( TimerHandle_t pxTimer )
{
	// El timer esta en reload c/1 sec, aqui contamos los secs para
	// completar poleo y lo indico prendiendo la flag correspondiente.
	// En consigna continua poleo c/60s.
	// En modo service poleo c/15s
	// En otro modo, poleo c/systemVars.timerPoll

	// Ajusto los timers.
	if ( RANGE_counters.secs4poll > 0 ) {
		--RANGE_counters.secs4poll;
	}

	// Control del poleo
	if ( RANGE_counters.secs4poll == 0 ) {
		RANGE_flags.start2poll = TRUE;

		switch(systemVars.wrkMode) {
		case WK_NORMAL:
			RANGE_counters.secs4poll = systemVars.timerPoll;
			break;
		case WK_MONITOR_FRAME:
			RANGE_counters.secs4poll = 15;
			break;
		case WK_SERVICE:
			RANGE_counters.secs4poll = 0xFFFF;
			break;
		}
	}

	// Como se ejecuta 1 vez por sec, aqui chequeo si las entradas digitales cambian
	pv_checkDIN4Change();

}
//--------------------------------------------------------------------------------------
static void pv_RANGEgetNextEvent(void)
{
// Evaluo todas las condiciones que generan los eventos que disparan las transiciones.
// Tenemos un array de eventos y todos se evaluan.

u08 i;

	// Inicializo la lista de eventos.
	for ( i=0; i < rgEVENT_COUNT; i++ ) {
		rgEventos[i] = FALSE;
	}

	// Evaluo los eventos
	if ( RANGE_flags.msgReload == TRUE ) { rgEventos[rg_ev_RELOADCONFIG] = TRUE; }
	if ( RANGE_flags.start2poll == TRUE ) { rgEventos[rg_ev_START2POLL] = TRUE; }
	if ( RANGE_flags.msgPollNow == TRUE ) { rgEventos[rg_ev_POLL_NOW] = TRUE; }
	if ( RANGE_flags.dInChange == TRUE ) { rgEventos[rg_ev_DIN_CHANGE] = TRUE; }
	if ( RANGE_flags.poll == TRUE ) { rgEventos[rg_ev_POLL] = TRUE; }

}
/*------------------------------------------------------------------------------------*/
static void pv_RANGEfsm(void)
{
	// El manejar la FSM con un switch por estado y no por transicion me permite
	// priorizar las transiciones.
	// Luego de c/transicion debe venir un break así solo evaluo de a 1 transicion por loop.
	//

	switch ( tkRANGE_state ) {
	case rgST_R00:
		tkRANGE_state = rgTR_00();
		break;
	case rgST_R01:
		if (  rgEventos[rg_ev_RELOADCONFIG] ) {
			tkRANGE_state = rgTR_01();
		} else if ( rgEventos[rg_ev_POLL_NOW] ){
			tkRANGE_state = rgTR_02();
		} else if (rgEventos[rg_ev_START2POLL] ) {
			tkRANGE_state = rgTR_03();
		} else if (rgEventos[rg_ev_DIN_CHANGE] ) {
			tkRANGE_state = rgTR_04();
		}
		break;
	case rgST_R02:
		if (  rgEventos[rg_ev_POLL] ) {
			tkRANGE_state = rgTR_05();
		} else {
			tkRANGE_state = rgTR_06();
		}
		break;
	default:
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("tkRange::ERROR state NOT DEFINED..\r\n\0"));
		FreeRTOS_write( &pdUART1, range_printfBuff,sizeof(range_printfBuff) );
		tkRANGE_state  = rgST_R00;
		break;

	}
}
/*------------------------------------------------------------------------------------*/
static int rgTR_00(void)
{
	// Inicializo el sistema aqui

	RANGE_flags.msgReload = FALSE;
	RANGE_flags.start2poll = FALSE;
	RANGE_flags.saveFrameInBD = FALSE;
	RANGE_counters.secs4poll = 15;
	RANGE_flags.poll = FALSE;

	pv_RANGEprintExitMsg(0);
	return(rgST_R01);
}
/*------------------------------------------------------------------------------------*/
static int rgTR_01(void)
{
	// MSG de autoreload configuration
	RANGE_flags.msgReload = FALSE;
	RANGE_counters.secs4poll = 15;
	RANGE_flags.poll = FALSE;

	pv_RANGEprintExitMsg(1);
	return(rgST_R01);
}
//------------------------------------------------------------------------------------
static int rgTR_02(void)
{

	// Tengo un mensaje que debo polear( x modo cmd )
	RANGE_flags.msgPollNow = FALSE;
	RANGE_flags.saveFrameInBD = FALSE;

	pv_pollInit();

	pv_RANGEprintExitMsg(2);
	return(rgST_R02);
}
//------------------------------------------------------------------------------------
static int rgTR_03(void)
{

	// Inicio un poleo.
	RANGE_flags.start2poll = FALSE;

	pv_pollInit();

	// En modo monitor o service frame no guardo en memoria.
	switch(systemVars.wrkMode) {
	case WK_NORMAL:
		RANGE_flags.saveFrameInBD = TRUE;
		break;
	case WK_MONITOR_FRAME:
		RANGE_flags.saveFrameInBD = FALSE;
		break;
	case WK_SERVICE:
		RANGE_flags.saveFrameInBD = FALSE;
		break;
	}

	pv_RANGEprintExitMsg(3);
	return(rgST_R02);
}
//------------------------------------------------------------------------------------
static int rgTR_04(void)
{
	// Cambio el DIN: debo polear
	RANGE_flags.dInChange = FALSE;
	RANGE_flags.saveFrameInBD = TRUE;

	pv_pollInit();

	pv_RANGEprintExitMsg(4);
	return(rgST_R02);
}
//------------------------------------------------------------------------------------
static int rgTR_05(void)
{
	// Este ciclo lo hago MAX_TOTAL_POLL_COUNTS o MAX_GOOD_POLL_COUNTS.
	// dependiendo si tengo poleos malos o no.

u16 distancia;
s08 qualityData;

	vTaskDelay( ( TickType_t)( 250 / portTICK_RATE_MS ) );

	// Poleo: Mido el nivel
	//u_rangeSignal(RUN);
//	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	qualityData = BAD;
	pv_rangeMeter_Fire(&distancia, &qualityData );
//	u_rangeSignal(STOP);

	snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("Range:: Dist=%d, Qty=%d\r\n\0"), distancia, qualityData);
	u_debugPrint(D_DATA, range_printfBuff, sizeof(range_printfBuff) );


	// Si la medida fue correcta, la guardo
	if ( qualityData == GOOD ) {

		if ( RANGE_counters.goodPollCounts > 0)
			RANGE_counters.goodPollCounts--;

		//snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("DEBUG::rgTR_05_B:: GPC=%d\r\n\0"), RANGE_counters.goodPollCounts);
		//FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );

		rangeDataStruct.distancia[RANGE_counters.goodPollCounts] = distancia;
		rangeDataStruct.quality[RANGE_counters.goodPollCounts] = qualityData;

		// Si tengo todos los poleos necesarios bien, salgo.
		if ( RANGE_counters.goodPollCounts == 0 )
			RANGE_flags.poll = FALSE;

	}

	// Si llegue al maximo permitido de poleos, salgo
	if ( --RANGE_counters.totalPollCounts == 0 )
		RANGE_flags.poll = FALSE;

	pv_RANGEprintExitMsg(5);
	return(rgST_R02);
}
//------------------------------------------------------------------------------------
static int rgTR_06(void)
{
	// Apago el sensor.
	// Completo el frame con fechaHora y datos digitales.
	// Imprimo
	// Si corresponde, salvo en BD.

u16 pos = 0;
size_t bWrite;
StatBuffer_t pxFFStatBuffer;
u16 distancia;
u08 i;
u08 goods;

	u_rangeSignal(STOP);

	// Promedio entre la cantidad de poleos correctos que tenga
	distancia = 0;
	goods = 0;
	for (i = 0; i < MAX_GOOD_POLL_COUNTS; i++ ) {

		if ( rangeDataStruct.quality[i] == GOOD ) {
			distancia += rangeDataStruct.distancia[i];
			goods++;
		}
	}
	distancia /= goods;

	// Armo el frame.
	RTC_read(&Aframe.rtc);
	// PW
	Aframe.inputs[0] = distancia;
	// Digital
	Aframe.inputs[1] = din0;
	Aframe.inputs[2] = din1;

	// Guardo en BD ?
	if ( RANGE_flags.saveFrameInBD ) {
		RANGE_flags.saveFrameInBD = FALSE;
		bWrite = FF_fwrite( &Aframe, sizeof(Aframe));
		FF_stat(&pxFFStatBuffer);

		if ( bWrite != sizeof(Aframe) ) {
			// Error de escritura ??
			snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("WR ERROR: (%d)\r\n\0"),pxFFStatBuffer.errno);
		} else {
			// Stats de memoria
			snprintf_P( range_printfBuff, sizeof(range_printfBuff), PSTR("MEM [%d/%d/%d][%d/%d]\r\n\0"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		}
		u_debugPrint(D_BASIC, range_printfBuff, sizeof(range_printfBuff) );
	}

	// Imprimo el frame.
	if ( goods == 0 ) {
		pos = snprintf_P( range_printfBuff, sizeof(range_printfBuff), PSTR("frame::(ERROR){" ));
	} else {
		pos = snprintf_P( range_printfBuff, sizeof(range_printfBuff), PSTR("frame::{" ));
	}

	// timeStamp.
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );
	// Valores
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%s=%d,"),systemVars.chName[0],Aframe.inputs[0] );
	// Valores digitales
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%s=%d,"), systemVars.chName[1],Aframe.inputs[1]);
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%s=%d"), systemVars.chName[2],Aframe.inputs[2]);

	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("}\r\n\0") );
	FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );

	// Me preparo para un nuevo poleo
	RANGE_flags.start2poll = FALSE;

	pv_RANGEprintExitMsg(6);
	return(rgST_R01);
}
//------------------------------------------------------------------------------------
static void pv_RANGEprintExitMsg(u08 code)
{

u32 tickCount;

	tickCount = xTaskGetTickCount();
	snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR(".[%06lu] tkRange::exit rgTR_%02d\r\n\0"), tickCount,code);
	u_debugPrint(D_DATA, range_printfBuff, sizeof(range_printfBuff) );
}
//------------------------------------------------------------------------------------
s16 u_readTimeToNextPoll(void)
{
s16 retVal = -1;

	// Lo determina en base al time elapsed y el timerPoll.
	// El -1 indica un modo en que no esta poleando.
	if ( ( systemVars.wrkMode == WK_NORMAL ) || ( systemVars.wrkMode == WK_MONITOR_FRAME )) {
		retVal = RANGE_counters.secs4poll;
	}

	return (retVal);
}
/*------------------------------------------------------------------------------------*/
void u_readDataFrame (frameData_t *dFrame)
{

	memcpy(dFrame, &Aframe, sizeof(Aframe) );
}
//----------------------------------------------------------------------------------------
static s08 pv_awaitLineHIGH(void)
{
	// Espero que PB2 este arriba.

u08 pin;
TickType_t xTicksToWait = 200;
TimeOut_t xTimeOut;
s08 retS = FALSE;

	vTaskSetTimeOutState( &xTimeOut );

	while (1) {
		pin  = ( ( RM_PW_PIN & _BV(RM_PW_BIT) ) >> RM_PW_BIT );
		// High ??
		if ( pin == 1 ) {
			// Lo apago inmediatamente
			TIMER1_STOP;
			retS = TRUE;
			break;
		}
		// Timeout ??
	     if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
	         break;
	}
	return(retS);

}
//----------------------------------------------------------------------------------------
static s08 pv_awaitLineLOW(void)
{
	// Espero que PB2 este low.

u08 pin;
TickType_t xTicksToWait = 200;
TimeOut_t xTimeOut;
s08 retS = FALSE;

	vTaskSetTimeOutState( &xTimeOut );

	while (1) {
		pin  = ( ( RM_PW_PIN & _BV(RM_PW_BIT) ) >> RM_PW_BIT );
		// Low ??
		if ( pin == 0 ) {
			// Lo prendo inmediatamente
			TCNT1 = 0;
			TIMER1_START;
			retS = TRUE;
			break;
		}
		// Timeout ??
	     if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
	         break;
	}
	return(retS);
}

static void pv_rangeMeter_Fire(u16 *distancia, s08 *status )
{
	// Genera un disparo y mide

	*status = BAD;

	if ( ! pv_awaitLineHIGH() ) {		// Espero un flanco de bajada
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("RangeMeter ERROR: no detect H1\r\n\0"));
		FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
		return;
	}

	// Espero que baje y lo prendo.
	if ( ! pv_awaitLineLOW() ) {
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("RangeMeter ERROR: no detect Low\r\n\0"));
		FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
		return;
	}

	//TCNT1 = 0;
	//TIMER1_START;

	if ( ! pv_awaitLineHIGH() ) {		// Espero el flanco de subida
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("RangeMeter ERROR: no detect H2\r\n\0"));
		FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
		return;
	}

	TIMER1_STOP;

	*distancia = (TCNT1 / 58);
	if ( *distancia < 700 ) {
		*status = GOOD;
	}

	//snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("DEBUG::RangeMeter:: Distancia=%d, status=%d\r\n\0"), *distancia, *status);
	//FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );

}
//----------------------------------------------------------------------------------------
static s08 pv_checkDIN4Change(void)
{
	// Leo las entradas digitales y determino si cambiaron
u08 din;
s08 changed = FALSE;

	din = ( ( RM_DIN0_PIN & _BV(RM_DIN0_BIT) ) >> RM_DIN0_BIT );
	if ( din != din0 ) {
		din0 = din;
		changed = TRUE;
	}

	din = ( ( RM_DIN1_PIN & _BV(RM_DIN1_BIT) ) >> RM_DIN1_BIT );
	if ( din != din1 ) {
		din1 = din;
		changed = TRUE;
	}

	RANGE_flags.dInChange = changed;
	return(changed);

}
//----------------------------------------------------------------------------------------
static void pv_pollInit(void)
{
u08 i;

	for (i = 0; i < MAX_GOOD_POLL_COUNTS; i++ ) {
		rangeDataStruct.distancia[i] = 0;
		rangeDataStruct.quality[i] = BAD;
	}

	RANGE_flags.poll = TRUE;
	RANGE_counters.goodPollCounts = MAX_GOOD_POLL_COUNTS;
	RANGE_counters.totalPollCounts = MAX_TOTAL_POLL_COUNTS;

	u_rangeSignal(RUN);
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
}
//----------------------------------------------------------------------------------------
