/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "sp5KV5_PZ.h"

static char cmd_printfBuff[CHAR128];
char *argv[16];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static u08 pv_makeArgv(void);

void pv_cmdRdRTC(void);
void pv_cmdRdEE(void);
void pv_cmdRdMCP(void);
void pv_cmdRdDIN(void);
s08 pv_cmdWrDebugLevel(char *s);
s08 pv_cmdWrkMode(char *s0, char *s1);
s08 pv_cmdWrEE(char *s0, char *s1);
s08 pv_cmdWrLog(char *s);
static void pv_readMemory(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdRedialFunction(void);
static void cmdStatusFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
/*------------------------------------------------------------------------------------*/
void tkCmd(void * pvParameters)
{

u08 c;
u08 ticks;
( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	cmdlineInit();
	cmdlineSetOutputFunc(pvFreeRTOS_UART1_writeChar);

	cmdlineAddCommand((u08 *)("cls"), cmdClearScreen );
	cmdlineAddCommand((u08 *)("help"), cmdHelpFunction);
	cmdlineAddCommand((u08 *)("reset"), cmdResetFunction);
	cmdlineAddCommand((u08 *)("read"), cmdReadFunction);
	cmdlineAddCommand((u08 *)("write"), cmdWriteFunction);
	cmdlineAddCommand((u08 *)("redial"), cmdRedialFunction);
	cmdlineAddCommand((u08 *)("status"), cmdStatusFunction);


	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	ticks = 1;
	FreeRTOS_ioctl( &pdUART1,ioctlSET_TIMEOUT, &ticks );

	// loop
	for( ;; )
	{
		u_clearWdg(WDG_CMD);

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		while ( FreeRTOS_read( &pdUART1, &c, 1 ) == 1 ) {
			cmdlineInputFunc(c);
		}

		/* run the cmdline execution functions */
		cmdlineMainLoop();
	}
}
/*------------------------------------------------------------------------------------*/
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-----------------------------------------------------\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset {default,memory}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-redial\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-----------------------------------------------------\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write rtc YYMMDDhhmm\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR( "  wrkmode [service | monitor {sqe|frame}]\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerpoll, dlgid, gsmband\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debuglevel +/-{none,basic,mem,data,gprs,all} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  log {on,off} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  A{0..2} aname \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn, roaming {on|off}, port, ip, script, passwd\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  save\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) mcp devId regAddr regValue\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) ee addr string\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) gprspwr {0|1},gprssw {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) atcmd {cmd}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) rangectl {run|stop}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-----------------------------------------------------\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  mcp {0|1} regAddr\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dcd,din {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc, adc {ch}, frame\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee addr lenght\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) frame,memory,gprs\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  defaults \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	pv_makeArgv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		FF_rewind();
	}

	cmdClearScreen();
	// RESET
	u_reset();

}
/*------------------------------------------------------------------------------------*/
static void cmdRedialFunction(void)
{
	// Envio un mensaje a la tk_Gprs para que recargue la configuracion y disque al server
	// Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkGprsTx,TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

}
/*------------------------------------------------------------------------------------*/
static void cmdStatusFunction(void)
{

RtcTimeType_t rtcDateTime;
u16 pos;
u08 channel;
frameData_t Cframe;
StatBuffer_t pxFFStatBuffer;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Last reset info
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Wdg (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DlgId */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Fecha y Hora */
	RTC_read(&rtcDateTime);
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("rtc: %02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Server:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* APN */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn: %s\r\n\0"), systemVars.apn );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER IP:SERVER PORT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.serverAddress,systemVars.serverPort );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER SCRIPT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER PASSWD */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  passwd: %s\r\n\0"), systemVars.passwd );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Modem:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Modem band */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  band: "));
	switch ( systemVars.gsmBand) {
	case 0:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(900)"));
		break;
	case 1:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(1800)"));
		break;
	case 2:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (900/1800)"));
		break;
	case 3:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("pcs (1900)"));
		break;
	case 4:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("gsm (850)"));
		break;
	case 5:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (1900/850)"));
		break;
	case 6:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (900/1800/1900)"));
		break;
	case 7:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (850/1800/1900)"));
		break;
	case 8:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("cuatriband (850/900/1800/1900)"));
		break;
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* ROAMING */
	if ( systemVars.roaming == TRUE ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  roaming ON\r\n\0"));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  roaming OFF\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DLGIP */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dlg ip: %s\r\n\0"), systemVars.dlgIp );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SYSTEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">System:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Memoria */
	FF_stat(&pxFFStatBuffer);
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory: wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d \r\n"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* WRK mode (NORMAL / SERVICE) */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  wrkmode: "));
	/* WRK mode (NORMAL / SERVICE) */
	switch (systemVars.wrkMode) {
	case WK_NORMAL:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("normal\r\n"));
		break;
	case WK_SERVICE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("service\r\n"));
		break;
	case WK_MONITOR_FRAME:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("monitor_frame\r\n"));
		break;
	case WK_MONITOR_SQE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("monitor_sqe\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Timers */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerPoll [%ds]: %d\r\n\0"),systemVars.timerPoll, u_readTimeToNextPoll() );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DebugLevel */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debugLevel: "));
	if ( systemVars.debugLevel == D_NONE) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("none") );
	} else {
		if ( (systemVars.debugLevel & D_BASIC) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+basic")); }
		if ( (systemVars.debugLevel & D_DATA) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+data")); }
		if ( (systemVars.debugLevel & D_GPRS) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+gprs")); }
		if ( (systemVars.debugLevel & D_MEM) != 0)   { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+mem")); }
		if ( (systemVars.debugLevel & D_DEBUG) != 0)  { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+debug")); }
	}
	snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Log
	if ( systemVars.log == ON ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  log: ON\r\n\0"));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  log: OFF\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CONFIG */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Config:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	for ( channel = 0; channel < NRO_CHANNELS; channel++) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Ch%d->%s\r\n\0"),channel, systemVars.chName[channel] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	/* VALUES --------------------------------------------------------------------------------------- */
	memset(&Cframe,'\0', sizeof(frameData_t));
	u_readDataFrame (&Cframe);
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Values:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("  "));
	// TimeStamp.
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff),PSTR( "%04d%02d%02d,"),Cframe.rtc.year,Cframe.rtc.month,Cframe.rtc.day );
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%02d%02d%02d,"),Cframe.rtc.hour,Cframe.rtc.min, Cframe.rtc.sec );
	// Valores analogicos
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%d,"),systemVars.chName[0],Cframe.inputs[0] );
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%d,"),systemVars.chName[1],Cframe.inputs[1] );
	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("%s=%d"),systemVars.chName[2],Cframe.inputs[2] );

	pos += snprintf_P( &cmd_printfBuff[pos], sizeof(cmd_printfBuff), PSTR("\r\n\0") );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{
u08 argc;
char *p;

	argc = pv_makeArgv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		pv_cmdRdEE();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		pv_cmdRdRTC();
		return;
	}

	// MCP
	// read mcp 0|1|2 addr
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		pv_cmdRdMCP();
		return;
	}

	// DIN
	// read din 0|1
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0"))) {
		pv_cmdRdDIN();
		return;
	}

	// DEFAULT
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULTS\0"))) {
		u_loadDefaults();
		return;
	}

	// FRAME
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		while ( xTaskNotify(xHandle_tkRange, TKR_READ_FRAME , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
		return;
	}

	// MEMORY
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		pv_readMemory();
		return;
	}

	// GPRS RSP.
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0"))) {
		p = FreeRTOS_UART_getFifoPtr(&pdUART0);
		FreeRTOS_write( &pdUART1, "rx->", sizeof("rx->")  );
		FreeRTOS_write( &pdUART1, p, UART0_RXBUFFER_LEN );
		FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0")  );
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
s08 retS = FALSE;
u08 argc;

	argc = pv_makeArgv();

	// SAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		retS = u_saveSystemParams();
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PASSWD
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
			systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// APN
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ROAMING
	if (!strcmp_P( strupr(argv[1]), PSTR("ROAMING\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) { systemVars.roaming = TRUE; }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { systemVars.roaming = FALSE; }
		pv_snprintfP_OK();
		return;
	}

	// SERVER PORT
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverPort, '\0', sizeof(systemVars.serverPort));
			memcpy(systemVars.serverPort, argv[2], sizeof(systemVars.serverPort));
			systemVars.serverPort[PORT_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER IP
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverAddress, '\0', sizeof(systemVars.serverAddress));
			memcpy(systemVars.serverAddress, argv[2], sizeof(systemVars.serverAddress));
			systemVars.serverAddress[IP_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER SCRIPT
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* DEBUGLEVEL */
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUGLEVEL\0"))) {
		retS = pv_cmdWrDebugLevel(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* DEBUGLEVEL */
	if (!strcmp_P( strupr(argv[1]), PSTR("LOG\0"))) {
		retS = pv_cmdWrLog(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* WRKMODE */
	if (!strcmp_P( strupr(argv[1]), PSTR("WRKMODE\0"))) {
		retS = pv_cmdWrkMode(argv[2],argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CANALES ANALOGICOS
	if (!strcmp_P( strupr(argv[1]), PSTR("A0\0"))) {
		retS = u_configAnalogCh( 0, argv[2] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("A1\0"))) {
		retS = u_configAnalogCh( 1, argv[2] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("A2\0"))) {
		retS = u_configAnalogCh( 2, argv[2] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// TIMERPOLL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0"))) {
		retS = u_configTimerPoll(argv[2]);

		// tk_range: notifico en modo persistente. Si no puedo, me voy a resetear por watchdog. !!!!
		while ( xTaskNotify(xHandle_tkRange, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = u_wrRtc(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	//----------------------------------------------------------------------
	// COMANDOS USADOS PARA DIAGNOSTICO
	// DEBEMOS ESTAR EN MODO SERVICE
	//----------------------------------------------------------------------

	// GSMBAND:
	// Debo estar en modo service ya que para que tome el valor debe resetearse
	if (!strcmp_P( strupr(argv[1]), PSTR("GSMBAND\0"))) {
		if ( argv[2] == NULL ) {
			retS = FALSE;
		} else {
			systemVars.gsmBand = atoi(argv[2]);
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// EE: write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = pv_cmdWrEE( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSPWR\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setGprsPwr( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsSW
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSSW\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		retS = MCP_setGprsSw( (u08) atoi(argv[2]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// MCP
	// write mcp 0|1|2 addr value
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		switch( atoi(argv[2] )) {
		case 0:
			retS = MCP_write( MCP0_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
		case 1:
			retS = MCP_write( MCP1_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ATCMD
	// Envia un comando al modem.
	if (!strcmp_P( strupr(argv[1]), PSTR("ATCMD\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"%s\r",argv[2] );
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
		FreeRTOS_write( &pdUART0, cmd_printfBuff, sizeof(cmd_printfBuff) );

		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("sent->%s\r\n\0"),argv[2] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// RANGE control
	if (!strcmp_P( strupr(argv[1]), PSTR("RANGECTL\0")) && ( systemVars.wrkMode == WK_SERVICE) ) {
		if (!strcmp_P( strupr(argv[2]), PSTR("RUN"))) {
			u_rangeSignal(RUN);
			retS = TRUE;
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("STOP"))) {
			u_rangeSignal(STOP);
			retS = TRUE;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
// FUNCIONES PRIVADAS
//-------------------------------------------------------------------------------------
s08 pv_cmdWrEE(char *s0, char *s1)
{
u08 length = 0;
char *p;
s08 retS = FALSE;

	p = s1;
	while (*p != 0) {
		p++;
		length++;
	}
//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("S=[%s](%d)\r\n\0"),s1, length);
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	retS = EE_write( (u16)(atoi(s0)), s1, length );
	return(retS);
}
/*------------------------------------------------------------------------------------*/
s08 pv_cmdWrkMode(char *s0, char *s1)
{
s08 retS = FALSE;

	if ((!strcmp_P(strupr(s0), PSTR("SERVICE")))) {
		systemVars.wrkMode = WK_SERVICE;
		retS = TRUE;
		goto quit;
	}

	if ((!strcmp_P(strupr(s0), PSTR("MONITOR")))) {

		if ((!strcmp_P( strupr(s1), PSTR("SQE")))) {
			systemVars.wrkMode = WK_MONITOR_SQE;
			retS = TRUE;
			goto quit;
		}

		if ((!strcmp_P( strupr(s1), PSTR("FRAME")))) {
			systemVars.wrkMode = WK_MONITOR_FRAME;
			retS = TRUE;
			goto quit;
		}
	}

quit:

	if ( retS ) {
		// Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
		// tk_Range:
		while ( xTaskNotify(xHandle_tkRange, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
		// tk_Gprs:
		while ( xTaskNotify(xHandle_tkGprsTx, TK_PARAM_RELOAD , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
	}

	return(retS);
}
/*------------------------------------------------------------------------------------*/
s08 pv_cmdWrDebugLevel(char *s)
{

	if ((!strcmp_P( strupr(s), PSTR("NONE")))) {
		systemVars.debugLevel = D_NONE;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("+BASIC")))) {
		systemVars.debugLevel += D_BASIC;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-BASIC")))) {
		if ( ( systemVars.debugLevel & D_BASIC) != 0 ) {
			systemVars.debugLevel -= D_BASIC;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DATA")))) {
		systemVars.debugLevel += D_DATA;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DATA")))) {
		if ( ( systemVars.debugLevel & D_DATA) != 0 ) {
			systemVars.debugLevel -= D_DATA;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+MEM")))) {
		systemVars.debugLevel += D_MEM;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-MEM")))) {
		if ( ( systemVars.debugLevel & D_MEM) != 0 ) {
			systemVars.debugLevel -= D_MEM;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+GPRS")))) {
		systemVars.debugLevel += D_GPRS;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-GPRS")))) {
		if ( ( systemVars.debugLevel & D_GPRS) != 0 ) {
			systemVars.debugLevel -= D_GPRS;
			return(TRUE);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DEBUG")))) {
		systemVars.debugLevel += D_DEBUG;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DEBUG")))) {
		if ( ( systemVars.debugLevel & D_DEBUG) != 0 ) {
			systemVars.debugLevel -= D_DEBUG;
			return(TRUE);
		}
	}
	if ((!strcmp_P( strupr(s), PSTR("ALL")))) {
		systemVars.debugLevel = D_DATA + D_GPRS + D_MEM  + D_DEBUG;
		return(TRUE);
	}

	return(FALSE);
}
/*------------------------------------------------------------------------------------*/
s08 pv_cmdWrLog(char *s)
{

	if ((!strcmp_P( strupr(s), PSTR("ON")))) {
		systemVars.log = ON;
		return(TRUE);
	}

	if ((!strcmp_P( strupr(s), PSTR("OFF")))) {
		systemVars.log = OFF;
		return(TRUE);
	}

	return(FALSE);
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdRTC(void)
{
RtcTimeType_t rtcDateTime;
s08 retS = FALSE;
u08 pos;

	retS = RTC_read(&rtcDateTime);
	if (retS ) {
		pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n"));
		pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos ),PSTR("%02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
		pos += snprintf_P( &cmd_printfBuff[pos],(sizeof(cmd_printfBuff) - pos ),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdEE(void)
{
	// read ee address length
	// address: argv[2]
	// length: argv[3]

s08 retS = FALSE;

	memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	retS = EE_read( (u16)(atoi(argv[2])), cmd_printfBuff, atoi(argv[3]) );
	if ( retS ) {
		// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
		snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
	retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdMCP(void)
{
	// read mcp 0|1|2 addr
	// mcpDevide: argv[2]
	// devAddress: argv[3]

s08 retS = FALSE;
u08 regValue;

	switch( atoi(argv[2] )) {
	case 0:
		retS = MCP_read( MCP0_ADDR, atoi(argv[3]), &regValue );
		break;
	case 1:
		retS = MCP_read( MCP1_ADDR, atoi(argv[3]), &regValue );
		break;
	}
	if (retS ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n[reg 0X%03x]=[0X%03x]\r\n\0"),atoi(argv[3]),regValue);
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
void pv_cmdRdDIN(void)
{
s08 retS = FALSE;
u08 pin;

	switch( atoi(argv[2] )) {
	case 0:
		pin  = ( RM_DIN0_PIN & _BV(RM_DIN0_BIT) ) >> RM_DIN0_BIT;
		retS = TRUE;
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("DIN_0=%d\r\n\0"),pin);
		break;
	case 1:
		pin  = ( RM_DIN1_PIN & _BV(RM_DIN1_BIT) ) >> RM_DIN1_BIT;
		retS = TRUE;
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("DIN_1=%d\r\n\0"),pin);
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();

	return;

}
/*------------------------------------------------------------------------------------*/
static u08 pv_makeArgv(void)
{
// A partir de la linea de comando, genera un array de punteros a c/token
//
char *token = NULL;
char parseDelimiters[] = " ";
int i = 0;

	// inicialmente todos los punteros deben apuntar a NULL.
	memset(argv, 0, sizeof(argv) );

	// Genero los tokens delimitados por ' '.
	token = strtok(SP5K_CmdlineBuffer, parseDelimiters);
	argv[i++] = token;
	while ( (token = strtok(NULL, parseDelimiters)) != NULL ) {
		argv[i++] = token;
		if (i == 16) break;
	}
	return(( i - 1));
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_OK(void )
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_readMemory(void)
{
StatBuffer_t pxFFStatBuffer;
frameData_t Aframe;
size_t bRead;
u08 pos, channel;

	FF_seek();
	while(1) {
		bRead = FF_fread( &Aframe, sizeof(Aframe));
		if ( bRead != sizeof(Aframe))
			break;

		// imprimo
		FF_stat(&pxFFStatBuffer);
		pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("RD:[%d/%d/%d][%d/%d] "), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("frame::{" ));
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );

		for ( channel = 0; channel < NRO_CHANNELS; channel++) {
			pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%s=%d,"),systemVars.chName[channel],Aframe.inputs[channel] );
		}
		// Bateria
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("\r\n\0"));
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
}
/*------------------------------------------------------------------------------------*/
