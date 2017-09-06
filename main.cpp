/*-------------------------------------------------------------------
Description: Simple code for test DCM2 fanout board.

AH 2016.01.10, 2017.06.02

    included comment for testing github 9/6/17
-------------------------------------------------------------------*/
#include "predef.h"
#include <stdio.h>
#include <startnet.h>
#include <autoupdate.h>
#include <startnet.h>
#include <dhcpclient.h>
#include <tcp.h>
#include <string.h>
#include <math.h>

#include <ctype.h>
#include <taskmon.h>
#include <smarttrap.h>
#include "i2cmulti.h"

#include <stdlib.h>

# include <pins.h>

// Telnet defines
#define TCP_LISTEN_PORT 23 // Telent port number
#define RX_BUFSIZE (4096)
#define CMD_BUFSIZE 30

// Other defines
const char *AppName = "fanoutTest";

#define VER "\r\n\r\n   **** COMAP DCM2 test setup, v. 20170903 ****\r\n\r\n"

// I2C read/write; switch to 0 for no hardware
#if 1
#define I2CSEND1 I2CSendBuf(address, buffer, 1)
#define I2CSEND2 I2CSendBuf(address, buffer, 2)
#define I2CSEND3 I2CSendBuf(address, buffer, 3)
#define I2CREAD1 I2CReadBuf(address, buffer, 1)
#define I2CREAD2 I2CReadBuf(address, buffer, 2)
#else
#define I2CSEND1 0
#define I2CSEND2 0
#define I2CSEND3 0
#define I2CREAD1 0
#define I2CREAD2 0
#endif

// sub-bus addresses
#define I2C_SB0 0x01   // on-board switch setting for sub-buses
// subsub-bus addresses
#define I2C_SSB7 0x80  // on-board BEX
#define I2C_SSB5 0x20  // S5 firewire connector
#define I2C_SSB4 0x10  // S4 firewire connector

// For test board
// SPI masks
#define SPI_CLK0_M 0x04
#define SPI_DAT0_M 0x01
#define SPI_CSB1_M 0x02  // on-board BEX, CS& for P1, temp. sensor
#define SPI_CSB4_M 0x10  // on-board BEX, CS& for P4
// Inits
#define BEXREAD0 SPI_DAT0_M  // read P0, write P1..P7 for on-board TCA6408A
#define BEXINIT0 SPI_CSB1_M | SPI_CSB4_M
// BEX address
#define BEX_ADDR0 0x21

// For downconverter card
// SPI masks
#define QLOG_CS 0x01
#define ILOG_CS 0x02
#define Q_ATTEN_LE 0x04
#define I_ATTEN_LE 0x08
#define BOARD_T_CS 0x10
#define SPI_MISO_M 0x20
#define SPI_MOSI_M 0x40
#define SPI_CLK_M 0x80
// Inits
#define BEXREAD SPI_MISO_M  // read SPI_MISO_M, write all others on BEX
#define BEXINIT QLOG_CS | ILOG_CS | Q_ATTEN_LE | I_ATTEN_LE | BOARD_T_CS
// BEX address
#define BEX_ADDR 0x20
// other parameters
#define ADCVREF 3.3   // ADC voltage reference value
#define MAXATTEN 32   // Maximum attenuator setting
#define DBMSCALE -45.5  // Scale factor for logamp voltage to dBm conversion
#define DBMOFFSET 25.  // Offset value for logamp voltage to dBm conversion

int portErr[2] = {0, 0};  // keep track of which ports have a DCM2 attached; 0 means no error
                     // also use this to manually block port acces with an engr-type command

int iq_idx = 0;  // array index for tracking I, Q
int dcm_idx = 0; // index for retained values
int attenVal[4] = {999, 999, 999, 999}; // record of attenuator values for S4I, S4Q, S5I, S5Q
float logampOut[4] = {999., 999., 999., 999.};  // record of logamp output voltages for S4I, S4Q, S5I, S5Q

int I2CStatus = 0;  // 0 for successful completion of I2C transaction, else NetBurner I2C error code

//----- Global Vars -----
extern "C" {
void UserMain(void * pd);
}

// Telnet
char RXBuffer[RX_BUFSIZE];

//I2C
BYTE buffer[I2C_MAX_BUF_SIZE];
char I2CInputBuffer[I2C_MAX_BUF_SIZE];   // User created I2C input buffer
char* inbuf = I2CInputBuffer;            // Pointer to user I2C buffer
BYTE address = 0;
BYTE I2CStat;


// Functions

/*-------------------------------------------------------------------
Convert IP address to a string
-------------------------------------------------------------------*/
void IPtoString(IPADDR ia, char *s)
{
  PBYTE ipb= (PBYTE)&ia;
  siprintf(s, "%d.%d.%d.%d",(int)ipb[0],(int)ipb[1],(int)ipb[2],(int)ipb[3]);
}

// Allocate task stack for UDP listen task
DWORD TcpServerTaskStack[USER_TASK_STK_SIZE];


/*------------------------------------------------------------------
 * Set up I2C bus for fanout test
 ------------------------------------------------------------------*/
int openI2Cbus(BYTE addr_sb, BYTE addr_ssb)
{
	  address = 0x77;        // I2C switch address 0x77 for top-level switch
	  buffer[0] = addr_sb;   // I2C channel address  0x01 for second-level switch
	  I2CStat = I2CSEND1;

	  address = 0x73;        // I2C switch address 0x73 for second-level switch
	  buffer[0] = addr_ssb;  // I2C channel address for device: 0x80 for test BEX, 0x04 for J4, 0x02 for J5
	  I2CStat = I2CSEND1;

  return (I2CStat==0 ? 0 : -1);
}

/*------------------------------------------------------------------
 * Close I2C buses for fanout test
 ------------------------------------------------------------------*/
int closeI2Cbus(void)
{
	  address = 0x73;    // I2C switch address 0x73 for second-level switch
	  buffer[0] = 0x00;  // I2C channel address 0x00 to open all switches
	  I2CStat = I2CSEND1;

	  address = 0x77;    // I2C switch address 0x77 for top-level switch
	  buffer[0] = 0x00;  // I2C channel address 0x00 to open all switches
	  I2CStat = I2CSEND1;

  return (I2CStat==0 ? 0 : -1);
}

/*------------------------------------------------------------------
 * Configure BEX for fanout board
 ------------------------------------------------------------------*/
int configBEX(BYTE conf, BYTE addr)
{
	address = addr;    // I2C address for BEX chip on board
	buffer[0] = 0x03;  // configuration register
	buffer[1] = conf;  // conf = 0x01 for warm IF tester, LSB only reads
	I2CStat = I2CSEND2;

	return (I2CStat==0 ? 0 : 9000+I2CStat);
}

/*------------------------------------------------------------------
 * Initialize BEX
 ------------------------------------------------------------------*/
int initBEX(BYTE init, BYTE addr)
{
	address = addr;    // I2C address for BEX chip on board
	buffer[0] = 0x01;  // output port register
	buffer[1] = init;  // conf = 0x01 for warm IF tester, LSB only reads
	I2CStat = I2CSEND2;

	return (I2CStat==0 ? 0 : 9000+I2CStat);
}

/*------------------------------------------------------------------
 * Read BEX
 ------------------------------------------------------------------*/
BYTE readBEXoutput(BYTE addr)
{
	address = addr;    // I2C address for BEX chip on board
	buffer[0] = 0x01;  // output port register
	I2CSEND1;
	I2CREAD1;

	return (buffer[0]);
}

/*------------------------------------------------------------------
 * Write/read parallel interface
 ------------------------------------------------------------------*/
int bex(char *inpSt, BYTE addr)
{
  BYTE address = addr, buffer[10], cmd;
  char dev[10];
  int inp;

  printf("Input string is %s \r\n", inpSt);
  sscanf(inpSt, "%s %d", dev, &inp);
  iprintf("Parsed command value %s, %d, 0x%x \r\n", dev, inp, (BYTE)inp);
  cmd = (BYTE)inp;
  iprintf("Command byte 0x%x \r\n", cmd);


  // Set I2C sub-bus as needed
  /*
    ...
  */

  // Set configuration register for P0 read, P1..7 write
  buffer[0] = 0x03;  
  buffer[1] = 0x01;
  I2CStat = I2CSEND2;


  // Write input variable to input port register
  buffer[0] = 0x01;  
  buffer[1] = cmd;
  printf("Send value is 0x%x \r\n",  buffer[1]);
  I2CStat = I2CSEND2;

  // Read port register and print results to screen
  buffer[0] = 0x00;
  I2CStat = I2CSEND1;
  I2CStat = I2CREAD1;
  printf("BEX value is 0x%x \r\n",  buffer[0]);

  return 0;
}


/*------------------------------------------------------------------
 * Turn on LED
 ------------------------------------------------------------------*/
//This function turns the LED on the fanout test board on.

/*******************************************************************/
/**
  \brief SPI bit-bang read AD7814 temperature sensor.

  This function reads an AD7814 temperature sensor by generating SPI
  bit-bang signals through a TCA6408A parallel interface chip.

  Requires previous call to function that sets I2C bus switches to
  address the interface before use, and close after.  Also requires
  previous single call to initialize interface chip.

  Use case:
 		  openI2Cbus(I2C_SB0, I2C_SSB7);   // select BEX on warm IF test board
   		  configBEX(BEXREAD0);            // configure BEX on warm IF test board
   		  printf("Temperature = %.2f\n", AD7814_SPI_bitbang(SPI_CLK_M, SPI_DAT_M, SPI_CSB1_M));
   		  closeI2Cbus();

  \param  val  value to convert and send
  \return temperature, or (9000 + NB I2C error code) for bus errors.
*/
int ledOn (void)
{
	int I2CStat;
	BYTE tmp;

	openI2Cbus(I2C_SB0, I2C_SSB7);	// select BEX

	// get command state of output pins on interface
	address = 0x21;      // I2C address for BEX chip on board
	buffer[0] = 0x01;    // output port register
	I2CStat = I2CSEND1;  // set register
	// kick out for I2C bus errors
	if (I2CStat) return (9000+I2CStat);
	I2CREAD1;            // get pin data
	tmp = buffer[0];

	// LED is on P7, address 0x80.  Low is LED on.
	buffer[0] = 0x01;  // write buffer
	buffer[1] = tmp & ~0x80;
	I2CSEND2;

	closeI2Cbus();
	return(0);
}

float AD7814_SPI_bitbang(BYTE spi_clk_m, BYTE spi_dat_m, BYTE spi_csb_m, BYTE addr)
{

	/* Mask definitions are hardware dependent
	*/

	BYTE x = 0;             // Working command byte
	int I2CStat = 0;        // Comms error
 	// var and var_m are defined below

	// get command state of output pins on interface
	x = readBEXoutput(addr);

    buffer[0] = 0x01;    // write register
	address = addr;      // I2C address for BEX chip on board
	// set up for read
	x |= spi_csb_m;       // ensure CS is high at start
	x |= spi_clk_m;      // ensure CLK is high at start
	buffer[1] = x;
	I2CStat = I2CSEND2;
	if (I2CStat) return (9000+I2CStat);

	x &= ~spi_csb_m;      // send CS low to initiate read
	buffer[1] = x;
	I2CSEND2;

	// read initial zero, one cycle
	x &= ~spi_clk_m;     // set clock low
	buffer[1] = x;
	I2CSEND2;
	x |= spi_clk_m;      // set clock high
	buffer[1] = x;
	I2CSEND2;

	// read MSB at clock high to get sign bit
	x &= ~spi_clk_m;      // set clock low
	buffer[1] = x;
	I2CSEND2;
	x |= spi_clk_m;       // set clock high
	buffer[1] = x;
	I2CSEND2;
	buffer[0] = 0x00;     // set input port register and get pin data
	I2CSEND1;
	I2CREAD1;

	int flag = 0;
	// define input variable and mask, deal with negative values
	short int val = 0;              // initialize value
	short int val_m = 0x0100;       // initialize mask
	if (buffer[0] & (unsigned short)spi_dat_m) {
		flag = 1;
		val = 0xfe00; // fill top bits of word if val < 0
	}

	// step through remainder of input word, reading at clock high
	while (val_m > 0) {
		// set clock low
		x &= ~spi_clk_m;
		buffer[0] = 0x01;
		buffer[1] = x;
		I2CSEND2;
		// set clock high
		x |= spi_clk_m;
		buffer[1] = x;
		I2CSEND2;
		// read bit
		buffer[0] = 0x00;
		I2CSEND1;
		I2CREAD1;
		// put bit in word and rotate mask for next-LSB
		if (buffer[0] & (unsigned short)spi_dat_m) val |= val_m;
		val_m >>= 1;
	}

	// all done, deselect with CS& high
	x |= spi_csb_m;
	buffer[0] = 0x01;
	buffer[1] = x;
	I2CStat = I2CSEND2;

	return ( I2CStat ? (float)(9000+I2CStat) : (float)val*0.25 );
}

/*******************************************************************/
/**
  \brief SPI bit-bang read AD7860 16-bit ADC.

  This function reads an AD7860 16-bit ADC by generating SPI
  bit-bang signals through a TCA6408A parallel interface chip.

  Requires previous call to function that sets I2C bus switches to
  address the interface before use, and close after.  Also requires
  previous single call to initialize interface chip.

  \param  val  value to convert and send
  \return voltage, or (9000 + NB I2C error code) for bus errors.
*/
float AD7860_SPI_bitbang(BYTE spi_clk_m, BYTE spi_dat_m, BYTE spi_csb_m, float vdd, BYTE addr)
{

	BYTE x ;                        // Working command byte
	int I2CStat = 0;                // Comms error counter
	short unsigned int val = 0;     // ADC value
	short unsigned int val_m;       // Mask

	// get command state of output pins on interface
	address = addr;      // I2C address for BEX chip on board
	buffer[0] = 0x01;    // output port register
	I2CStat = I2CSEND1;  // set register
	// kick out for I2C bus errors
	if (I2CStat) return (9000+I2CStat);
	I2CREAD1;            // get pin data
	x = buffer[0];       // update working byte

	// set up for read
	x |= spi_csb_m;       // ensure CS is high at start
	x |= spi_clk_m;      // ensure CLK is high at start
    buffer[0] = 0x01;    // write register
	buffer[1] = x;
	I2CSEND2;
	x &= ~spi_csb_m;      // send CS low to initiate read
	buffer[1] = x;
	I2CSEND2;

	// read three cycles of initial zeros
	val_m = 1 << (3 - 1);
	while (val_m > 0) {
		x &= ~spi_clk_m;     // set clock low
		buffer[1] = x;
		I2CSEND2;
		x |= spi_clk_m;      // set clock high
		buffer[1] = x;
		I2CSEND2;
		val_m >>= 1;
	}

	// step through 16-bit input word, msb to lsb, reading at clock high
	val_m = 1 << (16 - 1);
	while (val_m > 0) {
		// set clock low
		x &= ~spi_clk_m;
		buffer[0] = 0x01;
		buffer[1] = x;
		I2CSEND2;
		// set clock high
		x |= spi_clk_m;
		buffer[1] = x;
		I2CSEND2;
		// read bit
		buffer[0] = 0x00;
		I2CSEND1;
		I2CREAD1;
		// put bit in word and rotate mask for next-LSB
		if (buffer[0] & (unsigned short)spi_dat_m) val |= val_m;
		val_m >>= 1;
	}

	// all done. clock low, then deselect with CS& high
	buffer[0] = 0x01;
	x &= ~spi_clk_m;
	buffer[1] = x;
	x |= spi_csb_m;
	buffer[1] = x;
	I2CStat = I2CSEND2;

	return ( I2CStat ? (float)(9000+I2CStat) : (float)val*vdd/65536. );
}

/*******************************************************************/
/**
  \brief SPI bit-bang write to 6-bit step attenuator.

  This function converts an unsigned integer to a series of I2C commands
  for a HNC624 step attenuator.

  Requires previous call to function that sets I2C bus switches to
  address the interface before use, and close after.  Also requires
  previous single call to initialize interface chip.

  \param  atten  attenuation to convert and send
  \return NB I2C error code for bus errors.

*/
int HNC624_SPI_bitbang(BYTE spi_clk_m, BYTE spi_dat_m, BYTE spi_csb_m, float atten, BYTE addr)
{

	BYTE x = 0;                       // working byte
	int I2CStat = 0;                  // comms errors
	unsigned short int val, val_m;    // binary value word and mask

	val = (unsigned short)round(atten * 2.);  // convert from dB to bits
	if (val > 63) val = 63;

	// get command state of output pins on interface
	x = readBEXoutput(addr);

    buffer[0] = 0x01;    // write register
	address = addr;      // I2C address for BEX chip on board
	// set up for read
	x |= spi_csb_m;       // ensure CS is high at start
	x |= spi_clk_m;      // ensure CLK is high at start
	buffer[1] = x;
	I2CStat = I2CSEND2;
	if (I2CStat) return (9000+I2CStat);

	x &= ~spi_csb_m;      // send CS low to initiate read
	buffer[1] = x;
	I2CSEND2;

	// step through input word with mask; write data with clock low, then raise clock
	val_m = 1 << (6 - 1);
	while (val_m > 0) {
		x &= ~spi_clk_m; // set clock low
		if (val & val_m) { // look at bit value in word to set data bit
			x &= ~spi_dat_m;  // bit inversion: 0 is logical TRUE
		}
		else {
			x |= spi_dat_m;
		}
		buffer[1] = x;
		I2CSEND2;        // write to bus extender
		x |= spi_clk_m;  // set clock high, data will be valid here
		buffer[1] = x;
		I2CSEND2;        // write to bus extender
		val_m >>= 1;     // rotate to next-MSB
	}

	// done, set CS high to terminate SPI write
	x |= spi_csb_m;
	buffer[1] = x;
	I2CStat += I2CSEND2;

	return ( I2CStat );

}


/*------------------------------------------------------------------
 * Command parser
 *
 * In attenuator command, receiver and channel are 1s-based
 * Test board has only receivers and channels 1 and 2
 ------------------------------------------------------------------*/
int cmdParse(char *inpSt)
{

	// initialize variables
	char dev[10];
	int dcm;
	char iq[10];
	float atten;
	int errVal = 0;
	BYTE dcmSel = 0;
	BYTE phaSel = 0;
	float vadc;
	BYTE iq_cs = 0x01;
	char txt[80];
	long int i; //loop counter
	static const char *errMsg =
			"\r\n*** Unrecognized input: format must be one of\r\n"
			"A d c a  : Set attenuator for downconverter d, I or Q, a [dB]\r\n"
			"R d      : Read out monitor points for downconverter d\r\n"
			"JR d     : Read out monitor points for downconverter d, JSON format\r\n"
			"JL d c k : Loop read totpwr mon for dwncnv d, I or Q, loop k times, JSON\r\n"
			"B        : Read fanout board temperature sensor\r\n"
			"E        : Echo system values\r\n"
			"JE       : Echo system values, JSON format\r\n"
			"?        : This message\r\n"
			"quit     : Exit\r\n\r\n";


	closeI2Cbus();  // ensure bus switches are

	// read string and select action
	int narg = sscanf(inpSt, "%s %d %s %f", dev, &dcm, iq, &atten);
	//printf("Testing atten with DCM2 port = %d, chan = %d, atten = %f\r\n", dcm-1, chan-1, atten);
	// select DCM2
	if (narg >= 2) {
		switch (dcm) {
		case 4:
			if (portErr[0]) {
				iprintf("No DCM2 detected on port 4\r\n\r\n");
				return(-101);
			}
			dcmSel = I2C_SSB4;
			dcm_idx = 0;
			break;
		case 5:
			if (portErr[1]) {
				iprintf("No DCM2 detected on port 5\r\n\r\n");
				return(-102);
			}
			dcmSel = I2C_SSB5;
			dcm_idx = 1;
			break;
		default:
			iprintf("Invalid downconverter port number! Use either 4 or 5\r\n\r\n");
			iprintf("%s", errMsg);
			return (-1);
		}
	}
	// select channel within DCM2
	if (narg >= 3) {
		if (!strcasecmp(iq, "I")) {
			phaSel = I_ATTEN_LE;
			iq_idx = 0;
			iq_cs = ILOG_CS;
		}
		else if (!strcasecmp(iq, "Q")) {
			phaSel = Q_ATTEN_LE;
			iq_idx = 1;
			iq_cs = QLOG_CS;
		}
		else {
			iprintf("Invalid channel! Use either I or Q\r\n");
			iprintf("%s", errMsg);
			return (-1);
		}
	}

	// work out action
	if (narg == 4) {
		if (!strcasecmp(dev, "A")) {
			// open bus, configure for write, write, configure for read, exit
			openI2Cbus(I2C_SB0, dcmSel);   // select BEX on I2C_SSB4 or dcmSel
			if (atten > MAXATTEN) atten = MAXATTEN;
			if (atten < 0) atten = 0;
			int rtn = HNC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, phaSel, atten, BEX_ADDR);
			iprintf("\r\n%s", (rtn ? "I2C bus error" : "Atten. set ok"));
			attenVal[2*dcm_idx+iq_idx] = atten;
			closeI2Cbus();
		} else if (!strcasecmp(dev, "JL")) {
			// open bus
			openI2Cbus(I2C_SB0, dcmSel);
			// read ADC twice to clear potential first-read startup problem, check for valid range
			vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, iq_cs, ADCVREF, BEX_ADDR);
			vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, iq_cs, ADCVREF, BEX_ADDR);
#if(1)
			// set up non-changing part of text string
			sprintf(txt, "{\"testset\":{\"valid\":%s, \"Rx\":%d, \"IQ\":\"%c\", \"logampV\":[",
					( (vadc>0.0 && vadc<ADCVREF) ? "true" : "false"),
					dcm, toupper(iq[0]));
			// loop over number of reads, sending string to terminal each time
			for (i=0; i<(long int)(atten+0.5); i++) {
				vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, iq_cs, ADCVREF, BEX_ADDR);
				printf("%s%f]}}\r\n", txt, vadc);
			}
#else
			// print init string and first value
			printf("{\"testset\":{\"valid\":%s, \"Rx\":%d, \"IQ\":\"%c\", \"logampV\":[%f",
					( (vadc>0.0 && vadc<ADCVREF) ? "true" : "false"),
					dcm, toupper(iq[0]), vadc);
			// loop over number of reads, sending next value to string
			for (i=1; i<(long int)(atten+0.5); i++) {
				vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, iq_cs, ADCVREF, BEX_ADDR);
				printf(", %f", vadc);
			}
			// close json statment
			iprintf("]}}\r\n");
#endif
			// close bus
			closeI2Cbus();
		} else {
			iprintf("%s", errMsg);
			attenVal[2*dcm_idx+iq_idx] = 999;
		}
	} else if (narg == 2) {
		if (!strcasecmp(dev, "R")) {
			openI2Cbus(I2C_SB0, dcmSel);   // select BEX
			// read temperature sensor
			printf("\r\n%.2f [C] board temperature\r\n",
					AD7814_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, BOARD_T_CS, BEX_ADDR));
			// read I ADC; if close to rail, could be first read after power-up, so read again
			vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, ILOG_CS, ADCVREF, BEX_ADDR);
			if (vadc > (ADCVREF - 0.01)) vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, ILOG_CS, ADCVREF, BEX_ADDR);
			printf("%f [V] from I logamp ADC (approx. %.1f [dBm])\r\n", vadc, vadc*DBMSCALE+DBMOFFSET);
			logampOut[2*dcm_idx] = vadc;
			// read Q ADC; if close to rail, could be first read after power-up, so read again
			vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, QLOG_CS, ADCVREF, BEX_ADDR);
			if (vadc > (ADCVREF - 0.01)) vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, QLOG_CS, ADCVREF, BEX_ADDR);
			printf("%f [V] from Q logamp ADC (approx. %.1f [dBm])\r\n", vadc, vadc*DBMSCALE+DBMOFFSET);
			logampOut[2*dcm_idx+1] = vadc;
			closeI2Cbus();
		} else if (!strcasecmp(dev, "JR")) {
			openI2Cbus(I2C_SB0, dcmSel);   // select BEX
			// read I ADC; if close to rail, could be first read after power-up, so read again
			vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, ILOG_CS, ADCVREF, BEX_ADDR);
			if (vadc > (ADCVREF - 0.01)) vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, ILOG_CS, ADCVREF, BEX_ADDR);
			printf("{\"testset\":{\"valid\":true, \"Rx\":%d, \"Temp\":%.1f, \"I\":{\"PDV\":%f, \"PDdB\":%.1f}",
					dcm, AD7814_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, BOARD_T_CS, BEX_ADDR),
					vadc, vadc*DBMSCALE+DBMOFFSET);
			logampOut[2*dcm_idx] = vadc;
			// read Q ADC; if close to rail, could be first read after power-up, so read again
			vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, QLOG_CS, ADCVREF, BEX_ADDR);
			if (vadc > (ADCVREF - 0.01)) vadc = AD7860_SPI_bitbang(SPI_CLK_M, SPI_MISO_M, QLOG_CS, ADCVREF, BEX_ADDR);
			printf(", \"Q\":{\"PDV\":%f, \"PDdB\":%.1f}}}\r\n", vadc, vadc*DBMSCALE+DBMOFFSET);
			logampOut[2*dcm_idx+1] = vadc;
			closeI2Cbus();
		} else {
			iprintf("%s", errMsg);
		}
	} else if (narg == 1) {
		if (!strcasecmp(dev, "B")) {
			openI2Cbus(I2C_SB0, I2C_SSB7);
			printf("\r\nFanout board temperature = %.2f",
					AD7814_SPI_bitbang(SPI_CLK0_M, SPI_DAT0_M, SPI_CSB1_M, BEX_ADDR0));
			closeI2Cbus();
		} else if (!strcasecmp(dev, "E")) {
				printf("\r\n%d %d %5.3f %5.3f %3.1f %3.1f \r\n"
						"%d %d %5.3f %5.3f %3.1f %3.1f \r\n"
						"Atten_S4I Atten_S4Q Vdet_S4I Vdet_S4Q dBm_S4I dBm_S4Q \r\n"
						"Atten_S5I Atten_S5Q Vdet_S5I Vdet_S5Q dBm_S5I dBm_S5Q \r\n",
						attenVal[0], attenVal[1], logampOut[0], logampOut[1],
						logampOut[0]*DBMSCALE+DBMOFFSET, logampOut[1]*DBMSCALE+DBMOFFSET,
						attenVal[2], attenVal[3], logampOut[2], logampOut[3],
						logampOut[2]*DBMSCALE+DBMOFFSET, logampOut[3]*DBMSCALE+DBMOFFSET);
		} else if (!strcasecmp(dev, "JE")) {
				printf("{\"testset\":{\"valid\":true, \"Rx\":[4, 5], \"attnI\":[%d, %d], \"attnQ\":[%d, %d], "
						"\"PDVI\":[%f, %f], \"PDVQ\":[%f, %f], \"PDdBI\":[%.2f, %.2f], "
						"\"PDdBQ\":[%.2f, %.2f]}}\r\n",
						attenVal[0], attenVal[1], attenVal[2], attenVal[3],
						logampOut[0], logampOut[1], logampOut[2], logampOut[3],
						logampOut[0]*DBMSCALE+DBMOFFSET, logampOut[1]*DBMSCALE+DBMOFFSET,
						logampOut[2]*DBMSCALE+DBMOFFSET, logampOut[3]*DBMSCALE+DBMOFFSET);
		} else {
				iprintf("%s", errMsg);
		}
	} else {
		iprintf("%s", errMsg);
	}

	iprintf("\r\n");
	return errVal;
}


/*------------------------------------------------------------------
 * Blink LED
 ------------------------------------------------------------------*/
//This function blinks the LED on the fanout test board on and off.

int blinkLED (void)
{
	int i;  // loop counter
	BYTE address = 0x21, buffer[2];  // BEX comms

	openI2Cbus(I2C_SB0, I2C_SSB7);	// select BEX

	// Set configuration register for P0 read, P1..7 write
	buffer[0] = 0x03;
	buffer[1] = 0x01;
	I2CStat = I2CSEND2;

	buffer[0] = 0x01;  // write buffer
	// LED is on P7, address 0x80.  Low is LED on.
	for (i=0; i<3; i++) {
		buffer[1] = 0x00;
		I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 1 );
		buffer[1] = 0x80;
		I2CSEND2;
		OSTimeDly( TICKS_PER_SECOND * 1 );
	}

	// turn LED on at end
	buffer[1] = 0x00;
	I2CSEND2;

	closeI2Cbus();
	return(0);
}

/*******************************************************************/
/**
  \brief send reset signal to i2c switches

  \return Zero.
*/
//# include <pins.h>

int rstI2Csw(void)
{

	//J2-28 is assigned to QSPI_DOUT when not GPIO
	//using the QSPI register should not interfere with I2C, which is in the FECI register
	//following lines should be in system initialization
	//J2[28].function (PINJ2_28_GPIO);  // configure pin J2-28 for GPIO
	//J2[28].clr();  // starts high on reboot, so bring low; leaves a 3.2 us pulse to 0

	// send a reset signal, wait some time, then clear reset
	OSTimeDly(1); // needs a delay to make the pulse the right width, otherwise lots of jitter
	J2[28].set();
	OSTimeDly(1);
	J2[28].clr();
	printf("sent reset pulse, TICKS_PER_SECOND = %d\r\n", TICKS_PER_SECOND);

	return 0;
}

/*******************************************************************/
/**
  \brief read i2c pn values.

  \return Zero.
*/
//# include <pins.h>
int readI2C(void)
{

	int val = 0; // pin value buffer
	// set pins for GPIO to read
	J2[42].function (PINJ2_42_GPIO);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_GPIO);  // configure SDA as GPIO
	// read pins for BOOL values
	if (J2[42]) val += 1;
	if (J2[39]) val += 2;
	iprintf("SCL, SDA: 0x%d\r\n", val);
	// set pins for I2C again
	J2[42].function (PINJ2_42_SCL);  // configure SCL as GPIO
	J2[39].function (PINJ2_39_SDA);  // configure SDA as GPIO

	return 0;
}


/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/
/***********************************************************************************/

/*-------------------------------------------------------------------
  TCP Server Task
  Original NB code with AH add-on to capture and edit string from connection
  -------------------------------------------------------------------*/
void TcpServerTask(void * pd)
{
 
  int ListenPort = (int) pd;
  // Set up the listening TCP socket
  int fdListen = listen(INADDR_ANY, ListenPort, 5);
  if (fdListen > 0)
    {
      IPADDR client_addr;
      WORD port;
      while(1)
	{
	  // The accept() function will block until a TCP client requests
	  // a connection. Once a client connection is accepting, the
	  // file descriptor fdnet is used to read/write to it.
	  iprintf( "Waiting for connection on port %d...\n", ListenPort );
	  int fdnet = accept(fdListen, &client_addr, &port, 0);

	  // redirect input, output, and error to telnet session
	  ReplaceStdio(0, fdnet);
	  ReplaceStdio(1, fdnet);
	  ReplaceStdio(2, fdnet);

	  iprintf("Connected to: "); ShowIP(client_addr);
	  iprintf(":%d\n", port);
	  
	  writestring(fdnet, "Welcome to the NetBurner TCP Server; argus_v1\r\n");
	  char s[20];
	  IPtoString(EthernetIP, s);
	  siprintf(RXBuffer, "You are connected to IP Address %s, port %d\r\n",
		   s, TCP_LISTEN_PORT);
	  writestring(fdnet, RXBuffer);

	  while (fdnet > 0)
	    {
	      /* Loop while connection is valid. The read() function will return
		 0 or a negative number if the client closes the connection, so
		 we test the return value in the loop. Note: you can also use
		 ReadWithTimout() in place of read to enable the connection to
		 terminate after a period of inactivity.
	      */
	   
		  //// Hardware initialization at startup
		  // Initialize and pulse I2C switch reset line
		  J2[28].function (PINJ2_28_GPIO);  // configure pin J2-28 for GPIO
		  OSTimeDly (5);
		  J2[28].clr();  // send reset pulse out at start
		  OSTimeDly (5);
		  J2[28].set();  // set pin high to enable IF board I2C switches
		  // Configure system and turn on LED on completion
		  openI2Cbus(I2C_SB0, I2C_SSB7);
		  configBEX(BEXREAD0, BEX_ADDR0);
		  initBEX(BEXINIT0, BEX_ADDR0);
		  printf("\r\nTest board temperature = %.2f\r\n\r\n",
				  AD7814_SPI_bitbang(SPI_CLK0_M, SPI_DAT0_M, SPI_CSB1_M, BEX_ADDR0));
		  closeI2Cbus();
		  ledOn();

		  // Try configuring BEX on downconverter board attached to P4, record error state
		  openI2Cbus(I2C_SB0, I2C_SSB4);
		  configBEX(BEXREAD, BEX_ADDR);           // configure bus extender
		  I2CStatus = initBEX(BEXINIT, BEX_ADDR); // set initial value
		  portErr[0] = I2CStatus;              // record 0 for success, err code for fail
		  if (!I2CStatus) {
			  // HNC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, (Q_ATTEN_LE | I_ATTEN_LE), 40, BEX_ADDR);
			  iprintf("Found module on port 4\r\n");
		  }
		  closeI2Cbus();

		  // Try configuring BEX on downconverter board attached to P5, record error state
		  openI2Cbus(I2C_SB0, I2C_SSB5);
		  configBEX(BEXREAD, BEX_ADDR);           // configure bus extender
		  I2CStatus = initBEX(BEXINIT, BEX_ADDR); // set initial value
		  portErr[1] = I2CStatus;              // record 0 for success, err code for fail
		  if (!I2CStatus) {
			  // HNC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, (Q_ATTEN_LE | I_ATTEN_LE), 40, BEX_ADDR);
			  iprintf("Found module on port 5\r\n");
		  }
		  closeI2Cbus();


		  iprintf(VER);

		  char tmp[CMD_BUFSIZE];  // temporary buffer for string
	      tmp[0] = '\r';  // start with carriage return
	      int ctr = 0;  // character counter

	      int n = 0;  // return value for read; stop reading for n = 0
	      do {
	    	  n = read( fdnet, RXBuffer, RX_BUFSIZE - 1 );
	    	  RXBuffer[n] = '\0';
	    	  //iprintf( "Read %d bytes: %s\n", n, RXBuffer );  // read back buffer contents

		  /* Build up string, echo string to telnet screen, allow backspace */
	    	  if (RXBuffer[0] == '\r') {
              // if end of line, go process
	    		  //DO WORK HERE             ???  ZZZ

	    		  cmdParse(tmp);


	    		  //DONE WITH WORK, LOOP
	  		      tmp[1] = '\0'; // first character after carriage return
	    		  ctr = 0; // restart character counter
	    	  }
	    	  else if (RXBuffer[0] == '\b' && ctr > 0 && ctr < CMD_BUFSIZE) {  
	    	  // if backspace, clear and write space
	    		  tmp[ctr] = ' ';
	    		  tmp[ctr+1] = '\0';
	    		  write (fdnet, tmp, ctr+1);  // erase character on screen
	    		  tmp[ctr] = '\0';  // terminate correct string 
	    		  ctr--;  // point to right place for next character
	    	  }
	    	  else if (ctr < CMD_BUFSIZE-1) {
	    	  // add character to buffer
	    		  ctr++;
	    		  tmp[ctr] = RXBuffer[0];
	    		  tmp[ctr+1] = '\0';
	    	  }
	    	  write (fdnet, tmp, ctr+1);  // send current string to display

	    	  if (!strcmp("\rquit", tmp)) {
	    		  n = 0;    // signal to break out of loop
	    	  }

	      } while ( n > 0 );  // keep doing this until it's time to close

	      // Then close connection
	      iprintf("\r\nClosing client connection: ");
	      ShowIP(client_addr);
	      iprintf(":%d\n", port);
	      close(fdnet);
	      fdnet = 0;
	    }
	} // while(1)
    } // while listen
}


/*-------------------------------------------------------------------
  User Main
  ------------------------------------------------------------------*/
void UserMain(void * pd)
{
  InitializeStack(); // Initialize the TCP/IP Stack

  if ( EthernetIP == 0 )
    {
      iprintf( "Trying DHCP\r\n" );
      GetDHCPAddress();
      iprintf( "DHCP assigned the IP address of :" );
      ShowIP( EthernetIP );
      iprintf( "\r\n" );
    }
  else
    {
      iprintf( "Static IP address set to :" );
      ShowIP( EthernetIP );
      iprintf( "\r\n" );
    }

  EnableAutoUpdate(); // Enable network code updates
  OSChangePrio(MAIN_PRIO); // Set this task priority

  #ifdef _DEBUG
  InitializeNetworkGDB_and_Wait();
  #endif
  
  // Set up I2C device
  // start I2C interface
  I2CInit( 0xaa, 0x1a );   // Initialize I2C and set NB device slave address and I2C clock
  // Second argument is clock divisor for I2C bus, freqdiv
  // 0x16 for 97.6 kHz (fastest)
  // 0x17 for 78.1 kHz
  // 0x3b for 73.2 kHz
  // 0x18 for 65.1 kHz
  // 0x19 for 58.6 kHz
  // 0x1a for 48.8 kHz
  // 0x1c for 32.6 kHz
  // 0x1f for 19.5 kHz (slowest)


/*
  // try initializing system at power-up
  int I2CStatus;
  // set DCM2 connected to S4 to 0 dB atten
  openI2Cbus(I2C_SB0, I2C_SSB4);
  configBEX(BEXREAD, BEX_ADDR);           // configure bus extender
  I2CStatus = initBEX(BEXINIT, BEX_ADDR); // set initial value
  portErr[0] = I2CStatus;              // record 0 for success, err code for fail
  if (!I2CStatus) {
	  HNC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, (Q_ATTEN_LE | I_ATTEN_LE) , 0, BEX_ADDR);
  }
  closeI2Cbus();
  // set DCM2 connected to S5 to max atten
  openI2Cbus(I2C_SB0, I2C_SSB5);
  configBEX(BEXREAD, BEX_ADDR);           // configure bus extender
  I2CStatus = initBEX(BEXINIT, BEX_ADDR); // set initial value
  portErr[0] = I2CStatus;              // record 0 for success, err code for fail
  if (!I2CStatus) {
	  HNC624_SPI_bitbang(SPI_CLK_M, SPI_MOSI_M, (Q_ATTEN_LE | I_ATTEN_LE) , 40, BEX_ADDR);
  }
  closeI2Cbus();
*/

  // Create TCP Server task
  OSTaskCreate( TcpServerTask,
		(void *)TCP_LISTEN_PORT,
		&TcpServerTaskStack[USER_TASK_STK_SIZE] ,
		TcpServerTaskStack,
		MAIN_PRIO - 1); // higher priority than UserMain

  while (1)
    {
      // The work is happening in the while loop in TcpServerTask
      OSTimeDly( TICKS_PER_SECOND * 5 );
    }
}
