/*
 16-bit PWM: when comparator matches, toggling
 CCP0 - controlling LED, controled by ADC
 ADC - Capacitor voltage
 CCP1 - controlling DCbooster recharging
*/
// next update - house keeping
/************************************************/
#include <STC15F2K60S2.H>
#include <stdio.h>
/************************************************/

/************************************************/
/***** 			defined numbers		*******************/

#define MAIN_Fosc   11059200L				//SYSCLK
#define PCA_IDLE_DISABLE    0				//1: PCA does not work when MCU in IDLE; 0:  PCA works when MCU in IDLE
#define PCA_SOURCE_SELECT   4				//timebase for PCA
                                            //0：SYSCLK/12
                                            //1：SYSCLK/2
                                            //2：Timer0 overflow
                                            //3：P3.4 external clock(SYSCLK/2 max)
                                            //4：SYSCLK
                                            //5：SYSCLK/4
                                            //6：SYSCLK/6
                                            //7：SYSCLK/8
#define	PWM_DUTY        11059			//period for PWM() //1ms
#define	PWM_HIGH_MAX    9953			//to limit maxium duty cycle (90%), not used
#define	PWM_HIGH_MIN    1106			//minimum duty cycle (10%), not used
#define	PWM							8294			//initial value for pwm, PCA counts when P1.1 is supposed to be high, respectively


/************************************************/
/*******		global varibles    	*****************/
sbit CCP0 = P1^1;									// LED
sbit CCP1 = P1^0;									// DCbooster
unsigned int    PWM_high0, PWM_high1;				// PCA counts when high
unsigned int    PWM_low0, PWM_low1;					// PCA counts when low
unsigned int    CCAP0_tmp;									// 16bit comparator varible for voltage-meter
unsigned int		CCAP1_tmp;									// 16bit comparator varible for DC-booster

/************************************************/
/*******	  	functions  			*******************/
void PWMn_SetHighReg(unsigned int);
void PWMn_init(unsigned int);
void port_init(void);
void adc_init(void);
unsigned char adc_result(void);
void pause(int);


/******************** main **********************/
//========================================================================
// function		: void main(void)
// description	: 1kHz PWM with 75% duty cycle
//								ADC
//								1kHz PWM with duty cycle WRT ADC
// parameters	: None
// returns		: None
// date			: 2017-5-1
// version		: v1.2
// note			: needed to be modified
//========================================================================
void main(void)
{	
	unsigned int adc;
	port_init();		// initialize ports
	adc_init();			// initialize adc
	PWMn_init(PWM);	// initialize pwm
	while (1)
	{
		adc = adc_result();
		//P3 = adc;		// show ADC results with LEDs	// consuming too much current
		pause(255);
		if(adc <= 2)
			adc = 2;		// PWM will fail if DC is too low
		PWM_high0 = adc*43;	// when adc=255 which means Vin=5V, PWM DC will be near 100%
		PWM_low0 = PWM_DUTY - PWM_high0;
	}
}

//========================================================================
// function		: void PWMn_init(unsigned int high)
// description	: initialize PWM(CCP0, CCP1) with PCA
// parameters 	: high
// returns		: None
// Date			: 2017-5-1
//========================================================================
void PWMn_init(unsigned int high)
{
	CCON = 0;			// clear CF、CR、CCF0、CCF1
	CMOD = (PCA_IDLE_DISABLE << 7) | (PCA_SOURCE_SELECT << 1);
								// PCA still operates when MCU in IDLE mode, clock source SYSCLK
								// or CMOD &= 0x79;  
								// CMOD |= 0x09;
	CCAPM0 = 0x4D;// comparator on(TOGGLE PULSE)
	CCAPM1 = 0x4D;// comparator on(TOGGLE PULSE)
	CL = 0;
	CH = 0;				// clear PCA high 8 bits and low 8 bits
	CCAP0_tmp = 0;
	CCAP1_tmp = 0;// clear two 16-bit comparator capture varibles
	PWMn_SetHighReg(high);	//initialize DC data
	CR = 1;				// run PCA
	EA = 1;				// enable global interrupt
}

//========================================================================
// function		: void PWMn_SetHighReg(unsigned int high)
// description	: write in initial DC data
// parameters	: high
// Returns		: none
// date			: 2017-5-1
//========================================================================
void PWMn_SetHighReg(unsigned int high)
{
	CR = 0;						// stop PCA
	
	// **** PWM0 **** LED
	PWM_high0 = high;	// write in DC high level counts
	PWM_low0 = PWM_DUTY - high;	// write in DC low level counts
	
	// **** PWM1 **** DCbooster
	PWM_high1 = high;	// write in DC high level counts
	PWM_low1 = PWM_DUTY - high;	// write in DC low level counts
	
	CR = 1;						// run PCA
}

//========================================================================
// function		: void port_initial(void)
// description	: initialize port states(PP, OD, HI, etc.)
// parameters	: None
// returns 		: None
// date			: 2017-5-1
//========================================================================
void port_init(void)
{
	P_SW1 &= 0xCF;		// set output pin(P1.2/ECI, P1.1/CCP0, P1.0/CCP1, P3.7/CCP2)
	P1M1 &= ~0x03;
	P1M0 |=  0x03;		// P1.1, P1.0 in PUSH-PULL
	
	/*//***********
	P3M1 = 0;
	P3M0 = 0xFF;			// show adc results with leds*/
}

//========================================================================
// function		: void adc_init(void)
// description	: initialize adc, including port init
// parameters	: none
// Returns		: none
// date			: 2017-4-30
//========================================================================
void adc_init(void)
{
	P1M1 |= 0x04;
	P1M0 &= ~0x04;	// high impedance for p1.2
	P1ASF |= 0x04;	// use p1.0 for ADC input
	ADC_CONTR = 0xE2;
									// turn on adcpower
									// speed 300kHz
									// not start
									// p1.2 as input
	CLK_DIV &= 0xDF;// ADC_RES[7:0] - HIGH8, ADC_RESL[1:0] - LOW2
	ADC_RES = 0;		// clear result register
	pause(25);			// without the delay, the reference voltage will be unstable
}

//========================================================================
// function		: unsigned int adc_result(void)
// description	: begin adc conversion, return adc result
// parameters	: none
// Returns		: adcresult
// date			: 2017-4-30
//========================================================================
unsigned char adc_result(void)
{
	unsigned int adcresult;
	ADC_CONTR |= 0x08;	// start adc
	while((ADC_CONTR & 0x10) == 0x00);	// wait for adc complete
	ADC_CONTR &= 0xEF;	// ADC_FLAG = 0
	adcresult = ADC_RES;
	ADC_RES = 0;				// clear result register
	return adcresult;
}

//========================================================================
// function		: void pause(void)
// description	: a rough delay by software
// parameters	: int n, delay for n*1000 subtractions
// Returns		: none
// date			: 2017-4-30
//========================================================================
void pause(int n)
{
	int i, j;
	for(i=n;i>0;i--)
		for(j=500;j>0;j--);
}

/****************************  all interrupt services below  ***************************/
//========================================================================
// function		: void PCA_interrupt (void) interrupt 7
// description	: PCA interrupt request
// parameters	: None
// returns 		: None
// date			: 2017-5-1
//========================================================================
void PCA_interrupt (void) interrupt 7
{
	if(CCF0 == 1)				//PCA0 comparator interrupt
	{
		CCF0 = 0;					//clear interrupt flag
		if(CCP0 == 1)   CCAP0_tmp += PWM_high0;      //output high, giving PCA counts for high voltage
		else            CCAP0_tmp += PWM_low0;       //output low, giving PCA counts for low voltage
		CCAP0L = (unsigned char)CCAP0_tmp;          //capture register low 8 bits
		CCAP0H = (unsigned char)(CCAP0_tmp >> 8);   //capture register high 8 bits
	}
	if(CCF1 == 1)
	{
		CCF1 = 0;
		if(CCP1 == 1)		CCAP1_tmp += PWM;      //output high, giving PCA counts for high voltage
		else            CCAP1_tmp += (PWM_DUTY - PWM);       //output low, giving PCA counts for low voltage
		CCAP1L = (unsigned char)CCAP1_tmp;          //capture register low 8 bits
		CCAP1H = (unsigned char)(CCAP1_tmp >> 8);   //capture register high 8 bits
	}
}
