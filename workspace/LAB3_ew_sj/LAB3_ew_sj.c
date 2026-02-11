//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// sjew: fn defs
void setEPWM1A(float controleffort);
void setEPWM2A(float controleffort);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;

// sjew: global variables to count up and down in the duty cycles
int pwm_iterator = 1;
int motor_counting_up = 1;
float ctrl_effort = 0.0;


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LS7366#1 CS
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LS7366#2 CS	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LS7366#3 CS
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LS7366#4 CS
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// WIZNET RST
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	//PushButton 1
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_INPUT, GPIO_PULLUP);

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //F28027 CS
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
//    init_serialSCIC(&SerialC,115200);
//    init_serialSCID(&SerialD,115200);

    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);

    // PWM setup
    EALLOW;
    // sjew: PWM Setup for EPWM12A
    //       PERIOD: 20kHz, Duty Cycle 50%, High on Zero, Low on Comp A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPW12A
    // TBCTL
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 0x2; // Free Soft emulation = Free Run
    EPwm12Regs.TBCTL.bit.CLKDIV = 0; // Clock divide = 1
    EPwm12Regs.TBCTL.bit.CTRMODE = 0; // Count up Mode
    EPwm12Regs.TBCTL.bit.PHSEN = 0; // Do not load time time-base counter from the time-base phase registry
    // TBCTR
    EPwm12Regs.TBCTR = 0; // Start Timer at zero
    // TBPRD
    // Set period of the PWM to 20kHz
    // Counter frequency is 50MHz
    EPwm12Regs.TBPRD = 2500;
    // CMPA
    EPwm12Regs.CMPA.bit.CMPA = 1250; // Duty cycle at 50%, TBPRD / 2
    EPwm12Regs.AQCTLA.bit.CAU = 0x1; // PWM12A output pin is set low when CMPA is reached
    EPwm12Regs.AQCTLA.bit.ZRO = 0x2; // the pin be set high when the TBCTR register is zero
    EPwm12Regs.TBPHS.bit.TBPHS = 0;

    // sjew: setup for ePWM1a
    //       PERIOD: 20KHz, 50% default duty cycle, HIGH on zero and LOW on compare a
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0x2;
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm1Regs.TBCTL.bit.PHSEN = 0;
    EPwm1Regs.TBCTR = 0;
    EPwm1Regs.TBPRD = 2500; // 20 KHz (2500/50MHz)
    EPwm1Regs.CMPA.bit.CMPA = 1250;
    EPwm1Regs.AQCTLA.bit.CAU = 0x1; // set low
    EPwm1Regs.AQCTLA.bit.ZRO = 0x2; // set high
    EPwm1Regs.TBPHS.bit.TBPHS = 0;

    // sjew: setup for ePWM2a
    //       PERIOD: 20KHz, 50% default duty cycle, HIGH on zero and LOW on compare a
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x2;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500; // 20 KHz (2500/50MHz)
    EPwm2Regs.CMPA.bit.CMPA = 1250;
    EPwm2Regs.AQCTLA.bit.CAU = 0x1; // set low
    EPwm2Regs.AQCTLA.bit.ZRO = 0x2; // set high
    EPwm2Regs.TBPHS.bit.TBPHS = 0;
    EDIS;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
			// Normally on the Robot Car we only use the below UART_printfLine functions to write to the
			// on board LCD screen.  This below serial_printf is only used in lab 1 to print to a serial 
			// terminal over a USB cable like you will for your Homeworks.
			//serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
			
			//IMPORTANT!! %ld is for an int32_t.  To print an int16_t use %d
            UART_printfLine(1,"Timer2 Calls %ld",CpuTimer2.InterruptCount);
			UART_printfLine(2,"T0 %ld,T1 %ld",CpuTimer0.InterruptCount,CpuTimer1.InterruptCount);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%5) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
		
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    // sjew: if cmpa is equal to 0 or the entire period, flip the sign of the iteration step
    if(EPwm12Regs.CMPA.bit.CMPA == 0 || EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD) {
        pwm_iterator *= -1;
    }
    EPwm12Regs.CMPA.bit.CMPA += pwm_iterator; // sjew: increment cmpa by the iteration step (either +1 or -1)
    CpuTimer2.InterruptCount++;
	
    // sjew: if motor_counting_up not 0 increment ctrl_effort by 0.005
    //       and if the resultant ctrl_effort exceeds the limit of 10.0 then set motor_counting_up to 0
    if(motor_counting_up) {
        ctrl_effort += 0.005;
        if(ctrl_effort >= 10.0) {
            motor_counting_up = 0;
        }
    // sjew: if motor_counting_up is 0, decrement ctrl_effort by 0.05
    //       and if the resultant ctrl_effort exceeds the limit of -10.0 then set_motor_counting_up to 1
    } else {
        ctrl_effort -= 0.005;
        if(ctrl_effort <= -10.0) {
            motor_counting_up = 1;
        }
    }
    // sjew: set both motors to the current ctrl_effort
    setEPWM1A(ctrl_effort);
    setEPWM2A(ctrl_effort);


	if ((CpuTimer2.InterruptCount % 10) == 0) {
		UARTPrint = 1;
	}
}

void setEPWM1A(float controleffort) {
    // sjew: set max and min limits for controleffort
    if(controleffort < -10.0)
        controleffort = -10.0;
    if(controleffort > 10.0)
        controleffort = 10.0;
    int16_t offset = EPwm1Regs.TBPRD/2; // sjew: set an offset that would be a 50% duty cycle
    float unit_step = EPwm1Regs.TBPRD/20.0; // sjew: unit step of 5% of the total period (each 1.0 in control effort corresponds to 5% of the total period)
    EPwm1Regs.CMPA.bit.CMPA = offset+(unit_step*controleffort); // sjew: cmpa to 50% duty cycle + (5 * controleffort)% <- controleffort ranges from -10 to 10
}
void setEPWM2A(float controleffort) {
    // sjew: set min and max limits for controleffort
    if(controleffort < -10.0)
        controleffort = -10.0;
    if(controleffort > 10.0)
        controleffort = 10.0;
    int16_t offset = EPwm2Regs.TBPRD/2; // sjew: set an offset that would be 50% duty cycle
    float unit_step = EPwm2Regs.TBPRD/20.0; // sjew: unit step of 5% of the total period (each 1.0 in control effort corresponds to 5% of the total period)
    EPwm2Regs.CMPA.bit.CMPA = offset+(unit_step*controleffort); // sjew: cmpa to 50% duty cycle + (5 * controleffort)% <- controleffort ranges from -10 to 10
}

