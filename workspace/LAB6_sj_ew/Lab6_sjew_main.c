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
#include "F2837xD_SWPrioritizedIsrLevels.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define FEETINONEMETER 3.28083989501312
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI1_HighestPriority(void);
__interrupt void SWI2_MiddlePriority(void);
__interrupt void SWI3_LowestPriority(void);

// sjew: ISR dec
__interrupt void ADCC_ISR (void);

// sjew: fn decs
void setEPWM1A(float controleffort);
void setEPWM2A(float controleffort);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
float readEncWheel(void);

void PostSWI1(void);
void PostSWI3(void);

// sjew: ladar, linux command, and LV data vars
uint32_t timecount = 0;

extern datapts ladar_data[228]; //distance data from LADAR

extern float printLV1;
extern float printLV2;

extern float LADARrightfront;
extern float LADARfront;
float LADARrightback = 0.0;

extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern char G_command[]; //command for getting distance -120 to 120 degree
extern uint16_t G_len; //length of command
extern xy ladar_pts[228]; //xy data

extern uint16_t LADARpingpong;
extern float LADARxoffset;
extern float LADARyoffset;

extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

extern uint16_t NewLVData;

uint16_t LADARi = 0;
pose ROBOTps = {0,0,0}; //robot position
pose LADARps = {3.5/12.0,0,1};  // 3.5/12 for front mounting, theta is not used in this current code
float printLinux1 = 0;
float printLinux2 = 0;

// sjew: globals for tracking pose
float prev_avg_speed = 0.0;

// sjew: adc isr variables
uint32_t ADCC_count = 0;
float adcc2result_volts = 0;
float adcc3result_volts = 0;
float adcc4result_volts = 0;
float adcc5result_volts = 0;
// sjew: initializations for IMU data
float IMUx = 0;
float IMU4x = 0;
float IMUz = 0;
float IMU4z = 0;
// sjew: start the offsets at 1.23 because that is what they are assumed to be but then theyre adjusted later
float IMU_x_offset = 1.23;
float IMU_4x_offset = 1.23;
float IMU_z_offset = 1.23;
float IMU_4z_offset = 1.23;

float sum_x = 0;
float sum_4x = 0;
float sum_z = 0;
float sum_4z = 0;

float IMUz_prev = 0.0;
float IMU4z_prev = 0.0;

float accum_IMUz = 0.0;
float accum_IMU4z = 0.0;
float accum_gyro_z = 0.0;
float gyro_z_prev = 0.0;

uint8_t zeroing_flag = 0;

// sjew: global variables to track velocity, position, effort values
int pwm_iterator = 1;
int motor_counting_up = 1;
float ctrl_effort = 0.0;
float curr_left_enc= 0;
float curr_right_enc = 0;
float curr_p_right = 0;
float curr_p_left = 0;
float prev_p_right = 0;
float prev_p_left = 0;
float v_left = 0;
float v_right = 0;
float curr_knob_enc = 0;
float left_rpf = 9.83;
float right_rpf = 9.833;
float u_left = 0;
float u_right = 0;

// sjew: friction const
float visc_pos = 2.3335 * 0.6;
float static_pos = 1.8655 * 0.6;
float visc_neg = 2.2755 * 0.6;
float static_neg = -1.863 * 0.6;

// sjew: PID Constants
float v_ref = 0;
float kp = 3.0;
float ki = 25.0;
float e_left = 0.0;
float e_right = 0.0;
float accum_e_left = 0.0;
float accum_e_right = 0.0;
float prev_e_left = 0.0;
float prev_e_right = 0.0;
float e_steer = 0.0;
float kp_turn = 3.0;
float turn_command = 0.0;

// sjew: gryo vars
void setupSpib(void);
__interrupt void SPIB_isr(void);
//int16_t spivalue1 = 0;
//int16_t spivalue2 = 0;
int16_t gyro_x_raw = 0;
int16_t gyro_y_raw = 0;
int16_t gyro_z_raw = 0;
int16_t acc_x_raw = 0;
int16_t acc_y_raw = 0;
int16_t acc_z_raw = 0;

float acc_x = 0.0;
float acc_y = 0.0;
float acc_z = 0.0;
float gyro_x = 0.0;
float gyro_y = 0.0;
float gyro_z = 0.0;

float sum_gyroz = 0.0;
float gyro_z_offset = 0.0;
uint32_t SPIB_isr_cnt = 0;


// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;

// sjew: wall following params
float ref_right_wall = 1.0;
float left_turn_Start_threshold = 1.25;
float left_turn_Stop_threshold = 3;
float Kp_right_wall = -2.5;
float Kp_front_wall = -1.5;
float front_turn_velocity = 0.5;
float forward_velocity = 1.0;
float turn_command_saturation = 2;
int right_wall_follow_state = 2;
float right_wall_far = 1.5;
float right_turn_threshold = 3.75;



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

    // sjew: setups for ladar stuff
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

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

    // sjew: adcc isr init
    PieVectTable.ADCC1_INT = &ADCC_ISR;

    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    // sjew: set up software interrupts
    PieVectTable.EMIF_ERROR_INT = &SWI1_HighestPriority;
    PieVectTable.RAM_CORRECTABLE_ERROR_INT = &SWI2_MiddlePriority;
    PieVectTable.FLASH_CORRECTABLE_ERROR_INT = &SWI3_LowestPriority;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 100000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 100000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 50000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    DELAY_US(1000000);  // Delay 1 second giving LADAR Time to power on after system power on

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
    init_serialSCIC(&SerialC,19200);
    init_serialSCID(&SerialD,2083332);

    for (LADARi = 0; LADARi < 228; LADARi++) {
        ladar_data[LADARi].angle = ((3*LADARi+44)*0.3515625-135)*0.01745329; //0.017453292519943 is pi/180, multiplication is faster; 0.3515625 is 360/1024
    }

    // sjew: init eQEPs and set up pins to be PWM and not GPIO
    init_eQEPs();
    setupSpib();
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);

    EALLOW;
    // sjew: motor EPWM Setup
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


    // sjew: epwm trigger for sampling ADC at a 1ms rate
    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 0x2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 0x1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    //ADCC
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 0x2; //SOC0 will convert Channel you choose Does not have to be B0
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0xb;// EPWM4 ADCSOCA or another trigger you choose will trigger SOC0
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 0x3; //SOC1 will convert Channel you choose Does not have to be B1
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0xb;// EPWM4 ADCSOCA or another trigger you choose will trigger SOC1
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 0x4; //SOC2 will convert Channel you choose Does not have to be B2
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0xb;// EPWM4 ADCSOCA or another trigger you choose will trigger SOC2
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 0x5; //SOC3 will convert Channel you choose Does not have to be B3
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0xb;// EPWM4 ADCSOCA or another trigger you choose will trigger SOC3
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0x3; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    PieCtrlRegs.PIEIER12.bit.INTx10 = 1;
    PieCtrlRegs.PIEIER12.bit.INTx11 = 1;
    // sjew: enabled additional interrupts
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;


    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // sjew : lidar init setup
    char S_command[19] = "S1152000124000\n";//this change the baud rate to 115200
    uint16_t S_len = 19;
    serial_sendSCIC(&SerialC, S_command, S_len);


    DELAY_US(1000000);  // Delay letting LADAR change its Baud rate
    init_serialSCIC(&SerialC,115200);


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            // Normally on the Robot Car we only use the below UART_printfLine functions to write to the
            // on board LCD screen.  This below serial_printf is only used in lab 1 to print to a serial
            // terminal over a USB cable like you will for your Homeworks.
            //serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);

            //IMPORTANT!! %ld is for an int32_t.  To print an int16_t use %d
//            UART_printfLine(1, "br: %.2f fr: %.2f", LADARrightback, LADARrightfront);
//            UART_printfLine(2, "L fr: %.3f", right_wall_fo);
            UART_printfLine(1, "gyro_z heading: %.2f", ROBOTps.theta);
            UART_printfLine(2, "x: %.2f, y: %.2f", ROBOTps.x, ROBOTps.y);
//            UART_printfLine(2, "state: %d", right_wall_follow_state);
            UARTPrint = 0;
        }
    }
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
    // sjew: send g command to get ladar data
    serial_sendSCIC(&SerialC, G_command, G_len);

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

//  if ((CpuTimer2.InterruptCount % 10) == 0) {
//      UARTPrint = 1;
//  }
}

__interrupt void ADCC_ISR (void){
    // sjew: get the z, 4z, x, 4x adc values from the gyro and convert to voltages
    int16_t adcc2result = AdccResultRegs.ADCRESULT0;
    int16_t adcc3result = AdccResultRegs.ADCRESULT1;
    int16_t adcc4result = AdccResultRegs.ADCRESULT2;
    int16_t adcc5result = AdccResultRegs.ADCRESULT3;

    adcc2result_volts = adcc2result * 3.0 / 4095.0;
    adcc3result_volts = adcc3result * 3.0 / 4095.0;
    adcc4result_volts = adcc4result * 3.0 / 4095.0;
    adcc5result_volts = adcc5result * 3.0 / 4095.0;
    // sjew: convert voltages to deg/sec using the offset
    IMUx = (adcc2result_volts - IMU_x_offset ) * 400.0;
    IMU4x = (adcc3result_volts - IMU_4x_offset ) * 100.0;

    IMUz = (adcc5result_volts - IMU_z_offset ) * 400.0;
    IMU4z = (adcc4result_volts - IMU_4z_offset ) * 100.0;

    // sjew: send SPI request
    // Clear GPIO66 Low to act as a Slave Select. Right now, just to scope. Later, select the MPU9250 chip
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when two values are in the RX FIFO
//    SpibRegs.SPITXBUF = (0x8000 | 0x4600); // the 0x8000 set the read bit and 46 GYRO_YOUT_L
//    SpibRegs.SPITXBUF = 0x0000; // Send 16 zeros in order that we receive the 16 GyroZ reading
    SpibRegs.SPITXBUF = (0x8000 | 0x3A00); // the 0x8000 set the read bit and 3A INT_STATUS
    SpibRegs.SPITXBUF = 0x0000; //ACCEL_XOUT_H and ACCEL_XOUT_L
    SpibRegs.SPITXBUF = 0x0000; //ACCEL_YOUT_H and ACCEL_YOUT_L
    SpibRegs.SPITXBUF = 0x0000; //ACCEL_ZOUT_H and ACCEL_ZOUT_L
    SpibRegs.SPITXBUF = 0x0000; // TEMP_OUT_H and TEMP_OUT_L
    SpibRegs.SPITXBUF = 0x0000; // GYRO_XOUT_H and GYRO_XOUT_L
    SpibRegs.SPITXBUF = 0x0000; // GYRO_YOUT_H and GYRO_YOUT_L
    SpibRegs.SPITXBUF = 0x0000; // GYRO_ZOUT_H and GYRO_ZOUT_L




    // sjew: print every 500*1ms
    if ((ADCC_count % 100) == 0) {
        UARTPrint = 1;
    }

    ADCC_count++;

    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
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

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
//    SE423 8 Lab #3
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP2 pins for input
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO55 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP3 pins for input
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1; // Disable pull-up on GPIO54 (EQEP3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1; // Disable pull-up on GPIO55 (EQEP3B)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 5); // set GPIO6 and eQep2A
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 5); // set GPIO7 and eQep2B
    EQep3Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep3Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep3Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep3Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep3Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep3Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep3Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep3Regs.QPOSCNT = 0;
    EQep3Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    // SE423 9 Lab #3
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return -(raw*(1/2000.0*1/20.0*TWOPI));
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(1/2000.0*1/20.0*TWOPI)); // sjew: scaling factor of rotations/counts * wheel revs/rotation * radians/revolution
}

float readEncWheel(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep3Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(2*PI/4000.0)); // sjew: scaling factor of rotations/counts * wheel revs/rotation * radians/revolution
}

__interrupt void SPIB_isr(void){
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPIO 66 high to end Slave Select. Now to Scope.
                                 //Later, deselect the MPU9250
    //sj read off the first garbage value and then read the rest of the raw data
    int16_t temp = 0;
    temp = SpibRegs.SPIRXBUF;
    acc_x_raw = SpibRegs.SPIRXBUF;
    acc_y_raw = SpibRegs.SPIRXBUF;
    acc_z_raw = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    gyro_x_raw = SpibRegs.SPIRXBUF;
    gyro_y_raw = SpibRegs.SPIRXBUF;
    gyro_z_raw = SpibRegs.SPIRXBUF;
    // Later, when actually communicating with the MPU9250, do something with the data. Now do nothing.
    //sj scaling data
    acc_x = acc_x_raw*4.0/32767.0;
    acc_y = acc_y_raw*4.0/32767.0;
    acc_z = acc_z_raw*4.0/32767.0;

    gyro_x = gyro_x_raw * 250.0/32767.0;
    gyro_y = gyro_y_raw * 250.0/32767.0;
    gyro_z = gyro_z_raw * 250.0/32767.0;

    PostSWI1();

    SPIB_isr_cnt++;

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;// Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}

//
// Connected to PIEIER12_9 (use MINT12 and MG12_9 masks):
//
__interrupt void SWI1_HighestPriority(void)     // EMIF_ERROR
{

    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_9;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    GpioDataRegs.GPBSET.bit.GPIO61 = 1;

    uint16_t i = 0;//for loop

    // sjew: low level control
    if(timecount < 1000){

    } else if(timecount < 3000 && zeroing_flag == 0) {
        sum_x += adcc2result_volts;
        sum_4x += adcc3result_volts;
        sum_z += adcc5result_volts;
        sum_4z += adcc4result_volts;
        sum_gyroz += gyro_z;
    } else if(timecount == 3000 && zeroing_flag == 0) {
        // sjew: after the first three seconds, take the average of all the offsets and set the count flag to 1 ( completed )
        zeroing_flag = 1;
        IMU_z_offset = sum_z / 2000.0;
        IMU_4z_offset = sum_4z / 2000.0;
        IMU_x_offset = sum_x / 2000.0;
        IMU_4x_offset = sum_4x / 2000.0;
        gyro_z_offset = sum_gyroz / 2000.0;
        gyro_z_prev = gyro_z - gyro_z_offset;
    } else {
        // sjew: integrating for dead reckoning after offsets have been set
        accum_IMU4z = accum_IMU4z + 0.001 * (IMU4z + IMU4z_prev) / 2.0;
        accum_IMUz = accum_IMUz + 0.001 * (IMUz + IMUz_prev) / 2.0;
        IMUz_prev = IMUz;
        IMU4z_prev = IMU4z;
        gyro_z -= gyro_z_offset;
        accum_gyro_z = accum_gyro_z + 0.001 * ((gyro_z + gyro_z_prev)* PI/180.0) / 2.0;
        gyro_z_prev = gyro_z;
        ROBOTps.theta = accum_gyro_z;
    }

    // sjew: control loop
    curr_left_enc = readEncLeft();
    curr_right_enc = readEncRight();
    curr_knob_enc = readEncWheel();
    // sjew: position calculations
    curr_p_left = curr_left_enc * 1/ left_rpf;
    curr_p_right = curr_right_enc * 1/right_rpf;
    // sjew: vel calculations
    v_left = (curr_p_left - prev_p_left)/0.001;
    v_right = (curr_p_right - prev_p_right)/0.001;

    float avg_speed = (v_left + v_right) / 2;
    float pos_diff = (prev_avg_speed + avg_speed) / 2 * 0.001;
    ROBOTps.x += pos_diff * cosf(ROBOTps.theta);
    ROBOTps.y += pos_diff * sinf(ROBOTps.theta);
    prev_avg_speed = avg_speed;
    switch (right_wall_follow_state) {
    case 1:
        //Left Turn
        turn_command = Kp_front_wall*(14.5 - LADARfront);
        v_ref = front_turn_velocity;
        if (LADARfront > left_turn_Stop_threshold) {
            right_wall_follow_state = 2;
        }
        break;
    case 2:
        //Right Wall Follow
        float ladar_in = LADARrightfront;
        if(LADARfront < left_turn_Start_threshold) {
            right_wall_follow_state = 1;
            break;
        }
        if(LADARrightfront > right_wall_far) {
            if(LADARrightback > right_wall_far) {
                right_wall_follow_state = 3;
                break;
            } else {
                ladar_in = LADARrightback;
            }
        }
        turn_command = Kp_right_wall*(ref_right_wall - ladar_in);
        v_ref = forward_velocity;
        break;
    case 3:
        // sjew: turn right state
        turn_command = -Kp_front_wall*(14.5 - LADARfront);
        v_ref = front_turn_velocity;
        if (LADARrightfront < right_turn_threshold) { // sjew: keep going until threshold reached
            right_wall_follow_state = 2;
        }
        break;
    default:
        break;
    }

    // sjew: saturation
    if(turn_command > turn_command_saturation)
        turn_command = turn_command_saturation;
    if(turn_command < -turn_command_saturation)
        turn_command = -turn_command_saturation;


    // sjew: control output
    if(timecount < 4000) {
        u_left = curr_knob_enc; // sjew: eventually these will be determined by controls
        u_right = curr_knob_enc;
    } else {
        // sjew: error, accumulation, and velocity values
        e_steer = v_right - v_left + turn_command;
        e_left = v_ref - v_left + kp_turn * e_steer;
        e_right = v_ref - v_right - kp_turn * e_steer;

        // sjew: saving temp values for the integral wind up
        float tmp_accum_left =  accum_e_left + ((e_left + prev_e_left) / 2) * 0.001;
        float tmp_accum_right =  accum_e_right + ((e_right + prev_e_right) / 2) * 0.001;

        // sjew: control effort calc
        u_left = kp * e_left + ki * tmp_accum_left;
        u_right = kp * e_right + ki * tmp_accum_right;

        // sjew: friction comp
        if (v_left > 0.0){
            u_left = u_left + visc_pos* v_left + static_pos;
        }
        else{
            u_left = u_left + visc_neg* v_left + static_neg;
        }
        if (v_right > 0.0){
            u_right = u_right + visc_pos* v_right + static_pos;
        }
        else{
            u_right = u_right + visc_neg* v_right + static_neg;
        }

        // sjew: handle integral windup (only accumulate if u_left and u_right are within bounds)
        if(fabs(u_left) < 10.0) {
            accum_e_left = tmp_accum_left;
        }
        if(fabs(u_right) < 10.0) {
            accum_e_right = tmp_accum_right;
        }
        // sjew: always store prev error
        prev_e_left = e_left;
        prev_e_right = e_right;
    }

    // sjew: saturation limits for effort
    if(u_left < -10.0) {
        u_left = -10.0;
    }
    if(u_left > 10.0) {
        u_left = 10.0;
    }

    if(u_right < -10.0) {
        u_right = -10.0;
    }
    if(u_right > 10.0) {
        u_right = 10.0;
    }

    // sjew: set pwm outputs
    setEPWM1A(u_left);
    setEPWM2A(-u_right);
    // sjew: store current positions for next interrupt cycle to be used as previous positions
    prev_p_left = curr_p_left;
    prev_p_right = curr_p_right;

    if (newLinuxCommands == 1) {
        newLinuxCommands = 0;
//        v_ref = LinuxCommands[0];
//        turn_command = LinuxCommands[1];
        ref_right_wall = LinuxCommands[2];
        left_turn_Start_threshold = LinuxCommands[3];
        left_turn_Stop_threshold = LinuxCommands[4];
        Kp_right_wall = LinuxCommands[5];
        Kp_front_wall = LinuxCommands[6];
        front_turn_velocity = LinuxCommands[7];
        forward_velocity = LinuxCommands[8];
        turn_command_saturation = LinuxCommands[9];
        right_wall_far = 1.5 * ref_right_wall;
        right_turn_threshold = LinuxCommands[10];
    }


    if (NewLVData == 1) {
        NewLVData = 0;
        printLV1 = fromLVvalues[0];
        printLV2 = fromLVvalues[1];
    }

    if((timecount%250) == 0) {
        DataToLabView.floatData[0] = ROBOTps.x;
        DataToLabView.floatData[1] = ROBOTps.y;
        DataToLabView.floatData[2] = (float)timecount;
        DataToLabView.floatData[3] = ROBOTps.theta;
        DataToLabView.floatData[4] = ROBOTps.theta;
        DataToLabView.floatData[5] = ROBOTps.theta;
        DataToLabView.floatData[6] = ROBOTps.theta;
        DataToLabView.floatData[7] = ROBOTps.theta;
        LVsenddata[0] = '*';  // header for LVdata
        LVsenddata[1] = '$';
        for (i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }


    timecount++;



    //##############################################################################################################
    //
    // Restore registers saved:
    //
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_10 (use MINT12 and MG12_10 masks):
//
__interrupt void SWI2_MiddlePriority(void)     // RAM_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_10;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    if (LADARpingpong == 1) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_ping;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_ping;
            }
        }
        // sjew: track right back ladar
        LADARrightback = 19;
        for (LADARi = 11; LADARi <= 15 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightback) {
                LADARrightback = ladar_data[LADARi].distance_ping;
            }
        }

        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_ping*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_ping*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
    } else if (LADARpingpong == 0) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_pong;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_pong;
            }
        }
        // ew: track right back ladar vars
        LADARrightback = 19;
        for (LADARi = 11; LADARi <= 15 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightback) {
                LADARrightback = ladar_data[LADARi].distance_ping;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_pong*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_pong*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
}




    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_11 (use MINT12 and MG12_11 masks):
//
__interrupt void SWI3_LowestPriority(void)     // FLASH_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_11;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

void PostSWI1(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;
}

void PostSWI3(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx11 = 1;
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    // cut and paste all the SpibRegs initializations you found for part 3 here.
    // Also, do not forget to cut and paste the GPIO settings for
    // GPIO63, 64, 65, and 66, which are part of the SPIB setup.
    //-------------------------------------------------------------------------
    // SPI set up
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;

    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock
    // to create SCLK period

    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason

    SpibRegs.SPIFFTX.bit.SPIRST = 1; // Pull SPI FIFO out of reset,
                                       //SPI FIFO can resume transmitting or receiving.

    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL

    SpibRegs.SPIFFCT.bit.TXDLY = 0; // Set the delay between transmits to 0 spi clocks.

    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I do not think this is needed. Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; // Interrupt Level to 16 words or more received into FIFO causes
    //an interrupt. This is just the initial setting for the register. Will be changed below

    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    //Some code is given, most you have to fill yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers.
    // Remember, we are sending 16-bit transfers, so two registers at a time after the first 16-bit transfer
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = (0x1300 | 0x0000);
    // To address 00x14 write 0x00 and To address 00x15 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // To address 00x16 write 0x00 and To address 00x17 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // To address 00x18 write 0x00 and To address 00x19 write 0x13
    SpibRegs.SPITXBUF = (0x0000 | 0x0013);
    // To address 00x1A write 0x02 and To address 00x1B write 0x00
    SpibRegs.SPITXBUF = (0x0200 | 0x0000);
    // To address 00x1C write 0x08 and To address 00x1D write 0x06
    SpibRegs.SPITXBUF = (0x0800 | 0x0006);
    // To address 00x1E write 0x00 and To address 00x1F write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High

    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950
    // to get ready for the next transfer

    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    //Some code is given, most you have to fill yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = (0x2300 | 0x0000);
    // To address 00x24 write 0x40 and To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = (0x4000 | 0x008C);
    // To address 00x26 write 0x02 and To address 00x27 write 0x88
    SpibRegs.SPITXBUF = (0x0200 | 0x0088);
    // To address 00x28 write 0x0C and To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A);

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for the next transfer.

    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);

    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // sj offset calculations
    int16_t offset_a_x = -3255;
    offset_a_x = offset_a_x << 1;
    int16_t offset_a_x_H = (offset_a_x >> 8) & 0x00FF;
    int16_t offset_a_x_L = offset_a_x & 0x00FF;

    int16_t offset_a_y = 2653;
    offset_a_y = offset_a_y << 1;
    int16_t offset_a_y_H = offset_a_y >> 8;
    int16_t offset_a_y_L = offset_a_y & 0x00FF;

    int16_t offset_a_z = 3602;
    offset_a_z = offset_a_z << 1;
    int16_t offset_a_z_H = offset_a_z >> 8;
    int16_t offset_a_z_L = offset_a_z & 0x00FF;

    // The Remainder of this code is given to you.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | offset_a_x_H); // 0x7700 sj added the upper and lower offsets for x y and z
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | offset_a_x_L); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | offset_a_y_H); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | offset_a_y_L); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | offset_a_z_H); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | offset_a_z_L); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear the SPIB interrupt source, just in case any of the above initializations triggered it.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

