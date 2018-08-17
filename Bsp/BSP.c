/** @FILE NAME:    BSP.c
 *  @DESCRIPTION:  This file for ...
 *
 *  Copyright (c) 2018 EES Ltd.
 *  All Rights Reserved This program is the confidential and proprietary
 *  product of EES Ltd. Any Unauthorized use, reproduction or transfer
 *  of this program is strictly prohibited.
 *
 *  @Author: HaiHoang
 *  @NOTE:   No Note at the moment
 *  @BUG:    No known bugs.
 *
 *<pre>
 *  MODIFICATION HISTORY:
 *
 *  Ver   Who       Date                Changes
 *  ----- --------- ------------------  ----------------------------------------
 *  1.00  HaiHoang  August 1, 2018      First release
 *
 *
 *</pre>
 ******************************************************************************/

/***************************** Include Files *********************************/
#include <DSP28x_Project.h>
#include <BSP.h>
#include <IQmathLib.h>
#include <DPlib.h>
#include <app.h>


/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/

extern __interrupt void cpu_timer0_isr(void);
extern __interrupt void cpu_timer1_isr(void);
extern __interrupt void cpu_timer2_isr(void);
extern __interrupt void adc_isr(void);
void Test_Pwm();
/************************** Variable Definitions *****************************/


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

#define _FLASH

void BSP_Init(void) {

    /*
     * SYSTEM
     */
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    InitFlash();
#endif
    InitSysCtrl();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    /*
     * GPIO
     */
    GPIO_Output_Init();

    /*
     * HW Timer
     */
    InitCpuTimers();
    TIMER_Cpu0_Init(TIMER0_PERIOD);
    //TIMER_Cpu1_Init(TIMER_BASE / App_Control_Freq);
    //TIMER_Cpu2_Init(TIMER_BASE / Inverter_GenSin_Freq);

    ADC_Init();
    UART_Init();
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void BSP_Start(void) {
    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM
}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param period in micro second
 *  @return Void.
 *  @note
 */

void TIMER_Cpu0_Init(uint32_t period) {
    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    EDIS;

    StopCpuTimer0();

    //InitCpuTimers();

#if (CPU_FRQ_60MHZ)
    ConfigCpuTimer(&CpuTimer0, 60, 1000000);
#endif
#if (CPU_FRQ_50MHZ)
    ConfigCpuTimer(&CpuTimer0, 50, 1000000);
#endif
#if (CPU_FRQ_40MHZ)
    ConfigCpuTimer(&CpuTimer0, 40, period);
#endif

    // Cpu timer0 on Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    IER |= M_INT1;

    StartCpuTimer0();
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param period in micro second
 *  @return Void.
 *  @note
 */

void TIMER_Cpu1_Init(uint32_t period) {
    EALLOW;
    PieVectTable.TINT1 = &cpu_timer1_isr;
    EDIS;

    StopCpuTimer1();

    //InitCpuTimers();

#if (CPU_FRQ_60MHZ)
    ConfigCpuTimer(&CpuTimer1, 60, 1000000);
#endif
#if (CPU_FRQ_50MHZ)
    ConfigCpuTimer(&CpuTimer1, 50, 1000000);
#endif
#if (CPU_FRQ_40MHZ)
    ConfigCpuTimer(&CpuTimer1, 40, period);
#endif

    // cpu timer 1 on interrupt 13 - not in any group
    IER |= M_INT13;

    StartCpuTimer1();
}


/*****************************************************************************/
/** @brief, init timer 2 interrupt, need call InitCpuTimer() before
 *
 *
 *  @param period in micro second
 *  @return Void.
 *  @note
 */

void TIMER_Cpu2_Init(uint32_t period) {
    EALLOW;
    PieVectTable.TINT2 = &cpu_timer2_isr;
    EDIS;

    StopCpuTimer2();

    //InitCpuTimers();

#if (CPU_FRQ_60MHZ)
    ConfigCpuTimer(&CpuTimer2, 60, 1000000);
#endif
#if (CPU_FRQ_50MHZ)
    ConfigCpuTimer(&CpuTimer2, 50, 1000000);
#endif
#if (CPU_FRQ_40MHZ)
    ConfigCpuTimer(&CpuTimer2, 40, period);
#endif

    IER |= M_INT14;

    StartCpuTimer2();
}





/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void UART_Init() {

    EALLOW;
    //
    // Enable pull-up for GPIO28 (SCIRXDA)
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;

    //
    // Enable pull-up for GPIO19 (SCIRXDA)
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;

    //
    // Enable pull-up for GPIO7  (SCIRXDA)
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;

    //
    // Disable pull-up for GPIO29 (SCITXDA)
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;

    //
    // Disable pull-up for GPIO18 (SCITXDA)
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;

    //
    // Disable pull-up for GPIO12 (SCITXDA)
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;

    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.
    // This will select asynch (no qualification) for the selected pins.
    //
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDA)
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;   // Asynch input GPIO7 (SCIRXDA)

    //
    // Configure SCI-A pins using GPIO regs
    // This specifies which of the possible GPIO pins will be SCI functional
    // pins.
    //
    //GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA
    //GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDA
    //GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;    // Configure GPIO7  for SCIRXDA

    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA
    //GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDA
    //GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;   // Configure GPIO12 for SCITXDA

    EDIS;

    // 1 stop bit,  No loopback, No parity, 8 char bits, async mode,
    // idle-line protocol
    //
    SciaRegs.SCICCR.all =0x0007;

    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
    SciaRegs.SCICTL1.all =0x0003;
    SciaRegs.SCICTL2.all =0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA =0;
    SciaRegs.SCICTL2.bit.RXBKINTENA =0;
    SciaRegs.SCIHBAUD    =0x0000;

    SciaRegs.SCICTL1.all =0x0023;       // Relinquish SCI from Reset

    //SciaRegs.SCILBAUD  =0x00A2;      // A2h = 19.2Kbaud @ LSPCLK = 100/4 MHz
    //SciaRegs.SCILBAUD  =0x0050;      // 50h = 38.4Kbaud @ LSPCLK = 100/4 MHz
    //SciaRegs.SCILBAUD = 0x0035;      // 35h = 57.6Kbaud @ LSPCLK = 100/4 MHz
    SciaRegs.SCILBAUD = SCI_PRD;        //0x001B;        // 1Bh = 115.2Kbaud @ LSPCLK = 100/4 MHz

    //SciaRegs.SCIFFTX.all=0xE040;      // ENable FIFO enhancement
    SciaRegs.SCIFFTX.all=0x8040;        // DISable FIFO enhancement
    SciaRegs.SCIFFRX.all=0x204f;
    SciaRegs.SCIFFCT.all=0x0;
    SciaRegs.SCIPRI.bit.SOFT=0x0;
    SciaRegs.SCIPRI.bit.FREE=0x1;

}

void UART_Send(int ch) {
    while (SciaRegs.SCICTL2.bit.TXRDY != 1);
    SciaRegs.SCITXBUF=ch;
}



/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

void SPI_Init() {

}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

void ADC_Init() {

    InitAdc();
    AdcOffsetSelfCal();

    EALLOW;             // This is needed to write to EALLOW protected register
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;      // Use CBU event as trigger
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;                // Generate pulse on 1st event
    EPwm1Regs.ETCLR.bit.SOCA    = 1;                // Clear SOCA flag

    //PieVectTable.ADCINT1 = &adc_isr;
    EDIS;      // This is needed to disable write to EALLOW protected registers

    //PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE

    ChSel[0] = 0;
    ChSel[1] = 1;
    ChSel[2] = 2;
    ChSel[3] = 3;
    ChSel[4] = 4;

    ChSel[5] = 8;
    ChSel[6] = 9;


//    TrigSel[0] = ADCTRIG_CPU_TINT0;
//    TrigSel[1] = ADCTRIG_CPU_TINT0;
//    TrigSel[2] = ADCTRIG_CPU_TINT0;
//    TrigSel[3] = ADCTRIG_CPU_TINT0;
//    TrigSel[4] = ADCTRIG_CPU_TINT0;
//    TrigSel[5] = ADCTRIG_CPU_TINT0;

    TrigSel[0] = ADCTRIG_EPWM1_SOCA;
    TrigSel[1] = ADCTRIG_EPWM1_SOCA;
    TrigSel[2] = ADCTRIG_EPWM1_SOCA;
    TrigSel[3] = ADCTRIG_EPWM1_SOCA;
    TrigSel[4] = ADCTRIG_EPWM1_SOCA;
    TrigSel[5] = ADCTRIG_EPWM1_SOCA;
    TrigSel[6] = ADCTRIG_EPWM1_SOCA;

    ADC_SocConfig(ChSel, TrigSel, ACQPS, ADC_NUM_CHAN_USED, 0);

}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

void CAP_Init() {

}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

void I2C_Init() {

}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

void GPIO_Output_Init() {
    EALLOW;

    ////////////////////////// OUTPUT SIGNAL //////////////////////////////////////
    /*
     * DRV_EN1 - GPIO7
     */
    // For now using PWM instead GPIO
//    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;     // 0=GPIO,  1=EPWM4B,  2=SCIRX-A,  3=Resv
//    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;      // 1=OUTput,  0=INput
//    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;    // uncomment if --> Set Low initially
    //GpioDataRegs.GPASET.bit.GPIO7 = 1;      // uncomment if --> Set High initially

    /*
     * DRV_EN2 - GPIO5
     */
//    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;     // 0=GPIO,  1=EPWM3B,  2=Resv,  3=ECAP1
//    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;      // 1=OUTput,  0=INput
//    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;    // uncomment if --> Set Low initially
    //GpioDataRegs.GPASET.bit.GPIO5 = 1;      // uncomment if --> Set High initially

    /*
     * FAN_ON - GPIO29
     */
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;    // 0=GPIO,  1=SCITX-A,  2=I2C-SCL,  3=TZ3
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;   // uncomment if --> Set Low initially
    //GpioDataRegs.GPASET.bit.GPIO29 = 1;     // uncomment if --> Set High initially

    /*
     * RUN - GPIO33
     */
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;    // 0=GPIO,  1=I2C-SCL,  2=SYNCO,  3=ADCSOCB
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;   // uncomment if --> Set Low initially
    //GpioDataRegs.GPBSET.bit.GPIO33 = 1;   // uncomment if --> Set High initially

     /*
      * INVERTER_MODE - GPIO32 - Control relay
      */
     GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;    // 0=GPIO,  1=I2C-SDA,  2=SYNCI,  3=ADCSOCA
     GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;     // 1=OUTput,  0=INput
     GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;   // uncomment if --> Set Low initially
     //GpioDataRegs.GPBSET.bit.GPIO32 = 1;   // uncomment if --> Set High initially

     /*
      * AC_IN - GPIO16 - Display ac available
      */
     GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;    // 0=GPIO,  1=SPISIMO-A,  2=Resv,  3=TZ2
     GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;     // 1=OUTput,  0=INput
     GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;   // uncomment if --> Set Low initially
     //GpioDataRegs.GPASET.bit.GPIO16 = 1;     // uncomment if --> Set High initially

     /*
      * UPS_MODE - GPIO17 - display this device is running
      */
     GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;    // 0=GPIO,  1=SPISOMI-A,  2=Resv,  3=TZ3
     GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;     // 1=OUTput,  0=INput
     GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;   // uncomment if --> Set Low initially
     //GpioDataRegs.GPASET.bit.GPIO17 = 1;     // uncomment if --> Set High initially

     /*
      * BATT_LOW - GPIO19 - warning battery low
      */
     GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    // 0=GPIO,  1=SPISTE-A,  2=SCIRX-A,  3=ECAP1
     GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     // 1=OUTput,  0=INput
     GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;   // uncomment if --> Set Low initially
     //GpioDataRegs.GPASET.bit.GPIO19 = 1;     // uncomment if --> Set High initially


    EDIS;
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */

void GPIO_Input_Init() {

    EALLOW;
    ////////////////////////// INPUT SIGNAL //////////////////////////////////////
    /*
     * INVERTER_MODE - GPIO32
     */
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;    // 0=GPIO,  1=I2C-SDA,  2=SYNCI,  3=ADCSOCA
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;      // 1=OUTput,  0=INput
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;   // uncomment if --> Set Low initially
    //GpioDataRegs.GPBSET.bit.GPIO32 = 1;     // uncomment if --> Set High initially

    /*
     * AC_IN - GPIO16
     */
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;    // 0=GPIO,  1=SPISIMO-A,  2=Resv,  3=TZ2
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;
    //GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;   // uncomment if --> Set Low initially
    //GpioDataRegs.GPASET.bit.GPIO16 = 1;     // uncomment if --> Set High initially

    /*
     * UPS_MODE - GPIO17
     */
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;    // 0=GPIO,  1=SPISOMI-A,  2=Resv,  3=TZ3
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;
    //GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;   // uncomment if --> Set Low initially
    //GpioDataRegs.GPASET.bit.GPIO17 = 1;     // uncomment if --> Set High initially

    /*
     * BATT_LOW - GPIO19
     */
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    // 0=GPIO,  1=SPISTE-A,  2=SCIRX-A,  3=ECAP1
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;
    //GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;   // uncomment if --> Set Low initially
    //GpioDataRegs.GPASET.bit.GPIO19 = 1;     // uncomment if --> Set High initially

    EDIS;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
EPwmRet PWM_2ChCntUpDownBoostCfg(struct EPWM_REGS* pwm, uint32_t freq,
                                 int16_t mode, int16_t phase, int16_t channel) {

    uint16_t period = (FTBCLK / freq);

    if(pwm == NULL)
        return PWM_ERROR_HANDLE;

    EALLOW;
    // Time Base SubModule Registers
    pwm->TBCTL.bit.PRDLD     = TB_IMMEDIATE;  // set Immediate load
    pwm->TBPRD               = period >> 1;              // PWM frequency = 1 / period
    pwm->TBPHS.half.TBPHS    = 0;
    pwm->TBCTR               = 0;

    pwm->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    pwm->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    pwm->TBCTL.bit.CLKDIV = TB_DIV1;

    if(mode == 1) // config as a Master
    {
     pwm->TBCTL.bit.PHSEN      = TB_DISABLE;
     pwm->TBCTL.bit.SYNCOSEL   = TB_CTR_ZERO; // sync "down-stream"
    }
    if(mode == 0) // config as a Slave (Note: Phase+2 value used to compensate for logic delay)
    {
     pwm->TBCTL.bit.PHSEN      = TB_ENABLE;
     pwm->TBCTL.bit.SYNCOSEL   = TB_SYNC_IN;

     if ((0 <= phase)&&(phase <= 2))
         pwm->TBPHS.half.TBPHS = (2-phase);
     else if (phase > 2)
         pwm->TBPHS.half.TBPHS = (period-phase+2);
    }
    if(mode == 3) // config as a Slave,
    {
     pwm->TBCTL.bit.PHSEN      = TB_ENABLE;
     pwm->TBCTL.bit.SYNCOSEL   = TB_SYNC_IN;
    }

    // Counter Compare Submodule Registers

    pwm->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    pwm->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    pwm->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    pwm->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    pwm->CMPA.half.CMPA = 0;    //pwm->TBPRD >> 2;              // set duty 0% initially
    pwm->CMPB = pwm->TBPRD;     // - pwm->CMPA.half.CMPA;

#if 0 // for ver 1
    // normal
    if(channel == 1) {
        pwm->AQCTLA.bit.PRD = AQ_SET;
        pwm->AQCTLA.bit.CBD = AQ_CLEAR;

        pwm->AQCTLB.bit.ZRO = AQ_SET;
        pwm->AQCTLB.bit.CAU = AQ_CLEAR;

//        pwm->AQCTLB.bit.PRD = AQ_SET;
//        pwm->AQCTLB.bit.CBD = AQ_CLEAR;
    } else {
        pwm->AQCTLA.bit.ZRO = AQ_SET;
        pwm->AQCTLA.bit.CAU = AQ_CLEAR;

//        pwm->AQCTLB.bit.ZRO = AQ_SET;
//        pwm->AQCTLB.bit.CAU = AQ_CLEAR;

        pwm->AQCTLB.bit.PRD = AQ_SET;
        pwm->AQCTLB.bit.CBD = AQ_CLEAR;
    }
#endif

#if 1 // for ver2
    // normal
    if(channel == 1) {
//        pwm->AQCTLA.bit.PRD = AQ_SET;
//        pwm->AQCTLA.bit.CBD = AQ_CLEAR;
//
//        pwm->AQCTLB.bit.ZRO = AQ_SET;
//        pwm->AQCTLB.bit.CAU = AQ_CLEAR;

//        pwm->AQCTLB.bit.PRD = AQ_SET;
//        pwm->AQCTLB.bit.CBD = AQ_CLEAR;
    } else {
        pwm->AQCTLA.bit.ZRO = AQ_SET;
        pwm->AQCTLA.bit.CAU = AQ_CLEAR;

//        pwm->AQCTLB.bit.ZRO = AQ_SET;
//        pwm->AQCTLB.bit.CAU = AQ_CLEAR;

        pwm->AQCTLB.bit.PRD = AQ_SET;
        pwm->AQCTLB.bit.CBD = AQ_CLEAR;
    }

#endif

#if 0
    // invert
    if(channel == 1) {
        pwm->AQCTLA.bit.PRD = AQ_CLEAR;
        pwm->AQCTLA.bit.CBD = AQ_SET;

//        pwm->AQCTLB.bit.ZRO = AQ_SET;
//        pwm->AQCTLB.bit.CAU = AQ_CLEAR;

//        pwm->AQCTLB.bit.PRD = AQ_SET;
//        pwm->AQCTLB.bit.CBD = AQ_CLEAR;
    } else {
        pwm->AQCTLA.bit.ZRO = AQ_CLEAR;
        pwm->AQCTLA.bit.CAU = AQ_SET;

//        pwm->AQCTLB.bit.ZRO = AQ_SET;
//        pwm->AQCTLB.bit.CAU = AQ_CLEAR;

//        pwm->AQCTLB.bit.PRD = AQ_SET;
//        pwm->AQCTLB.bit.CBD = AQ_CLEAR;
    }
#endif

    EDIS;

    return PWM_SUCCESS;
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
//EPwmRet PWM_2ChUpDownBoostSetDuty(struct EPWM_REGS* pwm, uint16_t duty) {
//    if(duty > pwm->TBPRD)
//        return PWM_ERROR_DUTY;
//
//    pwm->CMPA.half.CMPA = duty;
//    pwm->CMPB = pwm->TBPRD - duty;
//    return PWM_SUCCESS;
//}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
EPwmRet PWM_2ChCntUpHalfCfg(struct EPWM_REGS *pwm, uint32_t freq,
                            int16_t mode, int16_t phase) {

    uint16_t period = (FTBCLK / freq) - 1;

    if(pwm == NULL)
        return PWM_ERROR_HANDLE;
    if(phase > (int16_t)(period))
        return PWM_ERROR_PHASE;

    EALLOW;
    // Time Base SubModule Registers
    pwm->TBCTL.bit.CTRMODE          = TB_COUNT_UP;
    pwm->TBCTL.bit.PRDLD            = TB_IMMEDIATE;            //TB_SHADOW
    //pwm->CMPCTL.bit.SHDWAMODE       = CC_SHADOW;
    //pwm->CMPCTL.bit.LOADAMODE       = CC_CTR_PRD;
    pwm->TBCTL.bit.HSPCLKDIV        = TB_DIV1;
    pwm->TBCTL.bit.CLKDIV           = TB_DIV1;

    pwm->TBPRD                      = period;                // PWM frequency = 1 / period
    pwm->TBPHS.half.TBPHS           = 0;
    pwm->TBCTR                      = 0;

    if(mode == 1) // config as a Master
    {
        pwm->TBCTL.bit.PHSEN        = TB_DISABLE;   // not set phase
        pwm->TBCTL.bit.SYNCOSEL     = TB_CTR_ZERO;  // sync "down-stream"
    }

    //
    // config as a Slave (Note: Phase+2 value used to compensate
    // for logic delay)
    //
    if(mode == 0)
    {
        pwm->TBCTL.bit.PHSEN = TB_ENABLE;
        pwm->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
        //pwm->TBPHS.half.TBPHS = (phase-2);
        pwm->TBPHS.half.TBPHS = (phase);
    }

    // Counter Compare Submodule Registers
    pwm->CMPA.half.CMPA = 0;                 // set duty A 0% initially
    pwm->CMPB = 0;                           // set duty B 0% initially

    // Action Qualifier SubModule Registers
    pwm->AQCTLA.bit.ZRO = AQ_SET;
    pwm->AQCTLA.bit.CAU = AQ_CLEAR;

    pwm->AQCTLB.bit.ZRO = AQ_SET;
    pwm->AQCTLB.bit.CBU = AQ_CLEAR;


    EDIS;

    return PWM_SUCCESS;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
EPwmRet PWM_2ChCntUpSetDutyHalf(struct EPWM_REGS* pwm, uint16_t channel, uint16_t duty) {
    if(pwm == NULL) {
        return PWM_ERROR_HANDLE;
    }
    if(duty > pwm->TBPRD) {
        return PWM_ERROR_DUTY;
    }

    if(channel == 1) {
        pwm->CMPA.half.CMPA = duty;
    } else {
        pwm->CMPB = duty;
    }

    return PWM_SUCCESS;
}

/*****************************************************************************/
/** @brief
 *         _______     _______
 *_________|     |_____|     |__________
 *  @param
 *  @return Void.
 *  @note
 */

#if 1

EPwmRet PWM_2ChCntUpDownFullCfg(struct EPWM_REGS *pwm, uint32_t freq, int16_t mode, int16_t phase) {

    uint16_t period = (FTBCLK / freq);

    if(pwm == NULL)
        return PWM_ERROR_HANDLE;
    if(phase > (int16_t)(period))
        return PWM_ERROR_PHASE;

    EALLOW;
    // Time Base SubModule Registers
    pwm->TBCTL.bit.PRDLD     = TB_IMMEDIATE;  // set Immediate load
    pwm->TBPRD               = period >> 1;              // PWM frequency = 1 / period
    pwm->TBPHS.half.TBPHS    = 0;
    pwm->TBCTR               = 0;

    pwm->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    pwm->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    pwm->TBCTL.bit.CLKDIV = TB_DIV1;

     if(mode == 1) // config as a Master
     {
         pwm->TBCTL.bit.PHSEN      = TB_DISABLE;
         pwm->TBCTL.bit.SYNCOSEL   = TB_CTR_ZERO; // sync "down-stream"
     }
     if(mode == 0) // config as a Slave (Note: Phase+2 value used to compensate for logic delay)
     {
         pwm->TBCTL.bit.PHSEN      = TB_ENABLE;
         pwm->TBCTL.bit.SYNCOSEL   = TB_SYNC_IN;

         if ((0 <= phase)&&(phase <= 2))
             pwm->TBPHS.half.TBPHS = (2-phase);
         else if (phase > 2)
             pwm->TBPHS.half.TBPHS = (period-phase+2);
     }
     if(mode == 3) // config as a Slave,
     {
         pwm->TBCTL.bit.PHSEN      = TB_ENABLE;
         pwm->TBCTL.bit.SYNCOSEL   = TB_SYNC_IN;
     }

     // Counter Compare Submodule Registers
     pwm->CMPA.half.CMPA = 0;              // set duty 0% initially
     pwm->CMPB = 0;
     pwm->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
     pwm->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

     pwm->CMPB = 0;                        // set duty 0% initially
     pwm->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
     pwm->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

     pwm->AQCTLA.bit.CAU = AQ_CLEAR;
     pwm->AQCTLA.bit.CAD = AQ_SET;

//     pwm->AQCTLB.bit.CBU = AQ_CLEAR;
//     pwm->AQCTLB.bit.CBD = AQ_SET;

     pwm->AQCTLB.bit.CAU = AQ_SET;
     pwm->AQCTLB.bit.CAD = AQ_CLEAR;


     // Action Qualifier SubModule Registers
     pwm->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; //Enable Dead-band module
     pwm->DBCTL.bit.POLSEL  = DB_ACTV_HIC;  //Active Hi complementary

     pwm->DBFED = 20;    //DeadBandFED;
     pwm->DBRED = 20;    //DeadBandRED;

    EDIS;

    return PWM_SUCCESS;
}

/*****************************************************************************/
/** @brief set up duty and AQ for pwm A-B
 *
 *
 *  @param channel:
 *      if channel = 1, set active channel is PWMA, passive is PWMB
 *      AQ-A: ZRO: AQ_SET, CAU: AQ_CLEAR
 *      AQ-B: CBU: AQ_SET, CBD: AQ_CLEAR
 *      else if channel =2, set active channel is PWMB, passive is PWMA
 *      AQ-B: ZRO: AQ_SET, CBU: AQ_CLEAR
 *      AQ-B: CAU: AQ_SET, CAD: AQ_CLEAR
 *  @return Void.
 *  @note
 */
EPwmRet PWM_2ChCntUpSetDutyFull(struct EPWM_REGS* pwm, uint16_t channel, uint16_t CMPA, uint16_t CMPB, uint16_t updateAQ) {
    if(pwm == NULL) {
        return PWM_ERROR_HANDLE;
    }

    if(CMPB > pwm->TBPRD) {
        return PWM_ERROR_DUTY;
    }

    pwm->CMPB = CMPB;
    pwm->CMPA.half.CMPA = CMPA;
    if(channel == 1) {
        if(updateAQ > 0) {
            pwm->AQCTLA.all = 0;
            pwm->AQCTLA.bit.CBU = AQ_SET;
            pwm->AQCTLA.bit.CBD = AQ_CLEAR;
            pwm->AQCTLB.all = 0;
            pwm->AQCTLB.bit.CAD = AQ_SET;
            pwm->AQCTLB.bit.CAU = AQ_CLEAR;
        }
    } else { // active is PWM B, passive is PWMA
        if(updateAQ > 0) {
            pwm->AQCTLB.all = 0;
            pwm->AQCTLB.bit.CBU = AQ_SET;
            pwm->AQCTLB.bit.CBD = AQ_CLEAR;
            pwm->AQCTLA.all = 0;
            pwm->AQCTLA.bit.CAD = AQ_SET;
            pwm->AQCTLA.bit.CAU = AQ_CLEAR;

        }
    }

    return PWM_SUCCESS;
}

#endif




/*****************************************************************************/
/** @brief ADC module with number channel start of convertion config
 *
 *Description: ADC configuration to support up to 16 conversions on
              Start of Conversion(SOC) based ADCs (type 3) found on F2802x
              and F3803x devices.  Independent selection of Channel, Trigger
              and acquisition window using ChSel[],TrigSel[] and ACQPS[].

 Dependencies:  Assumes the {DeviceName}-usDelay.asm is inlcuded in the
                project
 Version:       3.0

 Target:   TMS320F2802x(PiccoloA), TMS320F2803x(PiccoloB),

 The function call is: void ADC_SOC_CNF(int ChSel[], int Trigsel[],
                                        int ACQPS[], int IntChSel, int mode)

 Function arguments defined as:

 ChSel[]  = Channel selection made via a channel # array passed as an
            argument
 TrigSel[]= Source for triggering conversion of a channel,
            selection made via a trigger # array passed as argument
 ACQPS[]  = SOCx Acquisition Prescale. Controls the sample and hold window for SOCx. Minimum value
            allowed is 6
             AcqWidth is the S/H aperture in #ADCCLKS, # array passed as
            argument

 IntChSel = Channel number that would trigger an ADC Interrupt 1 on
            completion(EOC) if no channel triggers ADC interrupt pass
            value > 15
 Mode     = Operating mode:     0 = Start / Stop mode, needs trigger event
                                1 = Continuous mode, no trigger needed
                                2 = CLA Mode, start stop mode with auto clr
                                    INT Flag
 *
 *  @param
 *  @return Void.
 *  @note
 *
 *  int16    ChSel[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int16    TrigSel[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int16    ACQPS[16]={6,25,25,25,6,6,6,6,6,6,6,6,6,6,6,6};
 */



void ADC_SocConfig(int ChSel[], int Trigsel[],
                   int ACQPS[], int IntChSel, int mode) {

    (void)IntChSel;
    extern void DSP28x_usDelay(Uint32 Count);

    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL   = 0;    // Select reference 0-internal;1-external
    AdcRegs.ADCCTL1.bit.ADCBGPWD    = 1;    // Power up band gap
    AdcRegs.ADCCTL1.bit.ADCREFPWD   = 1;    // Power up reference
    AdcRegs.ADCCTL1.bit.ADCPWDN     = 1;    // Power up rest of ADC
    AdcRegs.ADCCTL1.bit.ADCENABLE   = 1;    // Enable ADC

    DSP28x_usDelay(1000);         // Delay before converting ADC channels

    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcRegs.ADCSOC0CTL.bit.ACQPS = ACQPS[0];
    AdcRegs.ADCSOC1CTL.bit.ACQPS = ACQPS[1];
    AdcRegs.ADCSOC2CTL.bit.ACQPS = ACQPS[2];
    AdcRegs.ADCSOC3CTL.bit.ACQPS = ACQPS[3];
    AdcRegs.ADCSOC4CTL.bit.ACQPS = ACQPS[4];
    AdcRegs.ADCSOC5CTL.bit.ACQPS = ACQPS[5];
    AdcRegs.ADCSOC6CTL.bit.ACQPS = ACQPS[6];
    AdcRegs.ADCSOC7CTL.bit.ACQPS = ACQPS[7];
    AdcRegs.ADCSOC8CTL.bit.ACQPS = ACQPS[8];
    AdcRegs.ADCSOC9CTL.bit.ACQPS = ACQPS[9];
    AdcRegs.ADCSOC10CTL.bit.ACQPS = ACQPS[10];
    AdcRegs.ADCSOC11CTL.bit.ACQPS = ACQPS[11];
    AdcRegs.ADCSOC12CTL.bit.ACQPS = ACQPS[12];
    AdcRegs.ADCSOC13CTL.bit.ACQPS = ACQPS[13];
    AdcRegs.ADCSOC14CTL.bit.ACQPS = ACQPS[14];
    AdcRegs.ADCSOC15CTL.bit.ACQPS = ACQPS[15];

    // ???
    AdcRegs.INTSEL1N2.bit.INT1SEL = 1;  //IntChSel; // IntChSel causes ADCInterrupt 1

    if (mode == 0)        // Start-Stop conv mode
    {
        // clear ADCINT1 flag to begin a new set of conversions
        AdcRegs.ADCINTFLG.bit.ADCINT1 = 0;  // clear interrupt flag for ADCINT1
        AdcRegs.INTSEL1N2.bit.INT1E     = 1;    // Enabled ADCINT1
        AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;    // Disable ADCINT1 Continuous mode
        AdcRegs.ADCINTSOCSEL1.all=0x0000;  // No ADCInterrupt will trigger SOCx
        AdcRegs.ADCINTSOCSEL2.all=0x0000;
    }

    if (mode == 1)        // Continuous conv mode
    {
        AdcRegs.INTSEL1N2.bit.INT1CONT = 1;   // set ADCInterrupt 1 to auto clr
        // ADCInterrupt 1 will trigger SOCx, TrigSel is ignored
        AdcRegs.ADCINTSOCSEL1.all=0xFF;
        AdcRegs.ADCINTSOCSEL2.all=0xFF;
    }

    if (mode == 2)        // CLA mode, Start Stop ADC with auto clr ADC Flag
    {
        AdcRegs.ADCINTFLG.bit.ADCINT1 = 0;  // clear interrupt flag for ADCINT1
        AdcRegs.INTSEL1N2.bit.INT1CONT = 1; // set ADCInterrupt 1 to auto clr
        AdcRegs.ADCINTSOCSEL1.all=0x0000;  // No ADCInterrupt will trigger SOCx
        AdcRegs.ADCINTSOCSEL2.all=0x0000;
    }

    //
    // Select the channel to be converted when SOCx is received
    //
    AdcRegs.ADCSOC0CTL.bit.CHSEL= ChSel[0];
    AdcRegs.ADCSOC1CTL.bit.CHSEL= ChSel[1];
    AdcRegs.ADCSOC2CTL.bit.CHSEL= ChSel[2];
    AdcRegs.ADCSOC3CTL.bit.CHSEL= ChSel[3];
    AdcRegs.ADCSOC4CTL.bit.CHSEL= ChSel[4];
    AdcRegs.ADCSOC5CTL.bit.CHSEL= ChSel[5];
    AdcRegs.ADCSOC6CTL.bit.CHSEL= ChSel[6];
    AdcRegs.ADCSOC7CTL.bit.CHSEL= ChSel[7];
    AdcRegs.ADCSOC8CTL.bit.CHSEL= ChSel[8];
    AdcRegs.ADCSOC9CTL.bit.CHSEL= ChSel[9];
    AdcRegs.ADCSOC10CTL.bit.CHSEL= ChSel[10];
    AdcRegs.ADCSOC11CTL.bit.CHSEL= ChSel[11];
    AdcRegs.ADCSOC12CTL.bit.CHSEL= ChSel[12];
    AdcRegs.ADCSOC13CTL.bit.CHSEL= ChSel[13];
    AdcRegs.ADCSOC14CTL.bit.CHSEL= ChSel[14];
    AdcRegs.ADCSOC15CTL.bit.CHSEL= ChSel[15];

    AdcRegs.ADCSOC0CTL.bit.TRIGSEL= Trigsel[0];
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL= Trigsel[1];
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL= Trigsel[2];
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL= Trigsel[3];
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL= Trigsel[4];
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL= Trigsel[5];
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL= Trigsel[6];
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL= Trigsel[7];
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL= Trigsel[8];
    AdcRegs.ADCSOC9CTL.bit.TRIGSEL= Trigsel[9];
    AdcRegs.ADCSOC10CTL.bit.TRIGSEL= Trigsel[10];
    AdcRegs.ADCSOC11CTL.bit.TRIGSEL= Trigsel[11];
    AdcRegs.ADCSOC12CTL.bit.TRIGSEL= Trigsel[12];
    AdcRegs.ADCSOC13CTL.bit.TRIGSEL= Trigsel[13];
    AdcRegs.ADCSOC14CTL.bit.TRIGSEL= Trigsel[14];
    AdcRegs.ADCSOC15CTL.bit.TRIGSEL= Trigsel[15];
    EDIS;

    AdcRegs.ADCSOCFRC1.all = 0xFFFF;        // kick-start ADC
}

#if 0
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
EPwmRet PWM_InverterConfigTripZone(struct EPWM_REGS *pwm) {

    EALLOW;

    pwm->DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP1OUT;               // DCAH = Comparator 1 output

    pwm->TZDCSEL.bit.DCAEVT1        = TZ_DCAH_HI;              // TZ_DCAH_HI;           // DCAEVT1 =  DCAH high
                                                               // (will become active as Comparator output goes high)
    pwm->DCACTL.bit.EVT1SRCSEL      = DC_EVT1;                 // DCAEVT1 = DCAEVT1 (not filtered)
    pwm->DCACTL.bit.EVT1FRCSYNCSEL  = DC_EVT_ASYNC;            // Take async path

    pwm->TZCTL.bit.TZA              = TZ_FORCE_LO;             // EPWM1A will go high
    pwm->TZCTL.bit.TZB              = TZ_FORCE_LO;             // EPWM1B will go low

    pwm->TZSEL.bit.DCAEVT1          = 1;

    pwm->TZEINT.bit.DCAEVT1         = 1;
    pwm->TZEINT.bit.OST             = 1;

    EDIS;

    return PWM_SUCCESS;
}

#endif

#if 1

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
EPwmRet PWM_InverterConfigTripZone(struct EPWM_REGS *pwm) {

    EALLOW;

    pwm->DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP1OUT;               // DCAH = Comparator 1 output

    pwm->TZDCSEL.bit.DCAEVT2        = TZ_DCAH_HI;              // TZ_DCAH_HI;           // DCAEVT1 =  DCAH high
                                                               // (will become active as Comparator output goes high)
    pwm->DCACTL.bit.EVT2SRCSEL      = DC_EVT2;                 // DCAEVT1 = DCAEVT1 (not filtered)
    pwm->DCACTL.bit.EVT2FRCSYNCSEL  = DC_EVT_ASYNC;            // Take async path

    pwm->TZCTL.bit.TZA              = TZ_FORCE_LO;             // EPWM1A will go high
    pwm->TZCTL.bit.TZB              = TZ_FORCE_LO;             // EPWM1B will go low

    pwm->TZSEL.bit.DCAEVT2          = 1;

    pwm->TZEINT.bit.DCAEVT2         = 1;
    pwm->TZEINT.bit.CBC             = 1;

    EDIS;

    return PWM_SUCCESS;
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
EPwmRet PWM_BoosterConfigTripZone(struct EPWM_REGS *pwm) {

    EALLOW;

    pwm->DCTRIPSEL.bit.DCAHCOMPSEL  = DC_COMP2OUT;               // DCAH = Comparator 1 output

    pwm->TZDCSEL.bit.DCAEVT2        = TZ_DCAH_HI;              // TZ_DCAH_HI;           // DCAEVT1 =  DCAH high
                                                               // (will become active as Comparator output goes high)
    pwm->DCACTL.bit.EVT2SRCSEL      = DC_EVT2;                 // DCAEVT1 = DCAEVT1 (not filtered)
    pwm->DCACTL.bit.EVT2FRCSYNCSEL  = DC_EVT_ASYNC;            // Take async path

    pwm->TZCTL.bit.TZA              = TZ_FORCE_LO;             // EPWM1A will go high
    pwm->TZCTL.bit.TZB              = TZ_FORCE_LO;             // EPWM1B will go low

    pwm->TZSEL.bit.DCAEVT2          = 1;

    pwm->TZEINT.bit.DCAEVT2         = 1;
    pwm->TZEINT.bit.CBC             = 1;

    EDIS;

    return PWM_SUCCESS;
}


#endif


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void COMP_InverterTripZoneConfig(struct COMP_REGS* cmp, uint16_t value) {
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD    = 1;
    GpioCtrlRegs.AIOMUX1.bit.AIO2   = 2;          // Configure AIO2 (disable) for CMP1A (analog input) operation

    cmp->COMPCTL.bit.COMPDACEN      = 1;        // Power up Comparator locally - enable compare blocks
    cmp->COMPCTL.bit.COMPSOURCE     = 0;        // Connect the inverting input to internal DAC
    cmp->COMPCTL.bit.SYNCSEL        = 0;        //Asynchronous version of Comparator output is passed
    cmp->COMPCTL.bit.CMPINV         = 0;        //Output of comparator is passed
    cmp->DACVAL.bit.DACVAL          = value;
    EDIS;
}
/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void COMP_BoosterTripZoneConfig(struct COMP_REGS* cmp, uint16_t value) {
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD    = 1;
    GpioCtrlRegs.AIOMUX1.bit.AIO4   = 2;          // Configure AIO4 (disable) for CMP2A (analog input) operation

    cmp->COMPCTL.bit.COMPDACEN      = 1;        // Power up Comparator locally - enable compare blocks
    cmp->COMPCTL.bit.COMPSOURCE     = 0;        // Connect the inverting input to internal DAC
    cmp->COMPCTL.bit.SYNCSEL        = 0;        //Asynchronous version of Comparator output is passed
    cmp->COMPCTL.bit.CMPINV         = 0;        //Output of comparator is passed
    cmp->DACVAL.bit.DACVAL          = value;

    EDIS;
}

/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void UART_SendStr(char *msg) {
    while(*msg) {
        UART_Send(*msg);
        msg++;
    }
}


/*****************************************************************************/
/** @brief
 *
 *
 *  @param
 *  @return Void.
 *  @note
 */
void UART_SendNum(int num) {
    uint8_t buf[7];
    buf[0] = num / 10000 + 0x30;
    buf[1] = ((num % 10000) / 1000) + 0x30;
    buf[2] = ((num % 1000) / 100) + 0x30;
    buf[3] = ((num % 100) / 10) + 0x30;
    buf[4] = (num % 10) + 0x30;
    buf[5] = ' ';
    buf[6] = 0;
    UART_SendStr((char*)buf);
}





