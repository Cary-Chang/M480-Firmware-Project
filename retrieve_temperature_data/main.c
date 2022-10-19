/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 15/09/16 5:25p $
 * @brief    NUC472/NUC442 SPI Driver Sample Code
 *           This is a SPI master mode demo and need to be tested with a slave device.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"
#include "gpio.h"

#define TEST_COUNT 16


#define R_CR0    (0x00)
#define W_CR0    (0x80)
#define CR0      (0x00)

#define R_CR1    (0x01)
#define W_CR1    (0x81)
#define CR1      (0x03)

#define R_MASK   (0x02)
#define W_MASK   (0x82)
#define MASK     (0xFF)

#define R_CJHF   (0x03)
#define W_CJHF   (0x83)

#define R_CJLF   (0x04)
#define W_CJLF   (0x84)

#define R_LTHFTH (0x05)
#define W_LTHFTH (0x85)

#define R_LTHFTL (0x06)
#define W_LTHFTL (0x86)

#define R_LTLFTH (0x07)
#define W_LTLFTH (0x87)

#define R_LTLFTL (0x08)
#define W_LTLFTL (0x88)

#define R_CJTO   (0x09)
#define W_CJTO   (0x89)

#define R_CJTH   (0x0A)
#define W_CJTH   (0x8A)

#define R_CJTL   (0x0B)
#define W_CJTL   (0x8B)

#define R_LTCBH  (0x0C)

#define R_LTCBM  (0x0D)

#define R_LTCBL  (0x0E)

#define R_SR     (0x0F)


#define CR0_50_60Hz_Enable  (1)
#define CR0_FAULTCLR_Enable (1 << 1)
#define CR0_FAULT_Enable    (1 << 2)
#define CR0_CJ_Enable       (1 << 3)
#define CR0_OCFAULT0_Enable (1 << 4)
#define CR0_OCFAULT1_Enable (1 << 5)
#define CR0_1SHOT_Enable    (1 << 6)
#define CR0_CMODE_Enable    (1 << 7)

#define CR1_TC_TYPE0_Enable   (1)
#define CR1_TC_TYPE1_Enable   (1 << 1)
#define CR1_TC_TYPE2_Enable   (1 << 2)
#define CR1_TC_TYPE3_Enable   (1 << 3)
#define CR1_TC_AVGSEL0_Enable (1 << 4)
#define CR1_TC_AVGSEL1_Enable (1 << 5)
#define CR1_TC_AVGSEL2_Enable (1 << 6)

#define MASK_Open_FAULT_Mask_Enable    (1)
#define MASK_OV_UV_FAULT_Mask_Enable   (1 << 1)
#define MASK_TC_Low_FAULT_Mask_Enable  (1 << 2)
#define MASK_TC_High_FAULT_Mask_Enable (1 << 3)
#define MASK_CJ_Low_FAULT_Mask_Enable  (1 << 4)
#define MASK_CJ_High_FAULT_Mask_Enable (1 << 5)

#define SR_OPEN_Mask      (1)
#define SR_OVUV_Mask      (1 << 1)
#define SR_TCLOW_Mask     (1 << 2)
#define SR_TCHIGH_Mask    (1 << 3)
#define SR_CJLOW_Mask     (1 << 4)
#define SR_CJHIGH_Mask    (1 << 5)
#define SR_TC_Range_Mask  (1 << 6)
#define SR_CJ_Range_Mask  (1 << 7)

uint32_t TemperatureData[4] = {0};
float Temperature;
float TemperatureArray[20] = {0};

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_84MHz_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* SPI0: GPE4=SS0, GPE3=MOSI0, GPE2=MISO0, GPE5=CLK */
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE2MFP_SPI0_MISO0 | SYS_GPE_MFPL_PE3MFP_SPI0_MOSI0 | SYS_GPE_MFPL_PE4MFP_SPI0_SS0 | SYS_GPE_MFPL_PE5MFP_SPI0_CLK);

    /* Lock protected registers */
    SYS_LockReg();
}

uint8_t SetupData[2] = {0};
uint8_t temp;
uint8_t ReceivedData[4];
volatile uint32_t count = 0;

void TMR0_IRQHandler(void)
{
    // Clear interrupt flag
    TIMER_ClearIntFlag(TIMER0);

    SPI_SET_SS0_LOW(SPI0);
    SPI_WRITE_TX(SPI0, R_LTCBH);
    SPI_WRITE_TX(SPI0, 0x00);
    SPI_WRITE_TX(SPI0, 0x00);
    SPI_WRITE_TX(SPI0, 0x00);
    while(SPI0->STATUS & SPI_STATUS_BUSY_Msk);
    
    SPI_SET_SS0_HIGH(SPI0);
    TemperatureData[0] = SPI_READ_RX(SPI0);
    TemperatureData[1] = SPI_READ_RX(SPI0);
    TemperatureData[2] = SPI_READ_RX(SPI0);
    TemperatureData[3] = SPI_READ_RX(SPI0);
   
    if((TemperatureData[1] & 0x80)==0)
    {
          Temperature = (((float)(TemperatureData[1] << 11 | TemperatureData[2] << 3 | TemperatureData[3] >> 5)) / 128.0);
    }
    else
    {
          TemperatureData[1] &= TemperatureData[1] & 0x7F;
          Temperature = -(((float)(TemperatureData[1] << 11 | TemperatureData[2] << 3 | TemperatureData[3] >> 5)) / 128.0);
    }
    TemperatureArray[count] = Temperature;
    count++;
    
}


int main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI0 as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 1MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_1, 8, 1000000);
    SPI_SET_SS0_HIGH(SPI0);

    /* Enable the automatic hardware slave select function. Select the SPI0_SS0 pin and configure as low-active. */
//    SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS_ACTIVE_LOW);

    SPI_TRIGGER(SPI0); /*Transfer control enabled*/

    /* Set Tx FIFO threshold, enable Tx FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_SetFIFOThreshold(SPI0, 4, 4); /*Set Tx FIFO threshold and Rx FIFO threshold configurations*/ //?//
//    SPI_SET_SUSPEND_CYCLE(SPI0, 4); /*Suspend interval = 4.5 SPICLK clock cycle*/
    SPI_EnableInt(SPI0, SPI_FIFO_TXTHIEN_MASK); /*Receive time-out interrupt enabled and TX FIFO threshold interrupt enabled*/
    
    SetupData[0] = W_CR0;
    SetupData[1] = (CR0 | CR0_CMODE_Enable | CR0_OCFAULT0_Enable);
//    SPI_ClearTxFIFO(SPI0);
    SPI_SET_SS0_LOW(SPI0);
    SPI_WRITE_TX(SPI0, SetupData[0]);
    SPI_WRITE_TX(SPI0, SetupData[1]);
    SPI_WRITE_TX(SPI0, 0x3);
    SPI_WRITE_TX(SPI0, 0x00);
    while(SPI0->STATUS & SPI_STATUS_BUSY_Msk);
    SPI_SET_SS0_HIGH(SPI0);
    
    SPI_ClearRxFIFO(SPI0);
    
    SPI_SET_SS0_LOW(SPI0);
    SPI_WRITE_TX(SPI0, R_CR0);
    SPI_WRITE_TX(SPI0, 0x00);
    SPI_WRITE_TX(SPI0, 0x00);
    SPI_WRITE_TX(SPI0, 0x00);
    while(SPI0->STATUS & SPI_STATUS_BUSY_Msk);

    SPI_SET_SS0_HIGH(SPI0);
    
    for(int i = 0; i < 4; i++)
      ReceivedData[i] = SPI_READ_RX(SPI0);
       
//    NVIC_EnableIRQ(SPI0_IRQn); /*Enable External Interrupt*/

    /*Initial Timer0 to periodic mode with 1Hz */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);

    /* Enable timer wake up system */
    TIMER_EnableWakeup(TIMER0);
    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    /* Start Timer0 counting */
    TIMER_Start(TIMER0);
    /* Unlock protected registers */
    SYS_UnlockReg();
    
    while(count<20);
    TIMER_DisableInt(TIMER0);
    
    while(1);
}

