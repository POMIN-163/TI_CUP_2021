/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:03
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-07 15:59:18
 * @Description:
 */
#include "tc_adc.h"

uint16_t srcBuffer[1024];
uint32_t adc_fs = 1024000;
volatile bool bgetConvStatus = false;

/* The control table used by the uDMA controller.  This table must be aligned
 * to a 1024 byte boundary. */
#if defined(__ICCARM__)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(__TI_ARM__)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__((aligned(1024)));
#endif

void adc_init(void) {
    /* Enable the clock to GPIO Port E and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))) {
    }

    /* Configure PE0-PE3 as ADC input channel */
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    /* Enable the clock to ADC-0 and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))) {
    }

    /* Configure Sequencer 2 to sample the analog channel : AIN0-AIN3. The
     * end of conversion and interrupt generation is set for AIN3 */
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                                 ADC_CTL_END);

    /* Enable sample sequence 2 with a timer signal trigger.  Sequencer 2
     * will do a single sample when the timer generates a trigger on timeout*/
    MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 3);

    /* Clear the interrupt status flag before enabling. This is done to make
     * sure the interrupt flag is cleared before we sample. */
    MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS3);
    MAP_ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS3);

    /* Enable the DMA request from ADC0 Sequencer 2 */
    MAP_ADCSequenceDMAEnable(ADC0_BASE, 3);

    /* Since sample sequence 2 is now configured, it must be enabled. */
    MAP_ADCSequenceEnable(ADC0_BASE, 3);

    /* Enable the Interrupt generation from the ADC-0 Sequencer */
    MAP_IntEnable(INT_ADC0SS3);

    /* Enable the DMA and Configure Channel for TIMER0A for Ping Pong mode of
     * transfer */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA))) {
    }

    MAP_uDMAEnable();

    /* Point at the control table to use for channel control structures. */
    MAP_uDMAControlBaseSet(pui8ControlTable);

    /* Map the ADC0 Sequencer 2 DMA channel */
    MAP_uDMAChannelAssign(UDMA_CH17_ADC0_3);

    /* Put the attributes in a known state for the uDMA ADC0 Sequencer 2
     * channel. These should already be disabled by default. */
    MAP_uDMAChannelAttributeDisable(UDMA_CH17_ADC0_3,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

    /* Configure the control parameters for the primary control structure for
     * the ADC0 Sequencer 2 channel. The primary control structure is used for
     * copying the data from ADC0 Sequencer 2 FIFO to srcBuffer. The transfer
     * data size is 16 bits and the source address is not incremented while
     * the destination address is incremented at 16-bit boundary.
     */
    MAP_uDMAChannelControlSet(UDMA_CH17_ADC0_3 | UDMA_PRI_SELECT,
                              UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                              UDMA_ARB_1);

    /* Set up the transfer parameters for the ADC0 Sequencer 2 primary control
     * structure. The mode is Basic mode so it will run to completion. */
    MAP_uDMAChannelTransferSet(UDMA_CH17_ADC0_3 | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void*)&ADC0->SSFIFO3, (void*)&srcBuffer,
                               sizeof(srcBuffer) / 2);

    /* The uDMA ADC0 Sequencer 2 channel is primed to start a transfer. As
     * soon as the channel is enabled and the Timer will issue an ADC trigger,
     * the ADC will perform the conversion and send a DMA Request. The data
     * transfers will begin. */
    MAP_uDMAChannelEnable(UDMA_CH17_ADC0_3);

    /* Enable Timer-0 clock and configure the timer in periodic mode with
     * a frequency of 1 KHz. Enable the ADC trigger generation from the
     * timer-0. */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))) {
    }

    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (/* g_ui32SysClock / adc_fs */ 5));
    MAP_TimerADCEventSet(TIMER0_BASE, TIMER_ADC_TIMEOUT_A);
    MAP_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}
/**
 * @brief 一阶滤波
 *
 * @param com 采样的原始数值
 * @return uint16_t 经过一阶滤波后的采样值
**/
uint16_t lowV(uint16_t com) {
    static unsigned int iLastData;    // 上一次值
    unsigned int iData;               // 本次计算值
    float dPower = 0.1;               // 滤波系数
    iData = (com * dPower) + (1 - dPower) * iLastData; // 计算
    iLastData = iData;                // 存贮本次数据
    return iData;                     // 返回数据
}

uint32_t adc_read(void) {
    uint8_t n_1024 = BUFF_SIZE / 1024;
    for (size_t i = 0; i < n_1024; i++) {
        /* Wait for the conversion to complete */
        while (!bgetConvStatus);
        bgetConvStatus = false;
        for (size_t j = 0; j < 1024; j++) {
            adc_buff[i * 1024 + j] = srcBuffer[j];
        }
    }
    // for (size_t i = 0; i < BUFF_SIZE; i++) {
    //     adc_buff[i] = srcBuffer[i];
    // }
    return 0;
}

void adc_write(uint32_t value) {

}

void ADC0SS3_IRQHandler(void) {
    uint32_t getIntStatus;

    /* Get the interrupt status from the ADC */
    getIntStatus = MAP_ADCIntStatusEx(ADC0_BASE, true);

    /* If the interrupt status for Sequencer-2 is set the
     * clear the status and read the data */
    if ((getIntStatus & ADC_INT_DMA_SS3) == ADC_INT_DMA_SS3) {
        /* Clear the ADC interrupt flag. */
        MAP_ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS3);

        /* Reconfigure the channel control structure and enable the channel */
        MAP_uDMAChannelTransferSet(UDMA_CH17_ADC0_3 | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void*)&ADC0->SSFIFO3, (void*)&srcBuffer,
                                   sizeof(srcBuffer) / 2);

        MAP_uDMAChannelEnable(UDMA_CH17_ADC0_3);

        /* Set conversion flag to true */
        bgetConvStatus = true;
    }
}
