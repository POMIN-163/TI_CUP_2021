/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdbool.h>

#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufMSP432E4.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/drivers/timer/TimerMSP432E4.h>

#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>

/* driverlib header files */
#include <ti/devices/msp432e4/driverlib/driverlib.h>

#define pinConfigChannel(config) (((config) >> 16) & 0x10F)
#define pinConfigPort(config) (((config << 4) & 0x000FF000) | 0x40000000)
#define pinConfigPin(config) ((config) & 0xFF)

/* Sequencer fifo address offsets used to calculate address for DMA*/
#define SSFIFO_BASE 0x48
#define SSFIFO_OFFSET 0x20

/* ADC Resolution */
#define ADCRESOLUTION 12

/* Maximum number of samples that can be taken with an SS FIFO */
#define MAX_SSFIFO_SIZE 8

void ADCBufMSP432E4_close(ADCBuf_Handle handle);
int_fast16_t ADCBufMSP432E4_control(ADCBuf_Handle handle, uint_fast16_t cmd,
    void * arg);
void ADCBufMSP432E4_init(ADCBuf_Handle handle);
ADCBuf_Handle ADCBufMSP432E4_open(ADCBuf_Handle handle,
    const ADCBuf_Params *params);
int_fast16_t ADCBufMSP432E4_convert(ADCBuf_Handle handle,
    ADCBuf_Conversion *conversions, uint_fast8_t channelCount);
int_fast16_t ADCBufMSP432E4_convertCancel(ADCBuf_Handle handle);
uint_fast8_t ADCBufMSP432E4_getResolution(ADCBuf_Handle handle);
int_fast16_t ADCBufMSP432E4_adjustRawValues(ADCBuf_Handle handle,
    void *sampleBuffer, uint_fast16_t sampleCount, uint32_t adcChannel);
int_fast16_t ADCBufMSP432E4_convertAdjustedToMicroVolts(ADCBuf_Handle handle,
    uint32_t adcChannel, void *adjustedSampleBuffer,
    uint32_t outputMicroVoltBuffer[], uint_fast16_t sampleCount);
static void initHw(ADCBufMSP432E4_Object *object,
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs);
static void configDMA(ADCBufMSP432E4_Object *object,
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs, ADCBuf_Conversion *conversions);
static void completeConversion(ADCBuf_Handle handle, uint8_t sequencer);
static int_fast16_t primeConvert(ADCBufMSP432E4_Object *object,
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs, ADCBuf_Conversion *conversions,
    uint_fast8_t channelCount);
static void blockingConvertCallback(ADCBuf_Handle handle,
    ADCBuf_Conversion *conversion, void *activeADCBuffer,
    uint32_t completedChannel, int_fast16_t status);
static void ADCBufMSP432E4_hwiIntFxn(uintptr_t arg);
static void ADCBufMSP423E4_noDMAhwiIntFxn(uintptr_t arg);
static uint_fast16_t getPowerResourceId(uint32_t baseAddress);

/* Global mutex ensuring exclusive access to ADC during conversions */
static SemaphoreP_Handle globalMutex[2] = {NULL, NULL};

/* Table of the maximum channels allowed per sequencer */
static const uint8_t adcMaxSamples[ADCBufMSP432E4_SEQUENCER_COUNT] = {8, 4, 4, 1};

/*
 *  timerSettings is used to keep track of the state
 *  of the timer that is used in timer mode.
 */
static struct {
    uint32_t timerSource;
    uint32_t timerInstances;
    uint32_t timerFrequency;
    bool     isInitialized;
} timerSettings;

/* Table of ADC sequencer interrupt vectors */
static const uint8_t adcInterrupts[2][4] = {
    {INT_ADC0SS0, INT_ADC0SS1, INT_ADC0SS2, INT_ADC0SS3},
    {INT_ADC1SS0, INT_ADC1SS1, INT_ADC1SS2, INT_ADC1SS3}
};

/* Table of ADC sequencer UDMA channels */
static const uint32_t udmaChannels[2][4] = {
    {UDMA_CH14_ADC0_0, UDMA_CH15_ADC0_1, UDMA_CH16_ADC0_2, UDMA_CH17_ADC0_3},
    {UDMA_CH24_ADC1_0, UDMA_CH25_ADC1_1, UDMA_CH26_ADC1_2, UDMA_CH27_ADC1_3}
 };

/* ADC function table for ADCBufMSP432E4 implementation */
const ADCBuf_FxnTable ADCBufMSP432E4_fxnTable = {
    ADCBufMSP432E4_close,
    ADCBufMSP432E4_control,
    ADCBufMSP432E4_init,
    ADCBufMSP432E4_open,
    ADCBufMSP432E4_convert,
    ADCBufMSP432E4_convertCancel,
    ADCBufMSP432E4_getResolution,
    ADCBufMSP432E4_adjustRawValues,
    ADCBufMSP432E4_convertAdjustedToMicroVolts
};

/*
 *  ======== blockingConvertCallback ========
 */
static void blockingConvertCallback(ADCBuf_Handle handle,
    ADCBuf_Conversion *conversion, void *activeADCBuffer,
    uint32_t completedChannel, int_fast16_t status)
{
    ADCBufMSP432E4_Object *object = handle->object;

    /* Indicate transfer complete */
    SemaphoreP_post(object->convertComplete);
}

/*
 *  ======== completeConversion ========
 */
static void completeConversion(ADCBuf_Handle handle, uint8_t sequencer)
{
    ADCBufMSP432E4_Object        *object = handle->object;

    /* Perform callback in a HWI context. The callback ideally is invoked in
     * SWI instead of HWI. This should get called once per sequencer group */
    object->callBackFxn(handle, &object->conversions[sequencer][0],
        (!object->pingpongFlag[sequencer]) ?
            object->conversions[sequencer][0].sampleBuffer :
            object->conversions[sequencer][0].sampleBufferTwo,
        object->conversions[sequencer][0].adcChannel,
        ADCBuf_STATUS_SUCCESS);

    if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS) {
        /* Toggle the pingpong flag */
        object->pingpongFlag[sequencer] ^= 1;

        /* Reset sample index */
        object->sampleIndex[sequencer] = 0;
        /* Toggle the pingpong flag */
        if (!object->pingpongFlag[sequencer]) {
            object->sampleBuffer[sequencer] =
                object->conversions[sequencer]->sampleBuffer;
        }
        else {
            object->sampleBuffer[sequencer] =
                object->conversions[sequencer]->sampleBufferTwo;
        }
    }
    else {
        /* Clear the object conversions if in the one shot mode */
        object->conversions[sequencer] = NULL;
    }
}

/*
 *  ======== initHW ========
 *  Configures ADC peripheral
 */
static void initHw(ADCBufMSP432E4_Object *object,
        ADCBufMSP432E4_HWAttrsV1 const *hwAttrs)
{
    uint8_t i;
    uintptr_t key;
    ClockP_FreqHz freq;

    /* Configure ADC Seqeuncer */
    for (i = 0; i < ADCBufMSP432E4_SEQUENCER_COUNT; i++) {
       if (hwAttrs->sequencePriority[i] < ADCBufMSP432E4_Seq_Disable) {
           MAP_ADCSequenceConfigure(hwAttrs->adcBase, i,
               hwAttrs->adcTriggerSource[i], hwAttrs->sequencePriority[i]);
       }
    }

    /* Get system clock frequency to calculate timer period*/
    ClockP_getCpuFreq(&freq);

    key = HwiP_disable();
    if (timerSettings.timerInstances && !timerSettings.isInitialized) {
        /* set timer to initialized */
        timerSettings.isInitialized = true;
        HwiP_restore(key);

        /* Initialize timer resource */
        MAP_TimerConfigure(timerSettings.timerSource,
            TIMER_CFG_PERIODIC);
        MAP_TimerLoadSet(timerSettings.timerSource, TIMER_A,
            freq.lo/timerSettings.timerFrequency);
        MAP_TimerADCEventSet(timerSettings.timerSource,
            TIMER_ADC_TIMEOUT_A);
        MAP_TimerControlTrigger(timerSettings.timerSource,
            TIMER_A, true);
    }
    else {
        HwiP_restore(key);
    }

    /* Set the ADC reference voltage */
    ADCReferenceSet(hwAttrs->adcBase, hwAttrs->refSource);

    /* Set ADC phase delay */
    ADCPhaseDelaySet(hwAttrs->adcBase, hwAttrs->modulePhase);
}

/*
 *  ======== ADCBufMSP432E4_init ========
 */
void ADCBufMSP432E4_init(ADCBuf_Handle handle)
{
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs = handle->hwAttrs;
    uintptr_t         key;
    SemaphoreP_Handle sem0;
    SemaphoreP_Handle sem1;

    /* Create a binary semaphore for thread safety per ADC module */
    sem0 = SemaphoreP_createBinary(1);
    sem1 = SemaphoreP_createBinary(1);
    /* sem == NULL will be detected in 'open' */

    key = HwiP_disable();

    /* Create Semaphore for ADC0 */
    if (globalMutex[0] == NULL) {
        /* use the binary sem created above */
        globalMutex[0] = sem0;

        HwiP_restore(key);
    }
    else {
        /* init already called */
        HwiP_restore(key);

        if (sem0) {
            /* delete unused Semaphore */
            SemaphoreP_delete(sem0);
        }
    }

    key = HwiP_disable();

    /* Create Semaphore for ADC1 */
    if (globalMutex[1] == NULL) {
        /* use the binary sem created above */
        globalMutex[1] = sem1;

        HwiP_restore(key);
    }
    else {
        /* init already called */
        HwiP_restore(key);

        if (sem1) {
            /* delete unused Semaphore */
            SemaphoreP_delete(sem1);
        }
    }

    /* Initialize UDMA peripheral */
    if (hwAttrs->useDMA) {
        UDMAMSP432E4_init();
    }
}

/*
 *  ======== configDMA ========
 *  This functions configures the DMA to automatically transfer ADC
 *  output data into a provided array
 *
 *  @pre    ADCBufMSP432E4_open() has to be called first.
 *
 *  @pre    There must not currently be a conversion in progress
 *
 *  @pre    Function assumes that the handle and transaction is not NULL
 *
 *  @param  object An ADCBufMSP432 handle->object returned from
 *          ADCBufMSP432E4_open()
 *
 *  @param  hwAttrs An ADCBufMSP432 handle->hwAttrs from board file
 *
 *  @param  conversion A pointer to an ADCBuf_Conversion
 *
 */
static void configDMA(ADCBufMSP432E4_Object *object,
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs, ADCBuf_Conversion *conversion)
{
    uint_fast8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    uint_fast8_t sequencer =
        hwAttrs->channelSetting[conversion[0].adcChannel].adcSequence;

    /* Enable the DMA request from selected ADC base and sequencer  */
    MAP_ADCSequenceDMAEnable(hwAttrs->adcBase, sequencer);

    /* Map the ADC sequencer to corresponding DMA channel*/
    MAP_uDMAChannelAssign(udmaChannels[base][sequencer]);

    /* Put the attributes in a known state for the ADC sequencer uDMA
     * channel. These should already be disabled by default. */
    MAP_uDMAChannelAttributeDisable(udmaChannels[base][sequencer],
            UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
            UDMA_ATTR_HIGH_PRIORITY |
            UDMA_ATTR_REQMASK);

    /* Configure the control parameters for the primary control structure for
     * the ADC sequencer channel. The primary control structure is used for
     * copying the data from the ADC Sequencer FIFO to conversion->sampleBuffer.
     * The transfer data size is 16 bits and the source address is not
     * incremented while the destination address is incremented at 16-bit
     * boundary.
     */
    MAP_uDMAChannelControlSet(udmaChannels[base][sequencer] | UDMA_PRI_SELECT,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
            UDMA_ARB_1);

    /* Set up the transfer parameters for the ADC Sequencer primary control
     * structure. The mode is pingpong mode so it will run to continuously. */
    MAP_uDMAChannelTransferSet(udmaChannels[base][sequencer] | UDMA_PRI_SELECT,
            UDMA_MODE_PINGPONG,
            (void *)(hwAttrs->adcBase + SSFIFO_BASE + sequencer*SSFIFO_OFFSET),
            (void *) conversion->sampleBuffer,
            conversion->samplesRequestedCount);

    /* Set up transfer parameters for ADC Sequencer alternate control structure */
    MAP_uDMAChannelControlSet(udmaChannels[base][sequencer] | UDMA_ALT_SELECT,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
            UDMA_ARB_1);
    MAP_uDMAChannelTransferSet(udmaChannels[base][sequencer] | UDMA_ALT_SELECT,
            UDMA_MODE_PINGPONG,
            (void *)(hwAttrs->adcBase + SSFIFO_BASE + sequencer*SSFIFO_OFFSET),
            (void *) conversion->sampleBufferTwo,
            conversion->samplesRequestedCount);

    /* The uDMA ADC Sequencer channel is primed to start a transfer. As
     * soon as the channel is enabled and the Timer will issue an ADC trigger,
     * the ADC will perform the conversion and send a DMA Request. The data
     * transfers will begin. */
    MAP_uDMAChannelEnable(udmaChannels[base][sequencer]);
}

/*
 *  ======== ADCBufCCMSP432E4_adjustRawValues ========
 */
int_fast16_t ADCBufMSP432E4_adjustRawValues(ADCBuf_Handle handle,
    void *sampleBuffer, uint_fast16_t sampleCount, uint32_t adcChannel)
{
    /* This hardware peripheral does not support Calibration */
    return (ADCBuf_STATUS_UNSUPPORTED);
}

/*
 *  ======== ADCBufMSP432E4_close ========
 */
void ADCBufMSP432E4_close(ADCBuf_Handle handle)
{
    uintptr_t         key;
    ADCBufMSP432E4_Object        *object = handle->object;
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs = handle->hwAttrs;
    uint8_t i;
    uint8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;

    key = HwiP_disable();

    /* Disable interrupts and the ADC */
    MAP_ADCIntDisableEx(hwAttrs->adcBase, 0xFFF);

    for (i = 0; i < ADCBufMSP432E4_SEQUENCER_COUNT; i++) {
        if (hwAttrs->useDMA) {
            MAP_ADCSequenceDMADisable(hwAttrs->adcBase, i);
            MAP_uDMAChannelDisable(udmaChannels[base][i]);
        }
        else {
            MAP_ADCSequenceDisable(hwAttrs->adcBase, i);
        }

        /* Destruct driver resources */
        if (object->sequencerHwiHandles[i]) {
            HwiP_delete(object->sequencerHwiHandles[i]);
            object->sequencerHwiHandles[i] = NULL;
        }

        if (hwAttrs->adcTriggerSource[i] == ADCBufMSP432E4_TIMER_TRIGGER) {
            timerSettings.timerInstances--;

            /* If no more timer instances, disable timer and free resources*/
            if (!timerSettings.timerInstances) {
                MAP_TimerDisable(timerSettings.timerSource, TIMER_A);
                TimerMSP432E4_freeTimerResource(timerSettings.timerSource,
                    TimerMSP432E4_timer32);
                timerSettings.timerSource = 0;
                timerSettings.timerFrequency = 0;
                timerSettings.isInitialized = false;
            }
        }
    }

    HwiP_restore(key);

    if (object->convertComplete) {
        SemaphoreP_delete(object->convertComplete);
    }

    if (object->dmaHandle != NULL) {
        UDMAMSP432E4_close(object->dmaHandle);
        object->dmaHandle = NULL;
    }

    object->isOpen = false;
}

/*
 *  ======== ADCBufMSP432E4_control ========
 */
int_fast16_t ADCBufMSP432E4_control(ADCBuf_Handle handle, uint_fast16_t cmd,
    void * arg)
{
    /* No implementation yet */
    return (ADCBuf_STATUS_UNDEFINEDCMD);
}

/*
 *  ======== primeConvert ========
 */
static int_fast16_t primeConvert(ADCBufMSP432E4_Object *object,
        ADCBufMSP432E4_HWAttrsV1 const *hwAttrs,
        ADCBuf_Conversion *conversions, uint_fast8_t channelCount)
{
    uint_fast8_t i=0;
    uint32_t channel;
    uint32_t port;
    uint8_t pin;
    uint_fast16_t powerID;
    ADCBufMSP432E4_Channels channelSetting =
            hwAttrs->channelSetting[conversions[0].adcChannel];
    uint_fast8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    uint8_t sequencer = channelSetting.adcSequence;
    uint32_t refVoltage = channelSetting.refVoltage;

    if ((channelCount > adcMaxSamples[sequencer])
        || (hwAttrs->sequencePriority[sequencer] >= ADCBufMSP432E4_Seq_Disable)) {
        return (ADCBuf_STATUS_ERROR);
    }

    /* Store the conversions struct array into object */
    object->conversions[sequencer] = conversions;
    /* Store the channel count into object */
    object->channelCount[sequencer] = channelCount;

    /* Store the samples count into object - one channel*/
    object->sampleBuffer[sequencer] = conversions->sampleBuffer;
    object->sampleCount[sequencer] =
        conversions->samplesRequestedCount;
    object->sampleIndex[sequencer] = 0;

    /* Initialize GPIOs and Temperature mode*/
    for (i=0; i < channelCount; i++) {

        channelSetting = hwAttrs->channelSetting[conversions[i].adcChannel];
        if (channelSetting.refVoltage != refVoltage) {
            return (ADCBuf_STATUS_ERROR);
        }

        channel = pinConfigChannel(channelSetting.adcPin);
        port = pinConfigPort(channelSetting.adcPin);
        pin = pinConfigPin(channelSetting.adcPin);

        /* If using temperature mode, skip GPIO initialization and use
           ADC_CTL_TS instead of channel */
        if (channelSetting.adcInternalSource
                == ADCBufMSP432E4_TEMPERATURE_MODE) {
            channel = ADCBufMSP432E4_TEMPERATURE_MODE;
        }
        else {
            powerID = getPowerResourceId(port);
            if (powerID == (uint16_t)(-1)) {
                return (ADCBuf_STATUS_ERROR);
            }
            Power_setDependency(powerID);
            GPIOPinTypeADC(port, pin);
        }

        /* Initialize differential pin channel for differential mode*/
        if (channelSetting.adcInputMode == ADCBufMSP432E4_DIFFERENTIAL) {
            if (channelSetting.adcDifferentialPin == ADCBufMSP432E4_PIN_NONE) {
                return (ADCBuf_STATUS_ERROR);
            }
            MAP_GPIOPinTypeADC(
                pinConfigPort(channelSetting.adcDifferentialPin),
                pinConfigPin(channelSetting.adcDifferentialPin)
                );
            channel = channel / 2;
            channel = channel | ADCBufMSP432E4_DIFFERENTIAL;
        }

        if (i == channelCount - 1) {
            channel |= ADC_CTL_IE | ADC_CTL_END;
        }
        MAP_ADCSequenceStepConfigure(hwAttrs->adcBase, sequencer, i,
            channel|object->samplingDuration);
    }

    /* If DMA enabled, setup peripheral*/
    if (hwAttrs->useDMA) {
        MAP_ADCIntClearEx(hwAttrs->adcBase, ADC_INT_DMA_SS0 << sequencer);
        MAP_ADCIntEnableEx(hwAttrs->adcBase, ADC_INT_DMA_SS0 <<sequencer);

        configDMA(object, hwAttrs, conversions);
    }
    else {
        MAP_ADCIntClear(hwAttrs->adcBase, sequencer);
        MAP_ADCIntEnable(hwAttrs->adcBase, sequencer);
    }

    /* Since sample sequence is now configured, it must be enabled. */
    MAP_ADCSequenceEnable(hwAttrs->adcBase, sequencer);

    /* Enable the Interrupt generation from the specified ADC Sequencer */
    MAP_IntEnable(adcInterrupts[base][sequencer]);

    /* Enable timer if using timer trigger */
    if (hwAttrs->adcTriggerSource[sequencer] == ADCBufMSP432E4_TIMER_TRIGGER) {
        MAP_TimerEnable(timerSettings.timerSource, TIMER_A);
    }

    return (ADCBuf_STATUS_SUCCESS);
}

/*
 *  ======== ADCBufMSP432E4_convert ========
 */
int_fast16_t ADCBufMSP432E4_convert(ADCBuf_Handle handle,
    ADCBuf_Conversion *conversions,
    uint_fast8_t channelCount)
{
    ADCBufMSP432E4_Object        *object = handle->object;
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs = handle->hwAttrs;
    int_fast16_t ret;

    /* Acquire the lock for this particular ADC handle */
    SemaphoreP_pend(object->mutex, SemaphoreP_WAIT_FOREVER);

    /* Execute core conversion */
    ret = primeConvert(object, hwAttrs, conversions, channelCount);
    if (ret == ADCBuf_STATUS_ERROR) {
        SemaphoreP_post(object->mutex);
        return (ret);
    }

    if (object->returnMode == ADCBuf_RETURN_MODE_BLOCKING) {
        /*
         * Wait for the transfer to complete here.
         * It's OK to block from here because the ADC's Hwi will unblock
         * upon errors
         */
        if (SemaphoreP_OK != SemaphoreP_pend(object->convertComplete,
            object->semaphoreTimeout)) {
            ret = ADCBuf_STATUS_ERROR;
        }
    }

    /* Release the lock for this particular ADC handle */
    SemaphoreP_post(object->mutex);

    /* Return the number of bytes transferred by the ADC */
    return (ret);
}

/*
 *  ======== ADCBufMSP432E4_convertAdjustedToMicroVolts ========
 */
int_fast16_t ADCBufMSP432E4_convertAdjustedToMicroVolts(ADCBuf_Handle handle,
    uint32_t adcChannel, void *adjustedSampleBuffer,
    uint32_t outputMicroVoltBuffer[], uint_fast16_t sampleCount)
{
    uint32_t i;
    ADCBufMSP432E4_HWAttrsV1 const  *hwAttrs = handle->hwAttrs;
    uint32_t refVoltage = hwAttrs->channelSetting[adcChannel].refVoltage;
    uint16_t *adjustedRawSampleBuf = (uint16_t *) adjustedSampleBuffer;

    for (i = 0; i < sampleCount; i++) {
        if (adjustedRawSampleBuf[i] == 0xFFF) {
            outputMicroVoltBuffer[i] = refVoltage;
        }
        else {
            outputMicroVoltBuffer[i] =
                (((uint64_t)adjustedRawSampleBuf[i] * refVoltage) / 0x1000);
        }
    }

    return (ADCBuf_STATUS_SUCCESS);
}

/*
 *  ======== ADCBufMSP432E4_convertCancel ========
 */
int_fast16_t ADCBufMSP432E4_convertCancel(ADCBuf_Handle handle)
{
    uintptr_t                key;
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs = handle->hwAttrs;
    ADCBufMSP432E4_Object    *object = handle->object;
    ADCBufMSP432E4_Channels channelSetting =
        hwAttrs->channelSetting[object->conversions[0]->adcChannel];
    uint8_t sequencer = channelSetting.adcSequence;

    key = HwiP_disable();
    if (hwAttrs->adcTriggerSource[sequencer] == ADCBufMSP432E4_TIMER_TRIGGER &&
        timerSettings.timerInstances == 1) {
        MAP_TimerDisable(hwAttrs->adcTimerSource, TIMER_A);
    }

    HwiP_restore(key);

    ADCSequenceDisable(hwAttrs->adcBase, sequencer);

    if (hwAttrs->useDMA) {
        MAP_ADCIntClearEx(hwAttrs->adcBase, ADC_INT_DMA_SS0 << sequencer);
    }
    else {
        MAP_ADCIntClear(hwAttrs->adcBase, sequencer);
    }

    /* Wait for BUSY to be cleared to ensure conversion is cancelled */
    while(ADCBusy(hwAttrs->adcBase));

    completeConversion(handle, sequencer);

    return (ADCBuf_STATUS_SUCCESS);
}

/*
 *  ======== ADCBufMSP432E4_getResolution ========
 */
uint_fast8_t ADCBufMSP432E4_getResolution(ADCBuf_Handle handle)
{
    return (ADCRESOLUTION);
}

void ADCBufMSP423E4_noDMAhwiIntFxn(uintptr_t arg)
{
    ADCBufMSP432E4_Object    *object = ((ADCBuf_Handle) arg)->object;
    ADCBufMSP432E4_HWAttrsV1 const  *hwAttrs = ((ADCBuf_Handle) arg)->hwAttrs;

    uint8_t i;
    uint32_t intStatus;
    uint32_t intMask;
    uint32_t adcBuffer[MAX_SSFIFO_SIZE];
    uint16_t *sampleBuffer;
    uint8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    uint8_t sequencer = 0;
    int32_t numData;

    intMask = ADC_INT_SS0;
    for (sequencer = 0; sequencer < ADCBufMSP432E4_SEQUENCER_COUNT; sequencer++) {
        /* Get the interrupt status of the ADC controller*/
        intStatus = MAP_ADCIntStatus(hwAttrs->adcBase, sequencer, true);
        if (intStatus & intMask) {
            MAP_ADCIntClear(hwAttrs->adcBase, sequencer);

            /* get ADC values and store in adcBuffer*/
            numData = MAP_ADCSequenceDataGet(hwAttrs->adcBase, sequencer,
                                             adcBuffer);

            /* Check if the expected number of samples have been stored in adcBuffer*/
            if (numData != object->channelCount[sequencer]) {
                return;
            }

            sampleBuffer = (!object->pingpongFlag[sequencer]) ?
                (uint16_t*) (object->conversions[sequencer]->sampleBuffer) :
                (uint16_t*) (object->conversions[sequencer]->sampleBufferTwo);

            /* store sample values from adcBuffer in sampleBuffer */
            for (i = 0; i < object->channelCount[sequencer]; i++) {
                sampleBuffer[object->sampleIndex[sequencer]] =
                    (uint16_t) adcBuffer[i];
                object->sampleIndex[sequencer]++;
            }

            /* ADC conversion complete */
            if (object->sampleIndex[sequencer] ==
                object->sampleCount[sequencer]) {

                if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_ONE_SHOT) {
                    MAP_ADCIntDisable(hwAttrs->adcBase, sequencer);
                    MAP_ADCSequenceDisable(hwAttrs->adcBase, sequencer);
                    MAP_IntDisable(adcInterrupts[base][sequencer]);

                    if (hwAttrs->adcTriggerSource[sequencer] ==
                        ADCBufMSP432E4_TIMER_TRIGGER &&
                        timerSettings.timerInstances == 1) {
                        MAP_TimerDisable(hwAttrs->adcTimerSource, TIMER_A);
                    }
                }
                completeConversion((ADCBuf_Handle)arg, sequencer);
            }
            else {
                MAP_ADCSequenceEnable(hwAttrs->adcBase, sequencer);
            }
        }
        intMask = intMask << 1;
    }
}

/*
 *  ======== ADCBufMSP432E4_hwiIntFxn ========
 */
void ADCBufMSP432E4_hwiIntFxn(uintptr_t arg)
{
    uint32_t intStatus;
    uint32_t intMask;
    ADCBufMSP432E4_Object    *object = ((ADCBuf_Handle) arg)->object;
    ADCBufMSP432E4_HWAttrsV1 const  *hwAttrs = ((ADCBuf_Handle) arg)->hwAttrs;

    uint8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    uint8_t sequencer = 0;

    /* check interrupt status */
    intStatus = MAP_ADCIntStatusEx(hwAttrs->adcBase, true);
    intMask = ADC_INT_DMA_SS0;

    /* clear interrupt flag */
    for (sequencer = 0; sequencer < ADCBufMSP432E4_SEQUENCER_COUNT; sequencer ++) {
        uint32_t dmaChannel = udmaChannels[base][sequencer];
        if ((intStatus & intMask) == intMask) {
            MAP_ADCIntClearEx(hwAttrs->adcBase, intMask);
            completeConversion((ADCBuf_Handle) arg, sequencer);

            if (object->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS) {
                /* Switch between primary and alternate buffers for DMA's
                   PingPong mode */
                if (object->pingpongFlag[sequencer] != 0) {
                    MAP_uDMAChannelControlSet(dmaChannel| UDMA_PRI_SELECT,
                        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                        UDMA_ARB_1);

                    MAP_uDMAChannelTransferSet(dmaChannel | UDMA_PRI_SELECT,
                        UDMA_MODE_PINGPONG,
                        (void *)(hwAttrs->adcBase + SSFIFO_BASE +
                            sequencer*SSFIFO_OFFSET),
                        (void *)object->conversions[sequencer]->sampleBuffer,
                        object->conversions[sequencer]->samplesRequestedCount);
                }
                else {
                    MAP_uDMAChannelControlSet(dmaChannel | UDMA_ALT_SELECT,
                        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 |
                        UDMA_ARB_1);
                    MAP_uDMAChannelTransferSet(dmaChannel | UDMA_ALT_SELECT,
                        UDMA_MODE_PINGPONG,
                        (void *)(hwAttrs->adcBase + SSFIFO_BASE +
                            sequencer*SSFIFO_OFFSET),
                        (void *)object->conversions[sequencer]->sampleBufferTwo,
                        object->conversions[sequencer]->samplesRequestedCount);
                }

                MAP_uDMAChannelEnable(dmaChannel);
                MAP_ADCSequenceDMAEnable(hwAttrs->adcBase, sequencer);
                /* Re-enable sample sequence */
                MAP_ADCSequenceEnable(hwAttrs->adcBase, sequencer);
            }
            else {
                /* If using one shot mode, disable interrupts, DMA,
                   and sequencer*/
                MAP_uDMAChannelDisable(dmaChannel);
                MAP_ADCIntDisableEx(hwAttrs->adcBase, intMask);
                MAP_ADCSequenceDisable(hwAttrs->adcBase, sequencer);

                if (hwAttrs->adcTriggerSource[sequencer] ==
                    ADCBufMSP432E4_TIMER_TRIGGER &&
                    timerSettings.timerInstances == 1) {
                    MAP_TimerDisable(hwAttrs->adcTimerSource, TIMER_A);
                }
            }
        }
        intMask = intMask << 1;
    }
}

/*
 *  ======== ADCBufMSP432E4_open ========
 */
ADCBuf_Handle ADCBufMSP432E4_open(ADCBuf_Handle handle,
                                const ADCBuf_Params *params)
{
    uintptr_t                key;
    uint32_t powerID;
    ADCBufMSP432E4_Object    *object = handle->object;
    ADCBufMSP432E4_HWAttrsV1 const *hwAttrs = handle->hwAttrs;
    uint8_t i=0;
    uint8_t base = (hwAttrs->adcBase == ADC0_BASE) ? 0 : 1;
    HwiP_Params          hwiParams;

    /* Confirm that 'init' has successfully completed */
    if (globalMutex[base] == NULL){
        ADCBufMSP432E4_init(handle);
        if (globalMutex[base] == NULL) {
            return (NULL);
        }
    }
    object->mutex = globalMutex[base];

    if ((ADCBuf_RETURN_MODE_CALLBACK == params->returnMode
        && params->callbackFxn == NULL) ||
        (params->recurrenceMode == ADCBuf_RECURRENCE_MODE_CONTINUOUS &&
        params->returnMode != ADCBuf_RETURN_MODE_CALLBACK)) {
        return (NULL);
    }

    key = HwiP_disable();

   if (object->isOpen) {
        HwiP_restore(key);
        return (NULL);
    }
    object->isOpen = true;

    HwiP_restore(key);

    /* Configure system clock */
    powerID = getPowerResourceId(hwAttrs->adcBase);
    if (powerID == (uint16_t)(-1)) {
        object->isOpen = false;
        return (NULL);
    }
    Power_setDependency(powerID);

    for (i = 0; i < ADCBufMSP432E4_SEQUENCER_COUNT; i++) {
        if (hwAttrs->sequencePriority[i] < ADCBufMSP432E4_Seq_Disable) {

            /* Allocate timer trigger resource */
            if(hwAttrs->adcTriggerSource[i] == ADCBufMSP432E4_TIMER_TRIGGER) {

                /* Check if first timer instance and allocate timer resource
                   accordingly*/
                key = HwiP_disable();
                if (!timerSettings.timerInstances) {
                    if (!TimerMSP432E4_allocateTimerResource(
                        hwAttrs->adcTimerSource, TimerMSP432E4_timer32)) {
                        object->isOpen = false;
                        Power_releaseDependency(powerID);
                        HwiP_restore(key);
                        return (NULL);
                    }
                    timerSettings.timerSource = hwAttrs->adcTimerSource;
                    timerSettings.timerFrequency = params->samplingFrequency;
                }
                HwiP_restore(key);

                /* Check if ADC instance is using the same timer source and
                   frequency. Fail if they are different */
                if (hwAttrs->adcTimerSource != timerSettings.timerSource ||
                    params->samplingFrequency != timerSettings.timerFrequency) {
                    object->isOpen = false;
                    Power_releaseDependency(powerID);
                    return (NULL);
                }

                timerSettings.timerInstances++;
            }

            /* Fail if using software automatic trigger and no DMA */
            if (hwAttrs->adcTriggerSource[i] ==
                ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER &&
                !hwAttrs->useDMA) {
                object->isOpen = false;
                Power_releaseDependency(powerID);
                return (NULL);
            }
        }
    }

    /* Creat Hwi object for ADC */
    if (hwAttrs->useDMA) {
        object->dmaHandle = UDMAMSP432E4_open();
        if (object->dmaHandle == NULL) {
            ADCBufMSP432E4_close(handle);
            return (NULL);
        }
    }

    HwiP_Params_init(&hwiParams);
    hwiParams.arg = (uintptr_t) handle;
    hwiParams.priority = hwAttrs->intPriority;

    /* Initialize interrupts for each sequencer if user sets priority
       less than ADCBufMSP432E4_Seq_Disable*/
    for (i = 0; i < ADCBufMSP432E4_SEQUENCER_COUNT; i++) {
        if (hwAttrs->sequencePriority[i] < ADCBufMSP432E4_Seq_Disable) {
            if (hwAttrs->useDMA) {
                object->sequencerHwiHandles[i] =
                    HwiP_create(adcInterrupts[base][i],
                        ADCBufMSP432E4_hwiIntFxn,
                        &hwiParams);
            }
            else {
                object->sequencerHwiHandles[i] =
                    HwiP_create(adcInterrupts[base][i],
                        ADCBufMSP423E4_noDMAhwiIntFxn,
                        &hwiParams);
            }
            if (object->sequencerHwiHandles[i] == NULL) {
                ADCBufMSP432E4_close(handle);
                return (NULL);
            }
        }
        object->pingpongFlag[i] = 0;
    }

    /* Configure driver to Callback or Blocking operating mode */
    if (params->returnMode == ADCBuf_RETURN_MODE_CALLBACK) {
        object->callBackFxn = params->callbackFxn;
    }
    else {
        /* Semaphore to block task for the duration of the ADC convert */
        object->convertComplete = SemaphoreP_createBinary(0);
        if (!object->convertComplete) {
            ADCBufMSP432E4_close(handle);
            return (NULL);
        }
        object->callBackFxn = blockingConvertCallback;
    }

    /*
     * Store ADC parameters & initialize peripheral.  These are used to
     * re/initialize the peripheral when opened or changing performance level.
     */

    object->returnMode = params->returnMode;
    object->recurrenceMode = params->recurrenceMode;
    object->semaphoreTimeout = params->blockingTimeout;
    object->samplingFrequency = params->samplingFrequency;

    /* Check the ExtensionParam is set */
    if (params->custom) {
        /* If MSP432E4 specific params were specified, use them */
        object->samplingDuration =
            ((ADCBufMSP432E4_ParamsExtension *)(params->custom))->
            samplingDuration;
    }
    else {
        /* Initialize MSP432E4 specific settings to defaults */
        object->samplingDuration =
            ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_4;
    }

     /* Initialize ADC related hardware */
    initHw(object, hwAttrs);
    return (handle);
}

static uint_fast16_t getPowerResourceId(uint32_t baseAddress) {
    uint_fast16_t resourceID;
    switch (baseAddress) {
    case GPIO_PORTB_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOB;
        break;
    case GPIO_PORTD_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOD;
        break;
    case GPIO_PORTE_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOE;
        break;
    case GPIO_PORTK_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOK;
        break;
    case GPIO_PORTP_BASE:
        resourceID = PowerMSP432E4_PERIPH_GPIOP;
        break;
    case ADC0_BASE:
        resourceID = PowerMSP432E4_PERIPH_ADC0;
        break;
    case ADC1_BASE:
        resourceID = PowerMSP432E4_PERIPH_ADC1;
        break;
    default:
        resourceID = (uint_fast16_t)-1;
        break;
    }
    return (resourceID);
}
