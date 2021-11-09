/*
 * Copyright (c) 2017-2020, Texas Instruments Incorporated
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

#include <stdbool.h>
#include <stdint.h>
#if defined(__IAR_SYSTEMS_ICC__)
#include <intrinsics.h>
#endif

#include <ti/devices/msp432e4/inc/msp432.h>

#include <ti/devices/msp432e4/driverlib/gpio.h>
#include <ti/devices/msp432e4/driverlib/interrupt.h>

#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432E4.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>

#include <ti/devices/msp432e4/driverlib/driverlib.h>
#include <ti/devices/msp432e4/driverlib/types.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_gpio.h>

/* Table of GPIO interrupt types */
static const uint8_t interruptType[] = {
    0,                  /* Undefined interrupt type */
    GPIO_FALLING_EDGE,
    GPIO_RISING_EDGE,
    GPIO_BOTH_EDGES,
    GPIO_LOW_LEVEL,
    GPIO_HIGH_LEVEL
};

#ifdef __MSP432E411Y__
static const uint32_t gpioBaseAddresses[] = {
    GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE,
    GPIO_PORTD_BASE, GPIO_PORTE_BASE, GPIO_PORTF_BASE,
    GPIO_PORTG_BASE, GPIO_PORTH_BASE, GPIO_PORTJ_BASE,
    GPIO_PORTK_BASE, GPIO_PORTL_BASE, GPIO_PORTM_BASE,
    GPIO_PORTN_BASE, GPIO_PORTR_BASE, GPIO_PORTS_BASE,
    GPIO_PORTT_BASE, GPIO_PORTP_BASE, GPIO_PORTQ_BASE
};
#else
static const uint32_t gpioBaseAddresses[] = {
    GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE,
    GPIO_PORTD_BASE, GPIO_PORTE_BASE, GPIO_PORTF_BASE,
    GPIO_PORTG_BASE, GPIO_PORTH_BASE, GPIO_PORTJ_BASE,
    GPIO_PORTK_BASE, GPIO_PORTL_BASE, GPIO_PORTM_BASE,
    GPIO_PORTN_BASE, 0              , 0,
    0              , GPIO_PORTP_BASE, GPIO_PORTQ_BASE
};
#endif

/* Table of GPIO input types */
static const uint8_t inPinTypes [] = {
    GPIO_PIN_TYPE_STD,        /* GPIO_CFG_IN_NOPULL */
    GPIO_PIN_TYPE_STD_WPU,    /* GPIO_CFG_IN_PU */
    GPIO_PIN_TYPE_STD_WPD     /* GPIO_CFG_IN_PD */
};

/* Table of GPIO output types */
static const uint8_t outPinTypes [] = {
    GPIO_PIN_TYPE_STD,    /* GPIO_CFG_OUT_STD */
    GPIO_PIN_TYPE_OD,     /* GPIO_CFG_OUT_OD_NOPULL */
    GPIO_PIN_TYPE_OD,     /* GPIO_CFG_OUT_OD_PU not supported by MSP432E4 driverlib */
    GPIO_PIN_TYPE_OD      /* GPIO_CFG_OUT_OD_PD not supported by MSP432E4 driverlib */
};

/* Table of port interrupt vectors. Used by setCallback() to create Hwis. */
static const uint8_t gpioInterruptVectors[] = {
    INT_GPIOA,  INT_GPIOB,  INT_GPIOC,
    INT_GPIOD,  INT_GPIOE,  INT_GPIOF,
    INT_GPIOG,  INT_GPIOH,  INT_GPIOJ,
    INT_GPIOK,  INT_GPIOL,  INT_GPIOM,
    INT_GPION,  INT_GPIOR,  INT_GPIOS,
    INT_GPIOT,  INT_GPIOP0, INT_GPIOP1,
    INT_GPIOP2, INT_GPIOP3, INT_GPIOP4,
    INT_GPIOP5, INT_GPIOP6, INT_GPIOP7,
    INT_GPIOQ0, INT_GPIOQ1, INT_GPIOQ2,
    INT_GPIOQ3, INT_GPIOQ4, INT_GPIOQ5,
    INT_GPIOQ6, INT_GPIOQ7
};

/* Table of GPIO drive strengths */
static const uint8_t outPinStrengths [] = {
    GPIO_STRENGTH_2MA,       /* GPIO_CFG_OUT_STR_LOW */
    GPIO_STRENGTH_6MA,       /* GPIO_CFG_OUT_STR_MED */
    GPIO_STRENGTH_12MA,      /* GPIO_CFG_OUT_STR_HIGH */
    GPIO_STRENGTH_4MA,
    GPIO_STRENGTH_8MA,
    GPIO_STRENGTH_8MA_SC,
    GPIO_STRENGTH_10MA
};

static const uint_fast16_t powerMgrIds[] = {
    PowerMSP432E4_PERIPH_GPIOA,     /* PORTA */
    PowerMSP432E4_PERIPH_GPIOB,     /* PORTB */
    PowerMSP432E4_PERIPH_GPIOC,     /* PORTC */
    PowerMSP432E4_PERIPH_GPIOD,     /* PORTD */
    PowerMSP432E4_PERIPH_GPIOE,     /* PORTE */
    PowerMSP432E4_PERIPH_GPIOF,     /* PORTF */
    PowerMSP432E4_PERIPH_GPIOG,     /* PORTG */
    PowerMSP432E4_PERIPH_GPIOH,     /* PORTH */
    PowerMSP432E4_PERIPH_GPIOJ,     /* PORTJ */
    PowerMSP432E4_PERIPH_GPIOK,     /* PORTK */
    PowerMSP432E4_PERIPH_GPIOL,     /* PORTL */
    PowerMSP432E4_PERIPH_GPIOM,     /* PORTM */
    PowerMSP432E4_PERIPH_GPION,     /* PORTN */
    PowerMSP432E4_PERIPH_GPIOR,     /* PORTR */
    PowerMSP432E4_PERIPH_GPIOS,     /* PORTS */
    PowerMSP432E4_PERIPH_GPIOT,     /* PORTT */
    PowerMSP432E4_PERIPH_GPIOP,     /* PORTP */
    PowerMSP432E4_PERIPH_GPIOQ,     /* PORTQ */
};

#define NUM_PORTS           (18)
#define NUM_PINS_PER_PORT    (8)

/* Returns the GPIO port base address */
#define getPortBase(port) (gpioBaseAddresses[port])

/* Returns the GPIO power resource ID */
#define getPowerResource(port) (powerMgrIds[port])

/* Uninitialized callbackInfo pinIndex */
#define CALLBACK_INDEX_NOT_CONFIGURED 0xFF

/* Device specific interpretation of the GPIO_PinConfig content */
typedef struct {
    uint8_t pin;
    uint8_t port;
    uint16_t config;
} PinConfig;

/*
 * User defined pin indexes assigned to a port's 8 pins. Used by port interrupt
 * function to located callback assigned to a pin.
 */
typedef struct {
    /* The port's 8 corresponding user defined pinId indices */
    uint8_t pinIndex[NUM_PINS_PER_PORT];
} PortCallbackInfo;


/* Table of callbacks per port. */
static PortCallbackInfo gpioCallbackInfo[NUM_PORTS];

/*
 * Bit mask used to determine if a Hwi has been created/constructed for a port
 * already.
 */
static uint32_t portHwiCreatedBitMask = 0;

/* Boolean to confirm that GPIO_init() has been called */
static volatile bool initCalled = false;

__attribute__((weak))extern const GPIOMSP432E4_Config GPIOMSP432E4_config;

/*
 *  ======== getPinNumber ========
 *
 *  Internal function to efficiently find the index of the right most set bit.
 */
static inline uint32_t getPinNumber(uint32_t x)
{
    uint32_t tmp;

#if defined(__TI_COMPILER_VERSION__)
    tmp = __clz(__rbit(x));
#elif defined(__GNUC__)
    tmp = __builtin_ctz(x);
#elif defined(__IAR_SYSTEMS_ICC__)
    tmp = __CLZ(__RBIT(x));
#else
    #error "Unsupported compiler used"
#endif

    tmp = tmp & 0x7;  /* force 0..7 to keep Klocwork happy */

    return (tmp);
}

/*
 *  ======== getInPinTypesIndex ========
 */
static inline uint32_t getInPinTypesIndex(uint32_t pinConfig)
{
    uint32_t index;

    index = (pinConfig & GPIO_CFG_IN_TYPE_MASK) >> GPIO_CFG_IN_TYPE_LSB;

    /*
     * If index is out-of-range, default to 0. This should never
     * happen, but it's needed to keep Klocwork checker happy.
     */
    if (index >= sizeof(inPinTypes) / sizeof(inPinTypes[0])) {
        index = 0;
    }

    return (index);
}

/*
 *  ======== getInterruptTypeIndex ========
 */
static inline uint32_t getInterruptTypeIndex(uint32_t pinConfig)
{
    uint32_t index;

    index = (pinConfig & GPIO_CFG_INT_MASK) >> GPIO_CFG_INT_LSB;

    /*
     * If index is out-of-range, default to 0. This should never
     * happen, but it's needed to keep Klocwork checker happy.
     */
    if (index >= sizeof(interruptType) / sizeof(interruptType[0])) {
        index = 0;
    }

    return (index);
};

/*
 *  ======== getOutPinStrengthsIndex ========
 */
static inline uint32_t getOutPinStrengthsIndex(uint32_t pinConfig)
{
    uint32_t index;

    index = (pinConfig & GPIO_CFG_OUT_STRENGTH_MASK) >>
        GPIO_CFG_OUT_STRENGTH_LSB;

    /*
     * If index is out-of-range, default to 0. This should never
     * happen, but it's needed to keep Klocwork checker happy.
     */
    if (index >= sizeof(outPinStrengths) / sizeof(outPinStrengths[0])) {
        index = 0;
    }

    return (index);
}

/*
 *  ======== getGpioIntIndex ========
 *
 *  Calculates the index into gpioInterruptVectors for a given pinConfig.
 */
static uint8_t getGpioIntIndex(PinConfig *config) {
    /*
     * Ports which have a single interrupt for all pins are stored within the
     * first 15 entries.  Need only return the index.
     */
    if (config->port <= 0x0F) {
        return config->port;
    }

    /*
     *  Ports P & Q have interrupts per each pin on the port.  These have been
     *  placed at the end of the gpioInterruptVectors array.
     *  The gpioInterruptVectors index for each interrupt is determined below.
     */
    return (((config->port - 14) * 8) + getPinNumber(config->pin));
}

/*
 *  ======== GPIO_clearInt ========
 */
void GPIO_clearInt(uint_least8_t index)
{
    PinConfig *config = (PinConfig *) &GPIOMSP432E4_config.pinConfigs[index];

    GPIOIntClear(getPortBase(config->port), config->pin);
}

/*
 *  ======== GPIO_disableInt ========
 */
void GPIO_disableInt(uint_least8_t index)
{
    uintptr_t    key;
    PinConfig   *config = (PinConfig *) &GPIOMSP432E4_config.pinConfigs[index];

    /* Make atomic update */
    key = HwiP_disable();

    GPIOIntDisable(getPortBase(config->port), config->pin);

    HwiP_restore(key);
}

/*
 *  ======== GPIO_enableInt ========
 */
void GPIO_enableInt(uint_least8_t index)
{
    uintptr_t  key;
    PinConfig *config = (PinConfig *) &GPIOMSP432E4_config.pinConfigs[index];

    /* Make atomic update */
    key = HwiP_disable();

    GPIOIntEnable(getPortBase(config->port), config->pin);

    HwiP_restore(key);
}

/*
 *  ======== GPIO_getConfig ========
 */
void GPIO_getConfig(uint_least8_t index, GPIO_PinConfig *pinConfig)
{
    *pinConfig = GPIOMSP432E4_config.pinConfigs[index];
}

/*
 *  ======== GPIO_hwiIntFxn ========
 *  Hwi function that processes GPIO interrupts.
 */
void GPIO_hwiIntFxn(uintptr_t portIndex)
{
    uint32_t          gpioPins;
    uint32_t          gpioBase;
    uint32_t          gpioIndex;
    uint32_t          bitNum;
    PortCallbackInfo *portCallbackInfo;

    portCallbackInfo = &gpioCallbackInfo[portIndex];
    gpioBase = getPortBase(portIndex);

    /* Find out which pins have their interrupt flags set */
    gpioPins = GPIOIntStatus(gpioBase, 0xFF) & 0xFF;

    /* Clear all the set bits at once */
    GPIOIntClear(gpioBase, gpioPins);

    /* Match each set bit to its corresponding callback function */
    while (gpioPins) {
        /* Gets the lowest order set bit number */
        bitNum = getPinNumber(gpioPins);
        gpioIndex = portCallbackInfo->pinIndex[bitNum];
        if (gpioIndex != CALLBACK_INDEX_NOT_CONFIGURED) {
            GPIOMSP432E4_config.callbacks[gpioIndex](gpioIndex);
        }
        gpioPins &= ~(1 << bitNum);
    }
}

/*
 *  ======== GPIO_init ========
 */
void GPIO_init()
{
    unsigned int i, j;
    uintptr_t    key;
    SemaphoreP_Handle sem;
    static SemaphoreP_Handle initSem;

    /* speculatively create a binary semaphore */
    sem = SemaphoreP_createBinary(1);

    /* There is no way to inform user of this fatal error. */
    if (sem == NULL) {
        return;
    }

    key = HwiP_disable();

    if (initSem == NULL) {
        initSem = sem;
        HwiP_restore(key);
    }
    else {
        /* init already called */
        HwiP_restore(key);
        /* delete unused Semaphore */
        SemaphoreP_delete(sem);
    }

    /* now use the semaphore to protect init code */
    SemaphoreP_pend(initSem, SemaphoreP_WAIT_FOREVER);

    /* Only perform init once */
    if (initCalled) {
        SemaphoreP_post(initSem);
        return;
    }

    /* Initialize all entries with 'not configured' key */
    for (i = 0; i < NUM_PORTS; i++) {
        for (j = 0; j < NUM_PINS_PER_PORT; j++) {
            gpioCallbackInfo[i].pinIndex[j] = CALLBACK_INDEX_NOT_CONFIGURED;
        }
    }

    /* Configure pins and create Hwis per static array content */
    for (i = 0; i < GPIOMSP432E4_config.numberOfPinConfigs; i++) {
        if (!(GPIOMSP432E4_config.pinConfigs[i] & GPIO_DO_NOT_CONFIG)) {
            GPIO_setConfig(i, GPIOMSP432E4_config.pinConfigs[i]);
        }
        if (i < GPIOMSP432E4_config.numberOfCallbacks) {
            if (GPIOMSP432E4_config.callbacks[i] != NULL) {
                GPIO_setCallback(i, GPIOMSP432E4_config.callbacks[i]);
            }
        }
    }

    initCalled = true;

    SemaphoreP_post(initSem);
}

/*
 *  ======== GPIO_read ========
 */
uint_fast8_t GPIO_read(uint_least8_t index)
{
    unsigned int value;
    PinConfig   *config = (PinConfig *) &GPIOMSP432E4_config.pinConfigs[index];

    value = GPIOPinRead(getPortBase(config->port), config->pin);

    value = (value & config->pin) ? 1 : 0;

    return (value);
}

/*
 *  ======== GPIO_setCallback ========
 */
void GPIO_setCallback(uint_least8_t index, GPIO_CallbackFxn callback)
{
    uintptr_t    key;
    uint32_t     pinNum;
    uint32_t     portIndex;
    PinConfig   *config = (PinConfig *) &GPIOMSP432E4_config.pinConfigs[index];

    /*
     * Ignore bogus callback indexes.
     * Required to prevent out-of-range callback accesses if
     * there are configured pins without callbacks
     */
    if (index >= GPIOMSP432E4_config.numberOfCallbacks) {
        return;
    }

    pinNum = getPinNumber(config->pin);
    portIndex = config->port;

    /* Make atomic update */
    key = HwiP_disable();

    /* Store index into corresponding port's callbackInfo pinIndex entry */
    gpioCallbackInfo[portIndex].pinIndex[pinNum] =
        (callback) ? index : CALLBACK_INDEX_NOT_CONFIGURED;

    /*
     * Only update callBackFunctions entry if different. This allows the
     * callBackFunctions array to be in flash for static systems.
     */
    if (GPIOMSP432E4_config.callbacks[index] != callback) {
        GPIOMSP432E4_config.callbacks[index] = callback;
    }

    HwiP_restore(key);
}

/*
 *  ======== GPIO_setConfig ========
 */
int_fast16_t GPIO_setConfig(uint_least8_t index, GPIO_PinConfig pinConfig)
{
    uintptr_t      key;
    HwiP_Params    hwiParams;
    HwiP_Handle    handle;
    uint32_t       portBase;
    uint8_t        direction;
    uint8_t        pinMask;
    uint8_t        gpioPortIntIdx;
    uint32_t       gpioPortIntBitMask;
    uint8_t        strength;
    uint8_t        gpioType;
    uint8_t        gpioTypeIdx;
    GPIO_PinConfig gpioPinConfig;
    PinConfig     *config = (PinConfig *) &GPIOMSP432E4_config.pinConfigs[index];

    if (pinConfig & GPIO_DO_NOT_CONFIG) {
        return (GPIO_STATUS_SUCCESS);
    }

    portBase = getPortBase(config->port);
    pinMask = config->pin;

    /* Enable clocks for the GPIO port */
    Power_setDependency(getPowerResource(config->port));

    if ((pinConfig & GPIO_CFG_IN_INT_ONLY) == 0) {
        /* Get GPIO configuration settings */

        strength = outPinStrengths[getOutPinStrengthsIndex(pinConfig)];
        gpioTypeIdx = getInPinTypesIndex(pinConfig);

        /* Determine settings for GPIO as input or output */
        if (pinConfig & GPIO_CFG_INPUT) {
            direction = GPIO_DIR_MODE_IN;
            gpioType = inPinTypes[gpioTypeIdx];
        }
        else {
            direction = GPIO_DIR_MODE_OUT;
            gpioType = outPinTypes[gpioTypeIdx];
        }

        /* Make atomic update */
        key = HwiP_disable();

        /* Configure pad as tristate if transitioning from input to output */
        if ((GPIOMSP432E4_config.pinConfigs[index] & GPIO_CFG_INPUT) &&
            (direction == GPIO_DIR_MODE_OUT)) {
            HWREG(portBase + GPIO_O_DEN) &= (uint32_t) ~pinMask;
        }

        /* Configure the GPIO pin */
        GPIODirModeSet(portBase, pinMask, direction);

        /* Set output value */
        if (direction == GPIO_DIR_MODE_OUT) {
            GPIOPinWrite(portBase, pinMask,
                ((pinConfig & GPIO_CFG_OUT_HIGH) ? pinMask : 0));
        }

        /* Enable GPIO pin */
        GPIOPadConfigSet(portBase, pinMask, strength, gpioType);

        /* Update pinConfig with the latest GPIO configuration */
        gpioPinConfig = GPIOMSP432E4_config.pinConfigs[index];
        gpioPinConfig &= ~GPIO_CFG_IO_MASK;
        gpioPinConfig |= (pinConfig & GPIO_CFG_IO_MASK);
        GPIOMSP432E4_config.pinConfigs[index] = gpioPinConfig;

        HwiP_restore(key);
    }

    /* Set type of interrupt and then clear it */
    if (pinConfig & GPIO_CFG_INT_MASK) {
        /* Calculate the gpioInterruptVectors index for the GPIO pin */
        gpioPortIntIdx = getGpioIntIndex(config);
        gpioPortIntBitMask = 1 << gpioPortIntIdx;

        /* If Hwi has not already been created, do so */
        if ((portHwiCreatedBitMask & gpioPortIntBitMask) == 0) {
            HwiP_Params_init(&hwiParams);
            hwiParams.arg = (uintptr_t) config->port;
            hwiParams.priority = GPIOMSP432E4_config.intPriority;
            handle = HwiP_create(gpioInterruptVectors[gpioPortIntIdx],
                GPIO_hwiIntFxn, &hwiParams);
            if (!handle) {
                return (GPIO_STATUS_ERROR);
            }
        }

        key = HwiP_disable();

        /* Mark the Hwi as created */
        portHwiCreatedBitMask |= gpioPortIntBitMask;

        GPIOIntTypeSet(portBase, pinMask,
                       interruptType[getInterruptTypeIndex(pinConfig)]);
        GPIOIntClear(portBase, pinMask);

        /* Update pinConfig with the latest interrupt configuration */
        gpioPinConfig = GPIOMSP432E4_config.pinConfigs[index];
        gpioPinConfig &= ~(GPIO_CFG_INT_MASK);
        gpioPinConfig |= (pinConfig & GPIO_CFG_INT_MASK);
        GPIOMSP432E4_config.pinConfigs[index] = gpioPinConfig;

        HwiP_restore(key);
    }

    return (GPIO_STATUS_SUCCESS);
}

/*
 *  ======== GPIO_toggle ========
 */
void GPIO_toggle(uint_least8_t index)
{
    uintptr_t    key;
    uint32_t     value;
    PinConfig   *config = (PinConfig *) &GPIOMSP432E4_config.pinConfigs[index];

    /* Make atomic update */
    key = HwiP_disable();

    value = GPIOPinRead(getPortBase(config->port), config->pin);
    value ^= (uint32_t)config->pin;
    GPIOPinWrite(getPortBase(config->port), config->pin, value);

    /* Update pinConfig with new output value */
    GPIOMSP432E4_config.pinConfigs[index] ^= GPIO_CFG_OUT_HIGH;

    HwiP_restore(key);
}

/*
 *  ======== GPIO_write ========
 */
void GPIO_write(uint_least8_t index, unsigned int value)
{
    uintptr_t    key;
    uint32_t     output;
    PinConfig   *config = (PinConfig *) &GPIOMSP432E4_config.pinConfigs[index];

    key = HwiP_disable();

    if (value) {
        output = config->pin;

        /* Set the pinConfig output bit to high */
        GPIOMSP432E4_config.pinConfigs[index] |= GPIO_CFG_OUT_HIGH;
    }
    else {
        /* Clear output from pinConfig */
        GPIOMSP432E4_config.pinConfigs[index] &= ~GPIO_CFG_OUT_HIGH;

        output = value;
    }

    GPIOPinWrite(getPortBase(config->port), config->pin, output);

    HwiP_restore(key);
}

/*
 *  ======== GPIOMSP432E4_getGpioBaseAddr ========
 */
uint32_t GPIOMSP432E4_getGpioBaseAddr(uint8_t port)
{
    if (port >= NUM_PORTS) {
        return ((uint32_t)-1);
    }

    return (gpioBaseAddresses[port]);
}

/*
 *  ======== GPIOMSP432E4_getPowerResourceId ========
 */
uint_fast16_t GPIOMSP432E4_getPowerResourceId(uint8_t port)
{
    if (port >= NUM_PORTS) {
        return ((uint_fast16_t)-1);
    }

    return (getPowerResource(port));
}

/*
 *  ======== GPIOMSP432E4_undoPinConfig ========
 */
void GPIOMSP432E4_undoPinConfig(uint32_t pinConfig)
{
    uint8_t   pin, port;
    uint32_t  shift;
    GPIO_Type *cmsisObj;

    /* Undo the pin config */
    port = GPIOMSP432E4_getPortFromPinConfig(pinConfig);
    pin = GPIOMSP432E4_getPinFromPinConfig(pinConfig);
    shift = (GPIOMSP432E4_getPinMapFromPinConfig(pinConfig) >> 8) & 0xff;
    cmsisObj = (GPIO_Type *)GPIOMSP432E4_getGpioBaseAddr(port);
    cmsisObj->PCTL &= ~(0xf << shift);
    cmsisObj->AFSEL &= ~((uint32_t) pin);
}
