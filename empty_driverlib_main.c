
//
// FILE: empty_driverlib_main.c
// Project: Ollagajo - 1kW PV-CLLC Controller
// MCU: TMS320F28388D CPU1
// C2000Ware: 26.01.00.00
// Version: v2.9 - Control Timing and Trip Diagnostics
//
// Development firmware. Not validated on physical hardware.
//
// IMPORTANT:
// - PWM GPIO mux is not configured.
// - ePWM1 through ePWM8 counters remain stopped.
// - One-Shot Trip is never intentionally cleared.
// - No gate-driver, fan or relay output is enabled.
// - Sensor calibration remains provisional.
// - Active fault polarity must be confirmed on the PCB.
// - There is no closed-loop voltage controller yet.
//

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"

// ============================================================
// 1. HARDWARE AND CONTROL CONSTANTS
// ============================================================

#define PVCLLC_ADC_WINDOW              20U
#define PVCLLC_ADC_TIMEOUT             1000U
#define PVCLLC_ADC_RATE_HZ             1000UL
#define PVCLLC_TELEMETRY_TICKS         100UL

#define PVCLLC_VI_ALPHA                0.20f
#define PVCLLC_TEMP_ALPHA              0.05f

#define PVCLLC_ADC_SCALE               (3.3f / 4096.0f)

#define PVCLLC_VPV_GAIN                0.078f
#define PVCLLC_VDC_GAIN                0.057f

// Hardware mapping is provisional until PCB inspection.

#define PVCLLC_GPIO_GD_FAULT1          88U
#define PVCLLC_GPIO_GD_FAULT2          91U

#define PVCLLC_GPIO_FAN_EN             42U
#define PVCLLC_GPIO_RELAY1             46U
#define PVCLLC_GPIO_RELAY2             47U

#define PVCLLC_GPIO_LED_BLINK          68U
#define PVCLLC_GPIO_LED_RUN            69U
#define PVCLLC_GPIO_LED_ERROR          70U
#define PVCLLC_GPIO_LED_FORWARD        71U
#define PVCLLC_GPIO_LED_BACKWARD       73U
#define PVCLLC_GPIO_LED_COMM           74U

// Only the two GD_FAULT pins are initialized.

// ============================================================
// 2. DIAGNOSTIC FLAGS
// ============================================================

#define PVCLLC_STATE_NORMAL            0U
#define PVCLLC_STATE_WARNING           1U
#define PVCLLC_STATE_FAULT             2U

#define PVCLLC_DIAG_ADC_INVALID        0x0001U
#define PVCLLC_DIAG_ADC_TIMEOUT        0x0002U
#define PVCLLC_DIAG_ADC_OVERFLOW       0x0004U
#define PVCLLC_DIAG_ADC_OVERRUN        0x0008U

#define PVCLLC_DIAG_GD1_CHANGE         0x0010U
#define PVCLLC_DIAG_GD2_CHANGE         0x0020U
#define PVCLLC_DIAG_ADC_RAIL           0x0040U

#define PVCLLC_DIAG_ISR_BUDGET         0x0080U
#define PVCLLC_DIAG_PWM_TRIP           0x0100U
#define PVCLLC_DIAG_PWM_INIT           0x0200U

#define PVCLLC_FAULT_MASK              0x0307U
#define PVCLLC_WARNING_MASK            0x00F8U

#define PVCLLC_CLEAR_VALID_SAMPLES     100UL
#define PVCLLC_WARNING_HOLD_TICKS      100UL

#define PVCLLC_ADC_RAIL_THRESHOLD      4094U
#define PVCLLC_ADC_RAIL_LIMIT          5UL
#define PVCLLC_ADC_CHANNELS            9U

// ============================================================
// 3. ePWM / PFM PARAMETERS
// ============================================================

#define PVCLLC_PFM_NOMINAL_HZ          300000UL
#define PVCLLC_PFM_TEST_MIN_HZ         200000UL
#define PVCLLC_PFM_TEST_MAX_HZ         400000UL

#define PVCLLC_CONTROL_RATE_HZ         2500UL

#define PVCLLC_DEADTIME_TEST_NS        200UL
#define PVCLLC_EPWM_COUNT              8U

// CPU Timer1 period: nominally 400 us.
//
// The 200 us execution budget below is a provisional
// diagnostic threshold, not a validated deadline.

#define PVCLLC_CONTROL_PERIOD_TICKS \
    (DEVICE_SYSCLK_FREQ / PVCLLC_CONTROL_RATE_HZ)

#define PVCLLC_CONTROL_BUDGET_TICKS \
    (PVCLLC_CONTROL_PERIOD_TICKS / 2UL)

// PFM request rejection reasons.

#define PVCLLC_PFM_REJECT_NONE         0U
#define PVCLLC_PFM_REJECT_RANGE        1U
#define PVCLLC_PFM_REJECT_PERIOD       2U
#define PVCLLC_PFM_REJECT_DEADTIME     3U

// The current PWM period calculation assumes:
// EPWMCLKDIV = /1
// HSPCLKDIV = /1
// CLKDIV = /1
//
// The hardware clock tree must be verified before
// enabling physical PWM output.

static const uint32_t pvcllcEPWMBase[8] =
{
    EPWM1_BASE,
    EPWM2_BASE,
    EPWM3_BASE,
    EPWM4_BASE,
    EPWM5_BASE,
    EPWM6_BASE,
    EPWM7_BASE,
    EPWM8_BASE
};

// ============================================================
// 4. ADC RAW VARIABLES
// ============================================================

volatile uint16_t adcVpvRaw = 0U;
volatile uint16_t adcIpvRaw = 0U;
volatile uint16_t adcM1Raw = 0U;
volatile uint16_t adcM2Raw = 0U;

volatile uint16_t adcIdcRaw = 0U;
volatile uint16_t adcVdcRaw = 0U;
volatile uint16_t adcTR1Raw = 0U;
volatile uint16_t adcTR2Raw = 0U;
volatile uint16_t adcTR3Raw = 0U;

// ============================================================
// 5. ADC STATUS
// ============================================================

volatile uint32_t adcSampleCount = 0UL;
volatile uint32_t adcTimeoutCount = 0UL;
volatile uint32_t adcOverflowCount = 0UL;
volatile uint32_t adcPeriodCount = 0UL;
volatile uint32_t adcOverrunCount = 0UL;

volatile uint16_t adcDataValid = 0U;
volatile uint16_t adcFilterReady = 0U;

// ============================================================
// 6. GPIO FAULT DIAGNOSTICS
// ============================================================

volatile uint16_t gdFault1RawLevel = 0U;
volatile uint16_t gdFault2RawLevel = 0U;

volatile uint16_t gdFaultInputsReady = 0U;
volatile uint16_t gdFaultChangeThisCycle = 0U;

volatile uint32_t gdFault1TransitionCount = 0UL;
volatile uint32_t gdFault2TransitionCount = 0UL;
volatile uint32_t gpioDiagnosticSampleCount = 0UL;

// ============================================================
// 7. SOFTWARE PROTECTION STATE
// ============================================================

volatile uint16_t pvcllcSystemState =
    PVCLLC_STATE_WARNING;

volatile uint16_t pvcllcPreviousState =
    PVCLLC_STATE_WARNING;

volatile uint16_t pvcllcFaultLatched = 0U;
volatile uint16_t pvcllcFaultClearRequest = 0U;

volatile uint16_t pvcllcDiagnosticFlags = 0U;
volatile uint16_t pvcllcDiagnosticLatchedFlags = 0U;
volatile uint16_t pvcllcFaultReasonLatched = 0U;

volatile uint32_t pvcllcFaultEventCount = 0UL;
volatile uint32_t pvcllcWarningEventCount = 0UL;
volatile uint32_t pvcllcStateTransitionCount = 0UL;

volatile uint32_t pvcllcFaultClearCount = 0UL;
volatile uint32_t pvcllcFaultClearDeniedCount = 0UL;

volatile uint32_t pvcllcValidSampleStreak = 0UL;
volatile uint32_t pvcllcWarningHoldTicks = 0UL;
volatile uint32_t pvcllcInvalidAdcCycleCount = 0UL;

volatile uint16_t pvcllcGateEnableAllowed = 0U;

// ============================================================
// 8. ADC RAIL DIAGNOSTICS
// ============================================================

volatile uint16_t adcRailHighMask = 0U;
volatile uint16_t adcRailPersistentMask = 0U;
volatile uint16_t adcRailHistoryMask = 0U;

volatile uint32_t adcRailHighStreak[9] = {0UL};
volatile uint32_t adcRailEventCount = 0UL;

// ============================================================
// 9. ADC INPUT PIN VOLTAGES
// ============================================================

volatile float Vpv_ADC_V = 0.0f;
volatile float Ipv_ADC_V = 0.0f;

volatile float Vdc_ADC_V = 0.0f;
volatile float Idc_ADC_V = 0.0f;

volatile float M1_ADC_V = 0.0f;
volatile float M2_ADC_V = 0.0f;

volatile float TR1_ADC_V = 0.0f;
volatile float TR2_ADC_V = 0.0f;
volatile float TR3_ADC_V = 0.0f;

// ============================================================
// 10. EMA FILTERED ADC VOLTAGES
// ============================================================

volatile float Vpv_ADC_Filtered_V = 0.0f;
volatile float Ipv_ADC_Filtered_V = 0.0f;

volatile float Vdc_ADC_Filtered_V = 0.0f;
volatile float Idc_ADC_Filtered_V = 0.0f;

volatile float M1_ADC_Filtered_V = 0.0f;
volatile float M2_ADC_Filtered_V = 0.0f;

volatile float TR1_ADC_Filtered_V = 0.0f;
volatile float TR2_ADC_Filtered_V = 0.0f;
volatile float TR3_ADC_Filtered_V = 0.0f;

// ============================================================
// 11. PROVISIONAL PHYSICAL VOLTAGES
// ============================================================

volatile float Vpv_V = 0.0f;
volatile float Vdc_V = 0.0f;

volatile float Vpv_Filtered_V = 0.0f;
volatile float Vdc_Filtered_V = 0.0f;

// ============================================================
// 12. TELEMETRY SNAPSHOT
// ============================================================

volatile float telemetryVpvAdcV = 0.0f;
volatile float telemetryIpvAdcV = 0.0f;

volatile float telemetryVdcAdcV = 0.0f;
volatile float telemetryIdcAdcV = 0.0f;

volatile float telemetryM1AdcV = 0.0f;
volatile float telemetryM2AdcV = 0.0f;

volatile float telemetryTR1AdcV = 0.0f;
volatile float telemetryTR2AdcV = 0.0f;
volatile float telemetryTR3AdcV = 0.0f;

volatile uint16_t telemetryValid = 0U;
volatile uint16_t telemetrySystemState = 1U;
volatile uint16_t telemetryAdcRailMask = 0U;

volatile uint32_t telemetrySequence = 0UL;
volatile uint32_t telemetrySourceSampleCount = 0UL;

// v2.9 telemetry additions.

volatile uint32_t telemetryPfmAppliedHz = 0UL;
volatile uint32_t telemetryISRMaxTicks = 0UL;
volatile uint16_t telemetryPwmTripMissingMask = 0U;

// ============================================================
// 13. ePWM / PFM STATUS
// ============================================================

volatile uint32_t pvcllcPfmRequestedHz =
    PVCLLC_PFM_NOMINAL_HZ;

volatile uint32_t pvcllcPfmAppliedHz = 0UL;
volatile uint32_t pvcllcPfmActualHz = 0UL;

volatile uint16_t pvcllcPwmPeriodCounts = 0U;
volatile uint16_t pvcllcPwmCompareCounts = 0U;
volatile uint16_t pvcllcPwmDeadtimeCounts = 0U;

volatile uint16_t pvcllcPwmInitialized = 0U;
volatile uint16_t pvcllcPwmOutputsEnabled = 0U;
volatile uint16_t pvcllcPwmTripCommanded = 0U;

volatile uint32_t pvcllcPfmPollCount = 0UL;
volatile uint32_t pvcllcPfmUpdateCount = 0UL;
volatile uint32_t pvcllcPfmRejectCount = 0UL;
volatile uint32_t pvcllcPfmOverrunCount = 0UL;

// v2.9 PFM diagnostics.

volatile uint16_t pvcllcPfmLastRejectReason =
    PVCLLC_PFM_REJECT_NONE;

volatile uint32_t pvcllcPfmLastRejectedHz = 0UL;
volatile uint32_t pvcllcPfmLastAcceptedHz =
    PVCLLC_PFM_NOMINAL_HZ;

// ============================================================
// 14. ISR TIMING DIAGNOSTICS
// ============================================================

volatile uint32_t pvcllcControlISRCount = 0UL;
volatile uint32_t pvcllcControlISRLastTicks = 0UL;
volatile uint32_t pvcllcControlISRMaxTicks = 0UL;

volatile uint16_t pvcllcControlISRReady = 0U;

// These counts are diagnostic indications only.
// Actual ISR jitter and latency need hardware measurement.

volatile uint32_t pvcllcControlISRBudgetTicks =
    PVCLLC_CONTROL_BUDGET_TICKS;

volatile uint32_t pvcllcControlISRBudgetEventCount =
    0UL;

volatile uint32_t pvcllcControlISRWrapCount = 0UL;

volatile uint16_t pvcllcControlTimingWarning = 0U;

// ============================================================
// 15. TRIP ZONE DIAGNOSTICS
// ============================================================

// Bit 0 corresponds to ePWM1.
// Bit 7 corresponds to ePWM8.
//
// A set verified bit means the peripheral OST flag
// was observed. It does NOT prove physical pin safety.

volatile uint16_t pvcllcPwmTripVerifiedMask = 0U;
volatile uint16_t pvcllcPwmTripMissingMask = 0U;
volatile uint16_t pvcllcPwmTripHistoryMask = 0U;

volatile uint32_t pvcllcPwmTripReassertCount = 0UL;

volatile uint16_t pvcllcPwmInitError = 0U;

// ============================================================
// 16. FUNCTION DECLARATIONS
// ============================================================

static void PVCLLC_Timer_init(void);

static void PVCLLC_ADC_init(void);
static void PVCLLC_ADC_read(void);
static void PVCLLC_ADC_clearFlags(void);
static void PVCLLC_ADC_process(void);
static void PVCLLC_ADC_checkRails(void);

static void PVCLLC_GPIO_initDiagnostics(void);
static void PVCLLC_GPIO_pollDiagnostics(void);

static void PVCLLC_updateProtectionState(
    uint32_t timeoutBefore,
    uint32_t overflowBefore);

static float PVCLLC_lowPass(
    float oldValue,
    float newValue,
    float alpha);

static void PVCLLC_publishTelemetry(void);

static void PVCLLC_PWM_init(void);
static void PVCLLC_PWM_forceAllTrip(void);
static void PVCLLC_PWM_checkTrip(void);

static uint16_t PVCLLC_PWM_calculatePeriod(
    uint32_t frequencyHz);

static uint16_t PVCLLC_PWM_calculateDeadtime(void);

static void PVCLLC_PFM_processRequest(void);

// CPU Timer1 ISR.

__interrupt void PVCLLC_CPUtimer1ISR(void);

// ============================================================
// 17. MAIN
// ============================================================

void main(void)
{
    uint32_t timeoutBefore;
    uint32_t overflowBefore;

    Device_init();
    Device_initGPIO();

    Interrupt_initModule();
    Interrupt_initVectorTable();

    Board_init();
    C2000Ware_libraries_init();

    PVCLLC_ADC_init();
    PVCLLC_GPIO_initDiagnostics();

    // Configure internal ePWM registers.
    // Counters and physical output remain disabled.

    PVCLLC_PWM_init();

    // CPU Timer0: 1ms monitoring.
    // CPU Timer1: 400us interrupt.

    PVCLLC_Timer_init();

    Interrupt_register(
        INT_TIMER1,
        &PVCLLC_CPUtimer1ISR);

    Interrupt_enable(INT_TIMER1);

    CPUTimer_enableInterrupt(CPUTIMER1_BASE);

    pvcllcControlISRReady = 1U;

    CPUTimer_startTimer(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER1_BASE);

    EINT;
    ERTM;

    while(1)
    {
        // ----------------------------------------------------
        // 1ms ADC monitoring task
        // ----------------------------------------------------

        if(CPUTimer_getTimerOverflowStatus(
            CPUTIMER0_BASE))
        {
            CPUTimer_clearOverflowFlag(
                CPUTIMER0_BASE);

            adcPeriodCount++;

            timeoutBefore = adcTimeoutCount;
            overflowBefore = adcOverflowCount;

            PVCLLC_ADC_read();

            if(adcDataValid != 0U)
            {
                PVCLLC_ADC_checkRails();
            }

            PVCLLC_GPIO_pollDiagnostics();

            // The hardware Trip diagnostic is polled
            // separately from the control ISR.

            PVCLLC_PWM_checkTrip();

            PVCLLC_updateProtectionState(
                timeoutBefore,
                overflowBefore);

            if((adcPeriodCount %
                PVCLLC_TELEMETRY_TICKS) == 0UL)
            {
                if(adcDataValid != 0U)
                {
                    PVCLLC_publishTelemetry();
                }
                else
                {
                    telemetryValid = 0U;
                }
            }

            if(adcDataValid == 0U)
            {
                telemetryValid = 0U;
            }
        }

        // No physical output enable path exists.

        pvcllcGateEnableAllowed = 0U;
        pvcllcPwmOutputsEnabled = 0U;
    }
}

// ============================================================
// 18. CPU TIMER1 ISR - 400us
// ============================================================

__interrupt void PVCLLC_CPUtimer1ISR(void)
{
    uint32_t startCount;
    uint32_t endCount;
    uint32_t elapsedTicks;

    startCount =
        CPUTimer_getTimerCount(CPUTIMER1_BASE);

    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);

    pvcllcControlISRCount++;
    pvcllcPfmPollCount++;

    // Process PFM request.
    // All ePWM counters stay STOP/FREEZE.

    PVCLLC_PFM_processRequest();

    endCount =
        CPUTimer_getTimerCount(CPUTIMER1_BASE);

    // Timer1 is a down-counter.
    // This estimates execution time in timer ticks.

    if(startCount >= endCount)
    {
        elapsedTicks = startCount - endCount;
    }
    else
    {
        // Possible single timer wrap.
        // Multiple wraps cannot be resolved this way.

        elapsedTicks =
            startCount +
            PVCLLC_CONTROL_PERIOD_TICKS -
            endCount;

        pvcllcControlISRWrapCount++;
    }

    pvcllcControlISRLastTicks = elapsedTicks;

    if(elapsedTicks > pvcllcControlISRMaxTicks)
    {
        pvcllcControlISRMaxTicks = elapsedTicks;
    }

    // Provisional 50 percent period budget.

    if(elapsedTicks > pvcllcControlISRBudgetTicks)
    {
        pvcllcControlISRBudgetEventCount++;

        pvcllcControlTimingWarning = 1U;
    }
    else
    {
        pvcllcControlTimingWarning = 0U;
    }

    // Detect a possible full timer-period overflow
    // occurring while executing this ISR.

    if(CPUTimer_getTimerOverflowStatus(
        CPUTIMER1_BASE))
    {
        pvcllcPfmOverrunCount++;

        pvcllcControlTimingWarning = 1U;
    }

    // CPU Timer1 uses CPU INT13 directly.
    // No PIE group acknowledgment is required.
}

// ============================================================
// 19. CPU TIMERS
// ============================================================

static void PVCLLC_Timer_init(void)
{
    // Timer0 - 1ms polled ADC sampling.

    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_disableInterrupt(CPUTIMER0_BASE);

    CPUTimer_setPreScaler(
        CPUTIMER0_BASE, 0U);

    CPUTimer_setPeriod(
        CPUTIMER0_BASE,
        (DEVICE_SYSCLK_FREQ /
         PVCLLC_ADC_RATE_HZ) - 1UL);

    CPUTimer_setEmulationMode(
        CPUTIMER0_BASE,
        CPUTIMER_EMULATIONMODE_RUNFREE);

    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);

    // Timer1 - 400us interrupt.

    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_disableInterrupt(CPUTIMER1_BASE);

    CPUTimer_setPreScaler(
        CPUTIMER1_BASE, 0U);

    CPUTimer_setPeriod(
        CPUTIMER1_BASE,
        PVCLLC_CONTROL_PERIOD_TICKS - 1UL);

    CPUTimer_setEmulationMode(
        CPUTIMER1_BASE,
        CPUTIMER_EMULATIONMODE_RUNFREE);

    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);
}

// ============================================================
// 20. GPIO DIAGNOSTIC INITIALIZATION
// ============================================================

static void PVCLLC_GPIO_initDiagnostics(void)
{
    GPIO_setPinConfig(GPIO_88_GPIO88);

    GPIO_setDirectionMode(
        PVCLLC_GPIO_GD_FAULT1,
        GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(
        PVCLLC_GPIO_GD_FAULT1,
        GPIO_PIN_TYPE_STD);

    GPIO_setQualificationMode(
        PVCLLC_GPIO_GD_FAULT1,
        GPIO_QUAL_SYNC);

    GPIO_setPinConfig(GPIO_91_GPIO91);

    GPIO_setDirectionMode(
        PVCLLC_GPIO_GD_FAULT2,
        GPIO_DIR_MODE_IN);

    GPIO_setPadConfig(
        PVCLLC_GPIO_GD_FAULT2,
        GPIO_PIN_TYPE_STD);

    GPIO_setQualificationMode(
        PVCLLC_GPIO_GD_FAULT2,
        GPIO_QUAL_SYNC);

    gdFault1RawLevel =
        (uint16_t)GPIO_readPin(
            PVCLLC_GPIO_GD_FAULT1);

    gdFault2RawLevel =
        (uint16_t)GPIO_readPin(
            PVCLLC_GPIO_GD_FAULT2);

    gdFaultInputsReady = 1U;
}

// ============================================================
// 21. GPIO DIAGNOSTIC POLLING
// ============================================================

static void PVCLLC_GPIO_pollDiagnostics(void)
{
    uint16_t fault1;
    uint16_t fault2;

    fault1 = (uint16_t)GPIO_readPin(
        PVCLLC_GPIO_GD_FAULT1);

    fault2 = (uint16_t)GPIO_readPin(
        PVCLLC_GPIO_GD_FAULT2);

    gdFaultChangeThisCycle = 0U;

    if(fault1 != gdFault1RawLevel)
    {
        gdFault1TransitionCount++;

        gdFaultChangeThisCycle |=
            PVCLLC_DIAG_GD1_CHANGE;
    }

    if(fault2 != gdFault2RawLevel)
    {
        gdFault2TransitionCount++;

        gdFaultChangeThisCycle |=
            PVCLLC_DIAG_GD2_CHANGE;
    }

    gdFault1RawLevel = fault1;
    gdFault2RawLevel = fault2;

    gpioDiagnosticSampleCount++;
}

// ============================================================
// 22. ADC RAIL DIAGNOSTICS
// ============================================================

static void PVCLLC_ADC_checkRails(void)
{
    uint16_t values[9];
    uint16_t i;
    uint16_t bit;

    uint16_t highMask = 0U;
    uint16_t persistentMask = 0U;

    values[0] = adcVpvRaw;
    values[1] = adcIpvRaw;
    values[2] = adcM1Raw;
    values[3] = adcM2Raw;

    values[4] = adcIdcRaw;
    values[5] = adcVdcRaw;

    values[6] = adcTR1Raw;
    values[7] = adcTR2Raw;
    values[8] = adcTR3Raw;

    for(i = 0U; i < PVCLLC_ADC_CHANNELS; i++)
    {
        bit = (uint16_t)(1U << i);

        if(values[i] >= PVCLLC_ADC_RAIL_THRESHOLD)
        {
            highMask |= bit;

            if(adcRailHighStreak[i] <
               PVCLLC_ADC_RAIL_LIMIT)
            {
                adcRailHighStreak[i]++;
            }

            if(adcRailHighStreak[i] >=
               PVCLLC_ADC_RAIL_LIMIT)
            {
                persistentMask |= bit;

                if((adcRailPersistentMask & bit) == 0U)
                {
                    adcRailEventCount++;
                }
            }
        }
        else
        {
            adcRailHighStreak[i] = 0UL;
        }
    }

    adcRailHighMask = highMask;
    adcRailPersistentMask = persistentMask;
    adcRailHistoryMask |= persistentMask;
}

// ============================================================
// 23. SOFTWARE PROTECTION STATE
// ============================================================

static void PVCLLC_updateProtectionState(
    uint32_t timeoutBefore,
    uint32_t overflowBefore)
{
    uint16_t activeFaults;
    uint16_t currentWarnings;

    pvcllcPreviousState = pvcllcSystemState;
    pvcllcDiagnosticFlags = 0U;

    // ADC data validity.

    if(adcDataValid == 0U)
    {
        pvcllcDiagnosticFlags |=
            PVCLLC_DIAG_ADC_INVALID;

        pvcllcInvalidAdcCycleCount++;
        pvcllcValidSampleStreak = 0UL;
    }
    else if(pvcllcValidSampleStreak <
            PVCLLC_CLEAR_VALID_SAMPLES)
    {
        pvcllcValidSampleStreak++;
    }

    // ADC conversion diagnostics.

    if(adcTimeoutCount != timeoutBefore)
    {
        pvcllcDiagnosticFlags |=
            PVCLLC_DIAG_ADC_TIMEOUT;
    }

    if(adcOverflowCount != overflowBefore)
    {
        pvcllcDiagnosticFlags |=
            PVCLLC_DIAG_ADC_OVERFLOW;
    }

    // GPIO changes - not interpreted as active faults.

    pvcllcDiagnosticFlags |=
        gdFaultChangeThisCycle;

    // Persistent near-upper-rail ADC values.

    if((adcDataValid != 0U) &&
       (adcRailPersistentMask != 0U))
    {
        pvcllcDiagnosticFlags |=
            PVCLLC_DIAG_ADC_RAIL;
    }

    // Timer0 possible sampling overrun.

    if(CPUTimer_getTimerOverflowStatus(
        CPUTIMER0_BASE))
    {
        adcOverrunCount++;

        pvcllcDiagnosticFlags |=
            PVCLLC_DIAG_ADC_OVERRUN;
    }

    // v2.9 control ISR timing diagnostic.

    if(pvcllcControlTimingWarning != 0U)
    {
        pvcllcDiagnosticFlags |=
            PVCLLC_DIAG_ISR_BUDGET;
    }

    // v2.9 PWM internal trip diagnostic.

    if(pvcllcPwmTripMissingMask != 0U)
    {
        pvcllcDiagnosticFlags |=
            PVCLLC_DIAG_PWM_TRIP;
    }

    if(pvcllcPwmInitError != 0U)
    {
        pvcllcDiagnosticFlags |=
            PVCLLC_DIAG_PWM_INIT;
    }

    // Save diagnostic history.

    pvcllcDiagnosticLatchedFlags |=
        pvcllcDiagnosticFlags;

    activeFaults =
        pvcllcDiagnosticFlags & PVCLLC_FAULT_MASK;

    currentWarnings =
        pvcllcDiagnosticFlags & PVCLLC_WARNING_MASK;

    // Latch software faults.

    if(activeFaults != 0U)
    {
        pvcllcFaultReasonLatched |= activeFaults;

        if(pvcllcFaultLatched == 0U)
        {
            pvcllcFaultEventCount++;
        }

        pvcllcFaultLatched = 1U;
    }

    // Hold warnings temporarily.

    if(currentWarnings != 0U)
    {
        pvcllcWarningHoldTicks =
            PVCLLC_WARNING_HOLD_TICKS;
    }
    else if(pvcllcWarningHoldTicks > 0UL)
    {
        pvcllcWarningHoldTicks--;
    }

    // Manual software fault clear request.
    //
    // This never clears PWM Trip flags and
    // never enables gate-driver outputs.

    if(pvcllcFaultClearRequest != 0U)
    {
        if((pvcllcFaultLatched != 0U) &&
           (activeFaults == 0U) &&
           (pvcllcValidSampleStreak >=
            PVCLLC_CLEAR_VALID_SAMPLES))
        {
            pvcllcFaultLatched = 0U;
            pvcllcFaultReasonLatched = 0U;

            pvcllcFaultClearCount++;
        }
        else
        {
            pvcllcFaultClearDeniedCount++;
        }

        pvcllcFaultClearRequest = 0U;
    }

    // Determine software state.

    if(pvcllcFaultLatched != 0U)
    {
        pvcllcSystemState =
            PVCLLC_STATE_FAULT;
    }
    else if((adcDataValid == 0U) ||
            (pvcllcWarningHoldTicks > 0UL))
    {
        pvcllcSystemState =
            PVCLLC_STATE_WARNING;
    }
    else
    {
        pvcllcSystemState =
            PVCLLC_STATE_NORMAL;
    }

    if(pvcllcSystemState != pvcllcPreviousState)
    {
        pvcllcStateTransitionCount++;

        if(pvcllcSystemState ==
           PVCLLC_STATE_WARNING)
        {
            pvcllcWarningEventCount++;
        }
    }

    pvcllcGateEnableAllowed = 0U;
}

// ============================================================
// 24. ePWM FORCE ALL TRIPS
// ============================================================

static void PVCLLC_PWM_forceAllTrip(void)
{
    uint16_t i;

    for(i = 0U; i < PVCLLC_EPWM_COUNT; i++)
    {
        EPWM_forceTripZoneEvent(
            pvcllcEPWMBase[i],
            EPWM_TZ_FORCE_EVENT_OST);
    }

    pvcllcPwmTripCommanded = 1U;
    pvcllcPwmOutputsEnabled = 0U;
}

// ============================================================
// 25. ePWM TRIP VERIFICATION
// ============================================================

static void PVCLLC_PWM_checkTrip(void)
{
    uint16_t i;
    uint16_t bit;
    uint16_t flags;

    uint16_t verified = 0U;
    uint16_t missing = 0U;

    uint32_t base;

    if(pvcllcPwmInitialized == 0U)
    {
        pvcllcPwmTripMissingMask = 0x00FFU;
        return;
    }

    for(i = 0U; i < PVCLLC_EPWM_COUNT; i++)
    {
        bit = (uint16_t)(1U << i);
        base = pvcllcEPWMBase[i];

        flags = EPWM_getTripZoneFlagStatus(base);

        if((flags & EPWM_TZ_FLAG_OST) == 0U)
        {
            // Attempt to restore internal OST latch.
            // Never clear the trip flag in this version.

            EPWM_forceTripZoneEvent(
                base,
                EPWM_TZ_FORCE_EVENT_OST);

            pvcllcPwmTripReassertCount++;

            flags = EPWM_getTripZoneFlagStatus(base);
        }

        if((flags & EPWM_TZ_FLAG_OST) != 0U)
        {
            verified |= bit;
        }
        else
        {
            missing |= bit;
        }
    }

    pvcllcPwmTripVerifiedMask = verified;
    pvcllcPwmTripMissingMask = missing;

    pvcllcPwmTripHistoryMask |= missing;
}

// ============================================================
// 26. ePWM PERIOD CALCULATION
// ============================================================

static uint16_t PVCLLC_PWM_calculatePeriod(
    uint32_t frequencyHz)
{
    uint32_t counts;

    if(frequencyHz == 0UL)
    {
        return 0U;
    }

    // Up-down mode nominal frequency.
    // Requires EPWMCLK and TBCLK clock validation.

    counts =
        (DEVICE_SYSCLK_FREQ + frequencyHz) /
        (2UL * frequencyHz);

    if((counts < 10UL) || (counts > 65535UL))
    {
        return 0U;
    }

    return (uint16_t)counts;
}

// ============================================================
// 27. DEADTIME CALCULATION
// ============================================================

static uint16_t PVCLLC_PWM_calculateDeadtime(void)
{
    uint32_t ticks;

    ticks =
        ((DEVICE_SYSCLK_FREQ / 1000000UL) *
         PVCLLC_DEADTIME_TEST_NS + 500UL) /
        1000UL;

    if(ticks > 1023UL)
    {
        ticks = 1023UL;
    }

    return (uint16_t)ticks;
}

// ============================================================
// 28. ePWM1-8 INITIALIZATION
// ============================================================

static void PVCLLC_PWM_init(void)
{
    uint16_t i;
    uint16_t period;
    uint16_t compare;
    uint16_t deadtime;

    uint32_t base;

    pvcllcPwmInitialized = 0U;
    pvcllcPwmInitError = 0U;

    // Stop the PWM time-base synchronization clock.

    SysCtl_disablePeripheral(
        SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Existing development clock assumption.
    // Recheck for final PCB and silicon revision.

    SysCtl_setEPWMClockDivider(
        SYSCTL_EPWMCLK_DIV_1);

    period = PVCLLC_PWM_calculatePeriod(
        PVCLLC_PFM_NOMINAL_HZ);

    deadtime = PVCLLC_PWM_calculateDeadtime();

    if((period == 0U) ||
       (((uint32_t)deadtime * 2UL) >=
        (uint32_t)period))
    {
        pvcllcPwmInitError = 1U;

        // TBCLKSYNC stays disabled on initialization
        // failure. No physical output is configured.

        return;
    }

    compare = period / 2U;

    pvcllcPwmPeriodCounts = period;
    pvcllcPwmCompareCounts = compare;
    pvcllcPwmDeadtimeCounts = deadtime;

    for(i = 0U; i < PVCLLC_EPWM_COUNT; i++)
    {
        base = pvcllcEPWMBase[i];

        EPWM_disableADCTrigger(
            base, EPWM_SOC_A);

        EPWM_disableADCTrigger(
            base, EPWM_SOC_B);

        EPWM_disableInterrupt(base);

        EPWM_setClockPrescaler(
            base,
            EPWM_CLOCK_DIVIDER_1,
            EPWM_HSCLOCK_DIVIDER_1);

        EPWM_setTimeBaseCounterMode(
            base,
            EPWM_COUNTER_MODE_STOP_FREEZE);

        // Direct register loading is only used because
        // all eight PWM time-bases remain stopped.
        //
        // Live operation will require synchronized
        // shadow loading and verified phase relations.

        EPWM_setPeriodLoadMode(
            base,
            EPWM_PERIOD_DIRECT_LOAD);

        EPWM_disableCounterCompareShadowLoadMode(
            base,
            EPWM_COUNTER_COMPARE_A);

        EPWM_setTimeBasePeriod(base, period);
        EPWM_setTimeBaseCounter(base, 0U);

        EPWM_disablePhaseShiftLoad(base);

        EPWM_setCounterCompareValue(
            base,
            EPWM_COUNTER_COMPARE_A,
            compare);

        EPWM_setActionQualifierAction(
            base,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_HIGH,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(
            base,
            EPWM_AQ_OUTPUT_A,
            EPWM_AQ_OUTPUT_LOW,
            EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

        // Provisional complementary waveform pattern.
        // Not validated against the actual gate drivers.

        EPWM_setDeadBandCounterClock(
            base,
            EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

        EPWM_setRisingEdgeDeadBandDelayInput(
            base,
            EPWM_DB_INPUT_EPWMA);

        EPWM_setFallingEdgeDeadBandDelayInput(
            base,
            EPWM_DB_INPUT_EPWMA);

        EPWM_setDeadBandDelayPolarity(
            base,
            EPWM_DB_RED,
            EPWM_DB_POLARITY_ACTIVE_HIGH);

        EPWM_setDeadBandDelayPolarity(
            base,
            EPWM_DB_FED,
            EPWM_DB_POLARITY_ACTIVE_LOW);

        EPWM_setRisingEdgeDelayCount(
            base, deadtime);

        EPWM_setFallingEdgeDelayCount(
            base, deadtime);

        EPWM_setDeadBandDelayMode(
            base,
            EPWM_DB_RED,
            true);

        EPWM_setDeadBandDelayMode(
            base,
            EPWM_DB_FED,
            true);

        // Trip Zone.
        //
        // Output LOW is a provisional register state.
        // It is not assumed to be a safe physical
        // level for every possible gate driver.

        EPWM_disableTripZoneAdvAction(base);

        EPWM_setTripZoneAction(
            base,
            EPWM_TZ_ACTION_EVENT_TZA,
            EPWM_TZ_ACTION_LOW);

        EPWM_setTripZoneAction(
            base,
            EPWM_TZ_ACTION_EVENT_TZB,
            EPWM_TZ_ACTION_LOW);
    }

    SysCtl_enablePeripheral(
        SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // Force OST on every module.

    PVCLLC_PWM_forceAllTrip();

    pvcllcPfmAppliedHz =
        PVCLLC_PFM_NOMINAL_HZ;

    // Theoretical setting, not measured output.

    pvcllcPfmActualHz =
        DEVICE_SYSCLK_FREQ /
        (2UL * (uint32_t)period);

    pvcllcPwmInitialized = 1U;

    PVCLLC_PWM_checkTrip();
}

// ============================================================
// 29. PFM FREQUENCY REQUEST PROCESSING
// ============================================================

static void PVCLLC_PFM_processRequest(void)
{
    uint32_t requested;
    uint32_t base;

    uint16_t period;
    uint16_t compare;
    uint16_t i;

    if(pvcllcPwmInitialized == 0U)
    {
        return;
    }

    requested = pvcllcPfmRequestedHz;

    if(requested == pvcllcPfmAppliedHz)
    {
        return;
    }

    // Bounds are provisional development settings.

    if((requested < PVCLLC_PFM_TEST_MIN_HZ) ||
       (requested > PVCLLC_PFM_TEST_MAX_HZ))
    {
        pvcllcPfmRejectCount++;

        pvcllcPfmLastRejectedHz = requested;

        pvcllcPfmLastRejectReason =
            PVCLLC_PFM_REJECT_RANGE;

        pvcllcPfmRequestedHz =
            pvcllcPfmAppliedHz;

        return;
    }

    period = PVCLLC_PWM_calculatePeriod(requested);

    if(period == 0U)
    {
        pvcllcPfmRejectCount++;

        pvcllcPfmLastRejectedHz = requested;

        pvcllcPfmLastRejectReason =
            PVCLLC_PFM_REJECT_PERIOD;

        pvcllcPfmRequestedHz =
            pvcllcPfmAppliedHz;

        return;
    }

    // Prevent an obviously invalid internal
    // dead-band / time-base configuration.

    if(((uint32_t)pvcllcPwmDeadtimeCounts * 2UL) >=
       (uint32_t)period)
    {
        pvcllcPfmRejectCount++;

        pvcllcPfmLastRejectedHz = requested;

        pvcllcPfmLastRejectReason =
            PVCLLC_PFM_REJECT_DEADTIME;

        pvcllcPfmRequestedHz =
            pvcllcPfmAppliedHz;

        return;
    }

    compare = period / 2U;

    for(i = 0U; i < PVCLLC_EPWM_COUNT; i++)
    {
        base = pvcllcEPWMBase[i];

        // Retain STOP/FREEZE and forced OST.

        EPWM_setTimeBaseCounterMode(
            base,
            EPWM_COUNTER_MODE_STOP_FREEZE);

        EPWM_forceTripZoneEvent(
            base,
            EPWM_TZ_FORCE_EVENT_OST);

        EPWM_setTimeBasePeriod(base, period);

        EPWM_setCounterCompareValue(
            base,
            EPWM_COUNTER_COMPARE_A,
            compare);
    }

    pvcllcPwmPeriodCounts = period;
    pvcllcPwmCompareCounts = compare;

    pvcllcPfmAppliedHz = requested;

    pvcllcPfmActualHz =
        DEVICE_SYSCLK_FREQ /
        (2UL * (uint32_t)period);

    pvcllcPfmLastAcceptedHz = requested;

    pvcllcPfmLastRejectReason =
        PVCLLC_PFM_REJECT_NONE;

    pvcllcPfmUpdateCount++;

    pvcllcPwmOutputsEnabled = 0U;
}

// ============================================================
// 30. ADC INITIALIZATION
// ============================================================

static void PVCLLC_ADC_init(void)
{
    ADC_setPrescaler(
        ADCA_BASE, ADC_CLK_DIV_4_0);

    ADC_setPrescaler(
        ADCD_BASE, ADC_CLK_DIV_4_0);

    ADC_setMode(
        ADCA_BASE,
        ADC_RESOLUTION_12BIT,
        ADC_MODE_SINGLE_ENDED);

    ADC_setMode(
        ADCD_BASE,
        ADC_RESOLUTION_12BIT,
        ADC_MODE_SINGLE_ENDED);

    ADC_setInterruptPulseMode(
        ADCA_BASE,
        ADC_PULSE_END_OF_CONV);

    ADC_setInterruptPulseMode(
        ADCD_BASE,
        ADC_PULSE_END_OF_CONV);

    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCD_BASE);

    DEVICE_DELAY_US(1000);

    // ADCA: 4 channels.

    ADC_setupSOC(
        ADCA_BASE,
        ADC_SOC_NUMBER0,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN3,
        PVCLLC_ADC_WINDOW);

    ADC_setupSOC(
        ADCA_BASE,
        ADC_SOC_NUMBER1,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN2,
        PVCLLC_ADC_WINDOW);

    ADC_setupSOC(
        ADCA_BASE,
        ADC_SOC_NUMBER2,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN4,
        PVCLLC_ADC_WINDOW);

    ADC_setupSOC(
        ADCA_BASE,
        ADC_SOC_NUMBER3,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN5,
        PVCLLC_ADC_WINDOW);

    // ADCD: 5 channels.

    ADC_setupSOC(
        ADCD_BASE,
        ADC_SOC_NUMBER0,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN0,
        PVCLLC_ADC_WINDOW);

    ADC_setupSOC(
        ADCD_BASE,
        ADC_SOC_NUMBER1,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN1,
        PVCLLC_ADC_WINDOW);

    ADC_setupSOC(
        ADCD_BASE,
        ADC_SOC_NUMBER2,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN2,
        PVCLLC_ADC_WINDOW);

    ADC_setupSOC(
        ADCD_BASE,
        ADC_SOC_NUMBER3,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN3,
        PVCLLC_ADC_WINDOW);

    ADC_setupSOC(
        ADCD_BASE,
        ADC_SOC_NUMBER4,
        ADC_TRIGGER_SW_ONLY,
        ADC_CH_ADCIN4,
        PVCLLC_ADC_WINDOW);

    ADC_setInterruptSource(
        ADCA_BASE,
        ADC_INT_NUMBER1,
        ADC_SOC_NUMBER3);

    ADC_setInterruptSource(
        ADCD_BASE,
        ADC_INT_NUMBER1,
        ADC_SOC_NUMBER4);

    ADC_disableContinuousMode(
        ADCA_BASE,
        ADC_INT_NUMBER1);

    ADC_disableContinuousMode(
        ADCD_BASE,
        ADC_INT_NUMBER1);

    PVCLLC_ADC_clearFlags();

    ADC_enableInterrupt(
        ADCA_BASE,
        ADC_INT_NUMBER1);

    ADC_enableInterrupt(
        ADCD_BASE,
        ADC_INT_NUMBER1);
}

// ============================================================
// 31. ADC FLAG CLEAR
// ============================================================

static void PVCLLC_ADC_clearFlags(void)
{
    ADC_clearInterruptStatus(
        ADCA_BASE,
        ADC_INT_NUMBER1);

    ADC_clearInterruptStatus(
        ADCD_BASE,
        ADC_INT_NUMBER1);

    ADC_clearInterruptOverflowStatus(
        ADCA_BASE,
        ADC_INT_NUMBER1);

    ADC_clearInterruptOverflowStatus(
        ADCD_BASE,
        ADC_INT_NUMBER1);
}

// ============================================================
// 32. ADC ACQUISITION
// ============================================================

static void PVCLLC_ADC_read(void)
{
    uint32_t waitCount;

    adcDataValid = 0U;

    PVCLLC_ADC_clearFlags();

    ADC_forceMultipleSOC(
        ADCA_BASE,
        ADC_FORCE_SOC0 |
        ADC_FORCE_SOC1 |
        ADC_FORCE_SOC2 |
        ADC_FORCE_SOC3);

    ADC_forceMultipleSOC(
        ADCD_BASE,
        ADC_FORCE_SOC0 |
        ADC_FORCE_SOC1 |
        ADC_FORCE_SOC2 |
        ADC_FORCE_SOC3 |
        ADC_FORCE_SOC4);

    // Wait for ADCA completion.

    waitCount = 0UL;

    while(!ADC_getInterruptStatus(
        ADCA_BASE, ADC_INT_NUMBER1))
    {
        DEVICE_DELAY_US(1);

        if(++waitCount >= PVCLLC_ADC_TIMEOUT)
        {
            adcTimeoutCount++;

            PVCLLC_ADC_clearFlags();

            return;
        }
    }

    // Wait for ADCD completion.

    waitCount = 0UL;

    while(!ADC_getInterruptStatus(
        ADCD_BASE, ADC_INT_NUMBER1))
    {
        DEVICE_DELAY_US(1);

        if(++waitCount >= PVCLLC_ADC_TIMEOUT)
        {
            adcTimeoutCount++;

            PVCLLC_ADC_clearFlags();

            return;
        }
    }

    // Check overflow flags.

    if(ADC_getInterruptOverflowStatus(
        ADCA_BASE, ADC_INT_NUMBER1) ||
       ADC_getInterruptOverflowStatus(
        ADCD_BASE, ADC_INT_NUMBER1))
    {
        adcOverflowCount++;

        PVCLLC_ADC_clearFlags();

        return;
    }

    // ADCA results.

    adcVpvRaw = ADC_readResult(
        ADCARESULT_BASE, ADC_SOC_NUMBER0);

    adcIpvRaw = ADC_readResult(
        ADCARESULT_BASE, ADC_SOC_NUMBER1);

    adcM1Raw = ADC_readResult(
        ADCARESULT_BASE, ADC_SOC_NUMBER2);

    adcM2Raw = ADC_readResult(
        ADCARESULT_BASE, ADC_SOC_NUMBER3);

    // ADCD results.

    adcIdcRaw = ADC_readResult(
        ADCDRESULT_BASE, ADC_SOC_NUMBER0);

    adcVdcRaw = ADC_readResult(
        ADCDRESULT_BASE, ADC_SOC_NUMBER1);

    adcTR1Raw = ADC_readResult(
        ADCDRESULT_BASE, ADC_SOC_NUMBER2);

    adcTR2Raw = ADC_readResult(
        ADCDRESULT_BASE, ADC_SOC_NUMBER3);

    adcTR3Raw = ADC_readResult(
        ADCDRESULT_BASE, ADC_SOC_NUMBER4);

    PVCLLC_ADC_process();

    PVCLLC_ADC_clearFlags();

    adcSampleCount++;
    adcDataValid = 1U;
}

// ============================================================
// 33. ADC RAIL DIAGNOSTICS
// ============================================================

static void PVCLLC_ADC_checkRails(void)
{
    uint16_t values[9];
    uint16_t i;
    uint16_t bit;

    uint16_t highMask = 0U;
    uint16_t persistentMask = 0U;

    values[0] = adcVpvRaw;
    values[1] = adcIpvRaw;
    values[2] = adcM1Raw;
    values[3] = adcM2Raw;

    values[4] = adcIdcRaw;
    values[5] = adcVdcRaw;

    values[6] = adcTR1Raw;
    values[7] = adcTR2Raw;
    values[8] = adcTR3Raw;

    for(i = 0U; i < PVCLLC_ADC_CHANNELS; i++)
    {
        bit = (uint16_t)(1U << i);

        if(values[i] >= PVCLLC_ADC_RAIL_THRESHOLD)
        {
            highMask |= bit;

            if(adcRailHighStreak[i] <
               PVCLLC_ADC_RAIL_LIMIT)
            {
                adcRailHighStreak[i]++;
            }

            if(adcRailHighStreak[i] >=
               PVCLLC_ADC_RAIL_LIMIT)
            {
                persistentMask |= bit;

                if((adcRailPersistentMask & bit) == 0U)
                {
                    adcRailEventCount++;
                }
            }
        }
        else
        {
            adcRailHighStreak[i] = 0UL;
        }
    }

    adcRailHighMask = highMask;
    adcRailPersistentMask = persistentMask;

    adcRailHistoryMask |= persistentMask;
}

// ============================================================
// 34. EMA FILTER
// ============================================================

static float PVCLLC_lowPass(
    float oldValue,
    float newValue,
    float alpha)
{
    return oldValue +
           alpha * (newValue - oldValue);
}

// ============================================================
// 35. INTERNAL TELEMETRY SNAPSHOT
// ============================================================

static void PVCLLC_publishTelemetry(void)
{
    telemetryValid = 0U;

    telemetryVpvAdcV = Vpv_ADC_Filtered_V;
    telemetryIpvAdcV = Ipv_ADC_Filtered_V;

    telemetryVdcAdcV = Vdc_ADC_Filtered_V;
    telemetryIdcAdcV = Idc_ADC_Filtered_V;

    telemetryM1AdcV = M1_ADC_Filtered_V;
    telemetryM2AdcV = M2_ADC_Filtered_V;

    telemetryTR1AdcV = TR1_ADC_Filtered_V;
    telemetryTR2AdcV = TR2_ADC_Filtered_V;
    telemetryTR3AdcV = TR3_ADC_Filtered_V;

    telemetrySystemState = pvcllcSystemState;
    telemetryAdcRailMask = adcRailPersistentMask;

    telemetryPfmAppliedHz = pvcllcPfmAppliedHz;

    telemetryISRMaxTicks =
        pvcllcControlISRMaxTicks;

    telemetryPwmTripMissingMask =
        pvcllcPwmTripMissingMask;

    telemetrySourceSampleCount = adcSampleCount;

    telemetrySequence++;
    telemetryValid = 1U;
}

// ============================================================
// 36. ADC PROCESSING AND FILTERING
// ============================================================

static void PVCLLC_ADC_process(void)
{
    Vpv_ADC_V =
        (float)adcVpvRaw * PVCLLC_ADC_SCALE;

    Ipv_ADC_V =
        (float)adcIpvRaw * PVCLLC_ADC_SCALE;

    Vdc_ADC_V =
        (float)adcVdcRaw * PVCLLC_ADC_SCALE;

    Idc_ADC_V =
        (float)adcIdcRaw * PVCLLC_ADC_SCALE;

    M1_ADC_V =
        (float)adcM1Raw * PVCLLC_ADC_SCALE;

    M2_ADC_V =
        (float)adcM2Raw * PVCLLC_ADC_SCALE;

    TR1_ADC_V =
        (float)adcTR1Raw * PVCLLC_ADC_SCALE;

    TR2_ADC_V =
        (float)adcTR2Raw * PVCLLC_ADC_SCALE;

    TR3_ADC_V =
        (float)adcTR3Raw * PVCLLC_ADC_SCALE;

    if(adcFilterReady == 0U)
    {
        Vpv_ADC_Filtered_V = Vpv_ADC_V;
        Ipv_ADC_Filtered_V = Ipv_ADC_V;

        Vdc_ADC_Filtered_V = Vdc_ADC_V;
        Idc_ADC_Filtered_V = Idc_ADC_V;

        M1_ADC_Filtered_V = M1_ADC_V;
        M2_ADC_Filtered_V = M2_ADC_V;

        TR1_ADC_Filtered_V = TR1_ADC_V;
        TR2_ADC_Filtered_V = TR2_ADC_V;
        TR3_ADC_Filtered_V = TR3_ADC_V;

        adcFilterReady = 1U;
    }
    else
    {
        Vpv_ADC_Filtered_V = PVCLLC_lowPass(
            Vpv_ADC_Filtered_V,
            Vpv_ADC_V,
            PVCLLC_VI_ALPHA);

        Ipv_ADC_Filtered_V = PVCLLC_lowPass(
            Ipv_ADC_Filtered_V,
            Ipv_ADC_V,
            PVCLLC_VI_ALPHA);

        Vdc_ADC_Filtered_V = PVCLLC_lowPass(
            Vdc_ADC_Filtered_V,
            Vdc_ADC_V,
            PVCLLC_VI_ALPHA);

        Idc_ADC_Filtered_V = PVCLLC_lowPass(
            Idc_ADC_Filtered_V,
            Idc_ADC_V,
            PVCLLC_VI_ALPHA);

        M1_ADC_Filtered_V = PVCLLC_lowPass(
            M1_ADC_Filtered_V,
            M1_ADC_V,
            PVCLLC_TEMP_ALPHA);

        M2_ADC_Filtered_V = PVCLLC_lowPass(
            M2_ADC_Filtered_V,
            M2_ADC_V,
            PVCLLC_TEMP_ALPHA);

        TR1_ADC_Filtered_V = PVCLLC_lowPass(
            TR1_ADC_Filtered_V,
            TR1_ADC_V,
            PVCLLC_TEMP_ALPHA);

        TR2_ADC_Filtered_V = PVCLLC_lowPass(
            TR2_ADC_Filtered_V,
            TR2_ADC_V,
            PVCLLC_TEMP_ALPHA);

        TR3_ADC_Filtered_V = PVCLLC_lowPass(
            TR3_ADC_Filtered_V,
            TR3_ADC_V,
            PVCLLC_TEMP_ALPHA);
    }

    // Provisional physical voltage conversion.

    Vpv_V = Vpv_ADC_V / PVCLLC_VPV_GAIN;
    Vdc_V = Vdc_ADC_V / PVCLLC_VDC_GAIN;

    Vpv_Filtered_V =
        Vpv_ADC_Filtered_V / PVCLLC_VPV_GAIN;

    Vdc_Filtered_V =
        Vdc_ADC_Filtered_V / PVCLLC_VDC_GAIN;
}
