/*
 * MpptController.c
 *
 * Converter state machine and MPPT control implementation.
 */

#include "MpptController.h"

#include "Measurements.h"
#include "PowerStage.h"

#include <stdint.h>

/*
 * Duty-cycle command for the two half bridges of the four-switch buck-boost.
 * Values are normalised fractions, not timer counts: 0.25 means 25% duty.
 */
typedef struct MpptDutyCommand
{
  float buck_duty;
  float boost_duty;
} MpptDutyCommand_t;

typedef struct PnOParameters
{
  float previous_power_w;
  float reference_gain;
  float gain;
  float step;
  float power_sum_w;
  uint8_t power_sample_count;
  uint8_t negative_delta_count;
  bool previous_power_valid;
} PnoParameters_t;

/* Conservative startup command. */
#define STARTUP_BUCK_DUTY          (0.1f)
#define STARTUP_BOOST_DUTY         (0.0f)

#define PnOMode                        (1)

/* The four-switch conversion ratio is M = D_buck / (1 - D_boost).
 * Derive the maximum gain from the real power-stage duty limit so the MPPT
 * never walks through commands that PowerStage_SetDuty() would clip. */
#define PNO_INITIAL_GAIN               (0.10f)
#define PNO_GAIN_MIN                   (0.0f)
#define PNO_GAIN_MAX                   (POWER_STAGE_DUTY_MAX / \
                                        (1.0f - POWER_STAGE_DUTY_MAX))

/* Average several complete, settled operating-point readings before making a
 * decision. A possible power decrease must also be observed twice; the command
 * is held during confirmation rather than perturbing farther downhill. */
#define PNO_POWER_AVERAGE_SAMPLES      (2U)
#define PNO_NEGATIVE_CONFIRMATIONS     (2U)

/* Ignore changes smaller than the larger of the absolute and relative noise
 * bands. These are initial bench-tuning values for approximately 100 W tests. */
#define PNO_POWER_DEADBAND_MIN_W       (0.50f)
#define PNO_POWER_DEADBAND_RELATIVE    (0.005f)

/* Variable perturbation: large relative power changes move quickly up the
 * hill; small changes near the MPP reduce the oscillation amplitude. */
#define PNO_STEP_INITIAL               (0.010f)
#define PNO_STEP_MIN                   (0.0025f)
#define PNO_STEP_MAX                   (0.040f)
#define PNO_STEP_PER_RELATIVE_POWER    (0.50f)
#define PNO_POWER_NORMALISATION_MIN_W  (1.0f)

static MpptDutyCommand_t MpptController_CalculateDuty(float panel_voltage_v, float panel_current_a);
static MpptDutyCommand_t PnO_Init(void);
static MpptDutyCommand_t PnO_Run(float panel_voltage_v, float panel_current_a);
static MpptDutyCommand_t getDutiesFromGain(float gain);
static float PnO_ClampGain(float gain);
static float PnO_Absolute(float value);
static float PnO_ClampStepMagnitude(float step_magnitude);
static float PnO_CalculateStepMagnitude(float delta_power_w,
                                        float reference_power_w);
static float PnO_CalculatePowerDeadband(float reference_power_w);
static void PnO_AdvanceGain(void);

volatile bool mppt_start_requested = false;
volatile bool stop_requested = false;
volatile bool fault_reset_requested= false;

MpptDutyCommand_t mppt_command = {0};
PnoParameters_t pno_par = {0};
uint32_t mppt_state_entered_ms = 0U;
uint32_t fast_task_last_ms = 0U;
uint32_t mppt_task_last_ms = 0U;
uint32_t telemetry_task_last_ms = 0U;

/**
  * @brief  Start the MPPT algorithm */
void MpptController_Startup(void)
{
  #if (PnOMode)
    mppt_command = PnO_Init();
  #else
    /* Start with a small fixed command. Students should verify this on an
    * oscilloscope before using real PV panels or a battery/load. */
    mppt_command.buck_duty = STARTUP_BUCK_DUTY;
    mppt_command.boost_duty = STARTUP_BOOST_DUTY;
  #endif
  
  
  PowerStage_SetDuty(mppt_command.buck_duty, mppt_command.boost_duty);
  if (!PowerStage_Enable())
  {
    enterFault(FAULT_PWM_START);
  }
  
  
}

/**
  * @brief  Student MPPT algorithm hook.
  * @param  panel_voltage_v Input/PV voltage in volts.
  * @param  panel_current_a Input/PV current in amps.
  * @retval Buck and boost duty-cycle command requested by the MPPT algorithm.
  */
static MpptDutyCommand_t MpptController_CalculateDuty(float panel_voltage_v, float panel_current_a)
{
  /* Start from the previous command. If students do not change anything below,
   * the converter will hold its last duty cycle instead of jumping abruptly. */
  MpptDutyCommand_t next_command = mppt_command;

  /* MPPT is based on input-side PV power, not output power. The guide defines
   * this as P_PV = V_in * I_in. */
  const float panel_power_w = panel_voltage_v * panel_current_a;
  (void)panel_power_w;

  /*
   * STUDENT MPPT CODE GOES HERE.
   *
   * Use panel_voltage_v, panel_current_a, and panel_power_w to decide how the
   * converter should move. Typical algorithms compare the present power with a
   * previous power value, then perturb the duty command in the direction that
   * increased PV power.
   *
   * Set one or both command fields:
   *
   *   next_command.buck_duty
   *   next_command.boost_duty
   *
   * The values are normalised duties from 0.0 to 1.0. PowerStage_SetDuty() will
   * clamp them to the configured power-stage duty range before writing the timer
   * compare registers.
   *
   * Keep the MPPT step size small. A large duty step can collapse the PV voltage
   * or cause a current transient before the ADC readings settle.
   */

  #if (PnOMode)
    next_command = PnO_Run(panel_voltage_v, panel_current_a);
  #endif
  
  

  return next_command;
}

/**
  * @brief  Runs the medium-rate MPPT update and applies the returned duty command.
  */
void MpptController_Update(void)
{
  const Measurements_t *measurements = Measurements_GetLatest();

  /* The MPPT algorithm receives input-side PV voltage and current. */
  MpptDutyCommand_t next_command =
      MpptController_CalculateDuty(measurements->v_in_v, measurements->i_in_a);

  /* The power-stage layer clamps and applies the returned command. */
  mppt_command = next_command;
  PowerStage_SetDuty(mppt_command.buck_duty, mppt_command.boost_duty);
}

MpptDutyCommand_t PnO_Init(void)
{
  pno_par.previous_power_w = 0.0f;
  pno_par.gain = PnO_ClampGain(PNO_INITIAL_GAIN);
  pno_par.reference_gain = pno_par.gain;
  pno_par.step = PNO_STEP_INITIAL;
  pno_par.power_sum_w = 0.0f;
  pno_par.power_sample_count = 0U;
  pno_par.negative_delta_count = 0U;
  pno_par.previous_power_valid = false;

  return getDutiesFromGain(pno_par.gain);
}


MpptDutyCommand_t PnO_Run(float panel_voltage_v, float panel_current_a)
{
  const float panel_power_w = panel_voltage_v * panel_current_a;
  float average_power_w;
  float delta_power_w;
  float power_deadband_w;
  float step_magnitude;
  const float step_direction = (pno_par.step >= 0.0f) ? 1.0f : -1.0f;

  pno_par.power_sum_w += panel_power_w;
  pno_par.power_sample_count++;

  /* Hold the present command until a complete operating-point average exists. */
  if (pno_par.power_sample_count < PNO_POWER_AVERAGE_SAMPLES)
  {
    return getDutiesFromGain(pno_par.gain);
  }

  average_power_w = pno_par.power_sum_w / (float)PNO_POWER_AVERAGE_SAMPLES;
  pno_par.power_sum_w = 0.0f;
  pno_par.power_sample_count = 0U;

  /* The first complete average establishes the reference operating point. */
  if (!pno_par.previous_power_valid)
  {
    pno_par.previous_power_w = average_power_w;
    pno_par.reference_gain = pno_par.gain;
    pno_par.previous_power_valid = true;
    PnO_AdvanceGain();
    return getDutiesFromGain(pno_par.gain);
  }

  delta_power_w = average_power_w - pno_par.previous_power_w;
  power_deadband_w = PnO_CalculatePowerDeadband(pno_par.previous_power_w);

  if (delta_power_w < -power_deadband_w)
  {
    pno_par.negative_delta_count++;

    if (pno_par.negative_delta_count < PNO_NEGATIVE_CONFIRMATIONS)
    {
      /* Re-measure this same gain. Do not move farther in a possibly wrong
       * direction, and retain the previous operating point as the reference. */
      return getDutiesFromGain(pno_par.gain);
    }

    /* A confirmed power loss rejects the probe. Return immediately to the last
     * known-good gain and reverse the next probe direction. */
    step_magnitude =
        PnO_CalculateStepMagnitude(delta_power_w, pno_par.previous_power_w);
    pno_par.step = -step_direction * step_magnitude;
    pno_par.negative_delta_count = 0U;
    pno_par.gain = pno_par.reference_gain;
    pno_par.previous_power_valid = false;
    return getDutiesFromGain(pno_par.gain);
  }

  pno_par.negative_delta_count = 0U;

  if (delta_power_w > power_deadband_w)
  {
    /* Accept a clear improvement as the new reference, retain direction, and
     * adapt the next probe to the relative power change. */
    step_magnitude =
        PnO_CalculateStepMagnitude(delta_power_w, pno_par.previous_power_w);
    pno_par.step = step_direction * step_magnitude;
    pno_par.previous_power_w = average_power_w;
    pno_par.reference_gain = pno_par.gain;
    PnO_AdvanceGain();
    return getDutiesFromGain(pno_par.gain);
  }

  /* An inconclusive probe is not allowed to walk across the whole power curve.
   * Reject it, return to the reference, and test the opposite side next time
   * using the minimum step. This bounds steady-state MPP oscillation. */
  pno_par.step = -step_direction * PNO_STEP_MIN;
  pno_par.gain = pno_par.reference_gain;
  pno_par.previous_power_valid = false;
  return getDutiesFromGain(pno_par.gain);
}

MpptDutyCommand_t getDutiesFromGain(float gain)
{
  MpptDutyCommand_t duties;
  gain = PnO_ClampGain(gain);

  if (gain <= POWER_STAGE_DUTY_MAX)
  {
    duties.buck_duty = gain;
    duties.boost_duty = 0.0f;
  }
  else
  {
    /* Start boost action as soon as buck reaches its real 95% limit. This keeps
     * M = D_buck / (1 - D_boost) continuous and removes the old 0.95-to-1.0
     * region in which every requested buck duty was clipped to the same value. */
    duties.buck_duty = POWER_STAGE_DUTY_MAX;
    duties.boost_duty = 1.0f - (POWER_STAGE_DUTY_MAX / gain);
  }

  return duties;
}

static float PnO_ClampGain(float gain)
{
  if (gain < PNO_GAIN_MIN)
  {
    return PNO_GAIN_MIN;
  }

  if (gain > PNO_GAIN_MAX)
  {
    return PNO_GAIN_MAX;
  }

  return gain;
}

static float PnO_Absolute(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float PnO_ClampStepMagnitude(float step_magnitude)
{
  if (step_magnitude < PNO_STEP_MIN)
  {
    return PNO_STEP_MIN;
  }

  if (step_magnitude > PNO_STEP_MAX)
  {
    return PNO_STEP_MAX;
  }

  return step_magnitude;
}

static float PnO_CalculateStepMagnitude(float delta_power_w,
                                        float reference_power_w)
{
  float normalisation_power_w = PnO_Absolute(reference_power_w);

  if (normalisation_power_w < PNO_POWER_NORMALISATION_MIN_W)
  {
    normalisation_power_w = PNO_POWER_NORMALISATION_MIN_W;
  }

  return PnO_ClampStepMagnitude(PNO_STEP_PER_RELATIVE_POWER *
                                PnO_Absolute(delta_power_w) /
                                normalisation_power_w);
}

static float PnO_CalculatePowerDeadband(float reference_power_w)
{
  const float relative_deadband_w =
      PNO_POWER_DEADBAND_RELATIVE * PnO_Absolute(reference_power_w);

  return (relative_deadband_w > PNO_POWER_DEADBAND_MIN_W)
             ? relative_deadband_w
             : PNO_POWER_DEADBAND_MIN_W;
}

static void PnO_AdvanceGain(void)
{
  /* Turn back immediately at a command boundary so the controller cannot keep
   * requesting a clipped value and making decisions from sensor noise alone. */
  if (((pno_par.step > 0.0f) && (pno_par.gain >= PNO_GAIN_MAX)) ||
      ((pno_par.step < 0.0f) && (pno_par.gain <= PNO_GAIN_MIN)))
  {
    pno_par.step = -pno_par.step;
  }

  pno_par.gain = PnO_ClampGain(pno_par.gain + pno_par.step);
}
