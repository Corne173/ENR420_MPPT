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
  float power_samples_w[8];
  float reference_power_w;
  float reference_gain;
  float gain;
  float step;
  uint8_t power_sample_count;
  uint32_t command_changed_ms;
  uint32_t last_sample_ms;
  bool reference_valid;
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

/* Every changed command is allowed to settle before a short, fresh measurement
 * window is collected. Samples from the previous operating point are never
 * mixed into the new power estimate. */
#define PNO_SETTLE_TIME_MS              (20U)
#define PNO_POWER_SAMPLE_COUNT          (8U)
#define PNO_TRIMMED_SAMPLE_COUNT        (PNO_POWER_SAMPLE_COUNT - 2U)

/* Ignore changes smaller than the larger of the absolute and relative noise
 * bands. These are initial bench-tuning values for approximately 100 W tests. */
#define PNO_POWER_DEADBAND_MIN_W       (0.10f)
#define PNO_POWER_DEADBAND_RELATIVE    (0.0005f)

/* Bounded multiplicative step control moves quickly after repeated clear
 * improvements without amplifying one noisy delta-power sample. A rejected or
 * inconclusive probe always returns to the fine step. */
#define PNO_STEP_INITIAL                (0.010f)
#define PNO_STEP_MIN                    (0.0025f)
#define PNO_STEP_MAX                    (0.020f)
#define PNO_STEP_GROWTH                 (1.50f)

static MpptDutyCommand_t MpptController_CalculateDuty(float panel_voltage_v,
                                                       float panel_current_a,
                                                       uint32_t sample_ms);
static MpptDutyCommand_t PnO_Init(uint32_t now_ms);
static MpptDutyCommand_t PnO_Run(float panel_power_w, uint32_t sample_ms);
static MpptDutyCommand_t getDutiesFromGain(float gain);
static float PnO_ClampGain(float gain);
static float PnO_Absolute(float value);
static float PnO_ClampStepMagnitude(float step_magnitude);
static float PnO_CalculatePowerDeadband(float reference_power_w);
static float PnO_CalculateTrimmedMean(void);
static void PnO_GrowStep(void);
static MpptDutyCommand_t PnO_AdvanceGain(uint32_t now_ms);
static void PnO_BeginSettling(uint32_t now_ms);
static void PnO_UpdateCommandStatus(MpptDutyCommand_t command);
static bool MpptController_CommandChanged(MpptDutyCommand_t first,
                                          MpptDutyCommand_t second);

volatile bool mppt_start_requested = false;
volatile bool stop_requested = false;
volatile bool fault_reset_requested= false;

static MpptDutyCommand_t mppt_command = {0};
static PnoParameters_t pno_par = {0};
static MpptControllerStatus_t mppt_status = {
    .phase = MPPT_CONTROLLER_PHASE_SETTLING};

/**
  * @brief  Start the MPPT algorithm */
void MpptController_Startup(void)
{
  #if (PnOMode)
    mppt_command = PnO_Init(HAL_GetTick());
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
static MpptDutyCommand_t MpptController_CalculateDuty(float panel_voltage_v,
                                                       float panel_current_a,
                                                       uint32_t sample_ms)
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
    next_command = PnO_Run(panel_power_w, sample_ms);
  #endif
  
  

  return next_command;
}

/**
  * @brief  Runs the medium-rate MPPT update and applies the returned duty command.
  */
void MpptController_Update(void)
{
  const Measurements_t *measurements = Measurements_GetLatest();
  MpptDutyCommand_t next_command;

  if (!measurements->valid)
  {
    return;
  }

  /* The MPPT algorithm receives input-side PV voltage and current. */
  next_command = MpptController_CalculateDuty(measurements->v_in_v,
                                               measurements->i_in_a,
                                               measurements->updated_ms);

  /* Rewriting the same compare values every millisecond is unnecessary and
   * obscures whether a real MPPT perturbation occurred. */
  if (MpptController_CommandChanged(mppt_command, next_command))
  {
    mppt_command = next_command;
    PowerStage_SetDuty(mppt_command.buck_duty, mppt_command.boost_duty);
  }
}

MpptDutyCommand_t PnO_Init(uint32_t now_ms)
{
  MpptDutyCommand_t command;

  pno_par.reference_power_w = 0.0f;
  pno_par.gain = PnO_ClampGain(PNO_INITIAL_GAIN);
  pno_par.reference_gain = pno_par.gain;
  pno_par.step = PNO_STEP_INITIAL;
  pno_par.power_sample_count = 0U;
  pno_par.reference_valid = false;
  pno_par.last_sample_ms = UINT32_MAX;

  mppt_status.reference_power_w = 0.0f;
  mppt_status.sampled_power_w = 0.0f;
  PnO_BeginSettling(now_ms);

  command = getDutiesFromGain(pno_par.gain);
  PnO_UpdateCommandStatus(command);
  return command;
}


MpptDutyCommand_t PnO_Run(float panel_power_w, uint32_t sample_ms)
{
  MpptDutyCommand_t command = getDutiesFromGain(pno_par.gain);
  float sampled_power_w;
  float delta_power_w;
  float power_deadband_w;

  if (mppt_status.phase == MPPT_CONTROLLER_PHASE_SETTLING)
  {
    if ((sample_ms - pno_par.command_changed_ms) < PNO_SETTLE_TIME_MS)
    {
      return command;
    }

    mppt_status.phase = MPPT_CONTROLLER_PHASE_SAMPLING;
    pno_par.power_sample_count = 0U;
  }

  /* The scheduler can call more than once for one measurement timestamp. Only
   * distinct 1 ms snapshots are admitted to the operating-point window. */
  if (sample_ms == pno_par.last_sample_ms)
  {
    return command;
  }

  pno_par.last_sample_ms = sample_ms;
  pno_par.power_samples_w[pno_par.power_sample_count] = panel_power_w;
  pno_par.power_sample_count++;

  if (pno_par.power_sample_count < PNO_POWER_SAMPLE_COUNT)
  {
    return command;
  }

  sampled_power_w = PnO_CalculateTrimmedMean();
  mppt_status.sampled_power_w = sampled_power_w;

  /* The initial command and every restored best command are sampled as a fresh
   * reference before another candidate is applied. */
  if (!pno_par.reference_valid)
  {
    pno_par.reference_power_w = sampled_power_w;
    pno_par.reference_gain = pno_par.gain;
    pno_par.reference_valid = true;
    mppt_status.reference_power_w = sampled_power_w;
    return PnO_AdvanceGain(sample_ms);
  }

  delta_power_w = sampled_power_w - pno_par.reference_power_w;
  power_deadband_w = PnO_CalculatePowerDeadband(pno_par.reference_power_w);

  if (delta_power_w > power_deadband_w)
  {
    /* A clear improvement becomes the new reference. Continue in the same
     * direction without re-measuring the operating point that was just sampled. */
    pno_par.reference_power_w = sampled_power_w;
    pno_par.reference_gain = pno_par.gain;
    mppt_status.reference_power_w = sampled_power_w;
    PnO_GrowStep();
    return PnO_AdvanceGain(sample_ms);
  }

  /* A loss or an inconclusive change is rejected after this single measurement
   * window. Restore the best command immediately, reverse, and use the fine
   * perturbation so steady-state hunting is bounded. */
  pno_par.step = (pno_par.step >= 0.0f) ? -PNO_STEP_MIN : PNO_STEP_MIN;
  pno_par.gain = pno_par.reference_gain;
  pno_par.reference_valid = false;
  PnO_BeginSettling(sample_ms);
  command = getDutiesFromGain(pno_par.gain);
  PnO_UpdateCommandStatus(command);
  return command;
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

static float PnO_CalculatePowerDeadband(float reference_power_w)
{
  const float relative_deadband_w =
      PNO_POWER_DEADBAND_RELATIVE * PnO_Absolute(reference_power_w);

  return (relative_deadband_w > PNO_POWER_DEADBAND_MIN_W)
             ? relative_deadband_w
             : PNO_POWER_DEADBAND_MIN_W;
}

static float PnO_CalculateTrimmedMean(void)
{
  float sum_w = 0.0f;
  float minimum_w = pno_par.power_samples_w[0];
  float maximum_w = pno_par.power_samples_w[0];

  for (uint32_t index = 0U; index < PNO_POWER_SAMPLE_COUNT; index++)
  {
    const float sample_w = pno_par.power_samples_w[index];
    sum_w += sample_w;
    if (sample_w < minimum_w)
    {
      minimum_w = sample_w;
    }
    if (sample_w > maximum_w)
    {
      maximum_w = sample_w;
    }
  }

  return (sum_w - minimum_w - maximum_w) /
         (float)PNO_TRIMMED_SAMPLE_COUNT;
}

static void PnO_GrowStep(void)
{
  const float direction = (pno_par.step >= 0.0f) ? 1.0f : -1.0f;
  const float grown_magnitude =
      PnO_ClampStepMagnitude(PnO_Absolute(pno_par.step) * PNO_STEP_GROWTH);

  pno_par.step = direction * grown_magnitude;
}

static MpptDutyCommand_t PnO_AdvanceGain(uint32_t now_ms)
{
  MpptDutyCommand_t command;
  float candidate_gain = pno_par.reference_gain + pno_par.step;

  /* Turn around before applying a command at a gain boundary. */
  if ((candidate_gain > PNO_GAIN_MAX) || (candidate_gain < PNO_GAIN_MIN))
  {
    pno_par.step = -pno_par.step;
    candidate_gain = pno_par.reference_gain + pno_par.step;
  }

  pno_par.gain = PnO_ClampGain(candidate_gain);
  PnO_BeginSettling(now_ms);
  command = getDutiesFromGain(pno_par.gain);
  PnO_UpdateCommandStatus(command);
  return command;
}

static void PnO_BeginSettling(uint32_t now_ms)
{
  pno_par.command_changed_ms = now_ms;
  pno_par.last_sample_ms = UINT32_MAX;
  pno_par.power_sample_count = 0U;
  mppt_status.phase = MPPT_CONTROLLER_PHASE_SETTLING;
}

static void PnO_UpdateCommandStatus(MpptDutyCommand_t command)
{
  mppt_status.gain = pno_par.gain;
  mppt_status.step = pno_par.step;
  mppt_status.buck_duty = command.buck_duty;
  mppt_status.boost_duty = command.boost_duty;
}

static bool MpptController_CommandChanged(MpptDutyCommand_t first,
                                          MpptDutyCommand_t second)
{
  return (first.buck_duty != second.buck_duty) ||
         (first.boost_duty != second.boost_duty);
}

const MpptControllerStatus_t *MpptController_GetStatus(void)
{
  return &mppt_status;
}

const char *MpptController_GetPhaseName(MpptControllerPhase_t phase)
{
  switch (phase)
  {
    case MPPT_CONTROLLER_PHASE_SETTLING:
      return "SETTLE";

    case MPPT_CONTROLLER_PHASE_SAMPLING:
      return "SAMPLE";

    default:
      return "UNKNOWN";
  }
}
