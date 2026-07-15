/*
 * MpptController.c
 *
 * Converter state machine and MPPT control implementation.
 */

#include "MpptController.h"

#include "Measurements.h"
#include "PowerStage.h"

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
  float Pprev;
  float Aprev;
  float increment;
  float step;
} PnoParameters_t;

/* Conservative startup command. */
#define STARTUP_BUCK_DUTY          (0.1f)
#define STARTUP_BOOST_DUTY         (0.0f)

#define PnOMode 1
#define PNO_GAIN_MIN (0.0f)
#define PNO_GAIN_MAX (20.0f)

static MpptDutyCommand_t MpptController_CalculateDuty(float panel_voltage_v, float panel_current_a);
static MpptDutyCommand_t PnO_Init(void);
static MpptDutyCommand_t PnO_Run(float panel_voltage_v, float panel_current_a);
static MpptDutyCommand_t getDutiesFromGain(float gain);
static float PnO_ClampGain(float gain);

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
  float Ainit = 0.1;

  MpptDutyCommand_t pno_init_command = getDutiesFromGain(Ainit);

  pno_par.increment=0.01f;
  pno_par.Pprev = 0.0f;
  pno_par.Aprev = PnO_ClampGain(Ainit);
  pno_par.step = pno_par.increment; 
  return pno_init_command;
}


MpptDutyCommand_t PnO_Run(float panel_voltage_v, float panel_current_a)
{
  float panel_power = panel_voltage_v * panel_current_a;
  float deltaP = panel_power - pno_par.Pprev;
  float Apno = pno_par.Aprev;

  MpptDutyCommand_t pno_command = {0};

  if (deltaP < 0.0f)
  {
    pno_par.step = -pno_par.step;
  }

  Apno = PnO_ClampGain(pno_par.Aprev + pno_par.step);

  pno_par.Aprev = Apno;
  pno_par.Pprev = panel_power;

  pno_command = getDutiesFromGain(Apno);
  return pno_command;
}

MpptDutyCommand_t getDutiesFromGain(float gain)
{
  MpptDutyCommand_t duties;
  gain = PnO_ClampGain(gain);

  if (gain < 1.0f) {
    duties.buck_duty = gain;
    duties.boost_duty = 0.0f;
  } else {
    duties.buck_duty = 1.0f;
    duties.boost_duty = -1.0f / gain + 1.0f;
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