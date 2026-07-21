#include <Measurements.h>
#include <MpptController.h>
#include <PowerStage.h>
#include <main.h>

#define TEST_CHECK(condition) do { if (!(condition)) { return __LINE__; } } while (0)

static Measurements_t test_measurements = {0};
static uint32_t test_tick_ms = 0U;
static float buck_commands[16] = {0};
static float boost_commands[16] = {0};
static uint32_t command_count = 0U;
static Fault_t entered_fault = FAULT_NONE;

static float Test_Absolute(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static bool Test_Near(float first, float second)
{
  return Test_Absolute(first - second) < 0.0001f;
}

static void Test_Feed(uint32_t tick_ms, float power_w)
{
  test_tick_ms = tick_ms;
  test_measurements.v_in_v = 1.0f;
  test_measurements.i_in_a = power_w;
  test_measurements.valid = true;
  test_measurements.updated_ms = tick_ms;
  MpptController_Update();
}

static void Test_FeedWindow(uint32_t first_tick_ms, const float *powers_w)
{
  for (uint32_t index = 0U; index < 8U; index++)
  {
    Test_Feed(first_tick_ms + index, powers_w[index]);
  }
}

uint32_t HAL_GetTick(void)
{
  return test_tick_ms;
}

void enterFault(Fault_t fault)
{
  entered_fault = fault;
}

const Measurements_t *Measurements_GetLatest(void)
{
  return &test_measurements;
}

bool PowerStage_Enable(void)
{
  return true;
}

void PowerStage_SetDuty(float buck_duty, float boost_duty)
{
  if (command_count < 16U)
  {
    buck_commands[command_count] = buck_duty;
    boost_commands[command_count] = boost_duty;
  }
  command_count++;
}

int main(void)
{
  static const float baseline_w[8] =
      {100.0f, 100.1f, 99.9f, 100.2f, 99.8f, 100.0f, 80.0f, 120.0f};
  static const float improvement_w[8] =
      {101.0f, 101.1f, 100.9f, 101.2f, 100.8f, 101.0f, 70.0f, 130.0f};
  static const float improvement_two_w[8] =
      {102.0f, 102.1f, 101.9f, 102.2f, 101.8f, 102.0f, 70.0f, 130.0f};
  static const float improvement_three_w[8] =
      {103.0f, 103.1f, 102.9f, 103.2f, 102.8f, 103.0f, 70.0f, 130.0f};
  static const float loss_w[8] =
      {99.0f, 99.1f, 98.9f, 99.2f, 98.8f, 99.0f, 60.0f, 140.0f};
  static const float reference_w[8] =
      {103.0f, 103.1f, 102.9f, 103.2f, 102.8f, 103.0f, 75.0f, 125.0f};
  static const float deadband_w[8] =
      {102.8f, 102.9f, 102.7f, 103.0f, 102.6f, 102.8f, 70.0f, 130.0f};
  const MpptControllerStatus_t *status;

  MpptController_Startup();
  TEST_CHECK(entered_fault == FAULT_NONE);
  TEST_CHECK(command_count == 1U);
  TEST_CHECK(Test_Near(buck_commands[0], 0.1000f));
  TEST_CHECK(Test_Near(boost_commands[0], 0.0f));

  for (uint32_t tick_ms = 1U; tick_ms < 20U; tick_ms++)
  {
    Test_Feed(tick_ms, 100.0f);
  }
  TEST_CHECK(command_count == 1U);

  Test_FeedWindow(20U, baseline_w);
  TEST_CHECK(command_count == 2U);
  TEST_CHECK(Test_Near(buck_commands[1], 0.1100f));
  status = MpptController_GetStatus();
  TEST_CHECK(status->phase == MPPT_CONTROLLER_PHASE_SETTLING);
  TEST_CHECK(Test_Near(status->reference_power_w, 100.0f));
  TEST_CHECK(Test_Near(status->step, 0.0100f));

  for (uint32_t tick_ms = 28U; tick_ms < 47U; tick_ms++)
  {
    Test_Feed(tick_ms, 101.0f);
  }
  TEST_CHECK(command_count == 2U);

  Test_FeedWindow(47U, improvement_w);
  TEST_CHECK(command_count == 3U);
  TEST_CHECK(Test_Near(buck_commands[2], 0.1250f));
  status = MpptController_GetStatus();
  TEST_CHECK(Test_Near(status->reference_power_w, 101.0f));
  TEST_CHECK(Test_Near(status->step, 0.0150f));

  Test_FeedWindow(74U, improvement_two_w);
  TEST_CHECK(command_count == 4U);
  TEST_CHECK(Test_Near(buck_commands[3], 0.1450f));
  status = MpptController_GetStatus();
  TEST_CHECK(Test_Near(status->reference_power_w, 102.0f));
  TEST_CHECK(Test_Near(status->step, 0.0200f));

  Test_FeedWindow(101U, improvement_three_w);
  TEST_CHECK(command_count == 5U);
  TEST_CHECK(Test_Near(buck_commands[4], 0.1650f));
  status = MpptController_GetStatus();
  TEST_CHECK(Test_Near(status->reference_power_w, 103.0f));
  TEST_CHECK(Test_Near(status->step, 0.0200f));

  Test_FeedWindow(128U, loss_w);
  TEST_CHECK(command_count == 6U);
  TEST_CHECK(Test_Near(buck_commands[5], 0.1450f));
  status = MpptController_GetStatus();
  TEST_CHECK(status->phase == MPPT_CONTROLLER_PHASE_SETTLING);
  TEST_CHECK(Test_Near(status->step, -0.0025f));

  Test_FeedWindow(155U, reference_w);
  TEST_CHECK(command_count == 7U);
  TEST_CHECK(Test_Near(buck_commands[6], 0.1425f));

  Test_FeedWindow(182U, deadband_w);
  TEST_CHECK(command_count == 8U);
  TEST_CHECK(Test_Near(buck_commands[7], 0.1450f));
  status = MpptController_GetStatus();
  TEST_CHECK(Test_Near(status->step, 0.0025f));

  for (uint32_t index = 1U; index < command_count; index++)
  {
    TEST_CHECK(Test_Absolute(buck_commands[index] - buck_commands[index - 1U]) <= 0.0201f);
    TEST_CHECK(Test_Near(boost_commands[index], 0.0f));
  }

  return 0;
}
