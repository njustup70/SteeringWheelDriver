#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "setpoint_ramp.h"

static uint32_t g_failures;

static float Wrap180(float angle)
{
    while (angle >= 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

typedef struct
{
    float settle_time_s;
    float max_velocity;
    float max_velocity_step;
    float total_travel_deg;
} RampRun_t;

static RampRun_t RunRamp(SetpointRamp_t *ramp,
                         float desired,
                         float dt,
                         uint32_t max_steps)
{
    RampRun_t run = {0};
    float previous_velocity = ramp->velocity_deg_s;
    float previous_output = ramp->output_angle_deg;
    uint32_t step;

    SetpointRamp_SetDesired(ramp, desired);

    for (step = 0U; step < max_steps; step++)
    {
        SetpointRamp_Update(ramp, dt);

        if (fabsf(ramp->velocity_deg_s) > run.max_velocity)
        {
            run.max_velocity = fabsf(ramp->velocity_deg_s);
        }

        if (fabsf(ramp->velocity_deg_s - previous_velocity) >
            run.max_velocity_step)
        {
            run.max_velocity_step =
                fabsf(ramp->velocity_deg_s - previous_velocity);
        }

        run.total_travel_deg += Wrap180(
            ramp->output_angle_deg - previous_output);
        previous_velocity = ramp->velocity_deg_s;
        previous_output = ramp->output_angle_deg;

        if ((fabsf(Wrap180(desired - ramp->output_angle_deg)) <
             SETPOINT_RAMP_ANGLE_EPS_DEG) &&
            (fabsf(ramp->velocity_deg_s) <
             SETPOINT_RAMP_VEL_EPS_DEG_S))
        {
            run.settle_time_s = (float)(step + 1U) * dt;
            return run;
        }
    }

    run.settle_time_s = -1.0f;
    return run;
}

static void Test_InitialAndReset(void)
{
    SetpointRamp_t ramp;

    SetpointRamp_Init(&ramp,
                      350.0f,
                      SETPOINT_RAMP_DEFAULT_MAX_SPEED_DEG_S,
                      SETPOINT_RAMP_DEFAULT_MAX_ACCEL_DEG_S2);
    Check(fabsf(ramp.output_angle_deg - 350.0f) < 0.001f,
          "initial output angle");
    Check(fabsf(ramp.desired_angle_deg - 350.0f) < 0.001f,
          "initial desired angle");
    Check(fabsf(ramp.velocity_deg_s) < 0.001f,
          "initial velocity");

    SetpointRamp_Reset(&ramp, -10.0f);
    Check(fabsf(ramp.output_angle_deg - 350.0f) < 0.001f,
          "reset output normalized");
    Check(fabsf(ramp.velocity_deg_s) < 0.001f,
          "reset velocity");
}

static void Test_SmallStep(void)
{
    SetpointRamp_t ramp;
    RampRun_t run;

    SetpointRamp_Init(&ramp, 0.0f, 120.0f, 600.0f);
    run = RunRamp(&ramp, 5.0f, 0.001f, 2000U);

    Check(fabsf(ramp.output_angle_deg - 5.0f) < 0.1f,
          "5 degree final angle");
    Check(run.settle_time_s > 0.0f, "5 degree settles");
    Check(run.settle_time_s < 0.4f, "5 degree settles under 400ms");
    Check(run.max_velocity <= 120.1f, "5 degree speed limit");
    Check(run.max_velocity_step <= 0.601f,
          "5 degree acceleration limit");
}

static void Test_MediumStep(void)
{
    SetpointRamp_t ramp;
    RampRun_t run;

    SetpointRamp_Init(&ramp, 0.0f, 120.0f, 600.0f);
    run = RunRamp(&ramp, 30.0f, 0.001f, 3000U);

    Check(fabsf(ramp.output_angle_deg - 30.0f) < 0.1f,
          "30 degree final angle");
    Check(run.settle_time_s > 0.3f, "30 degree takes time");
    Check(run.settle_time_s < 0.7f, "30 degree settles under 700ms");
    Check(run.max_velocity <= 120.1f, "30 degree speed limit");
    Check(run.max_velocity_step <= 0.601f,
          "30 degree acceleration limit");
}

static void Test_LargeStep(void)
{
    SetpointRamp_t ramp;
    RampRun_t run;

    SetpointRamp_Init(&ramp, 0.0f, 120.0f, 600.0f);
    run = RunRamp(&ramp, 100.0f, 0.001f, 3000U);

    Check(fabsf(ramp.output_angle_deg - 100.0f) < 0.1f,
          "100 degree final angle");
    Check(run.settle_time_s > 0.8f, "100 degree takes time");
    Check(run.settle_time_s < 1.4f, "100 degree settles under 1.4s");
    Check(run.max_velocity <= 120.1f, "100 degree speed limit");
    Check(run.max_velocity_step <= 0.601f,
          "100 degree acceleration limit");
}

static void Test_WrapShortestPath(void)
{
    SetpointRamp_t ramp;
    RampRun_t run;

    SetpointRamp_Init(&ramp, 359.0f, 120.0f, 600.0f);
    run = RunRamp(&ramp, 1.0f, 0.001f, 1000U);

    Check(fabsf(Wrap180(ramp.output_angle_deg - 1.0f)) < 0.1f,
          "wrap final angle");
    Check(run.settle_time_s > 0.0f, "wrap settles");
    Check(run.total_travel_deg > 0.0f, "wrap takes short positive path");
    Check(fabsf(run.total_travel_deg) < 3.0f,
          "wrap does not travel long way");
}

static void Test_TargetChange(void)
{
    SetpointRamp_t ramp;
    RampRun_t run;
    uint32_t i;

    SetpointRamp_Init(&ramp, 0.0f, 120.0f, 600.0f);
    SetpointRamp_SetDesired(&ramp, 90.0f);
    for (i = 0U; i < 300U; i++)
    {
        SetpointRamp_Update(&ramp, 0.001f);
    }

    run = RunRamp(&ramp, 0.0f, 0.001f, 3000U);
    Check(fabsf(Wrap180(ramp.output_angle_deg)) < 0.1f,
          "target change returns to zero");
    Check(run.settle_time_s > 0.0f, "target change settles");
    Check(run.max_velocity_step <= 0.601f,
          "target change acceleration limit");
}

int main(void)
{
    Test_InitialAndReset();
    Test_SmallStep();
    Test_MediumStep();
    Test_LargeStep();
    Test_WrapShortestPath();
    Test_TargetChange();

    if (g_failures != 0U)
    {
        printf("SetpointRamp host tests: %u failure(s)\n",
               (unsigned)g_failures);
        return 1;
    }

    printf("SetpointRamp host tests: all passed\n");
    return 0;
}
