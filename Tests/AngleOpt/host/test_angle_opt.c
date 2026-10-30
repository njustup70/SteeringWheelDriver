#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "angle_opt.h"

static uint32_t g_failures;

static void Check(int condition, const char *message)
{
    if (!condition)
    {
        printf("FAIL: %s\n", message);
        g_failures++;
    }
}

static void CheckResult(float current,
                        float target,
                        int32_t erpm,
                        float expected_angle,
                        int32_t expected_erpm,
                        uint8_t expected_reversed,
                        const char *message)
{
    AngleOptResult_t result;

    AngleOpt_Select(current, target, erpm, &result);

    Check(fabsf(result.target_angle_deg - expected_angle) < 0.001f,
          message);
    Check(result.target_erpm == expected_erpm, message);
    Check(result.reversed == expected_reversed, message);
    Check((result.target_angle_deg >= 0.0f) &&
          (result.target_angle_deg < 360.0f),
          "output angle normalized");
}

int main(void)
{
    CheckResult(0.0f, 0.0f, 100,
                0.0f, 100, 0U,
                "target 0 keeps direction");

    CheckResult(0.0f, 90.0f, 100,
                90.0f, 100, 0U,
                "90 degree tie keeps target");

    CheckResult(0.0f, 100.0f, 100,
                280.0f, -100, 1U,
                "100 degree target chooses reverse");

    CheckResult(359.0f, 1.0f, 100,
                1.0f, 100, 0U,
                "359 to 1 takes shortest path");

    CheckResult(1.0f, 359.0f, 100,
                359.0f, 100, 0U,
                "1 to 359 takes shortest path");

    CheckResult(0.0f, 180.0f, 100,
                0.0f, -100, 1U,
                "180 degree opposite is zero distance after reverse");

    CheckResult(0.0f, 100.0f, 0,
                100.0f, 0, 0U,
                "zero ERPM keeps target angle");

    CheckResult(-10.0f, 370.0f, 100,
                10.0f, 100, 0U,
                "negative and over-360 inputs normalize");

    CheckResult(720.0f, -360.0f, 100,
                0.0f, 100, 0U,
                "large angle inputs normalize");

    CheckResult(180.0f, 0.0f, 100,
                180.0f, -100, 1U,
                "opposite angle reverses ERPM");

    if (g_failures != 0U)
    {
        printf("AngleOpt host tests: %u failure(s)\n",
               (unsigned)g_failures);
        return 1;
    }

    printf("AngleOpt host tests: all passed\n");
    return 0;
}
