/*
 * File: revolve_pid.c
 *
 * Code generated for Simulink model 'revolve_pid'.
 *
 * Model version                  : 1.33
 * Simulink Coder version         : 8.3 (R2012b) 20-Jul-2012
 * TLC version                    : 8.3 (Jul 21 2012)
 * C/C++ source code generated on : Fri Feb 28 19:37:53 2014
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Atmel->AVR
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. MISRA-C:2004 guidelines
 *    3. Safety precaution
 * Validation result: Passed (12), Warnings (4), Error (0)
 */

#include "revolve_pid.h"
#include "revolve_pid_private.h"

/* External inputs (root inport signals with auto storage) */
ExternalInputs_revolve_pid revolve_pid_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_revolve_pid revolve_pid_Y;

/* Real-time model */
RT_MODEL_revolve_pid revolve_pid_M_;
RT_MODEL_revolve_pid *const revolve_pid_M = &revolve_pid_M_;

/* Model step function */
void revolve_pid_step(void)
{
  /* Outport: '<Root>/pid_out' incorporates:
   *  Gain: '<S1>/pid_Kp'
   *  Gain: '<S2>/wFront_Gain'
   *  Gain: '<S2>/wRear_Gain'
   *  Inport: '<Root>/slip_target'
   *  Inport: '<Root>/wFrontLeft'
   *  Inport: '<Root>/wFrontRight'
   *  Inport: '<Root>/wRearLeft'
   *  Inport: '<Root>/wRearRight'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S2>/Sum'
   *  Sum: '<S2>/Sum1'
   *  Sum: '<S2>/Sum2'
   */
  revolve_pid_Y.pid_out = (revolve_pid_U.slip_target - ((revolve_pid_U.wRearLeft
    + revolve_pid_U.wRearRight) * revolve_pid_P.wRear_Gain_Gain -
    (revolve_pid_U.wFrontLeft + revolve_pid_U.wFrontRight) *
    revolve_pid_P.wFront_Gain_Gain)) * revolve_pid_P.pid_Kp_Gain;
}

/* Model initialize function */
void revolve_pid_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(revolve_pid_M, (NULL));

  /* external inputs */
  (void) memset((void *)&revolve_pid_U, 0,
                sizeof(ExternalInputs_revolve_pid));

  /* external outputs */
  revolve_pid_Y.pid_out = 0.0F;
}

/* Model terminate function */
void revolve_pid_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
