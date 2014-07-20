/*
 * File: revolve_pid_data.c
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

/* Block parameters (auto storage) */
Parameters_revolve_pid revolve_pid_P = {
  0.5F,                                /* Computed Parameter: wFront_Gain_Gain
                                        * Referenced by: '<S2>/wFront_Gain'
                                        */
  1.5F,                                /* Computed Parameter: wRear_Gain_Gain
                                        * Referenced by: '<S2>/wRear_Gain'
                                        */
  10.0F                                /* Computed Parameter: pid_Kp_Gain
                                        * Referenced by: '<S1>/pid_Kp'
                                        */
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
