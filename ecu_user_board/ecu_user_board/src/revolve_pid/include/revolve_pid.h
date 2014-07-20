/*
 * File: revolve_pid.h
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

#ifndef RTW_HEADER_revolve_pid_h_
#define RTW_HEADER_revolve_pid_h_
#ifndef revolve_pid_COMMON_INCLUDES_
# define revolve_pid_COMMON_INCLUDES_
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#endif                                 /* revolve_pid_COMMON_INCLUDES_ */

#include "revolve_pid_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T pid_es;                     /* '<Root>/pid_es' */
  real32_T wFrontLeft;                 /* '<Root>/wFrontLeft' */
  real32_T wFrontRight;                /* '<Root>/wFrontRight' */
  real32_T wRearLeft;                  /* '<Root>/wRearLeft' */
  real32_T wRearRight;                 /* '<Root>/wRearRight' */
  real32_T slip_target;                /* '<Root>/slip_target' */
} ExternalInputs_revolve_pid;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  real32_T pid_out;                    /* '<Root>/pid_out' */
} ExternalOutputs_revolve_pid;

/* Parameters (auto storage) */
struct Parameters_revolve_pid_ {
  real32_T wFront_Gain_Gain;           /* Computed Parameter: wFront_Gain_Gain
                                        * Referenced by: '<S2>/wFront_Gain'
                                        */
  real32_T wRear_Gain_Gain;            /* Computed Parameter: wRear_Gain_Gain
                                        * Referenced by: '<S2>/wRear_Gain'
                                        */
  real32_T pid_Kp_Gain;                /* Computed Parameter: pid_Kp_Gain
                                        * Referenced by: '<S1>/pid_Kp'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_revolve_pid {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_revolve_pid revolve_pid_P;

/* External inputs (root inport signals with auto storage) */
extern ExternalInputs_revolve_pid revolve_pid_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExternalOutputs_revolve_pid revolve_pid_Y;

/* Model entry point functions */
extern void revolve_pid_initialize(void);
extern void revolve_pid_step(void);
extern void revolve_pid_terminate(void);

/* Real-time Model object */
extern RT_MODEL_revolve_pid *const revolve_pid_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'revolve_pid'
 * '<S1>'   : 'revolve_pid/PID controller'
 * '<S2>'   : 'revolve_pid/Slip calculation'
 */
#endif                                 /* RTW_HEADER_revolve_pid_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
