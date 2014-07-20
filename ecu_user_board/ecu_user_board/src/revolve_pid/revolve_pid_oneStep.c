#include "revolve_pid.h"               /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */

/*
 * Associating rt_OneStep with a real-time clock or interrupt service routine
 * is what makes the generated code "real-time".  The function rt_OneStep is
 * always associated with the base rate of the model.  Subrates are managed
 * by the base rate from inside the generated code.  Enabling/disabling
 * interrupts and floating point context switches are target specific.  This
 * example code indicates where these should take place relative to executing
 * the generated code step function.  Overrun behavior should be tailored to
 * your application needs.  This example simply sets an error status in the
 * real-time model and returns from rt_OneStep.
 */
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = 0;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    rtmSetErrorStatus(revolve_pid_M, "Overrun");
    return;
  }

  OverrunFlag = TRUE;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  revolve_pid_step();

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = FALSE;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

/*
 * The example "main" function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific.  This example
 * illustates how you do this relative to initializing the model.
 */
//int_T main(int_T argc, const char_T *argv[]);
//int_T main(int_T argc, const char_T *argv[]) {
//   /* Initialize model */
//   revolve_pid_initialize();
// 
//   /* Attach rt_OneStep to a timer or interrupt service routine with
//    * period 0.2 seconds (the model's base sample time) here.  The
//    * call syntax for rt_OneStep is
//    *
//    *  rt_OneStep();
//    */
//   while (rtmGetErrorStatus(revolve_pid_M) == (NULL)) {
//     /*  Perform other application tasks here */
//   }
// 
//   /* Disable rt_OneStep() here */
// 
//   /* Terminate model */
//   revolve_pid_terminate();
//   return 0;
//}