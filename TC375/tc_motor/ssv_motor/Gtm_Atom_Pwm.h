#ifndef GTM_ATOM_PWM_H_
#define GTM_ATOM_PWM_H_

#include "Ifx_Types.h"

#define ISR_PRIORITY_ATOM   20                                  /* Interrupt priority number                        */
#define LED                 IfxGtm_ATOM0_4_TOUT14_P00_5_OUT     /* LED which will be driven by the PWM              */
#define PWM_A               IfxGtm_ATOM0_1_TOUT1_P02_1_OUT
#define PWM_B               IfxGtm_ATOM1_3_TOUT105_P10_3_OUT

#define CLK_FREQ            1000000.0f                          /* CMU clock frequency, in Hertz                    */
#define PWM_PERIOD          1000                                 /* PWM period for the ATOM, in ticks                */

// Function Prototypes
void GtmAtomPwm_init(void);
void GtmAtomPwm_setDutyCycleA(uint32 dutyCycle);
void GtmAtomPwm_setDutyCycleB(uint32 dutyCycle);

#endif /* GTM_ATOM_PWM_H_ */
