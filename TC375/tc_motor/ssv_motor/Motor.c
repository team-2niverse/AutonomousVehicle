#include "Motor.h"
#include "Gtm_Atom_Pwm.h"
#include "IfxCpu.h"

void Motor_init(void) {
    MODULE_P10.IOCR0.B.PC1 = 0x10;  // chA: initialize direction
    MODULE_P02.IOCR4.B.PC7 = 0x10;  // chA: initialize brake

    MODULE_P10.IOCR0.B.PC2 = 0x10;  // chB: initialize direction
    MODULE_P02.IOCR4.B.PC6 = 0x10;  // chB: initialize brake

    MODULE_P02.OUT.B.P7 = 1;        // chA: set brake
    MODULE_P02.OUT.B.P6 = 1;        // chB: set brake

    MODULE_P10.OUT.B.P1 = 1;        // chA: set direction
    MODULE_P10.OUT.B.P2 = 1;        // chB: set direction

    GtmAtomPwm_init();

    GtmAtomPwm_setDutyCycleA(0);
    GtmAtomPwm_setDutyCycleB(0);
}

void Motor_stopLeft(void) {
    MODULE_P02.OUT.B.P7 = 1;
    GtmAtomPwm_setDutyCycleA(0);
}

void Motor_stopRight(void) {
    MODULE_P02.OUT.B.P6 = 1;
    GtmAtomPwm_setDutyCycleB(0);
}

void Motor_driveLeft(uint8 dir, uint8 duty) {
    if (MODULE_P02.OUT.B.P7 == 0) {
        if (MODULE_P10.OUT.B.P1 != dir) {
            Motor_stopLeft();
            MODULE_P10.OUT.B.P1 = dir;
            MODULE_P02.OUT.B.P7 = 0;
        }
    } else {
        MODULE_P10.OUT.B.P1 = dir;
        MODULE_P02.OUT.B.P7 = 0;
    }
    GtmAtomPwm_setDutyCycleA((uint32)(duty*3.92));
}

void Motor_driveRight(uint8 dir, uint8 duty) {
    if (MODULE_P02.OUT.B.P6 == 0) {
        if (MODULE_P10.OUT.B.P2 != dir) {
            Motor_stopRight();
            MODULE_P10.OUT.B.P2 = dir;
            MODULE_P02.OUT.B.P6 = 0;
        }
    } else {
        MODULE_P10.OUT.B.P2 = dir;
        MODULE_P02.OUT.B.P6 = 0;
    }
    GtmAtomPwm_setDutyCycleB((uint32)(duty*3.92));
}
