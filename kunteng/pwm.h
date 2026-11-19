/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PWM_H
#define _PWM_H

void pwm_init_bipolar_4q (void);
void pwm_phase_a_disable (void);
void pwm_phase_a_enable_pwm (void);
void pwm_phase_a_enable_low (void);
void pwm_phase_b_disable (void);
void pwm_phase_b_enable_pwm (void);
void pwm_phase_b_enable_low (void);
void pwm_phase_c_disable (void);
void pwm_phase_c_enable_pwm (void);
void pwm_phase_c_enable_low (void);

#endif /* _PWM_H_ */
