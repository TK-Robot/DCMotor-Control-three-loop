/**
 * @file Sds1601Calibration.h
 * @brief SDS1601 8.4 V output-shaft equivalent calibration.
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_SDS1601CALIBRATION_H
#define TRIPLE_CASCADECONTROLDCMOTOR_SDS1601CALIBRATION_H

/* Datasheet anchors: 7.5 kg.cm stall torque and 0.07 s/60 deg at 8.4 V.
 * R comes from the existing electrical identification. Friction-current
 * ratios come from the measured I ~= 40.2 mA + 0.97 mA/rpm relationship. */
#define SDS1601_RATED_MAX_VOLTAGE_MV                 8400U
#define SDS1601_TORQUE_CONSTANT_UNM_PER_A          235000U
#define SDS1601_BACK_EMF_UV_PER_RPM                 55500U
#define SDS1601_TERMINAL_RESISTANCE_MOHM             2650U
#define SDS1601_COULOMB_FRICTION_UNM                  9450U
#define SDS1601_VISCOUS_FRICTION_NNM_PER_RPM        228000U
#define SDS1601_FRICTION_DEADBAND_CPS                  300U

/* Previous output-shaft calibration, used only for exact runtime migration. */
#define SDS1601_LEGACY_TORQUE_CONSTANT_UNM_PER_A    245000U
#define SDS1601_LEGACY_BACK_EMF_UV_PER_RPM           40000U
#define SDS1601_LEGACY_COULOMB_FRICTION_UNM            9850U
#define SDS1601_LEGACY_VISCOUS_FRICTION_NNM_PER_RPM 237000U

#endif /* TRIPLE_CASCADECONTROLDCMOTOR_SDS1601CALIBRATION_H */
