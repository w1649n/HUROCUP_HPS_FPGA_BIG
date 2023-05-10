#ifndef _ALTERA_HPS_0_H_
#define _ALTERA_HPS_0_H_

/*
 * This file was automatically generated by the swinfo2header utility.
 * 
 * Created from SOPC Builder system 'soc_system' in
 * file './soc_system.sopcinfo'.
 */

/*
 * This file contains macros for module 'hps' and devices
 * connected to the following masters:
 *   h2f_axi_master
 *   h2f_lw_axi_master
 * 
 * Do not include this header file and another header file created for a
 * different module or master group at the same time.
 * Doing so may result in duplicate macro names.
 * Instead, use the system header file which has macros with unique names.
 */

/*
 * Macros for device 'read_motor_data_rf_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_MOTOR_DATA_RF_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_MOTOR_DATA_RF_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_MOTOR_DATA_RF_PULSE_COMPONENT_NAME read_motor_data_rf_pulse
#define READ_MOTOR_DATA_RF_PULSE_BASE 0x0
#define READ_MOTOR_DATA_RF_PULSE_SPAN 16
#define READ_MOTOR_DATA_RF_PULSE_END 0xf
#define READ_MOTOR_DATA_RF_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_MOTOR_DATA_RF_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_MOTOR_DATA_RF_PULSE_CAPTURE 0
#define READ_MOTOR_DATA_RF_PULSE_DATA_WIDTH 1
#define READ_MOTOR_DATA_RF_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_MOTOR_DATA_RF_PULSE_DRIVEN_SIM_VALUE 0
#define READ_MOTOR_DATA_RF_PULSE_EDGE_TYPE NONE
#define READ_MOTOR_DATA_RF_PULSE_FREQ 80000000
#define READ_MOTOR_DATA_RF_PULSE_HAS_IN 0
#define READ_MOTOR_DATA_RF_PULSE_HAS_OUT 1
#define READ_MOTOR_DATA_RF_PULSE_HAS_TRI 0
#define READ_MOTOR_DATA_RF_PULSE_IRQ_TYPE NONE
#define READ_MOTOR_DATA_RF_PULSE_RESET_VALUE 0

/*
 * Macros for device 'motor_data_rf', class 'altera_avalon_pio'
 * The macros are prefixed with 'MOTOR_DATA_RF_'.
 * The prefix is the slave descriptor.
 */
#define MOTOR_DATA_RF_COMPONENT_TYPE altera_avalon_pio
#define MOTOR_DATA_RF_COMPONENT_NAME motor_data_rf
#define MOTOR_DATA_RF_BASE 0x10
#define MOTOR_DATA_RF_SPAN 16
#define MOTOR_DATA_RF_END 0x1f
#define MOTOR_DATA_RF_BIT_CLEARING_EDGE_REGISTER 0
#define MOTOR_DATA_RF_BIT_MODIFYING_OUTPUT_REGISTER 0
#define MOTOR_DATA_RF_CAPTURE 0
#define MOTOR_DATA_RF_DATA_WIDTH 32
#define MOTOR_DATA_RF_DO_TEST_BENCH_WIRING 0
#define MOTOR_DATA_RF_DRIVEN_SIM_VALUE 0
#define MOTOR_DATA_RF_EDGE_TYPE NONE
#define MOTOR_DATA_RF_FREQ 80000000
#define MOTOR_DATA_RF_HAS_IN 1
#define MOTOR_DATA_RF_HAS_OUT 0
#define MOTOR_DATA_RF_HAS_TRI 0
#define MOTOR_DATA_RF_IRQ_TYPE NONE
#define MOTOR_DATA_RF_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_motor_data_rf', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_MOTOR_DATA_RF_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_MOTOR_DATA_RF_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_MOTOR_DATA_RF_COMPONENT_NAME set_hps_read_motor_data_rf
#define SET_HPS_READ_MOTOR_DATA_RF_BASE 0x20
#define SET_HPS_READ_MOTOR_DATA_RF_SPAN 16
#define SET_HPS_READ_MOTOR_DATA_RF_END 0x2f
#define SET_HPS_READ_MOTOR_DATA_RF_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_MOTOR_DATA_RF_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_MOTOR_DATA_RF_CAPTURE 0
#define SET_HPS_READ_MOTOR_DATA_RF_DATA_WIDTH 1
#define SET_HPS_READ_MOTOR_DATA_RF_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_MOTOR_DATA_RF_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_MOTOR_DATA_RF_EDGE_TYPE NONE
#define SET_HPS_READ_MOTOR_DATA_RF_FREQ 80000000
#define SET_HPS_READ_MOTOR_DATA_RF_HAS_IN 1
#define SET_HPS_READ_MOTOR_DATA_RF_HAS_OUT 0
#define SET_HPS_READ_MOTOR_DATA_RF_HAS_TRI 0
#define SET_HPS_READ_MOTOR_DATA_RF_IRQ_TYPE NONE
#define SET_HPS_READ_MOTOR_DATA_RF_RESET_VALUE 0

/*
 * Macros for device 'motor_data_lf', class 'altera_avalon_pio'
 * The macros are prefixed with 'MOTOR_DATA_LF_'.
 * The prefix is the slave descriptor.
 */
#define MOTOR_DATA_LF_COMPONENT_TYPE altera_avalon_pio
#define MOTOR_DATA_LF_COMPONENT_NAME motor_data_lf
#define MOTOR_DATA_LF_BASE 0x30
#define MOTOR_DATA_LF_SPAN 16
#define MOTOR_DATA_LF_END 0x3f
#define MOTOR_DATA_LF_BIT_CLEARING_EDGE_REGISTER 0
#define MOTOR_DATA_LF_BIT_MODIFYING_OUTPUT_REGISTER 0
#define MOTOR_DATA_LF_CAPTURE 0
#define MOTOR_DATA_LF_DATA_WIDTH 32
#define MOTOR_DATA_LF_DO_TEST_BENCH_WIRING 0
#define MOTOR_DATA_LF_DRIVEN_SIM_VALUE 0
#define MOTOR_DATA_LF_EDGE_TYPE NONE
#define MOTOR_DATA_LF_FREQ 80000000
#define MOTOR_DATA_LF_HAS_IN 1
#define MOTOR_DATA_LF_HAS_OUT 0
#define MOTOR_DATA_LF_HAS_TRI 0
#define MOTOR_DATA_LF_IRQ_TYPE NONE
#define MOTOR_DATA_LF_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_motor_data_lf', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_MOTOR_DATA_LF_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_MOTOR_DATA_LF_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_MOTOR_DATA_LF_COMPONENT_NAME set_hps_read_motor_data_lf
#define SET_HPS_READ_MOTOR_DATA_LF_BASE 0x40
#define SET_HPS_READ_MOTOR_DATA_LF_SPAN 16
#define SET_HPS_READ_MOTOR_DATA_LF_END 0x4f
#define SET_HPS_READ_MOTOR_DATA_LF_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_MOTOR_DATA_LF_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_MOTOR_DATA_LF_CAPTURE 0
#define SET_HPS_READ_MOTOR_DATA_LF_DATA_WIDTH 1
#define SET_HPS_READ_MOTOR_DATA_LF_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_MOTOR_DATA_LF_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_MOTOR_DATA_LF_EDGE_TYPE NONE
#define SET_HPS_READ_MOTOR_DATA_LF_FREQ 80000000
#define SET_HPS_READ_MOTOR_DATA_LF_HAS_IN 1
#define SET_HPS_READ_MOTOR_DATA_LF_HAS_OUT 0
#define SET_HPS_READ_MOTOR_DATA_LF_HAS_TRI 0
#define SET_HPS_READ_MOTOR_DATA_LF_IRQ_TYPE NONE
#define SET_HPS_READ_MOTOR_DATA_LF_RESET_VALUE 0

/*
 * Macros for device 'read_motor_data_lf_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_MOTOR_DATA_LF_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_MOTOR_DATA_LF_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_MOTOR_DATA_LF_PULSE_COMPONENT_NAME read_motor_data_lf_pulse
#define READ_MOTOR_DATA_LF_PULSE_BASE 0x50
#define READ_MOTOR_DATA_LF_PULSE_SPAN 16
#define READ_MOTOR_DATA_LF_PULSE_END 0x5f
#define READ_MOTOR_DATA_LF_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_MOTOR_DATA_LF_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_MOTOR_DATA_LF_PULSE_CAPTURE 0
#define READ_MOTOR_DATA_LF_PULSE_DATA_WIDTH 1
#define READ_MOTOR_DATA_LF_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_MOTOR_DATA_LF_PULSE_DRIVEN_SIM_VALUE 0
#define READ_MOTOR_DATA_LF_PULSE_EDGE_TYPE NONE
#define READ_MOTOR_DATA_LF_PULSE_FREQ 80000000
#define READ_MOTOR_DATA_LF_PULSE_HAS_IN 0
#define READ_MOTOR_DATA_LF_PULSE_HAS_OUT 1
#define READ_MOTOR_DATA_LF_PULSE_HAS_TRI 0
#define READ_MOTOR_DATA_LF_PULSE_IRQ_TYPE NONE
#define READ_MOTOR_DATA_LF_PULSE_RESET_VALUE 0

/*
 * Macros for device 'read_press_sensor_left_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_PRESS_SENSOR_LEFT_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_PRESS_SENSOR_LEFT_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_PRESS_SENSOR_LEFT_PULSE_COMPONENT_NAME read_press_sensor_left_pulse
#define READ_PRESS_SENSOR_LEFT_PULSE_BASE 0x60
#define READ_PRESS_SENSOR_LEFT_PULSE_SPAN 16
#define READ_PRESS_SENSOR_LEFT_PULSE_END 0x6f
#define READ_PRESS_SENSOR_LEFT_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_PRESS_SENSOR_LEFT_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_PRESS_SENSOR_LEFT_PULSE_CAPTURE 0
#define READ_PRESS_SENSOR_LEFT_PULSE_DATA_WIDTH 1
#define READ_PRESS_SENSOR_LEFT_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_PRESS_SENSOR_LEFT_PULSE_DRIVEN_SIM_VALUE 0
#define READ_PRESS_SENSOR_LEFT_PULSE_EDGE_TYPE NONE
#define READ_PRESS_SENSOR_LEFT_PULSE_FREQ 80000000
#define READ_PRESS_SENSOR_LEFT_PULSE_HAS_IN 0
#define READ_PRESS_SENSOR_LEFT_PULSE_HAS_OUT 1
#define READ_PRESS_SENSOR_LEFT_PULSE_HAS_TRI 0
#define READ_PRESS_SENSOR_LEFT_PULSE_IRQ_TYPE NONE
#define READ_PRESS_SENSOR_LEFT_PULSE_RESET_VALUE 0

/*
 * Macros for device 'read_press_sensor_right_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_PRESS_SENSOR_RIGHT_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_PRESS_SENSOR_RIGHT_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_PRESS_SENSOR_RIGHT_PULSE_COMPONENT_NAME read_press_sensor_right_pulse
#define READ_PRESS_SENSOR_RIGHT_PULSE_BASE 0x70
#define READ_PRESS_SENSOR_RIGHT_PULSE_SPAN 16
#define READ_PRESS_SENSOR_RIGHT_PULSE_END 0x7f
#define READ_PRESS_SENSOR_RIGHT_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_PRESS_SENSOR_RIGHT_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_PRESS_SENSOR_RIGHT_PULSE_CAPTURE 0
#define READ_PRESS_SENSOR_RIGHT_PULSE_DATA_WIDTH 1
#define READ_PRESS_SENSOR_RIGHT_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_PRESS_SENSOR_RIGHT_PULSE_DRIVEN_SIM_VALUE 0
#define READ_PRESS_SENSOR_RIGHT_PULSE_EDGE_TYPE NONE
#define READ_PRESS_SENSOR_RIGHT_PULSE_FREQ 80000000
#define READ_PRESS_SENSOR_RIGHT_PULSE_HAS_IN 0
#define READ_PRESS_SENSOR_RIGHT_PULSE_HAS_OUT 1
#define READ_PRESS_SENSOR_RIGHT_PULSE_HAS_TRI 0
#define READ_PRESS_SENSOR_RIGHT_PULSE_IRQ_TYPE NONE
#define READ_PRESS_SENSOR_RIGHT_PULSE_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_press_sensor_right', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_PRESS_SENSOR_RIGHT_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_COMPONENT_NAME set_hps_read_press_sensor_right
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_BASE 0x80
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_SPAN 16
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_END 0x8f
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_CAPTURE 0
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_DATA_WIDTH 1
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_EDGE_TYPE NONE
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_FREQ 80000000
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_HAS_IN 1
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_HAS_OUT 0
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_HAS_TRI 0
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_IRQ_TYPE NONE
#define SET_HPS_READ_PRESS_SENSOR_RIGHT_RESET_VALUE 0

/*
 * Macros for device 'press_sensor_right', class 'altera_avalon_pio'
 * The macros are prefixed with 'PRESS_SENSOR_RIGHT_'.
 * The prefix is the slave descriptor.
 */
#define PRESS_SENSOR_RIGHT_COMPONENT_TYPE altera_avalon_pio
#define PRESS_SENSOR_RIGHT_COMPONENT_NAME press_sensor_right
#define PRESS_SENSOR_RIGHT_BASE 0x90
#define PRESS_SENSOR_RIGHT_SPAN 16
#define PRESS_SENSOR_RIGHT_END 0x9f
#define PRESS_SENSOR_RIGHT_BIT_CLEARING_EDGE_REGISTER 0
#define PRESS_SENSOR_RIGHT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PRESS_SENSOR_RIGHT_CAPTURE 0
#define PRESS_SENSOR_RIGHT_DATA_WIDTH 32
#define PRESS_SENSOR_RIGHT_DO_TEST_BENCH_WIRING 0
#define PRESS_SENSOR_RIGHT_DRIVEN_SIM_VALUE 0
#define PRESS_SENSOR_RIGHT_EDGE_TYPE NONE
#define PRESS_SENSOR_RIGHT_FREQ 80000000
#define PRESS_SENSOR_RIGHT_HAS_IN 1
#define PRESS_SENSOR_RIGHT_HAS_OUT 0
#define PRESS_SENSOR_RIGHT_HAS_TRI 0
#define PRESS_SENSOR_RIGHT_IRQ_TYPE NONE
#define PRESS_SENSOR_RIGHT_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_press_sensor_left', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_PRESS_SENSOR_LEFT_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_PRESS_SENSOR_LEFT_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_PRESS_SENSOR_LEFT_COMPONENT_NAME set_hps_read_press_sensor_left
#define SET_HPS_READ_PRESS_SENSOR_LEFT_BASE 0xa0
#define SET_HPS_READ_PRESS_SENSOR_LEFT_SPAN 16
#define SET_HPS_READ_PRESS_SENSOR_LEFT_END 0xaf
#define SET_HPS_READ_PRESS_SENSOR_LEFT_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_PRESS_SENSOR_LEFT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_PRESS_SENSOR_LEFT_CAPTURE 0
#define SET_HPS_READ_PRESS_SENSOR_LEFT_DATA_WIDTH 1
#define SET_HPS_READ_PRESS_SENSOR_LEFT_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_PRESS_SENSOR_LEFT_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_PRESS_SENSOR_LEFT_EDGE_TYPE NONE
#define SET_HPS_READ_PRESS_SENSOR_LEFT_FREQ 80000000
#define SET_HPS_READ_PRESS_SENSOR_LEFT_HAS_IN 1
#define SET_HPS_READ_PRESS_SENSOR_LEFT_HAS_OUT 0
#define SET_HPS_READ_PRESS_SENSOR_LEFT_HAS_TRI 0
#define SET_HPS_READ_PRESS_SENSOR_LEFT_IRQ_TYPE NONE
#define SET_HPS_READ_PRESS_SENSOR_LEFT_RESET_VALUE 0

/*
 * Macros for device 'press_sensor_left', class 'altera_avalon_pio'
 * The macros are prefixed with 'PRESS_SENSOR_LEFT_'.
 * The prefix is the slave descriptor.
 */
#define PRESS_SENSOR_LEFT_COMPONENT_TYPE altera_avalon_pio
#define PRESS_SENSOR_LEFT_COMPONENT_NAME press_sensor_left
#define PRESS_SENSOR_LEFT_BASE 0xb0
#define PRESS_SENSOR_LEFT_SPAN 16
#define PRESS_SENSOR_LEFT_END 0xbf
#define PRESS_SENSOR_LEFT_BIT_CLEARING_EDGE_REGISTER 0
#define PRESS_SENSOR_LEFT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PRESS_SENSOR_LEFT_CAPTURE 0
#define PRESS_SENSOR_LEFT_DATA_WIDTH 32
#define PRESS_SENSOR_LEFT_DO_TEST_BENCH_WIRING 0
#define PRESS_SENSOR_LEFT_DRIVEN_SIM_VALUE 0
#define PRESS_SENSOR_LEFT_EDGE_TYPE NONE
#define PRESS_SENSOR_LEFT_FREQ 80000000
#define PRESS_SENSOR_LEFT_HAS_IN 1
#define PRESS_SENSOR_LEFT_HAS_OUT 0
#define PRESS_SENSOR_LEFT_HAS_TRI 0
#define PRESS_SENSOR_LEFT_IRQ_TYPE NONE
#define PRESS_SENSOR_LEFT_RESET_VALUE 0

/*
 * Macros for device 'read_imu_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_IMU_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_IMU_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_IMU_PULSE_COMPONENT_NAME read_imu_pulse
#define READ_IMU_PULSE_BASE 0xc0
#define READ_IMU_PULSE_SPAN 16
#define READ_IMU_PULSE_END 0xcf
#define READ_IMU_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_IMU_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_IMU_PULSE_CAPTURE 0
#define READ_IMU_PULSE_DATA_WIDTH 1
#define READ_IMU_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_IMU_PULSE_DRIVEN_SIM_VALUE 0
#define READ_IMU_PULSE_EDGE_TYPE NONE
#define READ_IMU_PULSE_FREQ 80000000
#define READ_IMU_PULSE_HAS_IN 0
#define READ_IMU_PULSE_HAS_OUT 1
#define READ_IMU_PULSE_HAS_TRI 0
#define READ_IMU_PULSE_IRQ_TYPE NONE
#define READ_IMU_PULSE_RESET_VALUE 0

/*
 * Macros for device 'imu', class 'altera_avalon_pio'
 * The macros are prefixed with 'IMU_'.
 * The prefix is the slave descriptor.
 */
#define IMU_COMPONENT_TYPE altera_avalon_pio
#define IMU_COMPONENT_NAME imu
#define IMU_BASE 0xd0
#define IMU_SPAN 16
#define IMU_END 0xdf
#define IMU_BIT_CLEARING_EDGE_REGISTER 0
#define IMU_BIT_MODIFYING_OUTPUT_REGISTER 0
#define IMU_CAPTURE 0
#define IMU_DATA_WIDTH 32
#define IMU_DO_TEST_BENCH_WIRING 0
#define IMU_DRIVEN_SIM_VALUE 0
#define IMU_EDGE_TYPE NONE
#define IMU_FREQ 80000000
#define IMU_HAS_IN 1
#define IMU_HAS_OUT 0
#define IMU_HAS_TRI 0
#define IMU_IRQ_TYPE NONE
#define IMU_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_imu', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_IMU_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_IMU_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_IMU_COMPONENT_NAME set_hps_read_imu
#define SET_HPS_READ_IMU_BASE 0xe0
#define SET_HPS_READ_IMU_SPAN 16
#define SET_HPS_READ_IMU_END 0xef
#define SET_HPS_READ_IMU_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_IMU_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_IMU_CAPTURE 0
#define SET_HPS_READ_IMU_DATA_WIDTH 1
#define SET_HPS_READ_IMU_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_IMU_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_IMU_EDGE_TYPE NONE
#define SET_HPS_READ_IMU_FREQ 80000000
#define SET_HPS_READ_IMU_HAS_IN 1
#define SET_HPS_READ_IMU_HAS_OUT 0
#define SET_HPS_READ_IMU_HAS_TRI 0
#define SET_HPS_READ_IMU_IRQ_TYPE NONE
#define SET_HPS_READ_IMU_RESET_VALUE 0

/*
 * Macros for device 'sensor_setting', class 'altera_avalon_pio'
 * The macros are prefixed with 'SENSOR_SETTING_'.
 * The prefix is the slave descriptor.
 */
#define SENSOR_SETTING_COMPONENT_TYPE altera_avalon_pio
#define SENSOR_SETTING_COMPONENT_NAME sensor_setting
#define SENSOR_SETTING_BASE 0xf0
#define SENSOR_SETTING_SPAN 16
#define SENSOR_SETTING_END 0xff
#define SENSOR_SETTING_BIT_CLEARING_EDGE_REGISTER 0
#define SENSOR_SETTING_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SENSOR_SETTING_CAPTURE 0
#define SENSOR_SETTING_DATA_WIDTH 32
#define SENSOR_SETTING_DO_TEST_BENCH_WIRING 0
#define SENSOR_SETTING_DRIVEN_SIM_VALUE 0
#define SENSOR_SETTING_EDGE_TYPE NONE
#define SENSOR_SETTING_FREQ 80000000
#define SENSOR_SETTING_HAS_IN 1
#define SENSOR_SETTING_HAS_OUT 0
#define SENSOR_SETTING_HAS_TRI 0
#define SENSOR_SETTING_IRQ_TYPE NONE
#define SENSOR_SETTING_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_sensor_setting', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_SENSOR_SETTING_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_SENSOR_SETTING_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_SENSOR_SETTING_COMPONENT_NAME set_hps_read_sensor_setting
#define SET_HPS_READ_SENSOR_SETTING_BASE 0x100
#define SET_HPS_READ_SENSOR_SETTING_SPAN 16
#define SET_HPS_READ_SENSOR_SETTING_END 0x10f
#define SET_HPS_READ_SENSOR_SETTING_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_SENSOR_SETTING_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_SENSOR_SETTING_CAPTURE 0
#define SET_HPS_READ_SENSOR_SETTING_DATA_WIDTH 1
#define SET_HPS_READ_SENSOR_SETTING_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_SENSOR_SETTING_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_SENSOR_SETTING_EDGE_TYPE NONE
#define SET_HPS_READ_SENSOR_SETTING_FREQ 80000000
#define SET_HPS_READ_SENSOR_SETTING_HAS_IN 1
#define SET_HPS_READ_SENSOR_SETTING_HAS_OUT 0
#define SET_HPS_READ_SENSOR_SETTING_HAS_TRI 0
#define SET_HPS_READ_SENSOR_SETTING_IRQ_TYPE NONE
#define SET_HPS_READ_SENSOR_SETTING_RESET_VALUE 0

/*
 * Macros for device 'read_sensor_setting_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_SENSOR_SETTING_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_SENSOR_SETTING_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_SENSOR_SETTING_PULSE_COMPONENT_NAME read_sensor_setting_pulse
#define READ_SENSOR_SETTING_PULSE_BASE 0x110
#define READ_SENSOR_SETTING_PULSE_SPAN 16
#define READ_SENSOR_SETTING_PULSE_END 0x11f
#define READ_SENSOR_SETTING_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_SENSOR_SETTING_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_SENSOR_SETTING_PULSE_CAPTURE 0
#define READ_SENSOR_SETTING_PULSE_DATA_WIDTH 1
#define READ_SENSOR_SETTING_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_SENSOR_SETTING_PULSE_DRIVEN_SIM_VALUE 0
#define READ_SENSOR_SETTING_PULSE_EDGE_TYPE NONE
#define READ_SENSOR_SETTING_PULSE_FREQ 80000000
#define READ_SENSOR_SETTING_PULSE_HAS_IN 0
#define READ_SENSOR_SETTING_PULSE_HAS_OUT 1
#define READ_SENSOR_SETTING_PULSE_HAS_TRI 0
#define READ_SENSOR_SETTING_PULSE_IRQ_TYPE NONE
#define READ_SENSOR_SETTING_PULSE_RESET_VALUE 0

/*
 * Macros for device 'pc_command', class 'altera_avalon_pio'
 * The macros are prefixed with 'PC_COMMAND_'.
 * The prefix is the slave descriptor.
 */
#define PC_COMMAND_COMPONENT_TYPE altera_avalon_pio
#define PC_COMMAND_COMPONENT_NAME pc_command
#define PC_COMMAND_BASE 0x120
#define PC_COMMAND_SPAN 16
#define PC_COMMAND_END 0x12f
#define PC_COMMAND_BIT_CLEARING_EDGE_REGISTER 0
#define PC_COMMAND_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PC_COMMAND_CAPTURE 0
#define PC_COMMAND_DATA_WIDTH 8
#define PC_COMMAND_DO_TEST_BENCH_WIRING 0
#define PC_COMMAND_DRIVEN_SIM_VALUE 0
#define PC_COMMAND_EDGE_TYPE NONE
#define PC_COMMAND_FREQ 80000000
#define PC_COMMAND_HAS_IN 1
#define PC_COMMAND_HAS_OUT 0
#define PC_COMMAND_HAS_TRI 0
#define PC_COMMAND_IRQ_TYPE NONE
#define PC_COMMAND_RESET_VALUE 0

/*
 * Macros for device 'read_database_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_DATABASE_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_DATABASE_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_DATABASE_PULSE_COMPONENT_NAME read_database_pulse
#define READ_DATABASE_PULSE_BASE 0x130
#define READ_DATABASE_PULSE_SPAN 16
#define READ_DATABASE_PULSE_END 0x13f
#define READ_DATABASE_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_DATABASE_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_DATABASE_PULSE_CAPTURE 0
#define READ_DATABASE_PULSE_DATA_WIDTH 1
#define READ_DATABASE_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_DATABASE_PULSE_DRIVEN_SIM_VALUE 0
#define READ_DATABASE_PULSE_EDGE_TYPE NONE
#define READ_DATABASE_PULSE_FREQ 80000000
#define READ_DATABASE_PULSE_HAS_IN 0
#define READ_DATABASE_PULSE_HAS_OUT 1
#define READ_DATABASE_PULSE_HAS_TRI 0
#define READ_DATABASE_PULSE_IRQ_TYPE NONE
#define READ_DATABASE_PULSE_RESET_VALUE 0

/*
 * Macros for device 'database', class 'altera_avalon_pio'
 * The macros are prefixed with 'DATABASE_'.
 * The prefix is the slave descriptor.
 */
#define DATABASE_COMPONENT_TYPE altera_avalon_pio
#define DATABASE_COMPONENT_NAME database
#define DATABASE_BASE 0x140
#define DATABASE_SPAN 16
#define DATABASE_END 0x14f
#define DATABASE_BIT_CLEARING_EDGE_REGISTER 0
#define DATABASE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define DATABASE_CAPTURE 0
#define DATABASE_DATA_WIDTH 32
#define DATABASE_DO_TEST_BENCH_WIRING 0
#define DATABASE_DRIVEN_SIM_VALUE 0
#define DATABASE_EDGE_TYPE NONE
#define DATABASE_FREQ 80000000
#define DATABASE_HAS_IN 1
#define DATABASE_HAS_OUT 0
#define DATABASE_HAS_TRI 0
#define DATABASE_IRQ_TYPE NONE
#define DATABASE_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_database', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_DATABASE_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_DATABASE_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_DATABASE_COMPONENT_NAME set_hps_read_database
#define SET_HPS_READ_DATABASE_BASE 0x150
#define SET_HPS_READ_DATABASE_SPAN 16
#define SET_HPS_READ_DATABASE_END 0x15f
#define SET_HPS_READ_DATABASE_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_DATABASE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_DATABASE_CAPTURE 0
#define SET_HPS_READ_DATABASE_DATA_WIDTH 1
#define SET_HPS_READ_DATABASE_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_DATABASE_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_DATABASE_EDGE_TYPE NONE
#define SET_HPS_READ_DATABASE_FREQ 80000000
#define SET_HPS_READ_DATABASE_HAS_IN 1
#define SET_HPS_READ_DATABASE_HAS_OUT 0
#define SET_HPS_READ_DATABASE_HAS_TRI 0
#define SET_HPS_READ_DATABASE_IRQ_TYPE NONE
#define SET_HPS_READ_DATABASE_RESET_VALUE 0

/*
 * Macros for device 'walkdata', class 'altera_avalon_pio'
 * The macros are prefixed with 'WALKDATA_'.
 * The prefix is the slave descriptor.
 */
#define WALKDATA_COMPONENT_TYPE altera_avalon_pio
#define WALKDATA_COMPONENT_NAME walkdata
#define WALKDATA_BASE 0x160
#define WALKDATA_SPAN 16
#define WALKDATA_END 0x16f
#define WALKDATA_BIT_CLEARING_EDGE_REGISTER 0
#define WALKDATA_BIT_MODIFYING_OUTPUT_REGISTER 0
#define WALKDATA_CAPTURE 0
#define WALKDATA_DATA_WIDTH 32
#define WALKDATA_DO_TEST_BENCH_WIRING 0
#define WALKDATA_DRIVEN_SIM_VALUE 0
#define WALKDATA_EDGE_TYPE NONE
#define WALKDATA_FREQ 80000000
#define WALKDATA_HAS_IN 1
#define WALKDATA_HAS_OUT 0
#define WALKDATA_HAS_TRI 0
#define WALKDATA_IRQ_TYPE NONE
#define WALKDATA_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_walkdata', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_WALKDATA_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_WALKDATA_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_WALKDATA_COMPONENT_NAME set_hps_read_walkdata
#define SET_HPS_READ_WALKDATA_BASE 0x170
#define SET_HPS_READ_WALKDATA_SPAN 16
#define SET_HPS_READ_WALKDATA_END 0x17f
#define SET_HPS_READ_WALKDATA_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_WALKDATA_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_WALKDATA_CAPTURE 0
#define SET_HPS_READ_WALKDATA_DATA_WIDTH 1
#define SET_HPS_READ_WALKDATA_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_WALKDATA_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_WALKDATA_EDGE_TYPE NONE
#define SET_HPS_READ_WALKDATA_FREQ 80000000
#define SET_HPS_READ_WALKDATA_HAS_IN 1
#define SET_HPS_READ_WALKDATA_HAS_OUT 0
#define SET_HPS_READ_WALKDATA_HAS_TRI 0
#define SET_HPS_READ_WALKDATA_IRQ_TYPE NONE
#define SET_HPS_READ_WALKDATA_RESET_VALUE 0

/*
 * Macros for device 'read_walkdata_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_WALKDATA_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_WALKDATA_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_WALKDATA_PULSE_COMPONENT_NAME read_walkdata_pulse
#define READ_WALKDATA_PULSE_BASE 0x180
#define READ_WALKDATA_PULSE_SPAN 16
#define READ_WALKDATA_PULSE_END 0x18f
#define READ_WALKDATA_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_WALKDATA_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_WALKDATA_PULSE_CAPTURE 0
#define READ_WALKDATA_PULSE_DATA_WIDTH 1
#define READ_WALKDATA_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_WALKDATA_PULSE_DRIVEN_SIM_VALUE 0
#define READ_WALKDATA_PULSE_EDGE_TYPE NONE
#define READ_WALKDATA_PULSE_FREQ 80000000
#define READ_WALKDATA_PULSE_HAS_IN 0
#define READ_WALKDATA_PULSE_HAS_OUT 1
#define READ_WALKDATA_PULSE_HAS_TRI 0
#define READ_WALKDATA_PULSE_IRQ_TYPE NONE
#define READ_WALKDATA_PULSE_RESET_VALUE 0

/*
 * Macros for device 'led_debug', class 'altera_avalon_pio'
 * The macros are prefixed with 'LED_DEBUG_'.
 * The prefix is the slave descriptor.
 */
#define LED_DEBUG_COMPONENT_TYPE altera_avalon_pio
#define LED_DEBUG_COMPONENT_NAME led_debug
#define LED_DEBUG_BASE 0x190
#define LED_DEBUG_SPAN 16
#define LED_DEBUG_END 0x19f
#define LED_DEBUG_BIT_CLEARING_EDGE_REGISTER 0
#define LED_DEBUG_BIT_MODIFYING_OUTPUT_REGISTER 0
#define LED_DEBUG_CAPTURE 0
#define LED_DEBUG_DATA_WIDTH 4
#define LED_DEBUG_DO_TEST_BENCH_WIRING 0
#define LED_DEBUG_DRIVEN_SIM_VALUE 0
#define LED_DEBUG_EDGE_TYPE NONE
#define LED_DEBUG_FREQ 80000000
#define LED_DEBUG_HAS_IN 0
#define LED_DEBUG_HAS_OUT 1
#define LED_DEBUG_HAS_TRI 0
#define LED_DEBUG_IRQ_TYPE NONE
#define LED_DEBUG_RESET_VALUE 0

/*
 * Macros for device 'locus_timer', class 'altera_avalon_pio'
 * The macros are prefixed with 'LOCUS_TIMER_'.
 * The prefix is the slave descriptor.
 */
#define LOCUS_TIMER_COMPONENT_TYPE altera_avalon_pio
#define LOCUS_TIMER_COMPONENT_NAME locus_timer
#define LOCUS_TIMER_BASE 0x1a0
#define LOCUS_TIMER_SPAN 16
#define LOCUS_TIMER_END 0x1af
#define LOCUS_TIMER_BIT_CLEARING_EDGE_REGISTER 0
#define LOCUS_TIMER_BIT_MODIFYING_OUTPUT_REGISTER 0
#define LOCUS_TIMER_CAPTURE 0
#define LOCUS_TIMER_DATA_WIDTH 8
#define LOCUS_TIMER_DO_TEST_BENCH_WIRING 0
#define LOCUS_TIMER_DRIVEN_SIM_VALUE 0
#define LOCUS_TIMER_EDGE_TYPE NONE
#define LOCUS_TIMER_FREQ 80000000
#define LOCUS_TIMER_HAS_IN 0
#define LOCUS_TIMER_HAS_OUT 1
#define LOCUS_TIMER_HAS_TRI 0
#define LOCUS_TIMER_IRQ_TYPE NONE
#define LOCUS_TIMER_RESET_VALUE 0

/*
 * Macros for device 'locus_timer_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'LOCUS_TIMER_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define LOCUS_TIMER_PULSE_COMPONENT_TYPE altera_avalon_pio
#define LOCUS_TIMER_PULSE_COMPONENT_NAME locus_timer_pulse
#define LOCUS_TIMER_PULSE_BASE 0x1b0
#define LOCUS_TIMER_PULSE_SPAN 16
#define LOCUS_TIMER_PULSE_END 0x1bf
#define LOCUS_TIMER_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define LOCUS_TIMER_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define LOCUS_TIMER_PULSE_CAPTURE 0
#define LOCUS_TIMER_PULSE_DATA_WIDTH 1
#define LOCUS_TIMER_PULSE_DO_TEST_BENCH_WIRING 0
#define LOCUS_TIMER_PULSE_DRIVEN_SIM_VALUE 0
#define LOCUS_TIMER_PULSE_EDGE_TYPE NONE
#define LOCUS_TIMER_PULSE_FREQ 80000000
#define LOCUS_TIMER_PULSE_HAS_IN 0
#define LOCUS_TIMER_PULSE_HAS_OUT 1
#define LOCUS_TIMER_PULSE_HAS_TRI 0
#define LOCUS_TIMER_PULSE_IRQ_TYPE NONE
#define LOCUS_TIMER_PULSE_RESET_VALUE 0

/*
 * Macros for device 'locus_idle', class 'altera_avalon_pio'
 * The macros are prefixed with 'LOCUS_IDLE_'.
 * The prefix is the slave descriptor.
 */
#define LOCUS_IDLE_COMPONENT_TYPE altera_avalon_pio
#define LOCUS_IDLE_COMPONENT_NAME locus_idle
#define LOCUS_IDLE_BASE 0x1c0
#define LOCUS_IDLE_SPAN 16
#define LOCUS_IDLE_END 0x1cf
#define LOCUS_IDLE_BIT_CLEARING_EDGE_REGISTER 0
#define LOCUS_IDLE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define LOCUS_IDLE_CAPTURE 0
#define LOCUS_IDLE_DATA_WIDTH 1
#define LOCUS_IDLE_DO_TEST_BENCH_WIRING 0
#define LOCUS_IDLE_DRIVEN_SIM_VALUE 0
#define LOCUS_IDLE_EDGE_TYPE NONE
#define LOCUS_IDLE_FREQ 80000000
#define LOCUS_IDLE_HAS_IN 1
#define LOCUS_IDLE_HAS_OUT 0
#define LOCUS_IDLE_HAS_TRI 0
#define LOCUS_IDLE_IRQ_TYPE NONE
#define LOCUS_IDLE_RESET_VALUE 0

/*
 * Macros for device 'command', class 'altera_avalon_pio'
 * The macros are prefixed with 'COMMAND_'.
 * The prefix is the slave descriptor.
 */
#define COMMAND_COMPONENT_TYPE altera_avalon_pio
#define COMMAND_COMPONENT_NAME command
#define COMMAND_BASE 0x1d0
#define COMMAND_SPAN 16
#define COMMAND_END 0x1df
#define COMMAND_BIT_CLEARING_EDGE_REGISTER 0
#define COMMAND_BIT_MODIFYING_OUTPUT_REGISTER 0
#define COMMAND_CAPTURE 0
#define COMMAND_DATA_WIDTH 3
#define COMMAND_DO_TEST_BENCH_WIRING 0
#define COMMAND_DRIVEN_SIM_VALUE 0
#define COMMAND_EDGE_TYPE NONE
#define COMMAND_FREQ 80000000
#define COMMAND_HAS_IN 1
#define COMMAND_HAS_OUT 0
#define COMMAND_HAS_TRI 0
#define COMMAND_IRQ_TYPE NONE
#define COMMAND_RESET_VALUE 0

/*
 * Macros for device 'parameter_for_walkinggait', class 'altera_avalon_pio'
 * The macros are prefixed with 'PARAMETER_FOR_WALKINGGAIT_'.
 * The prefix is the slave descriptor.
 */
#define PARAMETER_FOR_WALKINGGAIT_COMPONENT_TYPE altera_avalon_pio
#define PARAMETER_FOR_WALKINGGAIT_COMPONENT_NAME parameter_for_walkinggait
#define PARAMETER_FOR_WALKINGGAIT_BASE 0x1e0
#define PARAMETER_FOR_WALKINGGAIT_SPAN 16
#define PARAMETER_FOR_WALKINGGAIT_END 0x1ef
#define PARAMETER_FOR_WALKINGGAIT_BIT_CLEARING_EDGE_REGISTER 0
#define PARAMETER_FOR_WALKINGGAIT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PARAMETER_FOR_WALKINGGAIT_CAPTURE 0
#define PARAMETER_FOR_WALKINGGAIT_DATA_WIDTH 32
#define PARAMETER_FOR_WALKINGGAIT_DO_TEST_BENCH_WIRING 0
#define PARAMETER_FOR_WALKINGGAIT_DRIVEN_SIM_VALUE 0
#define PARAMETER_FOR_WALKINGGAIT_EDGE_TYPE NONE
#define PARAMETER_FOR_WALKINGGAIT_FREQ 80000000
#define PARAMETER_FOR_WALKINGGAIT_HAS_IN 1
#define PARAMETER_FOR_WALKINGGAIT_HAS_OUT 0
#define PARAMETER_FOR_WALKINGGAIT_HAS_TRI 0
#define PARAMETER_FOR_WALKINGGAIT_IRQ_TYPE NONE
#define PARAMETER_FOR_WALKINGGAIT_RESET_VALUE 0

/*
 * Macros for device 'set_hps_read_parameter', class 'altera_avalon_pio'
 * The macros are prefixed with 'SET_HPS_READ_PARAMETER_'.
 * The prefix is the slave descriptor.
 */
#define SET_HPS_READ_PARAMETER_COMPONENT_TYPE altera_avalon_pio
#define SET_HPS_READ_PARAMETER_COMPONENT_NAME set_hps_read_parameter
#define SET_HPS_READ_PARAMETER_BASE 0x1f0
#define SET_HPS_READ_PARAMETER_SPAN 16
#define SET_HPS_READ_PARAMETER_END 0x1ff
#define SET_HPS_READ_PARAMETER_BIT_CLEARING_EDGE_REGISTER 0
#define SET_HPS_READ_PARAMETER_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SET_HPS_READ_PARAMETER_CAPTURE 0
#define SET_HPS_READ_PARAMETER_DATA_WIDTH 1
#define SET_HPS_READ_PARAMETER_DO_TEST_BENCH_WIRING 0
#define SET_HPS_READ_PARAMETER_DRIVEN_SIM_VALUE 0
#define SET_HPS_READ_PARAMETER_EDGE_TYPE NONE
#define SET_HPS_READ_PARAMETER_FREQ 80000000
#define SET_HPS_READ_PARAMETER_HAS_IN 1
#define SET_HPS_READ_PARAMETER_HAS_OUT 0
#define SET_HPS_READ_PARAMETER_HAS_TRI 0
#define SET_HPS_READ_PARAMETER_IRQ_TYPE NONE
#define SET_HPS_READ_PARAMETER_RESET_VALUE 0

/*
 * Macros for device 'read_parameter_pulse', class 'altera_avalon_pio'
 * The macros are prefixed with 'READ_PARAMETER_PULSE_'.
 * The prefix is the slave descriptor.
 */
#define READ_PARAMETER_PULSE_COMPONENT_TYPE altera_avalon_pio
#define READ_PARAMETER_PULSE_COMPONENT_NAME read_parameter_pulse
#define READ_PARAMETER_PULSE_BASE 0x200
#define READ_PARAMETER_PULSE_SPAN 16
#define READ_PARAMETER_PULSE_END 0x20f
#define READ_PARAMETER_PULSE_BIT_CLEARING_EDGE_REGISTER 0
#define READ_PARAMETER_PULSE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define READ_PARAMETER_PULSE_CAPTURE 0
#define READ_PARAMETER_PULSE_DATA_WIDTH 1
#define READ_PARAMETER_PULSE_DO_TEST_BENCH_WIRING 0
#define READ_PARAMETER_PULSE_DRIVEN_SIM_VALUE 0
#define READ_PARAMETER_PULSE_EDGE_TYPE NONE
#define READ_PARAMETER_PULSE_FREQ 80000000
#define READ_PARAMETER_PULSE_HAS_IN 0
#define READ_PARAMETER_PULSE_HAS_OUT 1
#define READ_PARAMETER_PULSE_HAS_TRI 0
#define READ_PARAMETER_PULSE_IRQ_TYPE NONE
#define READ_PARAMETER_PULSE_RESET_VALUE 0

/*
 * Macros for device 'package_interrupt', class 'altera_avalon_pio'
 * The macros are prefixed with 'PACKAGE_INTERRUPT_'.
 * The prefix is the slave descriptor.
 */
#define PACKAGE_INTERRUPT_COMPONENT_TYPE altera_avalon_pio
#define PACKAGE_INTERRUPT_COMPONENT_NAME package_interrupt
#define PACKAGE_INTERRUPT_BASE 0x210
#define PACKAGE_INTERRUPT_SPAN 16
#define PACKAGE_INTERRUPT_END 0x21f
#define PACKAGE_INTERRUPT_IRQ 3
#define PACKAGE_INTERRUPT_BIT_CLEARING_EDGE_REGISTER 0
#define PACKAGE_INTERRUPT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PACKAGE_INTERRUPT_CAPTURE 1
#define PACKAGE_INTERRUPT_DATA_WIDTH 1
#define PACKAGE_INTERRUPT_DO_TEST_BENCH_WIRING 0
#define PACKAGE_INTERRUPT_DRIVEN_SIM_VALUE 0
#define PACKAGE_INTERRUPT_EDGE_TYPE RISING
#define PACKAGE_INTERRUPT_FREQ 80000000
#define PACKAGE_INTERRUPT_HAS_IN 1
#define PACKAGE_INTERRUPT_HAS_OUT 0
#define PACKAGE_INTERRUPT_HAS_TRI 0
#define PACKAGE_INTERRUPT_IRQ_TYPE EDGE
#define PACKAGE_INTERRUPT_RESET_VALUE 0

/*
 * Macros for device 'software_reset', class 'altera_avalon_pio'
 * The macros are prefixed with 'SOFTWARE_RESET_'.
 * The prefix is the slave descriptor.
 */
#define SOFTWARE_RESET_COMPONENT_TYPE altera_avalon_pio
#define SOFTWARE_RESET_COMPONENT_NAME software_reset
#define SOFTWARE_RESET_BASE 0x220
#define SOFTWARE_RESET_SPAN 16
#define SOFTWARE_RESET_END 0x22f
#define SOFTWARE_RESET_BIT_CLEARING_EDGE_REGISTER 0
#define SOFTWARE_RESET_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SOFTWARE_RESET_CAPTURE 0
#define SOFTWARE_RESET_DATA_WIDTH 1
#define SOFTWARE_RESET_DO_TEST_BENCH_WIRING 0
#define SOFTWARE_RESET_DRIVEN_SIM_VALUE 0
#define SOFTWARE_RESET_EDGE_TYPE NONE
#define SOFTWARE_RESET_FREQ 80000000
#define SOFTWARE_RESET_HAS_IN 0
#define SOFTWARE_RESET_HAS_OUT 1
#define SOFTWARE_RESET_HAS_TRI 0
#define SOFTWARE_RESET_IRQ_TYPE NONE
#define SOFTWARE_RESET_RESET_VALUE 0

/*
 * Macros for device 'sysid_qsys', class 'altera_avalon_sysid_qsys'
 * The macros are prefixed with 'SYSID_QSYS_'.
 * The prefix is the slave descriptor.
 */
#define SYSID_QSYS_COMPONENT_TYPE altera_avalon_sysid_qsys
#define SYSID_QSYS_COMPONENT_NAME sysid_qsys
#define SYSID_QSYS_BASE 0x1000
#define SYSID_QSYS_SPAN 8
#define SYSID_QSYS_END 0x1007
#define SYSID_QSYS_ID 2899645186
#define SYSID_QSYS_TIMESTAMP 1668048570

/*
 * Macros for device 'jtag_uart', class 'altera_avalon_jtag_uart'
 * The macros are prefixed with 'JTAG_UART_'.
 * The prefix is the slave descriptor.
 */
#define JTAG_UART_COMPONENT_TYPE altera_avalon_jtag_uart
#define JTAG_UART_COMPONENT_NAME jtag_uart
#define JTAG_UART_BASE 0x2000
#define JTAG_UART_SPAN 8
#define JTAG_UART_END 0x2007
#define JTAG_UART_IRQ 2
#define JTAG_UART_READ_DEPTH 64
#define JTAG_UART_READ_THRESHOLD 8
#define JTAG_UART_WRITE_DEPTH 64
#define JTAG_UART_WRITE_THRESHOLD 8

/*
 * Macros for device 'onchip_memory_data', class 'altera_avalon_onchip_memory2'
 * The macros are prefixed with 'ONCHIP_MEMORY_DATA_'.
 * The prefix is the slave descriptor.
 */
#define ONCHIP_MEMORY_DATA_COMPONENT_TYPE altera_avalon_onchip_memory2
#define ONCHIP_MEMORY_DATA_COMPONENT_NAME onchip_memory_data
#define ONCHIP_MEMORY_DATA_BASE 0x4000
#define ONCHIP_MEMORY_DATA_SPAN 16384
#define ONCHIP_MEMORY_DATA_END 0x7fff
#define ONCHIP_MEMORY_DATA_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define ONCHIP_MEMORY_DATA_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define ONCHIP_MEMORY_DATA_CONTENTS_INFO ""
#define ONCHIP_MEMORY_DATA_DUAL_PORT 0
#define ONCHIP_MEMORY_DATA_GUI_RAM_BLOCK_TYPE AUTO
#define ONCHIP_MEMORY_DATA_INIT_CONTENTS_FILE soc_system_onchip_memory_data
#define ONCHIP_MEMORY_DATA_INIT_MEM_CONTENT 1
#define ONCHIP_MEMORY_DATA_INSTANCE_ID NONE
#define ONCHIP_MEMORY_DATA_NON_DEFAULT_INIT_FILE_ENABLED 0
#define ONCHIP_MEMORY_DATA_RAM_BLOCK_TYPE AUTO
#define ONCHIP_MEMORY_DATA_READ_DURING_WRITE_MODE DONT_CARE
#define ONCHIP_MEMORY_DATA_SINGLE_CLOCK_OP 0
#define ONCHIP_MEMORY_DATA_SIZE_MULTIPLE 1
#define ONCHIP_MEMORY_DATA_SIZE_VALUE 16384
#define ONCHIP_MEMORY_DATA_WRITABLE 1
#define ONCHIP_MEMORY_DATA_MEMORY_INFO_DAT_SYM_INSTALL_DIR SIM_DIR
#define ONCHIP_MEMORY_DATA_MEMORY_INFO_GENERATE_DAT_SYM 1
#define ONCHIP_MEMORY_DATA_MEMORY_INFO_GENERATE_HEX 1
#define ONCHIP_MEMORY_DATA_MEMORY_INFO_HAS_BYTE_LANE 0
#define ONCHIP_MEMORY_DATA_MEMORY_INFO_HEX_INSTALL_DIR QPF_DIR
#define ONCHIP_MEMORY_DATA_MEMORY_INFO_MEM_INIT_DATA_WIDTH 32
#define ONCHIP_MEMORY_DATA_MEMORY_INFO_MEM_INIT_FILENAME soc_system_onchip_memory_data

/*
 * Macros for device 'ava_sensor_data', class 'Ava_Sensor_Data'
 * The macros are prefixed with 'AVA_SENSOR_DATA_'.
 * The prefix is the slave descriptor.
 */
#define AVA_SENSOR_DATA_COMPONENT_TYPE Ava_Sensor_Data
#define AVA_SENSOR_DATA_COMPONENT_NAME ava_sensor_data
#define AVA_SENSOR_DATA_BASE 0x8000
#define AVA_SENSOR_DATA_SPAN 8192
#define AVA_SENSOR_DATA_END 0x9fff

/*
 * Macros for device 'avalon_locus', class 'Avalon_Locus'
 * The macros are prefixed with 'AVALON_LOCUS_'.
 * The prefix is the slave descriptor.
 */
#define AVALON_LOCUS_COMPONENT_TYPE Avalon_Locus
#define AVALON_LOCUS_COMPONENT_NAME avalon_locus
#define AVALON_LOCUS_BASE 0xa000
#define AVALON_LOCUS_SPAN 8192
#define AVALON_LOCUS_END 0xbfff

/*
 * Macros for device 'ILC', class 'interrupt_latency_counter'
 * The macros are prefixed with 'ILC_'.
 * The prefix is the slave descriptor.
 */
#define ILC_COMPONENT_TYPE interrupt_latency_counter
#define ILC_COMPONENT_NAME ILC
#define ILC_BASE 0x30000
#define ILC_SPAN 256
#define ILC_END 0x300ff


#endif /* _ALTERA_HPS_0_H_ */
