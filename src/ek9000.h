/**
 * 
 * ek9000.h - Defines the basic ek9k device
 * 
 */ 
#pragma once

#include <stdint.h>

typedef struct
{
	/* Registers in the modbus memory map */
	uint16_t* reg_bus_coupler_id;
	uint16_t* reg_pdo_size_ao;
	uint16_t* reg_pdo_size_ai;
	uint16_t* reg_pdo_size_do;
	uint16_t* reg_pdo_size_di;
	uint16_t* reg_wdt_curr_time;
	uint16_t* reg_num_fallbacks_triggered;
	uint16_t* reg_connections;
	uint16_t* reg_hardware_ver;
	uint16_t* reg_soft_ver_main;
	uint16_t* reg_soft_ver_submain;
	uint16_t* reg_soft_ver_beta;
	uint16_t* reg_serial_num;
	uint16_t* reg_mfg_date_day;
	uint16_t* reg_mfg_date_mon;
	uint16_t* reg_mfg_date_year;
	uint16_t* reg_ebus_status;
	uint16_t* reg_wdt_reset;
	uint16_t* reg_wdt_time;
	uint16_t* reg_wdt_type;
	uint16_t* reg_fallback_mode;
	uint16_t* reg_writelock;
	uint16_t* reg_ebus_ctrl;
	uint16_t* reg_0x1400;
	uint16_t* reg_0x1401;
	uint16_t* reg_0x1402;
	uint16_t* reg_0x1403;
	uint16_t* reg_0x1404;
	uint16_t* reg_0x1405;
	uint16_t* reg_data_start;
	uint16_t* reg_term_ids;
} ek9000_t;
