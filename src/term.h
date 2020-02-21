/*

Custom terminal types

*/
#pragma once

#include <stdint.h>
#include <stddef.h>

#define TYPE_ANALOG  1
#define TYPE_DIGITAL 2

#define COE_RESPONSE_BAD 1
#define COE_RESPONSE_OK 0

/* Registers in the modbus memory map */
extern uint16_t* reg_bus_coupler_id;
extern uint16_t* reg_pdo_size_ao;
extern uint16_t* reg_pdo_size_ai;
extern uint16_t* reg_pdo_size_do;
extern uint16_t* reg_pdo_size_di;
extern uint16_t* reg_wdt_curr_time;
extern uint16_t* reg_num_fallbacks_triggered;
extern uint16_t* reg_connections;
extern uint16_t* reg_hardware_ver;
extern uint16_t* reg_soft_ver_main;
extern uint16_t* reg_soft_ver_submain;
extern uint16_t* reg_soft_ver_beta;
extern uint16_t* reg_serial_num;
extern uint16_t* reg_mfg_date_day;
extern uint16_t* reg_mfg_date_mon;
extern uint16_t* reg_mfg_date_year;
extern uint16_t* reg_ebus_status;
extern uint16_t* reg_wdt_reset;
extern uint16_t* reg_wdt_time;
extern uint16_t* reg_wdt_type;
extern uint16_t* reg_fallback_mode;
extern uint16_t* reg_writelock;
extern uint16_t* reg_ebus_ctrl;
extern uint16_t* reg_0x1400;
extern uint16_t* reg_0x1401;
extern uint16_t* reg_0x1402;
extern uint16_t* reg_0x1403;
extern uint16_t* reg_0x1404;
extern uint16_t* reg_0x1405;
extern uint16_t* reg_data_start;
extern uint16_t* reg_term_ids;

#pragma pack(1)
typedef struct
{
	uint16_t tid;
	uint16_t protoid;
	uint16_t length;
	uint8_t unitid;
	uint8_t function;
	uint16_t start_addr;
	uint16_t addr_cnt;
} modbus_header_t;
#pragma pack(0)

typedef enum
{
	COE_TYPE_INT16,
	COE_TYPE_BOOL,
	COE_TYPE_INT8,
	COE_TYPE_FLOAT32,
	COE_TYPE_STRING,
} coe_type_t;

typedef struct
{
	coe_type_t type;
	unsigned index;
	unsigned subindex;
	unsigned length;
	uint16_t* payload;
} coe_payload_t, coe_write_req_t;

typedef struct
{
	coe_type_t type;
	unsigned index;
	unsigned subindex;
} coe_read_req_t;

typedef struct term_context_s
{
	void* pdo_ai_buf;
	void* pdo_ao_buf;
	void* pdo_do_buf;
	void* pdo_di_buf;
	void* pvt;
} term_context_t;

typedef struct term_s 
{
	const char* name;
	int type;
	unsigned id;
	/* Pdo sizes */
	int pdo_size_ai;
	int pdo_size_ao;
	int pdo_size_di;
	int pdo_size_do;
	/* Init(int* pdo_size_ai, int* pdo_size_ao, int* pdo_size_di, int* pdo_size_do) */
	int(*pfnInit)(term_context_t*, void*,void*,void*,void*);
	/* Just signals that there's a pdo update */
	void(*pfnPDOUpdate)(term_context_t*);
	int(*pfnSendCoEData)(term_context_t*,coe_payload_t);
	int(*pfnReadCoEData)(term_context_t*,uint16_t*,coe_read_req_t);
	void(*pfnDestroy)(term_context_t*);
} term_t;

extern term_t el3064_table;
extern term_t el3174_table;
extern term_t el2008_table;

extern term_t* g_terms[];
