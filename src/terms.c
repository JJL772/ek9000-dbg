/*

Terminal implementations

*/
#include "term.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

term_t* g_terms[] =
{
	&el3064_table,
	NULL
};

//==========================================================//
//
// EL3064 ANALOG INPUT TERMINAL
//
//==========================================================//
static int el3064_init(term_context_t*,void*,void*,void*,void*);
static void el3064_pdo_update(term_context_t*);
static int el3064_send_coe_data(term_context_t*,coe_payload_t);
static int el3064_coe_read(term_context_t*,uint16_t*,coe_read_req_t);
static void el3064_destroy(term_context_t*);

#define EL3064_MAPPING_STANDARD 1
#define EL3064_MAPPING_COMPACT 2

typedef struct 
{
	unsigned mapping_type;
} el3064_data_t;

term_t el3064_table = {
	.name = "el3064",
	.type = TYPE_ANALOG,
	.id = 3064,
	.pdo_size_ai = 4,
	.pfnInit = el3064_init,
	.pfnPDOUpdate = el3064_pdo_update,
	.pfnSendCoEData = el3064_send_coe_data,
	.pfnReadCoEData = el3064_coe_read,
	.pfnDestroy = el3064_destroy,
	/* Rest initialzied to zero */
};

#pragma pack(1)

typedef struct 
{
	uint16_t txpdo_toggle : 1;
	uint16_t txpdo_state : 1;
	uint16_t _r1 : 9;
	uint16_t error : 1;
	uint16_t lim1 : 2;
	uint16_t lim2 : 2;
	uint16_t overrange : 1;
	uint16_t underrange : 1;
} el3064_pdo_status_t;

#pragma pack()

static int el3064_init(term_context_t* ctx, void* pdo_ai_buf, void* pdo_ao_buf, void* pdo_di_buf, void* pdo_do_buf)
{
	ctx->pvt = malloc(sizeof(el3064_data_t));
	((el3064_data_t*)ctx->pvt)->mapping_type = EL3064_MAPPING_STANDARD;
	return 0;
}

static void el3064_pdo_update(term_context_t* ctx)
{
	el3064_data_t* ppvt = ctx->pvt;
	uint16_t* ai = ctx->pdo_ai_buf;
	static int prev_vals[4] = {0,0,0,0};

	if(ppvt->mapping_type == EL3064_MAPPING_STANDARD)
	{
		/* 16 bit status info and 16-bit value */
		/* For each of these, we will just set a constant value */
		el3064_pdo_status_t tmp_status;
		tmp_status.error = 0;
		tmp_status.overrange = 0;
		tmp_status.underrange = 0;
		tmp_status.txpdo_state = 0;

		/* Use rand to generate some "noise" */
		ai[1] = rand() % 30;
		tmp_status.txpdo_toggle = ai[1] != prev_vals[0] ? 1 : 0;
		ai[0] = *(uint16_t*)&tmp_status;

		ai[3] = rand() % 30;
		tmp_status.txpdo_toggle = ai[3] != prev_vals[1] ? 1 : 0;
		ai[2] = *(uint16_t*)&tmp_status;

		ai[5] = rand() % 30;
		tmp_status.txpdo_toggle = ai[5] != prev_vals[2] ? 1 : 0;
		ai[4] = *(uint16_t*)&tmp_status;

		ai[7] = rand() % 30;
		tmp_status.txpdo_toggle = ai[7] != prev_vals[3] ? 1 : 0;
		ai[6] = *(uint16_t*)&tmp_status;

	}
	else
	{
		el3064_pdo_status_t tmp_status;
		tmp_status.error = 0;
		tmp_status.overrange = 0;
		tmp_status.underrange = 0;
		tmp_status.txpdo_state = 0;

		/* Generate some noise */
		ai[0] = rand() % 30;
		ai[1] = rand() % 30;
		ai[2] = rand() % 30;
		ai[3] = rand() % 30;
	}
}

static int el3064_send_coe_data(term_context_t* ctx, coe_payload_t payload)
{
	return -1;
}

static int el3064_coe_read(term_context_t* ctx, uint16_t* out, coe_read_req_t req)
{
	return -1;
}

static void el3064_destroy(term_context_t* ctx)
{
}
