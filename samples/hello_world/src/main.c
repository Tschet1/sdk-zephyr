/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include "nrf.h"
#include <helpers/nrf_vdma.h>
#include "hal/nrf_ppib.h"
#include "hal/nrf_dppi.h"

#include <zephyr/bluetooth/bluetooth.h>

#define HAL_CCM_AAR_IRK_SIZE                                          (16)
#define HAL_AAR_IRK_SIZE                                          (HAL_CCM_AAR_IRK_SIZE)
#define HAL_AAR_NR_OF_IRK                                         (16)
#define BTLE_DEVICE_ADDRESS__SIZE                 (6)
#define HAL_AAR_INVALID_IRK_INDEX HAL_AAR_NR_OF_IRK
#define BTLE_IRK__SIZE                            (16)
#define AAR_DMA_ATTR_HASH  11
#define AAR_DMA_ATTR_PRAND 12
#define AAR_DMA_ATTR_IRK   13
#define AAR_DMA_ATTR_OUT  11

struct
{
  bool aar_in_use_flag;
} m_hal_aar;

typedef enum
{
  HAL_AAR_STATUS_UNRESOLVED = 0,
  HAL_AAR_STATUS_RESOLVED,
  HAL_AAR_STATUS_IN_PROGRESS,
  HAL_AAR_STATUS_ERROR,
} hal_aar_status_t;

typedef uint8_t (hal_aar_data_t)[HAL_AAR_IRK_SIZE];
void hal_aar_configure(hal_aar_data_t * p_aar_irk_data, uint8_t nirk, const uint8_t * p_pdu_addr_field);
void hal_aar_enable_now(void);
bool hal_aar_wait_for_resolution(void);

hal_aar_status_t hal_aar_status_get( void );


int main(void)
{
	printk("Start!\n");
	uint8_t irk_data[8 * 2][BTLE_IRK__SIZE];  /**< Array of Local & Peer IRKs. */
	uint8_t addr[] = {0x72, 0xa3, 0xf7, 0x42, 0x34, 0x5c};
	uint8_t addr2[] = {0x5c, 0x34, 0x42, 0xf7, 0xa3, 0x72};

	for(uint8_t i=0; i<BTLE_IRK__SIZE; ++i)
	{
		irk_data[0][i] = 0xbb;
	}
	for(uint8_t i=0; i<BTLE_IRK__SIZE; ++i)
	{
		irk_data[1][i] = 0xaa;
	}

	bool res;
	printk("try resolve matching\n");
	hal_aar_configure((hal_aar_data_t *) &irk_data, 2, addr2);
	hal_aar_enable_now();
	res = hal_aar_wait_for_resolution();
	printk("resolved? %u\n", res);
	/* --> expect success */

	printk("try resolve non matching\n");
	hal_aar_configure((hal_aar_data_t *) &irk_data, 2, addr);
	hal_aar_enable_now();
	res = hal_aar_wait_for_resolution();
	printk("resolved? %u\n", res);
	/* --> expect fail */

	printk("try resolve again matching\n");
	hal_aar_configure((hal_aar_data_t *) &irk_data, 2, addr2);
	hal_aar_enable_now();
	res = hal_aar_wait_for_resolution();
	printk("resolved? %u\n", res);
	/* --> expect success */

	return 0;
}


static uint16_t m_aar_resolved_index[16];
static inline bool m_aar_resolve_status_only_is_resolved(void);
static inline void m_hal_aar_events_clear(void);

void hal_aar_configure(hal_aar_data_t * p_aar_irk_data, uint8_t nirk, const uint8_t * p_pdu_addr_field)
{
  NRF_AAR00->ENABLE = (AAR_ENABLE_ENABLE_Enabled << AAR_ENABLE_ENABLE_Pos);

  static nrf_vdma_job_t s_in_joblist[4] = {0};
  if(m_hal_aar.aar_in_use_flag)
  {
    /* only update address to resolve */
    s_in_joblist[0] = (nrf_vdma_job_t) {.p_buffer = (uint8_t *)p_pdu_addr_field, {.size = BTLE_DEVICE_ADDRESS__SIZE / 2, .attributes = AAR_DMA_ATTR_HASH}};
    s_in_joblist[1] = (nrf_vdma_job_t) {.p_buffer = (uint8_t *)p_pdu_addr_field + 3, {.size = BTLE_DEVICE_ADDRESS__SIZE / 2, .attributes = AAR_DMA_ATTR_PRAND}};
    s_in_joblist[2] = (nrf_vdma_job_t) {0};
  }
  else
  {
    /* add full config */
    s_in_joblist[0] = (nrf_vdma_job_t) {.p_buffer = (uint8_t *)p_pdu_addr_field, {.size = BTLE_DEVICE_ADDRESS__SIZE / 2, .attributes = AAR_DMA_ATTR_HASH}};
    s_in_joblist[1] = (nrf_vdma_job_t) {.p_buffer = (uint8_t *)p_pdu_addr_field + 3, {.size = BTLE_DEVICE_ADDRESS__SIZE / 2, .attributes = AAR_DMA_ATTR_PRAND}};
    s_in_joblist[2] = (nrf_vdma_job_t) {.p_buffer = (uint8_t *)p_aar_irk_data, {.size = HAL_AAR_IRK_SIZE * nirk, .attributes = AAR_DMA_ATTR_IRK}};
  }

  NRF_AAR00->IN.PTR = (uint32_t)s_in_joblist;

  static nrf_vdma_job_t s_out_joblist[2] = {0};
  s_out_joblist[0] = (nrf_vdma_job_t){.p_buffer = (uint8_t *) m_aar_resolved_index, {.size = 2*nirk, .attributes = AAR_DMA_ATTR_OUT}};

  NRF_AAR00->MAXRESOLVED = nirk;
  NRF_AAR00->OUT.PTR = (uint32_t)s_out_joblist;

  m_hal_aar.aar_in_use_flag = true;
}

void hal_aar_enable_now(void)
{
  NRF_AAR00->ENABLE = (AAR_ENABLE_ENABLE_Enabled << AAR_ENABLE_ENABLE_Pos);
  m_hal_aar_events_clear();
   NRF_AAR00->TASKS_START = 1;
}

bool hal_aar_wait_for_resolution(void)
{
  hal_aar_status_t aar_status;

  do
  {
    aar_status = hal_aar_status_get();

  }
  while (aar_status == HAL_AAR_STATUS_IN_PROGRESS);
  return (aar_status == HAL_AAR_STATUS_RESOLVED);
}

hal_aar_status_t hal_aar_status_get( void )
{
  if ( m_hal_aar.aar_in_use_flag )
  {
    if (NRF_AAR00->EVENTS_END == 0)
    {
      return ( HAL_AAR_STATUS_IN_PROGRESS );
    }
    else if (m_aar_resolve_status_only_is_resolved()) // EVENTS_END is guaranteed to be 1 here, so don't check it again (optimization).
    {
      return ( HAL_AAR_STATUS_RESOLVED );
    }
    else
    {
      return ( HAL_AAR_STATUS_UNRESOLVED );
    }
  }
  return HAL_AAR_STATUS_IN_PROGRESS;
}

static inline bool m_aar_resolve_status_only_is_resolved(void)
{
  return ((NRF_AAR00->EVENTS_RESOLVED != 0) && (NRF_AAR00->OUT.AMOUNT > 0));
}


static inline void m_hal_aar_events_clear(void)
{
 NRF_AAR00->EVENTS_END         = 0;
 NRF_AAR00->EVENTS_RESOLVED    = 0;
 NRF_AAR00->EVENTS_NOTRESOLVED = 0;
}
