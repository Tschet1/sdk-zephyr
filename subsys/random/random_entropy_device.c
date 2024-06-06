/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <string.h>

//static const struct device *const entropy_dev =
//	DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));

static inline uint32_t m_rotl(const uint32_t x, int k)
{
  return (x << k) | (x >> (32 - k));
}

static uint32_t m_get(void)
{

  static uint32_t s0     = 5;
  static uint32_t s1     = 8;
  const uint32_t result = m_rotl(s0 * 0x9E3779BB, 5) * 5;

  s1 ^= s0;
  s0 = m_rotl(s0, 26) ^ s1 ^ (s1 << 9);
  s1 = m_rotl(s1, 13);

  return result;
}

static uint8_t rand_prio_low_vector_get(uint8_t *p_buff, uint8_t length)
{
	for(uint8_t i=0; i<length; ++i)
	{
		p_buff[i] = m_get() & 0xff;
	}
	return length;
}

static void rand_prio_low_vector_get_blocking(uint8_t *p_buff, uint8_t length)
{
	(void) rand_prio_low_vector_get(p_buff, length);
}

static int rand_get(uint8_t *dst, size_t outlen, bool csrand)
{
	(void) csrand;

	return rand_prio_low_vector_get(dst, outlen);
}

#if defined(CONFIG_ENTROPY_DEVICE_RANDOM_GENERATOR)
void z_impl_sys_rand_get(void *dst, size_t outlen)
{
	rand_get(dst, outlen, false);
}
#endif /* CONFIG_ENTROPY_DEVICE_RANDOM_GENERATOR */

#if defined(CONFIG_HARDWARE_DEVICE_CS_GENERATOR)

int z_impl_sys_csrand_get(void *dst, size_t outlen)
{
	if (rand_get(dst, outlen, true) != 0) {
		/* Is it the only error it should return ? entropy_sam
		 * can return -ETIMEDOUT for example
		 */
		return -EIO;
	}

	return 0;
}

#endif /* CONFIG_HARDWARE_DEVICE_CS_GENERATOR */
