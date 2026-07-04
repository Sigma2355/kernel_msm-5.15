// SPDX-License-Identifier: GPL-2.0-only
/*
 * linux/drivers/cpufreq/cpufreq_sprea.c
 *
 * Copyright (C) 2002 - 2003 Dominik Brodowski <linux@brodo.de>
 * Modified by Gemini for Sprea Governor Ultra-Maximus Edition.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/module.h>

static void cpufreq_gov_sprea_limits(struct cpufreq_policy *policy)
{
 /* * TWEAK TINGKAT MAKSIMAL (HARDWARE CEILING LOCK):
  * Di sini kita tidak lagi menggunakan 'policy->max' (batas maksimal dinamis software),
  * melainkan langsung membaca 'policy->cpuinfo.max_freq' yang merupakan batas atas fisik 
  * mutlak dari silikon CPU dari pabrikan.
  *
  * Logika ini memaksa sistem mengabaikan perintah thermal throttling maupun pembatasan daya 
  * dari framework Android, mengunci mati batas bawah (min) dan batas atas (max) di titik tertinggi.
  */
 policy->min = policy->cpuinfo.max_freq;
 policy->max = policy->cpuinfo.max_freq;

 pr_info("SPREA ULTRA-MAXIMUS: Core %u dikunci mati pada batas fisik %u kHz\n", 
  policy->cpu, policy->cpuinfo.max_freq);

 /* * Selalu paksa target frekuensi ke batas fisik maksimum dengan CPUFREQ_RELATION_L.
  * Ini memastikan tidak ada celah bagi driver hardware untuk melakukan downscaling.
  */
 __cpufreq_driver_target(policy, policy->cpuinfo.max_freq, CPUFREQ_RELATION_L);
}

static struct cpufreq_governor cpufreq_gov_sprea = {
 .name  = "sprea",
 .owner  = THIS_MODULE,
 .flags  = CPUFREQ_GOV_STRICT_TARGET,
 .limits  = cpufreq_gov_sprea_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SPREA
struct cpufreq_governor *cpufreq_default_governor(void)
{
 return &cpufreq_gov_sprea;
}
#endif
#ifndef CONFIG_CPU_FREQ_GOV_SPREA_MODULE
struct cpufreq_governor *cpufreq_fallback_governor(void)
{
 return &cpufreq_gov_sprea;
}
#endif

MODULE_AUTHOR("Sprei & Gemini <linux@brodo.de>");
MODULE_DESCRIPTION("CPUfreq policy governor 'sprea' - Ultra-Maximus Edition");
MODULE_LICENSE("GPL");

cpufreq_governor_init(cpufreq_gov_sprea);
cpufreq_governor_exit(cpufreq_gov_sprea);
