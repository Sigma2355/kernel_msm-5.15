// SPDX-License-Identifier: GPL-2.0
/*
* ATOS (AuTomatic Operate System)
*
* Modul kernel Linux ini menyediakan kerangka kerja untuk mengelola mode
* di tingkat kernel. Profil sekarang disederhanakan menjadi dua status:
* 0 : Mati (Sistem berjalan standar)
* 1 : Hidup (Performa / Memaksa IO 'twesu' dan Governor 'sprea')
*
* Menggunakan metode VFS Write untuk kompatibilitas mutlak di Kernel 5.15+.
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#ifdef CONFIG_AUTO_ATOS_MSM_DRM
#include <linux/msm_drm_notify.h>
#elif defined(CONFIG_AUTO_ATOS_MI_DRM)
#include <drm/drm_notifier_mi.h>
#elif defined(CONFIG_AUTO_ATOS_FB)
#include <linux/fb.h>
#endif
#include "version.h"
#include <linux/notifier.h>
#include <linux/kthread.h>
#include <linux/cpufreq.h>
#include <linux/blkdev.h>
#include <linux/cpu.h>
#include <linux/fs.h>      /* Untuk trik filp_open & kernel_write */
#include <linux/file.h>

#ifdef CONFIG_AUTO_ATOS_MSM_DRM
#define ATOS_EVENT_BLANK MSM_DRM_EVENT_BLANK
#define ATOS_BLANK_POWERDOWN MSM_DRM_BLANK_POWERDOWN
#define ATOS_BLANK_UNBLANK MSM_DRM_BLANK_UNBLANK
#define atos_events msm_drm_notifier
#elif defined(CONFIG_AUTO_ATOS_MI_DRM)
#define ATOS_EVENT_BLANK MI_DRM_EVENT_BLANK
#define ATOS_BLANK_POWERDOWN MI_DRM_BLANK_POWERDOWN
#define ATOS_BLANK_UNBLANK MI_DRM_BLANK_UNBLANK
#define atos_events mi_drm_notifier
#elif defined(CONFIG_AUTO_ATOS_FB)
#define ATOS_EVENT_BLANK FB_EVENT_BLANK
#define ATOS_BLANK_POWERDOWN FB_BLANK_POWERDOWN
#define ATOS_BLANK_UNBLANK FB_BLANK_UNBLANK
#define atos_events fb_event
#endif

static BLOCKING_NOTIFIER_HEAD(atos_mode_notifier);
unsigned int ATOS_MODE_CHANGE = 0x80000000;
EXPORT_SYMBOL(ATOS_MODE_CHANGE);

static unsigned int atos_override_mode;
static bool atos_override;
#ifdef CONFIG_AUTO_ATOS
static bool screen_on = true;
#endif

static bool auto_atos __read_mostly = !IS_ENABLED(CONFIG_AUTO_ATOS_NONE);
module_param(auto_atos, bool, 0664);
MODULE_PARM_DESC(auto_atos, "Aktifkan/nonaktifkan manajemen ATOS otomatis");

static unsigned int atos_mode = 0; // Default: Mati (0)

static struct kobject *atos_kobj;

DEFINE_MUTEX(atos_set_mode_rb_lock);
DEFINE_SPINLOCK(atos_set_mode_lock);

#define atos_err(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#define atos_info(fmt, ...) pr_info(fmt, ##__VA_ARGS__)

static void atos_trigger_mode_change_event(void);
static void atos_apply_performance_profile(void);

void atos_set_mode_rollback(unsigned int level, unsigned int duration_ms)
{
#ifdef CONFIG_AUTO_ATOS
        if (!screen_on) return;
#endif

        if (!auto_atos) return;

        mutex_lock(&atos_set_mode_rb_lock);
        if (unlikely(level > 1)) {
                mutex_unlock(&atos_set_mode_rb_lock);
                return;
        }

        atos_override_mode = level;
        atos_override = true;
        atos_trigger_mode_change_event();
        msleep(duration_ms);
        atos_override = false;
        atos_trigger_mode_change_event();
        mutex_unlock(&atos_set_mode_rb_lock);
}
EXPORT_SYMBOL(atos_set_mode_rollback);

static __always_inline int __atos_set_mode(unsigned int level)
{
        if (unlikely(level > 1))
                return -EINVAL;

        atos_mode = level;
        return 0;
}

void atos_set_mode(unsigned int level)
{
        int ret = 0;

#ifdef CONFIG_AUTO_ATOS
        if (!screen_on) return;
#endif

        if (!auto_atos) return;

        spin_lock(&atos_set_mode_lock);
        ret = __atos_set_mode(level);
        if (ret) {
                spin_unlock(&atos_set_mode_lock);
                return;
        }

        atos_trigger_mode_change_event();
        spin_unlock(&atos_set_mode_lock);
}
EXPORT_SYMBOL(atos_set_mode);

int atos_active_mode(void)
{
#ifdef CONFIG_AUTO_ATOS
        // Jika layar mati dan auto_atos aktif, paksa ke mode 0 (Mati)
        if (!screen_on && auto_atos)
                return 0;
#endif

        if (atos_override)
                return atos_override_mode;

        if (unlikely(atos_mode > 1)) {
                atos_mode = 0;
                atos_trigger_mode_change_event();
        }

        return atos_mode;
}
EXPORT_SYMBOL(atos_active_mode);

static __always_inline void atos_trigger_mode_change_event(void)
{
        unsigned int current_mode = atos_active_mode();
        blocking_notifier_call_chain(&atos_mode_notifier, ATOS_MODE_CHANGE,
                                     (void *)(uintptr_t)current_mode);

        // Jika mode 1, terapkan performa dengan metode force VFS write
        if (current_mode == 1) {
                atos_info("Mode Hidup: Mengaktifkan profil performa ATOS\n");
                atos_apply_performance_profile();
        } else {
                atos_info("Mode Mati: ATOS kembali ke normal\n");
        }
}

int atos_notifier_register_client(struct notifier_block *nb)
{
        return blocking_notifier_chain_register(&atos_mode_notifier, nb);
}
EXPORT_SYMBOL(atos_notifier_register_client);

int atos_notifier_unregister_client(struct notifier_block *nb)
{
        return blocking_notifier_chain_unregister(&atos_mode_notifier, nb);
}
EXPORT_SYMBOL(atos_notifier_unregister_client);

#ifdef CONFIG_AUTO_ATOS
static inline int atos_display_notifier_callback(struct notifier_block *self,
                                               unsigned long event, void *data)
{
        struct atos_events *evdata = data;
        unsigned int blank;

        if (event != ATOS_EVENT_BLANK)
                return 0;

        if (evdata && evdata->data) {
                blank = *(int *)(evdata->data);
                switch (blank) {
                case ATOS_BLANK_POWERDOWN:
                        if (!screen_on) break;
                        screen_on = false;
                        atos_trigger_mode_change_event();
                        break;
                case ATOS_BLANK_UNBLANK:
                        if (screen_on) break;
                        screen_on = true;
                        atos_trigger_mode_change_event();
                        break;
                default:
                        break;
                }
        }
        return NOTIFY_OK;
}

static struct notifier_block atos_display_notifier_block = {
        .notifier_call = atos_display_notifier_callback,
};

static inline int atos_register_display_notifier(void)
{
#ifdef CONFIG_AUTO_ATOS_MSM_DRM
        return msm_drm_register_client(&atos_display_notifier_block);
#elif defined(CONFIG_AUTO_ATOS_MI_DRM)
        return mi_drm_register_client(&atos_display_notifier_block);
#elif defined(CONFIG_AUTO_ATOS_FB)
        return fb_register_client(&atos_display_notifier_block);
#else
        return 0;
#endif
}

static inline void atos_unregister_display_notifier(void)
{
#ifdef CONFIG_AUTO_ATOS_MSM_DRM
        msm_drm_unregister_client(&atos_display_notifier_block);
#elif defined(CONFIG_AUTO_ATOS_MI_DRM)
        mi_drm_unregister_client(&atos_display_notifier_block);
#elif defined(CONFIG_AUTO_ATOS_FB)
        fb_unregister_client(&atos_display_notifier_block);
#endif
}
#else
static inline int atos_register_display_notifier(void) { return 0; }
static inline void atos_unregister_display_notifier(void) { }
#endif

/* Tampilkan state ASLI dari mode, bukan sekedar variabel statis */
static inline ssize_t atos_mode_show(struct kobject *kobj,
                                   struct kobj_attribute *attr, char *buf)
{
        return scnprintf(buf, PAGE_SIZE, "%u\n", atos_active_mode());
}

/* FUNGSI YANG DIPERBAIKI: Tangkap echo userspace dan jalankan trigger! */
static inline ssize_t atos_mode_store(struct kobject *kobj,
                                    struct kobj_attribute *attr,
                                    const char *buf, size_t count)
{
        unsigned int new_mode;
        int ret;

        ret = kstrtouint(buf, 10, &new_mode);
        if (ret) return ret;

        ret = __atos_set_mode(new_mode);
        if (ret) {
                atos_err("Perubahan mode tidak valid (hanya 0 atau 1)!\n");
                return ret;
        }

        /* INILAH YANG HILANG SEBELUMNYA! Pemanggil Eksekutornya */
        atos_trigger_mode_change_event();

        return count;
}

static struct kobj_attribute atos_mode_attribute =
        __ATTR(atos_mode, 0664, atos_mode_show, atos_mode_store);

static struct attribute *atos_attrs[] = {
        &atos_mode_attribute.attr,
        NULL,
};

static struct attribute_group atos_attr_group = {
        .attrs = atos_attrs,
};

static int __init atos_init(void)
{
        int ret = 0;

        atos_kobj = kobject_create_and_add("atos", kernel_kobj);
        if (!atos_kobj) return -ENOMEM;

        ret = sysfs_create_group(atos_kobj, &atos_attr_group);
        if (ret) goto err_kobj;

        ret = atos_register_display_notifier();
        if (ret) goto err_group;

        atos_info("ATOS dimuat. (0=Mati, 1=Hidup)\n");
        return 0;

err_group:
        sysfs_remove_group(atos_kobj, &atos_attr_group);
err_kobj:
        kobject_put(atos_kobj);
        return ret;
}
module_init(atos_init);

static void __exit atos_exit(void)
{
        atos_unregister_display_notifier();
        sysfs_remove_group(atos_kobj, &atos_attr_group);
        kobject_put(atos_kobj);
}
module_exit(atos_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ATOS (AuTomatic Operate System)");
MODULE_AUTHOR("Dimodifikasi untuk ATOS Project");

/*
* Metode VFS Cheat Code: Memaksa penggantian Governor CPU ke 'sprea'
* dengan mensimulasikan userspace echo secara langsung ke sysfs dalam kernel.
*/
static void atos_apply_cpu_governor(void)
{
        char path[64];
        struct file *file;
        loff_t pos;
        int cpu;
        const char *gov = "sprea\n";

        for_each_online_cpu(cpu) {
                snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor", cpu);
                file = filp_open(path, O_WRONLY, 0);
                if (!IS_ERR(file)) {
                        pos = 0;
                        kernel_write(file, gov, strlen(gov), &pos);
                        filp_close(file, NULL);
                        atos_info("ATOS: CPU%d governor sukses dipaksa ke sprea\n", cpu);
                }
        }
}

/*
* Metode VFS Cheat Code: Memaksa penggantian IO Scheduler ke 'twesu'
*/
static void atos_apply_performance_profile(void)
{
        struct gendisk *gd;
        struct class_dev_iter iter;
        char path[128];
        struct file *file;
        loff_t pos;
        const char *iosched = "twesu\n";

        // 1. Eksekusi Governor CPU
        atos_apply_cpu_governor();

        // 2. Eksekusi IO Scheduler
        class_dev_iter_init(&iter, &block_class, NULL, NULL);
        while ((gd = dev_to_disk(class_dev_iter_next(&iter)))) {
                if (!gd->queue) continue;
                
                snprintf(path, sizeof(path), "/sys/block/%s/queue/scheduler", gd->disk_name);
                file = filp_open(path, O_WRONLY, 0);
                if (!IS_ERR(file)) {
                        pos = 0;
                        kernel_write(file, iosched, strlen(iosched), &pos);
                        filp_close(file, NULL);
                        atos_info("ATOS: IO scheduler disk %s sukses dipaksa ke twesu\n", gd->disk_name);
                }
        }
        class_dev_iter_exit(&iter);
}