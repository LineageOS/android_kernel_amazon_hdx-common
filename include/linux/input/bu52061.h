#ifndef __BU52061_H__
#define __BU52061_H__
#include <linux/types.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <asm/atomic.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#if defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#if defined(CONFIG_ARCH_MSM8974_THOR)
#define MAX_SENSORS 1
#elif defined(CONFIG_ARCH_MSM8974_APOLLO)
#define MAX_SENSORS 4
#else
#define MAX_SENSORS 1
#endif

#ifdef CONFIG_PM_WAKELOCKS
/* kernel/power/wakelock.c */
extern ssize_t pm_show_wakelocks(char *buf, bool show_active);
extern int pm_wake_lock(const char *buf);
extern int pm_wake_unlock(const char *buf);
#endif /* !CONFIG_PM_WAKELOCKS */

struct bu52061_platform_data {
  struct input_dev *dev[MAX_SENSORS];
  atomic_t used;
#ifdef CONFIG_ARCH_MSM8974_THOR
  struct delayed_work irq_work[MAX_SENSORS];
#else
  struct work_struct irq_work[MAX_SENSORS];
#endif
  unsigned int irq[MAX_SENSORS];
  unsigned int irq_gpio[MAX_SENSORS];
  int (*init_irq)(void);
  int (*get_gpio_value)(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
  struct early_suspend early_suspend;
#else
#ifdef CONFIG_FB
  struct notifier_block fb_notif;
#endif
#endif
  struct device *pdev;
};



#endif // __BU52061_H__

