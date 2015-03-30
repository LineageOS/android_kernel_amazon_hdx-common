/*
 * Rohm BU52061NVX hall sensor driver
 * driver/input/misc/bu52061.c
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input/bu52061.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input.h>

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input/bu52061_ioctl.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
//----------------------------------------------------
#define HALL_EFFECT 33  // Hall sensor output pin -- gpio_wk0

#define KEY_PRESSED 1
#define KEY_RELEASED 0

#ifdef CONFIG_ARCH_MSM8974_APOLLO
#define TIMEOUT_VALUE 1900
#else
#define TIMEOUT_VALUE 1200
#endif
#define ENABLE_REGULATOR

struct regulator *vcc_reg = NULL;
struct timer_list hall_timer;

struct bu52061_platform_data *g_bu52061_data = NULL;

static const char *bu52061_irqs[] = {
  "bu52061_irq1",
#ifdef CONFIG_ARCH_MSM8974_APOLLO
  "bu52061_irq2",
  "bu52061_irq3",
  "bu52061_irq4",
#endif
};

static const char *bu52061_name[] = {
  "bu52061_1",
#ifdef CONFIG_ARCH_MSM8974_APOLLO
  "bu52061_2",
  "bu52061_3",
  "bu52061_4",
#endif
};

static const char *bu52061_phys[] = {
  "bu52061/input1",
#ifdef CONFIG_ARCH_MSM8974_APOLLO
  "bu52061/input2",
  "bu52061/input3",
  "bu52061/input4",
#endif
};

enum backlight_status {
  BL_OFF = 0,
  BL_ON
};

enum hall_status {
  HALL_CLOSED = 0,
  HALL_OPENED
};

enum hall_type {
  POWER = 0,
  CAMERA,
  COVER,
  NONE,
};

enum device_state {
  BOOT = 0,
  UP
};

struct hall_event_info {
  enum backlight_status bl_status;
  enum hall_status hall_current_status;
  unsigned int ignore_hall_event;
  enum hall_type type;
  enum backlight_status previous_bl_status;
};

static struct hall_event_info gHallEventInfo[] =
  {
#ifdef CONFIG_ARCH_MSM8974_THOR
    {BL_ON, HALL_CLOSED, 0, POWER,  BL_OFF},
#else
    {BL_ON, HALL_CLOSED, 0, POWER,  BL_OFF},
    {BL_ON, HALL_CLOSED, 1, NONE,   BL_OFF},
    {BL_ON, HALL_CLOSED, 0, COVER,  BL_OFF},
    {BL_ON, HALL_CLOSED, 0, CAMERA, BL_OFF},
#endif
  };

static const char *hall_name[] = {
  "POWER",
#ifdef CONFIG_ARCH_MSM8974_APOLLO
  "NONE",
  "COVER",
  "CAMERA",
#endif
};

static const char *WAKELOCK = "HallSensor 2000000000";
//----------------------------------------------------
#define BU52061_FTM_PORTING

#ifdef BU52061_FTM_PORTING
#include <linux/miscdevice.h>
#endif

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

#ifdef	CONFIG_PROC_FS      // Proc function
#define	BU52061_PROC_FILE	"driver/hall_sensor"
#define BU52061_COUNT_PROC_FILE       "driver/hall_sensor_count"
static struct proc_dir_entry *bu52061_proc_file;

static unsigned int *bu52061_count[] = {
  0,
#ifdef CONFIG_ARCH_MSM8974_APOLLO
  0,
  0,
  0,
#endif
};

static enum device_state mState = BOOT;

static int bu52061_read_proc(char *page, char **start, off_t off,
		int count, int*eof, void *data)
{
  u8 reg_val=0;
  int i;
  for(i=0; i<MAX_SENSORS; i++){
    reg_val = reg_val | (gHallEventInfo[i].hall_current_status << i) ;
    printk(KERN_DEBUG "%s: hall_%d_status=%d count = %d \n", __func__, i, gHallEventInfo[i].hall_current_status, (int)bu52061_count[i]);
  }
  return snprintf(page, count, "0x%x \n",reg_val);
}

static int bu52061_write_proc(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
  u8 msgbuf[count];
  u8 val;
  int i;

  if (copy_from_user(msgbuf, buffer, count))
    return -EFAULT;

  val=msgbuf[0];
  for(i=0; i<MAX_SENSORS; i++){
    gHallEventInfo[i].hall_current_status = (0x01 & (val >> i));
    bu52061_count[i]=0;
    printk(KERN_DEBUG "%s: hall_%d_write_proc val=%d count =%d \n",__func__, i, val, (int)bu52061_count[i]);
  }
  return count;
}

static int get_count_info(char *buf, char **start, off_t offset, int len, int *eof, void *data)
{
  int i;
  len = sprintf(buf, "Stats ");
  for(i=0; i<MAX_SENSORS; i++){
    len += sprintf(buf+len, " Hall[%d]=%d", i+1, (int)bu52061_count[i]);
  }
  len += sprintf(buf+len, " \n");

  return len;
}

static void create_bu52061_proc_file(void)
{
  bu52061_proc_file = create_proc_entry(BU52061_PROC_FILE, 0644, NULL);
  if (bu52061_proc_file) {
    bu52061_proc_file->read_proc = bu52061_read_proc;
    bu52061_proc_file->write_proc = bu52061_write_proc;

    if (!create_proc_read_entry(BU52061_COUNT_PROC_FILE, 0600, NULL, get_count_info,NULL))
      printk(KERN_INFO "%s: Error creating %s \n", __func__, BU52061_COUNT_PROC_FILE);

  } else
  printk(KERN_ERR "bu52061 proc file create failed!\n");
}

static void remove_bu52061_proc_file(void)
{
  remove_proc_entry(BU52061_PROC_FILE, NULL);
}
#endif      //Proc function

#ifdef BU52061_FTM_PORTING
static int bu52061_open(struct inode *inode, struct file *file)
{
  return 0;
}

static long bu52061_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int ret = 0;
  int i;
  struct ioctl_cmd data;
  memset(&data, 0, sizeof(data));
  switch(cmd) {
    case IOCTL_VALSET:
      if (!access_ok(VERIFY_READ,(void __user *)arg, _IOC_SIZE(cmd))) {
        ret = -EFAULT;
        goto done;
      }
      if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
        ret = -EFAULT;
        goto done;
      }
      printk(KERN_DEBUG "bu52061_ioctl received data = %d\n",data.halt_key);
      for (i=0; i<MAX_SENSORS; i++){
        gHallEventInfo[i].ignore_hall_event = data.halt_key;
      }
      break;
    default:
      ret = -EINVAL;
      break;
  }
 done:
  printk(KERN_DEBUG "bu52061_ioctl DONE \n");
  return ret;
}

static ssize_t bu52061_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos)
{
  unsigned int val=0;
  int i;
  for (i=0; i<MAX_SENSORS; i++){
    val = val | (gHallEventInfo[i].hall_current_status < i);
    printk(KERN_INFO "Hall sensor %d state:%d\n", i, gHallEventInfo[i].hall_current_status);
  }
  printk(KERN_INFO "Hall sensor state: 0x%x\n", val);

  if(copy_to_user(buffer, &val, sizeof(val))){
    printk(KERN_ERR "[line:%d] copy_to_user failed\n",  __LINE__ );
    return -EFAULT;
  }
  return 0;
}

static ssize_t bu52061_write(struct file *file, const char __user *buffer, size_t size, loff_t *f_ops)
{
  return 0;
}
static int bu52061_release(struct inode *inode, struct file *filp)
{
  return 0;
}

#endif //BU52061_FTM_PORTING

static void hall_handle_event(int i)
{
  char *envp[2];
  char buf[120];
  struct kobject *kobj = &g_bu52061_data->pdev->kobj;
  struct input_dev *dev = g_bu52061_data->dev[i];

  gHallEventInfo[i].hall_current_status = gpio_get_value(g_bu52061_data->irq_gpio[i]);

#ifdef CONFIG_PROC_FS
  bu52061_count[i]++;
#endif

  printk(KERN_DEBUG "BU52061: %s hall_%d_current_status = %s\n", hall_name[i], i,
         gHallEventInfo[i].hall_current_status ? "HALL_OPENED" : "HALL_CLOSED");
  printk(KERN_DEBUG "BU52061: %s gHallEventInfo[%d].bl_status = %s\n", hall_name[i], i,
         gHallEventInfo[i].bl_status ? "BL_ON" : "BL_OFF");

  if (gHallEventInfo[i].ignore_hall_event == false) {
    switch(gHallEventInfo[i].type) {
    case POWER:
#ifdef CONFIG_ARCH_MSM8974_APOLLO
      if (gHallEventInfo[2].hall_current_status == HALL_CLOSED)
#endif
        {
          /*Hall sensor State Machine: only two cases need to send power key event:
            1.close book-cover in Normal mode(BL_ON) 2.open book-cover in Suspend mode(BL_OFF) */
          if (((gHallEventInfo[i].bl_status == BL_ON) && (gHallEventInfo[i].hall_current_status == HALL_CLOSED)) ||
              ((gHallEventInfo[i].bl_status == BL_OFF) && (gHallEventInfo[i].hall_current_status == HALL_OPENED))) {

            if ((gHallEventInfo[i].previous_bl_status != gHallEventInfo[i].bl_status) || (mState == BOOT)) {
              printk(KERN_INFO "BU52061: KEY_POWER = %s\n", gHallEventInfo[i].bl_status ? "OFF" : "ON");
              gHallEventInfo[i].previous_bl_status = gHallEventInfo[i].bl_status;
              input_report_key(dev, KEY_POWER, KEY_PRESSED);
              input_sync(dev);
              mdelay(20);
              input_report_key(dev, KEY_POWER, KEY_RELEASED);
              input_sync(dev);
            }
          }
        }
      break;

    case COVER:
      if (gHallEventInfo[i].bl_status == BL_OFF) {
        input_report_key(dev, KEY_POWER, KEY_PRESSED);
        input_sync(dev);
        mdelay(20);
        input_report_key(dev, KEY_POWER, KEY_RELEASED);
        input_sync(dev);
        mdelay(20);
      }
      snprintf(buf, sizeof(buf),"COVER_STATE=%s", gHallEventInfo[i].hall_current_status ? "1" : "0");
      envp[0] = buf;
      envp[1] = NULL;
      kobject_uevent_env(kobj, KOBJ_CHANGE, envp);
      input_event(dev, EV_MSC, MSC_RAW, gHallEventInfo[i].hall_current_status);
      input_sync(dev);
      break;

    case CAMERA:
      snprintf(buf, sizeof(buf),"CAMERA_STATE=%s", gHallEventInfo[i].hall_current_status ? "1" : "0");
      envp[0] = buf;
      envp[1] = NULL;
      kobject_uevent_env(kobj, KOBJ_CHANGE, envp);
      input_event(dev, EV_MSC, MSC_RAW, gHallEventInfo[i].hall_current_status);
      input_sync(dev);
      break;

    case NONE:
      input_event(dev, EV_MSC, MSC_RAW, gHallEventInfo[i].hall_current_status);
      input_sync(dev);
      break;

    default:
      printk(KERN_ERR "BU52061: %s Unknown HALL TYPE \n", __func__);
      break;
    }
  }
}

static void hall_timeout_report(unsigned long arg)
{
  int i=0;
  struct input_dev *dev = g_bu52061_data->dev[i];

  for (i=0; i<MAX_SENSORS; i++){
    gHallEventInfo[i].hall_current_status = gpio_get_value(g_bu52061_data->irq_gpio[i]);
  }

  i = 0;
  printk(KERN_DEBUG "BU52061: %s %s hall_%d_current_status = %s\n", __func__, hall_name[i], i,
         gHallEventInfo[i].hall_current_status ? "HALL_OPENED" : "HALL_CLOSED");
  printk(KERN_DEBUG "BU52061: %s %s gHallEventInfo[%d].bl_status = %s\n", __func__, hall_name[i], i,
         gHallEventInfo[i].bl_status ? "BL_ON" : "BL_OFF");

  if (gHallEventInfo[i].ignore_hall_event == false) {
#ifdef CONFIG_ARCH_MSM8974_APOLLO
    if (gHallEventInfo[2].hall_current_status == HALL_CLOSED)
#endif
      {
        /*Hall sensor State Machine: only two cases need to send power key event:
          1.close book-cover in Normal mode(BL_ON) 2.open book-cover in Suspend mode(BL_OFF) */
        if (((gHallEventInfo[i].bl_status == BL_ON) && (gHallEventInfo[i].hall_current_status == HALL_CLOSED)) ||
            ((gHallEventInfo[i].bl_status == BL_OFF) && (gHallEventInfo[i].hall_current_status == HALL_OPENED))) {

          printk(KERN_INFO "BU52061: %s KEY_POWER = %s\n", __func__, gHallEventInfo[i].bl_status ? "OFF" : "ON");
          gHallEventInfo[i].previous_bl_status = gHallEventInfo[i].bl_status;
          input_report_key(dev, KEY_POWER, KEY_PRESSED);
          input_sync(dev);
          mdelay(20);
          input_report_key(dev, KEY_POWER, KEY_RELEASED);
          input_sync(dev);
        }
      }
  }
}

static void hall_init_timer(void)
{
  init_timer(&hall_timer);
  hall_timer.function = hall_timeout_report;
  hall_timer.data = 0;
  hall_timer.expires = jiffies + ((TIMEOUT_VALUE*HZ)/1000);
  add_timer(&hall_timer);
  printk(KERN_DEBUG "BU52061 hall_init_timer Done\n");
}

#ifdef CONFIG_ARCH_MSM8974_THOR
static void bu52061_irq_work(struct work_struct *work)
{
  hall_handle_event(0);
  mod_timer(&hall_timer, jiffies + ((TIMEOUT_VALUE*HZ)/1000));
#ifdef CONFIG_PM_WAKELOCKS
  pm_wake_lock(WAKELOCK);
#endif
}
#else
static void bu52061_irq_work(struct work_struct *work)
{
  int i;
  for (i=0; i<MAX_SENSORS; i++){
    if (&g_bu52061_data->irq_work[i] == work){
      hall_handle_event(i);
      if(i == 0){
        mod_timer(&hall_timer, jiffies + ((TIMEOUT_VALUE*HZ)/1000));
      }
      break;
    }
  }
}
#endif

static irqreturn_t bu52061_interrupt(int irq, void *dev_id)
{
  int i;
  printk(KERN_ERR "BU52061 bu52061_interrupt irq = %d \n", irq);

#ifdef CONFIG_PM_WAKELOCKS
  pm_wake_lock(WAKELOCK);
#endif

  for (i=0; i<MAX_SENSORS; i++){
    if (irq == g_bu52061_data->irq[i]){
#ifdef CONFIG_ARCH_MSM8974_THOR
      schedule_delayed_work(&g_bu52061_data->irq_work[i], msecs_to_jiffies(750));
#else
      schedule_work(&g_bu52061_data->irq_work[i]);
#endif
      break;
    }
  }
  return IRQ_HANDLED;
}

static int bu52061_input_open(struct input_dev *dev)
{
  return 0;
}
static void bu52061_input_close(struct input_dev *dev)
{

}

#ifdef ENABLE_REGULATOR
static int bu52061_config_regulator(struct device dev, bool on)
{
  int rc = 0;
  if (on) {
    vcc_reg = regulator_get(&dev, "vcc");
      if (IS_ERR(vcc_reg)) {
        rc = PTR_ERR(vcc_reg);
        printk(KERN_ERR "bu52061 regulator_get failed return = %d\n", rc);
        vcc_reg = NULL;
        return rc;
      }

      if (regulator_count_voltages(vcc_reg) > 0) {
        rc = regulator_set_voltage(vcc_reg, 1800000, 1800000);
        if (rc) {
          printk(KERN_ERR "bu52061 regulator_set_voltage failed, return %d\n", rc);
          regulator_put(vcc_reg);
          vcc_reg = NULL;
          return rc;
        }
      }

      rc = regulator_enable(vcc_reg);
      if (rc) {
        printk(KERN_ERR "bu52061 regulator_enable failed, return %d\n", rc);
        if (regulator_count_voltages(vcc_reg) > 0) {
          regulator_set_voltage(vcc_reg, 0, 1800000);
        }
        regulator_put(vcc_reg);
        vcc_reg = NULL;
        return rc;
      }
      printk(KERN_ERR "bu52061 regulator_enabled return %d\n", rc);
  } else {
    if (!IS_ERR_OR_NULL(vcc_reg)) {
      if (regulator_count_voltages(vcc_reg) > 0) {
        regulator_set_voltage(vcc_reg, 0, 1800000);
      }
      regulator_disable(vcc_reg);
      regulator_put(vcc_reg);
      vcc_reg = NULL;
      printk(KERN_ERR "bu52061 regulator_disabled return %d\n", rc);
    }
  }

  return rc;
}
#endif

#ifdef BU52061_FTM_PORTING

static const struct file_operations bu52061_dev_fops = {
  .owner = THIS_MODULE,
  .open = bu52061_open,
  .read = bu52061_read,
  .write = bu52061_write,
  .release = bu52061_release,
  .unlocked_ioctl = bu52061_ioctl,
};

static struct miscdevice bu52061_dev =
{
  .minor = MISC_DYNAMIC_MINOR,
  .name = "bu52061",
  .fops = &bu52061_dev_fops,
};

#endif //BU52061_FTM_PORTING

static int __devinit bu52061_probe(struct platform_device *pdev)
{
  //struct input_dev *input_dev;
  int ret;
  enum of_gpio_flags flags;
  int i;
  struct device_node *node;
  unsigned long irqflags;

  node = pdev->dev.of_node;
  if (node == NULL)
    return -ENODEV;

  g_bu52061_data= kzalloc(sizeof(struct bu52061_platform_data), GFP_KERNEL);
  if (!g_bu52061_data) {
    ret = -ENOMEM;
    printk(KERN_ERR "[%s:%d] allocate g_bu52061_data fail!\n", __func__, __LINE__);
    goto fail1;
  }

#ifdef CONFIG_PROC_FS
  create_bu52061_proc_file();
#endif

  //input device
  for (i=0; i<MAX_SENSORS; i++){
    g_bu52061_data->dev[i] = input_allocate_device();
    if (!g_bu52061_data->dev[i]) {
      ret = -ENOMEM;
      printk(KERN_ERR "[%s:%d] allocate input device fail!\n", __func__, __LINE__);
      goto fail2;
    }

    //input device settings
    g_bu52061_data->dev[i]->name = bu52061_name[i];
    g_bu52061_data->dev[i]->phys = bu52061_phys[i];

    __set_bit(EV_KEY, g_bu52061_data->dev[i]->evbit);
    __set_bit(EV_MSC, g_bu52061_data->dev[i]->evbit);
    __set_bit(MSC_RAW, g_bu52061_data->dev[i]->mscbit);
    __set_bit(KEY_POWER, g_bu52061_data->dev[i]->keybit);

    g_bu52061_data->dev[i]->open = bu52061_input_open;
    g_bu52061_data->dev[i]->close = bu52061_input_close;

    ret = input_register_device(g_bu52061_data->dev[i]);
    if (ret) {
      printk(KERN_ERR "[%s:%d]bu52061 input register device fail!\n", __func__, __LINE__);
      goto fail2;
    }

    g_bu52061_data->irq_gpio[i] = of_get_named_gpio_flags(node, "gpios", i, &flags);
    if (gpio_is_valid(g_bu52061_data->irq_gpio[i])) {
      ret = gpio_request(g_bu52061_data->irq_gpio[i], bu52061_irqs[i]);
      if (ret) {
        printk(KERN_ERR "[%s:%d] failed gpio %d request \n", __func__, __LINE__, g_bu52061_data->irq_gpio[i]);
        goto fail3;;
      }
      ret = gpio_direction_input(g_bu52061_data->irq_gpio[i]);
      if (ret) {
        printk(KERN_ERR "[%s:%d] failed set gpio %d directiont \n", __func__, __LINE__, g_bu52061_data->irq_gpio[i]);
        goto fail3;;
      }
    }
    g_bu52061_data->irq[i] = platform_get_irq(pdev, i);
    printk(KERN_ERR "[%s:%d] gpio %d setup properly irq %d \n", __func__, __LINE__, g_bu52061_data->irq_gpio[i], g_bu52061_data->irq[i]);

#ifdef CONFIG_ARCH_MSM8974_THOR
    INIT_DELAYED_WORK(&g_bu52061_data->irq_work[i], bu52061_irq_work);
#else
    INIT_WORK(&g_bu52061_data->irq_work[i], bu52061_irq_work);
#endif
    irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME | IRQF_EARLY_RESUME;
    ret = request_threaded_irq(g_bu52061_data->irq[i], NULL, bu52061_interrupt, irqflags, bu52061_irqs[i], NULL);
    if (ret) {
      printk(KERN_ERR "bu52061 request_irq %d failed, return:%d\n", g_bu52061_data->irq[i], ret);
      goto fail3;
    }

    if(g_bu52061_data->irq_gpio[i] != 33){
      enable_irq_wake(g_bu52061_data->irq[i]);
    }

    gHallEventInfo[i].hall_current_status = gpio_get_value(g_bu52061_data->irq_gpio[i]);

  }

  hall_init_timer();

#ifdef CONFIG_FB
  g_bu52061_data->fb_notif.notifier_call = fb_notifier_callback;
  ret = fb_register_client(&g_bu52061_data->fb_notif);
  if (ret)
    printk(KERN_ERR "Unable to register fb_notifier: %d\n", ret);
#endif

#ifdef ENABLE_REGULATOR
  ret = bu52061_config_regulator(pdev->dev, 1);
  if (ret) {
    printk(KERN_ERR "[%s:%d] bu52061 bu52061_config_regulator fail!\n", __func__, __LINE__);
    goto fail3;
  }
#endif

  g_bu52061_data->pdev = (&pdev->dev);
  mState = BOOT;
  printk(KERN_INFO "BU52061 Probe OK\n");
  return 0;

fail3:
  for (i=0; i<MAX_SENSORS; i++){
    if (gpio_is_valid(g_bu52061_data->irq_gpio[i])){
      gpio_free(g_bu52061_data->irq_gpio[i]);
    }
    if(g_bu52061_data->irq[i]){
      free_irq(g_bu52061_data->irq[i], NULL);
    }
  }
fail2:
  for (i=0; i<MAX_SENSORS; i++){
    if(g_bu52061_data->dev[i]){
      input_free_device(g_bu52061_data->dev[i]);
    }
  }
  kfree(g_bu52061_data);
fail1:
  return ret;
}

static int __devexit bu52061_remove(struct platform_device *pdev)
{
  int i;
  for (i=0; i<MAX_SENSORS; i++){
    free_irq(g_bu52061_data->irq[i], NULL);
    input_unregister_device(g_bu52061_data->dev[i]);
    input_free_device(g_bu52061_data->dev[i]);
    gpio_free(g_bu52061_data->irq_gpio[i]);
  }

#ifdef CONFIG_PROC_FS
  remove_bu52061_proc_file();
#endif

#ifdef CONFIG_FB
  fb_unregister_client(&g_bu52061_data->fb_notif);
#endif
  del_timer_sync(&hall_timer);

#ifdef CONFIG_PM_WAKELOCKS
  pm_wake_unlock(WAKELOCK);
#endif

#ifdef ENABLE_REGULATOR
  bu52061_config_regulator(pdev->dev, 0);
#endif

  kfree(g_bu52061_data);

  return 0;
}

static void bu52061_shutdown(struct platform_device *pdev)
{
  int i;
  printk(KERN_INFO "bu52061: shutdown\n");
  for (i=0; i<MAX_SENSORS; i++)
#ifdef CONFIG_ARCH_MSM8974_THOR
    cancel_delayed_work_sync(&g_bu52061_data->irq_work[i]);
#else
    cancel_work_sync(&g_bu52061_data->irq_work[i]);
#endif
}

static int bu52061_suspend(struct device *dev)
{
  int i;
  for(i=0; i<MAX_SENSORS; i++) {
    gHallEventInfo[i].bl_status = BL_OFF;
    gHallEventInfo[i].previous_bl_status = BL_ON;
  }
  mState = UP;
  printk(KERN_INFO "BU52061 bu52061_suspend\n");
  return 0;
}

static int bu52061_resume(struct device *dev)
{
	int i = 0;
	for (i = 0; i < MAX_SENSORS; i++) {
		gHallEventInfo[i].bl_status = BL_ON;
		gHallEventInfo[i].previous_bl_status = BL_OFF;
                check_irq_resend(irq_to_desc(g_bu52061_data->irq[i]), g_bu52061_data->irq[i]);
		if (i == 2)
			hall_handle_event(i);
	}
	printk(KERN_INFO "BU52061 bu52061_resume\n");
	return 0;
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
  struct fb_event *evdata = data;
  int *blank;
  if ((evdata && evdata->data) && (event == FB_EVENT_BLANK)) {
    blank = evdata->data;
    if (*blank == FB_BLANK_UNBLANK)
      bu52061_resume(g_bu52061_data->pdev);
    else if (*blank == FB_BLANK_POWERDOWN)
      bu52061_suspend(g_bu52061_data->pdev);
  }

  return 0;
}
#endif

static const struct dev_pm_ops bu52061_dev_pm_ops = {
  .suspend  = bu52061_suspend,
  .resume   = bu52061_resume,
};

static struct of_device_id bu52061_of_match[] = {
  {.compatible = "rohm,bu52061", },
  { },
};
MODULE_DEVICE_TABLE(of, bu52061_of_match);

static struct platform_driver bu52061_device_driver = {
  .probe    = bu52061_probe,
  .remove   = bu52061_remove,
  .shutdown = bu52061_shutdown,
  .driver   = {
    .name   = "bu52061",
    .owner  = THIS_MODULE,
    .of_match_table = of_match_ptr(bu52061_of_match),
    }
};

static int __init bu52061_init(void)
{
  printk(KERN_INFO "BU52061 sensors driver: init\n");

#ifdef BU52061_FTM_PORTING
  if (0 != misc_register(&bu52061_dev))
  {
    printk(KERN_ERR "bu52061_dev register failed.\n");
    return 0;
  }
else
  {
    printk(KERN_INFO "bu52061_dev register ok.\n");
  }
#endif

  return platform_driver_register(&bu52061_device_driver);
}

static void __exit bu52061_exit(void)
{
  printk(KERN_INFO "BU52061 sensors driver: exit\n");

#ifdef BU52061_FTM_PORTING
  misc_deregister(&bu52061_dev);
#endif
  platform_driver_unregister(&bu52061_device_driver);
}

module_init(bu52061_init);
module_exit(bu52061_exit);

MODULE_DESCRIPTION("Rohm BU52061NVX hall sensor driver");
MODULE_AUTHOR("Joss");
MODULE_LICENSE("GPL");
