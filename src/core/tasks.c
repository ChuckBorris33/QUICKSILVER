#include "tasks.h"

#include <stddef.h>

#include "driver/serial.h"
#include "driver/usb.h"
#include "flight/control.h"
#include "flight/gestures.h"
#include "flight/imu.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd/render.h"
#include "profile.h"
#include "project.h"
#include "rx/rx.h"

void task_main() {
  // all flight calculations and motors
  control();

  // attitude calculations for level mode
  imu_calc();

  // battery low logic
  vbat_calc();

  // check gestures
  if (flags.on_ground && !flags.gestures_disabled) {
    gestures();
  }

  // handle led commands
  led_update();

#if (RGB_LED_NUMBER > 0)
  // RGB led control
  rgb_led_lvc();
#ifdef RGB_LED_DMA
  rgb_dma_start();
#endif
#endif

  buzzer_update();
}

FAST_RAM task_t tasks[TASK_MAX] = {
    [TASK_GYRO] = CREATE_TASK("GYRO", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, sixaxis_read),
    [TASK_MAIN] = CREATE_TASK("MAIN", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, task_main),
    [TASK_RX] = CREATE_TASK("RX", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, rx_update),
#ifdef ENABLE_BLACKBOX
    [TASK_BLACKBOX] = CREATE_TASK("BLACKBOX", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, blackbox_update),
#endif
    [TASK_OSD] = CREATE_TASK("OSD", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, osd_display),
    [TASK_VTX] = CREATE_TASK("VTX", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, vtx_update),
    [TASK_USB] = CREATE_TASK("USB", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, usb_configurator),
};