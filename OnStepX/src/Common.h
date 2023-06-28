// -----------------------------------------------------------------------------------
// Common includes
#pragma once

#define SERIAL_LOCAL_MODE ON
#define STANDARD_IPSERIAL_CHANNEL ON
#define PERSISTENT_IPSERIAL_CHANNEL ON

#include <Arduino.h>
#include "Constants.h"
#include "lib/Constants.h"
#include "../Config.h"
#include "Config.defaults.h"

#ifdef ESP32
  #if OPERATIONAL_MODE == WIFI && WEB_SERVER == ON
    #define NV_WIFI_SETTINGS_BASE (NV_LAST+1) // bytes: 451 , 451
  #endif
  #define NV_PEC_BUFFER_BASE    (NV_LAST+452) // bytes: ?   , ? + (PEC_BUFFER_SIZE_LIMIT - 1)
#else
  #define NV_PEC_BUFFER_BASE      (NV_LAST+1) // bytes: ?   , ? + (PEC_BUFFER_SIZE_LIMIT - 1)
#endif

#include "HAL/HAL.h"
#include "lib/Macros.h"
#include "pinmaps/Models.h"
#include "lib/debug/Debug.h"
#include "lib/nv/NV.h"
extern NVS nv;

#if ST4_HAND_CONTROL == ON
  #define SERIAL_ST4_MASTER ON
#endif

#if AXIS1_DRIVER_MODEL != OFF && AXIS2_DRIVER_MODEL != OFF
  #define MOUNT_PRESENT
#endif

#if AXIS3_DRIVER_MODEL != OFF
  #define ROTATOR_PRESENT
#endif

#if AXIS4_DRIVER_MODEL != OFF || AXIS5_DRIVER_MODEL != OFF || AXIS6_DRIVER_MODEL != OFF || AXIS7_DRIVER_MODEL != OFF || AXIS8_DRIVER_MODEL != OFF || AXIS9_DRIVER_MODEL != OFF
  #define FOCUSER_PRESENT
#endif

#if FEATURE1_PURPOSE != OFF || FEATURE2_PURPOSE != OFF || FEATURE3_PURPOSE != OFF || FEATURE4_PURPOSE != OFF || FEATURE5_PURPOSE != OFF || FEATURE6_PURPOSE != OFF || FEATURE7_PURPOSE != OFF || FEATURE8_PURPOSE != OFF
  #define FEATURES_PRESENT
#endif

#if FEATURE1_PURPOSE == DEW_HEATER || FEATURE2_PURPOSE == DEW_HEATER || FEATURE3_PURPOSE == DEW_HEATER || FEATURE4_PURPOSE == DEW_HEATER || FEATURE5_PURPOSE == DEW_HEATER || FEATURE6_PURPOSE == DEW_HEATER || FEATURE7_PURPOSE == DEW_HEATER || FEATURE8_PURPOSE == DEW_HEATER
  #define DEW_HEATER_PRESENT

  #if (FEATURE1_TEMP & DS_MASK) == DS1820  || (FEATURE2_TEMP & DS_MASK) == DS1820  || (FEATURE3_TEMP & DS_MASK) == DS1820  || (FEATURE4_TEMP & DS_MASK) == DS1820  || \
      (FEATURE5_TEMP & DS_MASK) == DS1820  || (FEATURE6_TEMP & DS_MASK) == DS1820  || (FEATURE7_TEMP & DS_MASK) == DS1820  || (FEATURE8_TEMP & DS_MASK) == DS1820  || \
      (FEATURE1_TEMP & DS_MASK) == DS18S20 || (FEATURE2_TEMP & DS_MASK) == DS18S20 || (FEATURE3_TEMP & DS_MASK) == DS18S20 || (FEATURE4_TEMP & DS_MASK) == DS18S20 || \
      (FEATURE5_TEMP & DS_MASK) == DS18S20 || (FEATURE6_TEMP & DS_MASK) == DS18S20 || (FEATURE7_TEMP & DS_MASK) == DS18S20 || (FEATURE8_TEMP & DS_MASK) == DS18S20
    #define DS1820_DEVICES_PRESENT
  #endif

  #if FEATURE1_TEMP == THERMISTOR1 || FEATURE2_TEMP == THERMISTOR1 || FEATURE3_TEMP == THERMISTOR1 || FEATURE4_TEMP == THERMISTOR1 || \
      FEATURE5_TEMP == THERMISTOR1 || FEATURE6_TEMP == THERMISTOR1 || FEATURE7_TEMP == THERMISTOR1 || FEATURE8_TEMP == THERMISTOR1 || \
      FEATURE1_TEMP == THERMISTOR2 || FEATURE2_TEMP == THERMISTOR2 || FEATURE3_TEMP == THERMISTOR2 || FEATURE4_TEMP == THERMISTOR2 || \
      FEATURE5_TEMP == THERMISTOR2 || FEATURE6_TEMP == THERMISTOR2 || FEATURE7_TEMP == THERMISTOR2 || FEATURE8_TEMP == THERMISTOR2
    #define THERMISTOR_DEVICES_PRESENT
  #endif
#endif

#if (FOCUSER_TEMPERATURE & DS_MASK) == DS1820 || (FOCUSER_TEMPERATURE & DS_MASK) == DS18S20
  #ifndef DS1820_DEVICES_PRESENT
    #define DS1820_DEVICES_PRESENT
  #endif
#endif

#if FOCUSER_TEMPERATURE == THERMISTOR1 || FOCUSER_TEMPERATURE == THERMISTOR2
  #ifndef THERMISTOR_DEVICES_PRESENT
    #define THERMISTOR_DEVICES_PRESENT
  #endif
#endif

#include "lib/gpio/Gpio.h"
