/* ---------------------------------------------------------------------------------------------------------------------------------
 * Configuration for OnStepX
 *
 *          For more information on setting OnStep up see http://www.stellarjourney.com/index.php?r=site/equipment_onstep 
 *                      and join the OnStep Groups.io at https://groups.io/g/onstep
 * 
 *           *** Read the compiler warnings and errors, they are there to help guard against invalid configurations ***
 *
 * ---------------------------------------------------------------------------------------------------------------------------------
 * ADJUST THE FOLLOWING TO CONFIGURE YOUR CONTROLLER FEATURES ----------------------------------------------------------------------
 * <-Req'd = always must set, <-Often = usually must set, Option = optional, Adjust = adjust as req'd, Infreq = infrequently changed
*/
//      Parameter Name              Value   Default  Notes                                                                      Hint

// =================================================================================================================================
// CONTROLLER ======================================================================================================================

// PINMAP ------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Controller#PINMAP
#define PINMAP                    MaxESP4 //    OFF, Choose from: MiniPCB, MiniPCB2, MaxPCB2, MaxESP3, CNC3, STM32Blue,      <-Req'd
                                          //         MaxSTM3, FYSETC_S6_2, etc.  Other boards and more info. in ~/src/Constants.h

// SERIAL PORT COMMAND CHANNELS --------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Controller#SERIAL_PORTS
#define SERIAL_A_BAUD_DEFAULT        9600 //   9600, n. Where n=9600,19200,57600,115200,230400,460800 (common baud rates.)    Infreq
#define SERIAL_B_BAUD_DEFAULT         OFF //   9600, n. Baud rate as above. See (src/pinmaps/) for Serial port assignments.   Infreq
#define SERIAL_B_ESP_FLASHING         OFF //    OFF, ON Upload ESP8266 WiFi firmware through SERIAL_B with :ESPFLASH# cmd.    Option
#define SERIAL_C_BAUD_DEFAULT         OFF //    OFF, n. Baud rate as above. See (src/pinmaps/) for Serial port assignments.   Infreq
#define SERIAL_D_BAUD_DEFAULT         OFF //    OFF, n. Baud rate as above. See (src/pinmaps/) for Serial port assignments.   Infreq
#define SERIAL_E_BAUD_DEFAULT         OFF //    OFF, n. Baud rate as above. See (src/pinmaps/) for Serial port assignments.   Infreq

// STATUS --------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Controller#STATUS_LED
#define STATUS_LED                    OFF //    OFF, Steady illumination if no error, blinks w/error code otherwise.          Option

// RETICLE CONTROL ------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Controller#RETICLE_CONTROL
#define RETICLE_LED_DEFAULT           OFF //    OFF, n. Where n=0..255 (0..100%) activates feature sets default brightness.   Option
#define RETICLE_LED_MEMORY            OFF //    OFF, ON Remember reticle brightness across power cycles.                      Option
#define RETICLE_LED_INVERT            OFF //    OFF, ON Inverts state for cases where 0% is full brightness otherwise.        Option

// WEATHER SENSOR --------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Controller#WEATHER_SENSOR
#define WEATHER                       OFF //    OFF, BME280 (I2C 0x77,) BME280_0x76, BME280_SPI (see pinmap for CS.)          Option
                                          //         BMP280 (I2C 0x77,) BMP280_0x76, BMP280_SPI (see pinmap for CS.)
                                          //         BME280 or BMP280 for temperature, pressure.  BME280 for humidity also.

// SIGNALING ------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Controller#SIGNALING
#define STEP_WAVE_FORM             SQUARE // SQUARE, PULSE Step signal wave form faster rates. SQUARE best signal integrity.  Adjust
                                          //         Applies to all axes.


// SETTINGS FOR SERVO
#define SERIAL_ENCODER            Serial2 //
#define SERIAL_ENCODER_BAUD        921200 //

#define DRIVER_TMC_STEPPER                //  Enable for TMC driver

//#define SERVO_SAFETY_DISABLE              //   DONT USE THIS UNTIL YOU HAVE MORE CONFIDENCE
                                          //   or you are near the on/off switch of your PCB.



// =================================================================================================================================
// MOUNT ===========================================================================================================================

// Driver models (Step/Dir and Servo) many have specific requirements so be sure to follow the link below to help learn about these.
// Typically: A4988, DRV8825, LV8729, S109, TMC2130, TMC5160, TMC2209, etc.

// AXIS1 RA/AZM -------------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Axes
#define AXIS1_DRIVER_MODEL  SERVO_TMC2209 //    OFF, Enter motor driver model (above) in both axes to activate the mount.    <-Often

// If runtime axis settings are enabled changes in the section below may be ignored unless you reset to defaults:
// \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ 
#define AXIS1_STEPS_PER_DEGREE   14166.67 //  12800, n. Number of steps or encoder counts per degree:                        <-Req'd
                                          //         n = (stepper_steps * micro_steps * overall_gear_reduction)/360.0
#define AXIS1_REVERSE                 OFF //    OFF, ON Reverses movement direction, or reverse wiring instead to correct.   <-Often
#define AXIS1_LIMIT_MIN              -180 //   -180, n. Where n= -90..-360 (degrees.) Minimum "Hour Angle" or Azimuth.        Adjust
#define AXIS1_LIMIT_MAX               180 //    180, n. Where n=  90.. 360 (degrees.) Maximum "Hour Angle" or Azimuth.        Adjust

#define AXIS1_DRIVER_MICROSTEPS        16 //    OFF, n. Microstep mode when tracking.                                        <-Req'd
#define AXIS1_DRIVER_MICROSTEPS_GOTO  OFF //    OFF, n. Microstep mode used during slews. OFF uses _DRIVER_MICROSTEPS.        Option

// for TMC2130, TMC5160, TMC2209, TMC2226 STEP/DIR driver models:
#define AXIS1_DRIVER_IHOLD            OFF //    OFF, n, (mA.) Current during standstill. OFF uses IRUN/2.0                    Option
#define AXIS1_DRIVER_IRUN            1000 //    OFF, n, (mA.) Current during tracking, appropriate for stepper/driver/etc.    Option
#define AXIS1_DRIVER_IGOTO            OFF //    OFF, n, (mA.) Current during slews. OFF uses IRUN.                            Option
// /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /

#define AXIS1_DRIVER_STATUS            ON //    OFF, ON, HIGH, or LOW.  For driver status info/fault detection.               Option

#define AXIS1_DRIVER_DECAY      STEALTHCHOP //    OFF, Tracking decay mode default override. TMC default is STEALTHCHOP.        Infreq
#define AXIS1_DRIVER_DECAY_GOTO SPREADCYCLE //    OFF, Decay mode goto default override. TMC default is SPREADCYCLE.            Infreq

#define AXIS1_POWER_DOWN              OFF //    OFF, ON Powers off 30sec after movement stops or 10min after last<=1x guide.  Infreq

#define AXIS1_SENSE_HOME              OFF //    OFF, HIGH or LOW enables & state clockwise home position, as seen from front. Option
#define AXIS1_SENSE_LIMIT_MIN LIMIT_SENSE // ...NSE, HIGH or LOW state on limit sense switch stops movement.                  Option
#define AXIS1_SENSE_LIMIT_MAX LIMIT_SENSE // ...NSE, HIGH or LOW state on limit sense switch stops movement.                  Option
                                          //         Digital, optionally add: |HYST(n) Where n=0..1023 stability time in ms.
                                          //         Analog capable sense inputs also allow adding:
                                          //         |THLD(n) Where n=1..1023 (ADU) for Analog threshold.
                                          //         |HYST(n) Where n=0..1023 (ADU) for +/- Hystersis range.

#define AXIS1_ENCODER       SERIAL_BRIDGE // Default Value: AB  AB, AB_ESP32, CW_CCW, PULSE_DIR, PULSE_ONLY, AS37_H39B_B, SERIAL_BRIDGE
                                          //  For servo drivers, specifies the encoder type:
                                          //  AB for common AB quadrature encoders.
                                          //  AB_ESP32 for common AB quadrature encoders with fast hardware decode on ESP32 micro-controllers.
                                          //  CW_CCW for encoders with separate clockwise and counter-clockwise signals.
                                          //  PULSE_DIR for encoders with pulse and direction signals.
                                          //  PULSE_ONLY for encoders that send only a pulse.
                                          //  This is the simplest type of encoder where even a small button magnet on a motor shaft along with an hall-effect sensor can establish how many turns the motors shaft has taken and the speed.
                                          //  In this case the direction is inferred from the control input (which direction the OCS is telling the motor to run.)
#define AXIS1_ENCODER_ORIGIN            0 // Default Value: 0    Other Values: 0 to 4294967296 (counts)

#define AXIS1_ENCODER_REVERSE         OFF // Default Value: OFF  Other Values: OFF, ON  Reverses encoder count direction
#define AXIS1_TARGET_TOLERANCE          0 // Default Value: 0    Recommended Values: 0 to 120 (arc-seconds)  Servos don't always arrive exactly at the target coordinate and this allows a small margin of error so gotos end quickly.
#define AXIS1_SERVO_MAX_VELOCITY    22666 // Default Value: 100  Recommended Values: 0 to 1000000 (% or steps/s)  Velocity limit, in % for DC, in steps/s for SERVO_TMC2209.
// #define AXISn_VELOCITY_FACTOR           0 // 
#define AXIS1_SERVO_ACCELERATION     8000 // Default Value: 20   Recommended Values: 0 to 1000000 (% or steps/s)  Acceleration limit, in %/s/s for DC, in steps/s/s for SERVO_TMC2209.
#define AXIS1_SERVO_SYNC_THRESHOLD    OFF // Default Value: OFF  Recommended Values: 0 to 100000 (counts)  Sync threshold in counts (for absolute encoders) or OFF.
#define AXIS1_SERVO_FLTR              OFF // Default Value: OFF  Other Values: OFF, KALMAN
                                          //  Notes:
                                          //  The KALMAN filter smooths encoder readings trading resolution and responsiveness for accuracy.
                                          //  Tips:
                                          //  If using KALMAN the following two options must also be added to Config.h (for example):
                                          //  #define AXIS1_SERVO_FLTR_MEAS_U      20 // kalaman measurement uncertainty, in encoder ticks
                                          //  #define AXIS1_SERVO_FLTR_VARIANCE   0.1 // kalaman responsiveness, usually between 0.001 and 1
                                          //  If using KALMAN the required encoder library is here: https://github.com/denyssene/SimpleKalmanFilter
#define AXIS1_PID_P                   2.0 // Default Value: 2.0   Recommended Values: 0.0 to 200.0  Proportional, scale of immediate response to position error.
#define AXIS1_PID_I                   5.0 // Default Value: 5.0   Recommended Values: 0.0 to 200.0  Integral, rate of increasing response to position error over time.
                                          // See: https://en.wikipedia.org/wiki/PID_controller
#define AXIS1_PID_D                   5.0 // Default Value: 5.0   Recommended Values: 0.0 to 200.0  Derivative, overshoot suppression.
#define AXIS1_PID_P_GOTO              2.0 // Default Value: 2.0   Recommended Values: 0.0 to 200.0  Proportional, scale of immediate response to position error  Applied during faster motion.
#define AXIS1_PID_I_GOTO              5.0 // Default Value: 5.0   Recommended Values: 0.0 to 200.0 Integral, rate of increasing response to position error over time. Applied during faster motion.
#define AXIS1_PID_D_GOTO              5.0 // Default Value: 5.0   Recommended Values: 0.0 to 200.0  Derivative, overshoot suppression.  Applied during faster motion.
#define AXIS1_PID_SENSITIVITY           0 // Default Value: 0     Recommended Values: 0, 1 to 100 (%)
                                          //  When the default setting (0) is used the Axis slewing state selects between the tracking or slewing PID sets.
                                          //  When switching from tracking to slewing mode, the slewing PID (_GOTO) settings are immediately adopted.
                                          //  When switching from slewing mode to tracking mode, the PID settings are changed over a one second period transitioning smoothly from slewing to tracking values to help prevent oscillation.
                                          //  You can change the default one second period for this transition by adding the following setting to Config.h (for example, two seconds:)




// AXIS2 DEC/ALT ------------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Axes
#define AXIS2_DRIVER_MODEL  SERVO_TMC2209 //    OFF, Enter motor driver model (above) in both axes to activate the mount.    <-Often

// If runtime axis settings are enabled changes in the section below may be ignored unless you reset to defaults:
// \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
#define AXIS2_STEPS_PER_DEGREE   14166.67 //  12800, n. Number of steps or encoder counts per degree:                        <-Req'd
                                          //         n = (stepper_steps * micro_steps * overall_gear_reduction)/360.0
#define AXIS2_REVERSE                 OFF //    OFF, ON Reverses movement direction, or reverse wiring instead to correct.   <-Often
#define AXIS2_LIMIT_MIN               -90 //    -90, n. Where n=-90..0 (degrees.) Minimum allowed Declination or Altitude.    Infreq
#define AXIS2_LIMIT_MAX                90 //     90, n. Where n=0..90 (degrees.) Maximum allowed Declination or Altitude.     Infreq

#define AXIS2_DRIVER_MICROSTEPS        16 //    OFF, n. Microstep mode when tracking.                                        <-Req'd
#define AXIS2_DRIVER_MICROSTEPS_GOTO  OFF //    OFF, n. Microstep mode used during slews. OFF uses _DRIVER_MICROSTEPS.        Option

// for TMC2130, TMC5160, TMC2209, TMC2226 STEP/DIR driver models:
#define AXIS2_DRIVER_IHOLD            OFF //    OFF, n, (mA.) Current during standstill. OFF uses IRUN/2.0                    Option
#define AXIS2_DRIVER_IRUN            1000 //    OFF, n, (mA.) Current during tracking, appropriate for stepper/driver/etc.    Option
#define AXIS2_DRIVER_IGOTO            OFF //    OFF, n, (mA.) Current during slews. OFF uses IRUN.                            Option
// /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /

#define AXIS2_DRIVER_STATUS            ON //    OFF, ON, HIGH, or LOW.  Polling for driver status info/fault detection.       Option

#define AXIS2_DRIVER_DECAY       STEALTHCHOP //    OFF, Tracking decay mode default override. TMC default is STEALTHCHOP.        Infreq
#define AXIS2_DRIVER_DECAY_GOTO  SPREADCYCLE //    OFF, Decay mode goto default override. TMC default is SPREADCYCLE.            Infreq

#define AXIS2_POWER_DOWN              OFF //    OFF, ON Powers off 30sec after movement stops or 10min after last<=1x guide.  Option

#define AXIS2_SENSE_HOME              OFF //    OFF, HIGH or LOW enables & state clockwise home position, as seen from above. Option
#define AXIS2_SENSE_LIMIT_MIN LIMIT_SENSE // ...NSE, HIGH or LOW state on limit sense switch stops movement.                  Option
#define AXIS2_SENSE_LIMIT_MAX LIMIT_SENSE // ...NSE, HIGH or LOW state on limit sense switch stops movement.                  Option

#define AXIS2_ENCODER       SERIAL_BRIDGE // Default Value: AB  AB, AB_ESP32, CW_CCW, PULSE_DIR, PULSE_ONLY, AS37_H39B_B, SERIAL_BRIDGE
                                          //  For servo drivers, specifies the encoder type:
                                          //  AB for common AB quadrature encoders.
                                          //  AB_ESP32 for common AB quadrature encoders with fast hardware decode on ESP32 micro-controllers.
                                          //  CW_CCW for encoders with separate clockwise and counter-clockwise signals.
                                          //  PULSE_DIR for encoders with pulse and direction signals.
                                          //  PULSE_ONLY for encoders that send only a pulse.
                                          //  This is the simplest type of encoder where even a small button magnet on a motor shaft along with an hall-effect sensor can establish how many turns the motors shaft has taken and the speed.
                                          //  In this case the direction is inferred from the control input (which direction the OCS is telling the motor to run.)
#define AXIS2_ENCODER_ORIGIN            0 // Default Value: 0    Other Values: 0 to 4294967296 (counts)

#define AXIS2_ENCODER_REVERSE         OFF // Default Value: OFF  Other Values: OFF, ON  Reverses encoder count direction
#define AXIS2_TARGET_TOLERANCE          0 // Default Value: 0    Recommended Values: 0 to 120 (arc-seconds)  Servos don't always arrive exactly at the target coordinate and this allows a small margin of error so gotos end quickly.
#define AXIS2_SERVO_MAX_VELOCITY    11333 // Default Value: 100  Recommended Values: 0 to 1000000 (% or steps/s)  Velocity limit, in % for DC, in steps/s for SERVO_TMC2209.
// #define AXISn_VELOCITY_FACTOR           0 //
#define AXIS2_SERVO_ACCELERATION      100 // Default Value: 20   Recommended Values: 0 to 1000000 (% or steps/s)  Acceleration limit, in %/s/s for DC, in steps/s/s for SERVO_TMC2209.
#define AXIS2_SERVO_SYNC_THRESHOLD    OFF // Default Value: OFF  Recommended Values: 0 to 100000 (counts)  Sync threshold in counts (for absolute encoders) or OFF.
#define AXIS2_SERVO_FLTR              OFF // Default Value: OFF  Other Values: OFF, KALMAN
                                          //  Notes:
                                          //  The KALMAN filter smooths encoder readings trading resolution and responsiveness for accuracy.
                                          //  Tips:
                                          //  If using KALMAN the following two options must also be added to Config.h (for example):
                                          //  #define AXIS1_SERVO_FLTR_MEAS_U      20 // kalaman measurement uncertainty, in encoder ticks
                                          //  #define AXIS1_SERVO_FLTR_VARIANCE   0.1 // kalaman responsiveness, usually between 0.001 and 1
                                          //  If using KALMAN the required encoder library is here: https://github.com/denyssene/SimpleKalmanFilter
#define AXIS2_PID_P                   2.0 // Default Value: 2.0   Recommended Values: 0.0 to 200.0  Proportional, scale of immediate response to position error.
#define AXIS2_PID_I                   5.0 // Default Value: 5.0   Recommended Values: 0.0 to 200.0  Integral, rate of increasing response to position error over time.
                                          // See: https://en.wikipedia.org/wiki/PID_controller
#define AXIS2_PID_D                   5.0 // Default Value: 5.0   Recommended Values: 0.0 to 200.0  Derivative, overshoot suppression.
#define AXIS2_PID_P_GOTO              2.0 // Default Value: 2.0   Recommended Values: 0.0 to 200.0  Proportional, scale of immediate response to position error  Applied during faster motion.
#define AXIS2_PID_I_GOTO              5.0 // Default Value: 5.0   Recommended Values: 0.0 to 200.0 Integral, rate of increasing response to position error over time. Applied during faster motion.
#define AXIS2_PID_D_GOTO              5.0 // Default Value: 5.0   Recommended Values: 0.0 to 200.0  Derivative, overshoot suppression.  Applied during faster motion.
#define AXIS2_PID_SENSITIVITY           0 // Default Value: 0     Recommended Values: 0, 1 to 100 (%)
                                          //  When the default setting (0) is used the Axis slewing state selects between the tracking or slewing PID sets.
                                          //  When switching from tracking to slewing mode, the slewing PID (_GOTO) settings are immediately adopted.
                                          //  When switching from slewing mode to tracking mode, the PID settings are changed over a one second period transitioning smoothly from slewing to tracking values to help prevent oscillation.
                                          //  You can change the default one second period for this transition by adding the following setting to Config.h (for example, two seconds:)




// MOUNT -------------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Mount#MOUNT
#define MOUNT_TYPE                    GEM //    GEM, GEM         German Equatorial Mount, etc. that need meridian flips.     <-Req'd
                                          //         GEM_TA      GEM w/tangent arm Declination
                                          //         GEM_TAC     GEM w/tangent arm Declination and geometry correction
                                          //         FORK        Fork Mount
                                          //         FORK_TA     FORK w/tangent arm Declination
                                          //         FORK_TAC    FORK w/tangent arm Declination and geometry correction
                                          //         ALTAZM      Altitude/Azimuth Mount, Dobsonians, etc.
                                          //         ALTAZM_UNL  ALTAZM w/unlimited Azimuth motion

#define MOUNT_COORDS          TOPOCENTRIC // ...RIC, Applies refraction to coordinates to/from OnStep, except exactly         Infreq
                                          //              at the poles. Use TOPO_STRICT to apply refraction even in that case.
                                          //              Use OBSERVED_PLACE for no refraction.

#define MOUNT_ENABLE_IN_STANDBY       OFF //    OFF, ON Enables mount motor drivers while in standby.                         Infreq

// TIME AND LOCATION ---------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Mount#TLS
#define TIME_LOCATION_SOURCE       DS3231 //    OFF, DS3231 (I2C,) SD3031 (I2C,) TEENSY (T3.2 etc,) GPS, or NTP source.       Option
                                          //         Provides Date/Time, and if available, PPS & Lat/Long also.
#define TIME_LOCATION_PPS_SENSE      HIGH //    OFF, HIGH senses PPS (pulse per second,) signal rising edge, or use LOW for   Option
                                          //         falling edge, or use BOTH for rising and falling edges.
                                          //         Better tracking accuracy especially for Mega2560's w/ceramic resonator.

// STATUS ------------------------------------------------------ see https://onstep.groups.io/g/main/wiki/Configuration_Mount#STATUS
#define STATUS_MOUNT_LED              OFF //    OFF, ON Flashes proportional to rate of movement or solid on for slews.       Option
#define STATUS_BUZZER                 OFF //    OFF, ON, n. Where n=100..6000 (Hz freq.) for speaker. ON for piezo buzzer.    Option
#define STATUS_BUZZER_DEFAULT         OFF //    OFF, ON default starts w/buzzer enabled.                                      Option
#define STATUS_BUZZER_MEMORY          OFF //    OFF, ON to remember buzzer setting across power cycles.                       Option

// ST4 INTERFACE -------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Mount#ST4
// *** It is up to you to verify the interface meets the electrical specifications of any connected device, use at your own risk ***
#define ST4_INTERFACE                 OFF //    OFF, ON enables interface. <= 1X guides unless hand control mode.             Option
                                          //         During goto btn press: aborts slew or continue meridian flip pause home
#define ST4_HAND_CONTROL               ON //     ON, ON for hand controller special features and SHC support.                 Option
                                          //         Hold [E]+[W] btns >2s: Guide rate   [E]-  [W]+  [N] trk on/off [S] sync
                                          //         Hold [N]+[S] btns >2s: Usr cat item [E]-  [W]+  [N] goto [S] snd on/off
#define ST4_HAND_CONTROL_FOCUSER       ON //     ON, ON alternate to above: Focuser move [E]f1 [W]f2 [N]-     [S]+            Option

// GUIDING BEHAVIOUR ------------------------------------------ see https://onstep.groups.io/g/main/wiki/Configuration_Mount#GUIDING
#define GUIDE_TIME_LIMIT               10 //     10, n. Time limit n=0..120 seconds. Use 0 to disable.                        Adjust
#define GUIDE_DISABLE_BACKLASH        OFF //    OFF, Disable backlash takeup during guiding at <= 1X.                         Option

// LIMITS ------------------------------------------------------ see https://onstep.groups.io/g/main/wiki/Configuration_Mount#LIMITS
#define LIMIT_SENSE                   OFF //    OFF, HIGH or LOW state on limit sense switch stops movement.                  Option
#define LIMIT_STRICT                  OFF //    OFF, disables limits until unpark goto or sync. ON enables limits at startup. Option
                                          //         note that ON also disables all motion until date/time are set.

// PARKING ---------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Mount#PARKING
#define PARK_SENSE                    OFF //    OFF, HIGH or LOW state indicates mount is in the park orientation.            Option
#define PARK_SIGNAL                   OFF //    OFF, HIGH or LOW state park input signal triggers parking.                    Option
#define PARK_STATUS                   OFF //    OFF, signals with a HIGH or LOW state when successfully parked.               Option 
#define PARK_STRICT                   OFF //    OFF, ON Un-parking is only allowed if successfully parked.                    Option

// PEC ---------------------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Mount
#define PEC_STEPS_PER_WORM_ROTATION     0 //      0, n. Steps per worm rotation (0 disables else 720 sec buffer allocated.)  <-Req'd
                                          //         n = (AXIS1_STEPS_PER_DEGREE*360)/reduction_final_stage

#define PEC_SENSE                     OFF //    OFF, HIGH. Senses the PEC signal rising edge or use LOW for falling edge.     Option
                                          //         Ignored in ALTAZM mode.
#define PEC_BUFFER_SIZE_LIMIT         720 //    720, Seconds of PEC buffer allowed.                                           Infreq

// TRACKING BEHAVIOUR ---------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Mount#TRACKING
#define TRACK_BACKLASH_RATE            20 //     20, n. Where n=2..50 (x sidereal rate) during backlash takeup.               Option
                                          //         Too fast motors stall/gears slam or too slow and sluggish in backlash.
#define TRACK_AUTOSTART               OFF //    OFF, ON Start with tracking enabled.                                          Option
#define TRACK_COMPENSATION_DEFAULT    OFF //    OFF, No compensation or REFRACTION, REFRACTION_DUAL, MODEL, MODEL_DUAL.       Option
#define TRACK_COMPENSATION_MEMORY     OFF //    OFF, ON Remembers refraction/pointing model compensated tracking settings.    Option

// SLEWING BEHAVIOUR ------------------------------------------ see https://onstep.groups.io/g/main/wiki/Configuration_Mount#SLEWING
#define SLEW_RATE_BASE_DESIRED        1.0 //    1.0, n. Desired slew rate in deg/sec. Adjustable at run-time from            <-Req'd
                                          //         1/2 to 2x this rate, and as performace considerations require.
#define SLEW_RATE_MEMORY              OFF //    OFF, ON Remembers rates set across power cycles.                              Option
#define SLEW_ACCELERATION_DIST        5.0 //    5.0, n, (degrees.) Approx. distance for acceleration (and deceleration.)      Adjust
#define SLEW_RAPID_STOP_DIST          2.0 //    2.0, n, (degrees.) Approx. distance required to stop when a slew              Adjust
                                          //         is aborted or a limit is exceeded.
#define GOTO_FEATURE                   ON //     ON, Use OFF to disable mount Goto features.                                  Infreq
#define GOTO_OFFSET                  0.25 //   0.25, Offset in deg's for goto target unidirectional approach, 0.0 disables    Adjust
#define GOTO_OFFSET_ALIGN             OFF //    OFF, ON skips final phase of goto for align stars so user tends to approach   Option
                                          //         from the correct direction when centering.

// PIER SIDE BEHAVIOUR --------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Mount#PIERSIDE
#define MFLIP_SKIP_HOME               OFF //    OFF, ON Goto directly to the destination without visiting home position.      Option
#define MFLIP_AUTOMATIC_DEFAULT       OFF //    OFF, ON Start with automatic meridian flips enabled.                          Option
#define MFLIP_AUTOMATIC_MEMORY        OFF //    OFF, ON Remember automatic meridian flip setting across power cycles.         Option
#define MFLIP_PAUSE_HOME_DEFAULT      OFF //    OFF, ON Start with meridian flip pause at home enabed.                        Infreq
#define MFLIP_PAUSE_HOME_MEMORY       OFF //    OFF, ON Remember meridian flip pause at home setting across power cycles.     Infreq

#define PIER_SIDE_SYNC_CHANGE_SIDES   OFF //    OFF, ON Allows sync to change pier side, for GEM mounts.                      Option
#define PIER_SIDE_PREFERRED_DEFAULT  BEST //   BEST, BEST Stays on current side if possible. EAST or WEST switch if possible. Option
#define PIER_SIDE_PREFERRED_MEMORY    OFF //    OFF, ON Remember preferred pier side setting across power cycles.             Option

// ALIGN -------------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Mount#Align
#define ALIGN_AUTO_HOME               OFF //    OFF, ON uses home switches to find home first when starting an align.         Option
#define ALIGN_MODEL_MEMORY            OFF //    OFF, ON Restores any pointing model saved in NV at startup.                   Option
#define ALIGN_MAX_STARS              AUTO //   AUTO, Uses HAL specified default (either 6 or 9 stars.)                        Infreq
                                          //         Or use n. Where n=1 (for Sync only) or 3 to 9 (for Goto Assist.)

// =================================================================================================================================
// ROTATOR =========================================================================================================================

// Driver models (Step/Dir and Servo) many have specific requirements so be sure to follow the link below to help learn about these.
// Typically: A4988, DRV8825, LV8729, S109, TMC2130S, etc.

// AXIS3 ROTATOR --------------------------------------- see https://onstep.groups.io/g/developer/wiki/Configuration_Rotator_Focuser
#define AXIS3_DRIVER_MODEL        TMC2209 //    OFF, Enter motor driver model (above) to activate the rotator.                Option
#define AXIS3_SLEW_RATE_BASE_DESIRED  2.0 //    1.0, n. Desired slew rate in deg/sec. Adjustable at run-time from            <-Req'd
                                          //         1/2 to 2x this rate, and as performance considerations require.

// If runtime axis settings are enabled changes in the section below may be ignored unless you reset to defaults:
// \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
#define AXIS3_STEPS_PER_DEGREE  177.77778 //   64.0, n. Number of steps per degree for rotator/de-rotator.                    Adjust
                                          //         Alt/Az de-rotation: n = (num_circumference_pixels * 2)/360, minimum
#define AXIS3_REVERSE                 OFF //    OFF, ON Reverses movement direction, or reverse wiring instead to correct.    Option
#define AXIS3_LIMIT_MIN                 0 //      0, n. Where n=-360..0 (degrees.) Minimum allowed rotator angle.             Infreq
#define AXIS3_LIMIT_MAX               359 //    360, n. Where n=0..360 (degrees.) Maximum allowed rotator angle.              Infreq

#define AXIS3_DRIVER_MICROSTEPS        32 //    OFF, n. Microstep mode when tracking.                                         Option
#define AXIS3_DRIVER_MICROSTEPS_GOTO  OFF //    OFF, n. Microstep mode used during slews. OFF uses _DRIVER_MICROSTEPS.        Option

// for TMC2130, TMC5160, TMC2209, TMC2226 STEP/DIR driver models:
#define AXIS3_DRIVER_IHOLD            OFF //    OFF, n, (mA.) Current during standstill. OFF uses IRUN/2.0                    Option
#define AXIS3_DRIVER_IRUN            1000 //    OFF, n, (mA.) Current during tracking, appropriate for stepper/driver/etc.    Option
#define AXIS3_DRIVER_IGOTO            OFF //    OFF, n, (mA.) Current during slews. OFF uses IRUN.                            Option
// /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /

#define AXIS3_DRIVER_STATUS            ON //    OFF, ON, HIGH, or LOW.  For driver status info/fault detection.               Option

#define AXIS3_DRIVER_DECAY            OFF //    OFF, Tracking decay mode default override. TMC default is STEALTHCHOP.        Infreq
#define AXIS3_DRIVER_DECAY_GOTO       OFF //    OFF, Decay mode goto default override. TMC default is SPREADCYCLE.            Infreq

#define AXIS3_POWER_DOWN              OFF //    OFF, ON Powers off 30 seconds after movement stops.                           Option

#define AXIS3_SENSE_HOME             HIGH //    OFF, HIGH or LOW enables & state clockwise home position, as seen from above. Option
#define AXIS3_SENSE_HOME_PIN           13 //
#define SLEW_HOME_REFINE_TIME_LIMIT     0 //    120 is default
#define AXIS3_RAPID_STOP_TIME           0 //    1 second
#define AXIS3_ACCELERATION_TIME       7.0 //    2 default

#define AXIS3_SENSE_LIMIT_MIN         OFF //    OFF, HIGH or LOW state on limit sense switch stops movement.                  Option
#define AXIS3_SENSE_LIMIT_MAX         OFF //    OFF, HIGH or LOW state on limit sense switch stops movement.                  Option

// =================================================================================================================================
// FOCUSERS ========================================================================================================================

// Driver models (Step/Dir and Servo) many have specific requirements so be sure to follow the link below to help learn about these.
// Typically: A4988, DRV8825, LV8729, S109, TMC2130S, etc.

// AXIS4 FOCUSER 1 ------------------------------------- see https://onstep.groups.io/g/developer/wiki/Configuration_Rotator_Focuser
#define AXIS4_DRIVER_MODEL            OFF //    OFF, Enter motor driver model (above) to activate the focuser.                Option
#define AXIS4_SLEW_RATE_BASE_DESIRED  500 //    500, n, Where n=200..5000 (um/s.) Adjustable at run-time from                <-Req'd
                                          //         1/2 to 2x this rate, and as performance considerations require.
#define AXIS4_SLEW_RATE_MINIMUM        20 //     20, n. Where n=5..200 (um/s.) Minimum microns/second.                        Adjust

// If runtime axis settings are enabled changes in the section below may be ignored unless you reset to defaults:
// \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
#define AXIS4_STEPS_PER_MICRON        0.5 //    0.5, n. Steps per micrometer. Figure this out by testing or other means.      Adjust
#define AXIS4_REVERSE                 OFF //    OFF, ON Reverses movement direction, or reverse wiring instead to correct.    Option
#define AXIS4_LIMIT_MIN                 0 //      0, n. Where n=0..500 (millimeters.) Minimum allowed position.               Adjust
#define AXIS4_LIMIT_MAX                50 //     50, n. Where n=0..500 (millimeters.) Maximum allowed position.               Adjust

#define AXIS4_DRIVER_MICROSTEPS       OFF //    OFF, n. Microstep mode when tracking.                                         Option
#define AXIS4_DRIVER_MICROSTEPS_GOTO  OFF //    OFF, n. Microstep mode used during slews. OFF uses _DRIVER_MICROSTEPS.        Option

// for TMC2130, TMC5160, TMC2209, TMC2226 STEP/DIR driver models:
#define AXIS4_DRIVER_IHOLD            OFF //    OFF, n, (mA.) Current during standstill. OFF uses IRUN/2.0                    Option
#define AXIS4_DRIVER_IRUN             OFF //    OFF, n, (mA.) Current during tracking, appropriate for stepper/driver/etc.    Option
#define AXIS4_DRIVER_IGOTO            OFF //    OFF, n, (mA.) Current during slews. OFF uses IRUN.                            Option
// /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /\ /

#define AXIS4_DRIVER_STATUS           OFF //    OFF, ON, HIGH, or LOW.  For driver status info/fault detection.               Option

#define AXIS4_DRIVER_DECAY            OFF //    OFF, Tracking decay mode default override. TMC default is STEALTHCHOP.        Infreq
#define AXIS4_DRIVER_DECAY_GOTO       OFF //    OFF, Decay mode goto default override. TMC default is SPREADCYCLE.            Infreq

#define AXIS4_POWER_DOWN              OFF //    OFF, ON Powers off 30 seconds after movement stops.                           Option

#define AXIS4_SENSE_HOME              OFF //    OFF, HIGH or LOW enables & state clockwise home position, as seen from above. Option
#define AXIS4_SENSE_LIMIT_MIN         OFF //    OFF, HIGH or LOW state on limit sense switch stops movement.                  Option
#define AXIS4_SENSE_LIMIT_MAX         OFF //    OFF, HIGH or LOW state on limit sense switch stops movement.                  Option

// AXIS5 FOCUSER 2 -----------------------------------------------------------------------------------------------------------------
// Up to 6 focusers can be present (AXIS4 to AXIS9) simply copy the above text for focuser 1 and rename to AXIS5_ for focuser 2, etc

// FOCUSER TEMPERATURE --------------------------------- see https://onstep.groups.io/g/developer/wiki/Configuration_Rotator_Focuser
#define FOCUSER_TEMPERATURE           OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n for focuser temp.            Adjust

// =================================================================================================================================
// AUXILIARY FEATURES ==============================================================================================================

// FEATURES ------------------------------------------------------------- see https://onstep.groups.io/g/main/wiki/Configuration_Aux
// Note: Temporarily set DEBUG mode to VERBOSE and use "FEATURE1_TEMP DS1820" to list the DS18B20 device serial numbers.

#define FEATURE1_PURPOSE              OFF //    OFF, SWITCH, MOMENTARY_SWITCH, ANALOG_OUT, DEW_HEATER, INTERVALOMETER.        Option
#define FEATURE1_NAME          "FEATURE1" // "FE..", Name of feature being controlled.                                        Adjust
#define FEATURE1_TEMP                 OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n. For DEW_HEATER temperature. Adjust
#define FEATURE1_PIN                  OFF //    OFF, AUX for auxiliary pin, n. Where n is the pin#.                           Adjust
#define FEATURE1_VALUE_DEFAULT        OFF //    OFF, ON, n. Where n=0..255 for ANALOG_OUT purpose.                            Adjust
#define FEATURE1_ON_STATE            HIGH //   HIGH, LOW to invert so "ON" is 0V and "OFF" is Vcc (3.3V usually.)             Adjust

#define FEATURE2_PURPOSE              OFF //    OFF, SWITCH, MOMENTARY_SWITCH, ANALOG_OUT, DEW_HEATER, INTERVALOMETER.        Option
#define FEATURE2_NAME          "FEATURE2" // "FE..", Name of feature being controlled.                                        Adjust
#define FEATURE2_TEMP                 OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n. For DEW_HEATER temperature. Adjust
#define FEATURE2_PIN                  OFF //    OFF, AUX for auxiliary pin, n. Where n is the pin#.                           Adjust
#define FEATURE2_VALUE_DEFAULT        OFF //    OFF, ON, n. Where n=0..255 for ANALOG_OUT purpose.                            Adjust
#define FEATURE2_ON_STATE            HIGH //   HIGH, LOW to invert so "ON" is 0V and "OFF" is Vcc (3.3V usually.)             Adjust

#define FEATURE3_PURPOSE              OFF //    OFF, SWITCH, MOMENTARY_SWITCH, ANALOG_OUT, DEW_HEATER, INTERVALOMETER.        Option
#define FEATURE3_NAME          "FEATURE3" // "FE..", Name of feature being controlled.                                        Adjust
#define FEATURE3_TEMP                 OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n. For DEW_HEATER temperature. Adjust
#define FEATURE3_PIN                  OFF //    OFF, AUX for auxiliary pin, n. Where n is the pin#.                           Adjust
#define FEATURE3_VALUE_DEFAULT        OFF //    OFF, ON, n. Where n=0..255 for ANALOG_OUT purpose.                            Adjust
#define FEATURE3_ON_STATE            HIGH //   HIGH, LOW to invert so "ON" is 0V and "OFF" is Vcc (3.3V usually.)             Adjust

#define FEATURE4_PURPOSE              OFF //    OFF, SWITCH, MOMENTARY_SWITCH, ANALOG_OUT, DEW_HEATER, INTERVALOMETER.        Option
#define FEATURE4_NAME          "FEATURE4" // "FE..", Name of feature being controlled.                                        Adjust
#define FEATURE4_TEMP                 OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n. For DEW_HEATER temperature. Adjust
#define FEATURE4_PIN                  OFF //    OFF, AUX for auxiliary pin, n. Where n is the pin#.                           Adjust
#define FEATURE4_VALUE_DEFAULT        OFF //    OFF, ON, n. Where n=0..255 for ANALOG_OUT purpose.                            Adjust
#define FEATURE4_ON_STATE            HIGH //   HIGH, LOW to invert so "ON" is 0V and "OFF" is Vcc (3.3V usually.)             Adjust

#define FEATURE5_PURPOSE              OFF //    OFF, SWITCH, MOMENTARY_SWITCH, ANALOG_OUT, DEW_HEATER, INTERVALOMETER.        Option
#define FEATURE5_NAME          "FEATURE5" // "FE..", Name of feature being controlled.                                        Adjust
#define FEATURE5_TEMP                 OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n. For DEW_HEATER temperature. Adjust
#define FEATURE5_PIN                  OFF //    OFF, AUX for auxiliary pin, n. Where n is the pin#.                           Adjust
#define FEATURE5_VALUE_DEFAULT        OFF //    OFF, ON, n. Where n=0..255 for ANALOG_OUT purpose.                            Adjust
#define FEATURE5_ON_STATE            HIGH //   HIGH, LOW to invert so "ON" is 0V and "OFF" is Vcc (3.3V usually.)             Adjust

#define FEATURE6_PURPOSE              OFF //    OFF, SWITCH, MOMENTARY_SWITCH, ANALOG_OUT, DEW_HEATER, INTERVALOMETER.        Option
#define FEATURE6_NAME          "FEATURE6" // "FE..", Name of feature being controlled.                                        Adjust
#define FEATURE6_TEMP                 OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n. For DEW_HEATER temperature. Adjust
#define FEATURE6_PIN                  OFF //    OFF, AUX for auxiliary pin, n. Where n is the pin#.                           Adjust
#define FEATURE6_VALUE_DEFAULT        OFF //    OFF, ON, n. Where n=0..255 for ANALOG_OUT purpose.                            Adjust
#define FEATURE6_ON_STATE            HIGH //   HIGH, LOW to invert so "ON" is 0V and "OFF" is Vcc (3.3V usually.)             Adjust

#define FEATURE7_PURPOSE              OFF //    OFF, SWITCH, MOMENTARY_SWITCH, ANALOG_OUT, DEW_HEATER, INTERVALOMETER.        Option
#define FEATURE7_NAME          "FEATURE7" // "FE..", Name of feature being controlled.                                        Adjust
#define FEATURE7_TEMP                 OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n. For DEW_HEATER temperature. Adjust
#define FEATURE7_PIN                  OFF //    OFF, AUX for auxiliary pin, n. Where n is the pin#.                           Adjust
#define FEATURE7_VALUE_DEFAULT        OFF //    OFF, ON, n. Where n=0..255 for ANALOG_OUT purpose.                            Adjust
#define FEATURE7_ON_STATE            HIGH //   HIGH, LOW to invert so "ON" is 0V and "OFF" is Vcc (3.3V usually.)             Adjust

#define FEATURE8_PURPOSE              OFF //    OFF, SWITCH, MOMENTARY_SWITCH, ANALOG_OUT, DEW_HEATER, INTERVALOMETER.        Option
#define FEATURE8_NAME          "FEATURE8" // "FE..", Name of feature being controlled.                                        Adjust
#define FEATURE8_TEMP                 OFF //    OFF, THERMISTOR or n. Where n is the ds18b20 s/n. For DEW_HEATER temperature. Adjust
#define FEATURE8_PIN                  OFF //    OFF, AUX for auxiliary pin, n. Where n is the pin#.                           Adjust
#define FEATURE8_VALUE_DEFAULT        OFF //    OFF, ON, n. Where n=0..255 for ANALOG_OUT purpose.                            Adjust
#define FEATURE8_ON_STATE            HIGH //   HIGH, LOW to invert so "ON" is 0V and "OFF" is Vcc (3.3V usually.)             Adjust

// ---------------------------------------------------------------------------------------------------------------------------------
#define FileVersionConfig 5
#include "Extended.config.h"
