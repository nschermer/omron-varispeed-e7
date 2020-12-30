#include <Wire.h>

#include "memobus.h"

#include <LiquidCrystal_I2C.h>
#include <Button2.h>   // https://github.com/LennartHennigs/Button2
#include <ESPRotary.h> // https://github.com/LennartHennigs/ESPRotary

/* VFD settings */
#define VFD_HZ_TO_RPM(hz)   ((hz) * 30) /* (Hz x 60 x 2) / number of poles */
#define VFD_MIN_HZ          10
#define VFD_MAX_HZ          100
#define VFD_HZ_UNITS(hz)    (constrain((hz), VFD_MIN_HZ, VFD_MAX_HZ) * 100) /* o1-03 = 0 */

/* Rotory encoder */
#define ROTARY_PIN_CLK  5
#define ROTARY_PIN_DT   6
#define ROTARY_PIN_SW   4
#define ROTARY_CPS      4 /* clicks per step */

#define UNUSED __attribute__((unused))

#define INT_NTH_BIT(i,n) (((i) >> (n)) & 0b1)

ESPRotary         encoder(ROTARY_PIN_CLK, ROTARY_PIN_DT, ROTARY_CPS);
Button2           button(ROTARY_PIN_SW);
LiquidCrystal_I2C lcd(0x27, 20, 4);

bool     menu_update = true;
int      mode = 0;
int      ref_frequency = 0;
int      tapping_rev_perc = 150;
int      error_linenr = 0;
uint16_t last_inverter_status = 0;
int      fail_counter = 8; /* no fail first run */

/* update timers */
#if MEMOBUS_LOOPBACK
static unsigned long loopbackMillis = 0;
#endif
unsigned long potmeterMillis = 0;
unsigned long statusFastMillis = 0;
unsigned long statusSlowMillis = 0;

void setup() {
  /* usb serial */
  Serial.begin(9600);

  /* serial communication RS485 module */
  memobus_setup();

  /* rotary encoder handling */
  button.setTapHandler(button_click);
  encoder.setLeftRotationHandler(encoder_rotate);
  encoder.setRightRotationHandler(encoder_rotate);

  lcd.init();
  lcd.backlight();
}

void loop () {
  int n;
  unsigned long currentMillis = millis();
  char buf[16] = { 0 };
  uint16_t status[8] = { 0 };

  button.loop();
  encoder.loop();

  if (menu_update) {
    lcd.clear();
    lcd.setCursor(0, 0);

    switch (mode) {
      case 0:
        lcd.print(F("Mode: Decelerate    "));
        break;

      case 1:
        lcd.print(F("Mode: Brake         "));
        break;

      case 2:
        lcd.print(F("Mode: Tapping       "));
        print_tapping ();
        break;
    }

    /* Set stopping mode b1-03 and send enter command */
    if (!memobus_write_register(0x0182, mode == 0 ? 0 : 1)) {
      error_handler(__LINE__);
    } else if (!memobus_write_register(0x910, 0)) {
      error_handler(__LINE__);
    }

    lcd.setCursor(0, 1);
    lcd.print(F("Ref:      rpm"));

    lcd.setCursor(6, 1);
    lcd.print(VFD_HZ_TO_RPM(ref_frequency));

    lcd.setCursor(0, 2);
    lcd.print(F("VFD:  Not connected"));

    menu_update = false;
    last_inverter_status = 0;
  }

  /* analog value to 0 - 100% */
  n = map (analogRead(A0), 0, 1023, 10, 100);
  if (ref_frequency != n) {
    /* avoid excessive screen updates */
    if (potmeterMillis == 0)
      potmeterMillis = currentMillis;
  }

  if (potmeterMillis != 0
      && currentMillis - potmeterMillis >= 100) {
    if (ref_frequency != n) {
      /* update value */
      ref_frequency = n;

      /* update setpoint on VFD */
      if (!memobus_write_register(0x0002, VFD_HZ_UNITS(ref_frequency))) {
        error_handler(__LINE__);
      }

      lcd.setCursor(6, 1);
      lcd.print(VFD_HZ_TO_RPM(ref_frequency));
      if (ref_frequency < (1000 / VFD_HZ_TO_RPM(1)))
        lcd.print(" ");
    }

    /* stop timeout */
    potmeterMillis = 0;
  }

#if MEMOBUS_LOOPBACK
  /* loopcheck after 80% of CE time expired */
  if (currentMillis - loopbackMillis >= MEMOBUS_CE_TIME * 0.8) {
    if (!memobus_loopback_test()) {
      error_handler(__LINE__);
    }
    loopbackMillis = currentMillis;
  }
#endif

  if (currentMillis - statusFastMillis >= 250) {
    /* read status */
    if (memobus_read_register (0x0020, sizeof(status), status)) {

      if (last_inverter_status != status[0]) {
        /* 0x0020 inverter status */
        lcd.setCursor(6, 2);
        if (INT_NTH_BIT (status[0], 3)) {
          lcd.print(F("Fault   "));
        } else if (INT_NTH_BIT (status[0], 0)) {
          lcd.print(INT_NTH_BIT (status[0], 1) ? F("Reverse ") : F("Forward "));
        } else if (INT_NTH_BIT (status[0], 2)) {
          lcd.print(F("Ready   "));
        } else {
          lcd.print(F("Unknown "));
        }

        /* tapping mode */
        if (mode == 2) {
          /* forward to reverse speed */
          if (INT_NTH_BIT (status[0], 1)) {
            n = ref_frequency * tapping_rev_perc / 100;
          } else {
            n = ref_frequency;
          }

          /* update setpoint on VFD */
          if (!memobus_write_register(0x0002, VFD_HZ_UNITS(n))) {
            error_handler(__LINE__);
          }
        }

        last_inverter_status = status[0];
      }

      /* Output frequency */
      lcd.setCursor(14, 2);
      snprintf(buf, sizeof(buf), "%3dHz", min (status[4] / 100, 999));
      lcd.print(buf);

      /* Output power */
      lcd.setCursor(6, 3);
      lcd.print((double) min (status[7], 99) / 10, 1);
      lcd.print(F("kW"));

      /* reset afil counter */
      fail_counter = 0;
    } else {
      /* count fails */
      fail_counter++;

      if (fail_counter == 8) {
        /* force menu update (clear) */
        menu_update = true;
      }
    }

    statusFastMillis = currentMillis;

  } else if (currentMillis - statusSlowMillis >= 1000) {
    /* read status */
    if (memobus_read_register (0x0031, 1, status)) {
      lcd.setCursor(14, 3);
      snprintf(buf, sizeof(buf), "%3dVDC", min(status[0], 999));
      lcd.print(buf);
    }

    /* debug helper */
    lcd.setCursor(0, 3);
    if (error_linenr != 0)
      lcd.print(error_linenr);
    else
      lcd.print(F("   "));
    error_linenr = 0;

    statusSlowMillis = currentMillis;
  }
}

void error_handler(int line) {
  error_linenr = line;

  if (Serial) {
    Serial.print(F("Error line "));
    Serial.println(line);
  }
}

void print_tapping () {
  char buf[8] = { 0 };

  lcd.setCursor(15, 0);
  snprintf (buf, sizeof(buf), "%3d%%", min (tapping_rev_perc, 999));
  lcd.print(buf);
}

void encoder_rotate(UNUSED ESPRotary & r) {
  if (mode == 2) {
    tapping_rev_perc += encoder.getDirection() == RE_LEFT ? -10 : 10;

    if (tapping_rev_perc < 50)
      tapping_rev_perc = 50;
    if (tapping_rev_perc > 500)
      tapping_rev_perc = 500;

    print_tapping ();
  }
}

void button_click(UNUSED Button2 & b) {
  mode++;

  if (mode > 2)
    mode = 0;

  menu_update = true;
}
