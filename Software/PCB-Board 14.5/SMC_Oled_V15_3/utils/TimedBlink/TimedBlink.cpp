#include <Arduino.h>
#include "TimedBlink.h"

//############################################################################
//# HISTORY  TimedBlink.cpp
//#
//# 20.01.2023 cebra
//# - add: additional parameter to switch between PIN aktive low or high
//# - add: additional to blink, switch LED on or off
//# - chg:
//# - fix:
//# - del:
//############################################################################



/*
 * Resets all timers and state
 */
void TimedBlink::reset() {
  m_blinkTime   = 0UL;
  m_onForTime   = -1;
  m_offForTime  = -1;
  m_blinkState  = BLINK_OFF;
  m_resolution  = 100;
}

/*
 * Constructor. Only needs to know what pin to blink and set function to aktiv Low  = true, else aktiv High
 */
TimedBlink::TimedBlink(int pin, bool activlow) {
  m_pin = pin;
  m_pinAktivLow = activlow;
  reset();
}



/*
 * Turns LED on .
 */
void TimedBlink::On() {

	m_blinkState = ON;
  if (m_pinAktivLow == true)
  digitalWrite(m_pin, LOW);
  else digitalWrite(m_pin, HIGH);
}

/*
 * Turns LED on .
 */
void TimedBlink::Off() {
  reset();
  if (m_pinAktivLow == true)
  digitalWrite(m_pin, HIGH);
  else digitalWrite(m_pin, LOW);
}


/*
 * Sets the blink time ON
 */
void TimedBlink::setOnTime(int ms) {
  if (ms>0) { // no=op if ms is <= 0
    m_onForTime = ms;
    if (m_offForTime>0) {
      m_resolution = min(m_onForTime,m_offForTime)/10;
    }
  }
}

/*
 * Sets the blink time OFF
 */
void TimedBlink::setOffTime(int ms) {
  if (ms>0) { // no=op if ms is <= 0
    m_offForTime = ms;
    if (m_onForTime>0) {
      m_resolution = min(m_onForTime,m_offForTime);
    }
  }
}

/*
 * Sets the blink state ON or OFF
 */
void TimedBlink::setBlinkState(blink_t state)
{
  digitalWrite(m_pin, (state==BLINK_ON) ? HIGH : LOW);
  m_blinkState = state;
  m_blinkTime  = millis();
}

/*
 * Executes the blink. It allows to specify new on and off times. Use negative
 * values if you don't want to change what is already set.
 */

void TimedBlink::blink(int on_for, int off_for) {

  unsigned long ct = millis();

  if ((on_for!=-1) && (off_for != -1))							// in case LED is permanently on, switch off before blink
  {
	  if (m_blinkState==ON) setBlinkState(BLINK_OFF);			// If blinkstate already on, reset
  }
  if (m_blinkTime==0UL) m_blinkTime=ct;

  unsigned long diff = abs(ct - m_blinkTime);
  short set_to = -1;

  setOnTime(on_for);
  setOffTime(off_for);

  if (m_blinkState==BLINK_OFF || (m_blinkState==BLINK_ON))
  {
  if (m_blinkState==BLINK_OFF) {
    if (m_offForTime>0 && diff>m_offForTime) {
      setBlinkState(BLINK_ON);
    }
  } else {
    if (m_onForTime>0 && diff>m_onForTime) {
      setBlinkState(BLINK_OFF);
    }
  }
}

  else
  {

	  if (m_blinkState==OFF) Off();
	  if (m_blinkState==ON) On();

  }
}



/*
 * Call often to blink.
 */
void TimedBlink::blink() {
  blink(-1,-1);
}

/*
 * Equivalent to delay(d), but updates the blink.
 */
void TimedBlink::blinkDelay(int d)
{
  unsigned long ct = millis();
  while (millis()-ct<d) {
    blink();
    delay(m_resolution);
  }
}

/*
 * Turns off the blink.
 */
void TimedBlink::blinkOff() {
  reset();
  if (m_pinAktivLow == true)
  digitalWrite(m_pin, HIGH);
  else digitalWrite(m_pin, LOW);
}
