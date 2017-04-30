#include "Joint.h"
Joint::Joint(int servoNum, int minPos, int maxPos, int sanePos) {
  m_minPos = minPos;
  m_maxPos = maxPos;
  m_sanePos = sanePos;
  m_servoNum = servoNum;
  m_currPos = sanePos;
  m_pwmDriver = NULL;
  m_tweenDelayMillis = SANE_TWEEN_DELAY_MILLIS;
}

Joint::~Joint() {

}

void Joint::setTweenDelayMillis(int delayMillis) {
  m_tweenDelayMillis = constrain(0, delayMillis, MAX_TWEEN_DELAY_MILLIS);
}

int Joint::setPos(int targetPos) {
  // set a joint position  (constrained between minimum and maximum position)
  m_currPos = constrain(targetPos, m_minPos, m_maxPos);
  if (m_pwmDriver != NULL) {
    m_pwmDriver->setPWM(m_servoNum, 0, m_currPos);
  }
  return m_currPos;
}

int Joint::tweenPos(int targetPos) {
  int constrainedTargetPos = constrain(targetPos, m_minPos, m_maxPos);

  while (m_currPos != constrainedTargetPos) {
    if (m_currPos < constrainedTargetPos) {
      incrementPos();
    } else {
      decrementPos();
    }
    delay(m_tweenDelayMillis);
  }

  setPos(constrainedTargetPos);   // make sure we set arm to target position if initial position is unknown
  return m_currPos;
}

int Joint::getPos(void) {
  // return current joint position
  return m_currPos;
}

int Joint::incrementPos() {
  // increment joint position, constrained between minimum and maximum position)
  m_currPos = constrain(m_currPos + 1, m_minPos, m_maxPos);
  if (m_pwmDriver != NULL) {
    m_pwmDriver->setPWM(m_servoNum, 0, m_currPos);
  }
  return m_currPos;
}

int Joint::decrementPos() {
  // decrement joint position, constrained between minimum and maximum position
  m_currPos = constrain(m_currPos - 1, m_minPos, m_maxPos);
  if (m_pwmDriver != NULL) {
    m_pwmDriver->setPWM(m_servoNum, 0, m_currPos);
  }
  return m_currPos;
}

void Joint::setPwmDriver(Adafruit_PWMServoDriver *pwmDriver) {
  // sets pointer to PWM board driver object
  m_pwmDriver = pwmDriver;
}

