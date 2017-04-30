#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define MAX_TWEEN_DELAY_MILLIS 1000
#define SANE_TWEEN_DELAY_MILLIS 2
// called this way, it uses the default address 0x40

class Joint {
  public:
    Joint(int servoNum, int minPos, int maxPos, int sanePos);
    virtual ~Joint() ;

    int tweenPos(int targetPos);
    int setPos(int targetPos);
    int getPos();

    int incrementPos();
    int decrementPos();
    
    void setTweenDelayMillis(int delayMillis);

    void setPwmDriver(Adafruit_PWMServoDriver *pwmDriver);
    
  private:
    int m_minPos;
    int m_maxPos;
    int m_sanePos;
    int m_servoNum;
    int m_currPos;
    int m_tweenDelayMillis;
    Adafruit_PWMServoDriver *m_pwmDriver;
};
