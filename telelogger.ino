/******************************************************************************
* Arduino sketch of a vehicle data data logger and telemeter for Freematics Hub
* Works with Freematics ONE+ Model A and Model B
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
* Visit https://hub.freematics.com to view live and history telemetry data
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <FreematicsPlus.h>
// #include <httpd.h>
#include "config.h"
// #include "telestore.h"
// #include "teleclient.h"
// #include "telemesh.h"
#if BOARD_HAS_PSRAM
#include "esp32/himem.h"
#endif
#include "driver/adc.h"
#include "driver/ledc.h"

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_PWM_SIGNAL_UP 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CELL_CONNECTED 0x20
#define STATE_WIFI_CONNECTED 0x40
#define STATE_WORKING 0x80
#define STATE_STANDBY 0x100

typedef struct {
  byte pid;
  byte tier;
  int value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
};

#if ENABLE_MEMS
float accBias[3] = {0}; // calibrated reference accelerometer data
float accSum[3] = {0};
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
uint8_t accCount = 0;
#endif
int deviceTemp = 0;

// live data
uint16_t dtc[6] = {0};
GPS_DATA* gd = 0;

char isoTime[32] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;

int32_t dataInterval = 1000;

uint32_t lastCmdToken = 0;
String serialCommand;

byte ledMode = 0;

MEMS_I2C* mems = 0;

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};


class OBD : public COBD
{
protected:
  void idleTasks()
  {
    // do some quick tasks while waiting for OBD response
    delay(5);
  }
};

void printTimeoutStats()
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}


FreematicsESP32 sys;
State state;
OBD obd;
ledc_channel_config_t ledc_channel={};
ledc_timer_config_t ledc_timer={};

/*******************************************************************************
  Reading and processing OBD data
*******************************************************************************/
int readSpeed()
{ 
  int kph;
  byte pid = PID_SPEED;
  # if !DEBUG_MODE
    obd.readPID(pid, kph);
  # else
    // every 10 seconds, increase the speed by 10kph
    // using millis() to measure
    kph = (millis() / 10000) * 10;
    // modulo to get to 0-100
    kph = kph % 100;
  # endif

  if ((kph >= 2) & (kph <= MAX_SPEED_KMH)) lastMotionTime = millis();

  return kph;
}

void calibrateMEMS()
{
  if (state.check(STATE_MEMS_READY)) {
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
      float acc[3];
      if (!mems->read(acc)) continue;
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC BIAS:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
  }
}

void printTime()
{
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {
    // valid system time available
    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    Serial.print("UTC:");
    Serial.println(buf);
  }
}

uint32_t calculateFreq(int kph) {
  // calculate frequency using kph and MAX_FREQUENCY_KMH
  // 1. calculate ratio of kph to MAX_FREQUENCY_KMH
  // 2. multiply ratio by MAX_FREQUENCY_HZ
  // 3. set frequency to result

  float ratio = (float)kph / (float)MAX_SPEED_KMH;
  if(ratio > 1.0) {
    ratio = 1.0;
  }
  uint32_t freq = (uint32_t)(ratio * MAX_FREQUENCY);

  Serial.print("Speed: ");
  Serial.print(kph);
  Serial.print(" Ratio: ");
  Serial.print(ratio);
  Serial.print("Calculated frequency: ");
  Serial.println(freq);

  return freq;
}

void setupPwmSignal(int kph) {
  uint32_t freq = calculateFreq(kph);

  if(!state.check(STATE_PWM_SIGNAL_UP)) do {
    Serial.println("PWM not up; configuring PWM");
    ledc_timer.speed_mode       = LEDC_HIGH_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz          = freq;  // set output frequency at 1 Hz
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    // ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.timer_sel  = LEDC_TIMER_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = GPIO_NUM_26;
    ledc_channel.duty       = 500; // set duty at about 10%
    // set hpoint above 0
    ledc_channel.hpoint     = 1000; 
    ledc_channel_config(&ledc_channel);

    state.set(STATE_PWM_SIGNAL_UP);
    Serial.println("PWM now up");
  } while(!state.check(STATE_PWM_SIGNAL_UP));
}

void updatePwmSignal(int kph) {
  if (!state.check(STATE_PWM_SIGNAL_UP)) {
    Serial.println("Attempted to update PWM signal but not up..");
    setupPwmSignal(kph);
    return;
  }
  uint32_t freq = calculateFreq(kph);
  Serial.print("Updating PWM signal with frequency:");
  Serial.println(freq);
  ledc_set_freq(ledc_channel.speed_mode, ledc_channel.timer_sel, freq);
  Serial.println("Updated PWM signal");
}

void cancelPwmSignal() {
  if(state.check(STATE_PWM_SIGNAL_UP)) {
    Serial.println("Stopping PWM via LEDc"); 
    ledc_stop(ledc_channel.speed_mode, ledc_channel.channel, 0);
    state.clear(STATE_PWM_SIGNAL_UP);
    Serial.println("Stopped PWM via LEDc"); 
  }
}

void setVssPwmSpeed(int kph) {
  if(kph < 3) {
    // bring down PWM signal as we're not moving
      cancelPwmSignal();
  } else {
    updatePwmSignal(kph);
  }
 
}

/*******************************************************************************
  Initializing all data logging components
*******************************************************************************/
void initialize()
{
    // turn on buzzer at 2000Hz frequency 
  // sys.buzzer(2000);
  // delay(100);
  // // turn off buzzer
  // sys.buzzer(0);

  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }

  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    if (obd.init()) {
      Serial.println("OBD:OK");
      state.set(STATE_OBD_READY);
    } else {
      Serial.println("OBD:NO");
      // if not DEBUG_MODE, go into standby
      # if !DEBUG_MODE
        state.clear(STATE_OBD_READY);
        state.clear(STATE_WORKING);
        return;
      # else 
        Serial.println("In DEBUG_MODE, so continuing despite no ODB connection");
        state.set(STATE_OBD_READY);
      # endif      
      
    }
  }

  // check system time
  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);
}

bool waitMotion(long timeout)
{
  unsigned long t = millis();
  if (state.check(STATE_MEMS_READY)) {
    do {
      // calculate relative movement
      float motion = 0;
      float acc[3];
      if (!mems->read(acc)) continue;
      if (accCount == 10) {
        accCount = 0;
        accSum[0] = 0;
        accSum[1] = 0;
        accSum[2] = 0;
      }
      accSum[0] += acc[0];
      accSum[1] += acc[1];
      accSum[2] += acc[2];
      accCount++;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
  
      // check movement
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        //lastMotionTime = millis();
        Serial.println(motion);
        return true;
      }
    } while ((long)(millis() - t) < timeout || timeout == -1);
    return false;
  }
  return false;
}

/*******************************************************************************
  Collecting and processing data
*******************************************************************************/
void process()
{
  uint32_t startTime = millis();

  // process OBD data, only if connected
  if (state.check(STATE_OBD_READY)) {
    int kph;
    kph = readSpeed();
    if(kph > MAX_SPEED_KMH) {
      state.clear(STATE_WORKING);
      return;
    }
    setVssPwmSpeed(kph);
  } else if (obd.init(PROTO_AUTO, true)) {
    state.set(STATE_OBD_READY);
    Serial.println("[OBD] ECU ON");
  }

  // motion adaptive data interval control
  const int dataIntervals[] = DATA_INTERVAL_TABLE;
  const uint16_t stationaryTime[] = STATIONARY_TIME_TABLE;
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool stationary = true;
  for (byte i = 0; i < sizeof(stationaryTime) / sizeof(stationaryTime[0]); i++) {
    dataInterval = dataIntervals[i];
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      stationary = false;
      break;
    }
  }
  if (stationary) {
    // stationery timeout
    Serial.print("Stationary for ");
    Serial.print(motionless);
    Serial.println(" secs");
    // trip ended, go into standby
    state.clear(STATE_WORKING);
    return;
  }
  do {
    long t = dataInterval - (millis() - startTime);
  } while (millis() - startTime < dataInterval);
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
  state.set(STATE_STANDBY);
  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY | STATE_PWM_SIGNAL_UP) ;
  // this will put co-processor into sleep mode
  
  Serial.println("Going into STANDBY...");
  cancelPwmSignal();

  Serial.println("Entering ODB low power mode"); 
  # if !DEBUG_MODE
    obd.enterLowPowerMode();
  # endif

  calibrateMEMS();
  waitMotion(-1);

  Serial.println("Wake up event, waking up and resetting...");

  sys.resetLink();
#if RESET_AFTER_WAKEUP
  mems->end();  
  ESP.restart();
#endif  
  state.clear(STATE_STANDBY);
}

void setup()
{
  delay(500);

  // initialize USB serial
  Serial.begin(115200);

  if (sys.begin()) {
    Serial.print("TYPE:");
    Serial.println(sys.devType);
    # if !DEBUG_MODE
      obd.begin(sys.link);
    # endif
  }

if (!state.check(STATE_MEMS_READY)) do {
  Serial.print("MEMS:");
  mems = new ICM_20948_I2C;
  byte ret = mems->begin();
  if (ret) {
    state.set(STATE_MEMS_READY);
    Serial.println("ICM-20948");
    break;
  }
} while (0);
  state.set(STATE_WORKING);
  initialize();
}

void loop()
{
  // error handling
  if (!state.check(STATE_WORKING)) {
    standby();
    initialize();
    return;
  }

  // collect and log data
  process();
}
