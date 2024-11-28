#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <BluetoothSerial.h>

#define BTN_PIN 22
#define AUX_BTN_PIN 39
#define LED_PIN 27
#define SCL_PIN 21
#define SDA_PIN 25

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

const int baudRate = 9600;

LSM303 compass;
L3G gyro;

BluetoothSerial SerialBT;

void setupIMU();

int checkButton();
int checkAuxButton();
void clickEvent();
void doubleClickEvent();
void holdEvent();

int checkIMU();
int processIMUData(float gx, float gy, float gz, float ax, float ay, float az);
int detectUpStopPattern();


void setup()
{
  pinMode(BTN_PIN, INPUT_PULLDOWN);
  pinMode(AUX_BTN_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Serial setup
  Serial.begin(baudRate);

  // Blutooth setup
  SerialBT.begin("MagicRing");

  // IMU setup
  setupIMU();
}

void loop()
{
  pixels.clear();
  pixels.show();

  int event = checkButton();
  switch (event)
  {
  case 1:
    clickEvent();
    break;
  case 2:
    doubleClickEvent();
    break;
  case 3:
    holdEvent();
    break;
  default:
    break;
  }

  // i
  


  // IMU data - for debugging
  // gyro.read();
  // compass.read();
  // SerialBT.println("Gyro: " + String(gyro.g.x) + "," + String(gyro.g.y) + "," + String(gyro.g.z) + " | Accel: " + String(compass.a.x) + "," + String(compass.a.y) + "," + String(compass.a.z)  + " | Mag: " + String(compass.m.x) + "," + String(compass.m.y) + "," + String(compass.m.z));
  // delay(300);
}

void setupIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);

  compass.init();
  compass.enableDefault();

  gyro.init();
  gyro.enableDefault();
}

void clickEvent()
{
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();

  // Serial.println("Toggle-Volume");
  SerialBT.println("Toggle-Volume");
}
void doubleClickEvent()
{
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();

  // Serial.println("Toggle-Microphone");
  SerialBT.println("Toggle-Microphone");
}
void holdEvent()
{
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();

  int action = checkIMU();
  if (action == 1) {
    // Serial.println("Increase-Volume");
    SerialBT.println("Increase-Volume");
  }
  if (action == -1) {
    // Serial.println("Decrease-Volume");
    SerialBT.println("Decrease-Volume");
  }

  // int action = detectUpStopPattern();
  // if (action > 0) {
  //   Serial.println("Set-Volume " + String(action * 20));
  //   SerialBT.println("Set-Volume " + String(action * 20));
  // }

}

int threshold_up_and_z_accel = 1000;
int threshold_stop = 500;
int min_stop_duration = 50;
int end_stop_duration = 1000;
int max_num_ups = 5;
bool waitingForStop = false;
int numUps = 0;
unsigned long lastUpTime = 0;
unsigned long stopStartTime = 0;

int detectUpStopPattern() {
  gyro.read();
  unsigned long currentTime = millis();

  if (!waitingForStop) {
    if (gyro.g.z > threshold_up_and_z_accel) {
      waitingForStop = true;
      if (++numUps > max_num_ups) {
        numUps = 0;
        return max_num_ups;
      }
      // Serial.println("Up: " + String(numUps));
      lastUpTime = currentTime;
    }
  } else {
    if (abs(gyro.g.z) < threshold_stop) {
      if (stopStartTime == 0) {
        stopStartTime = millis(); // Start stop timer
      }

      // Check if it has been stable long enough
      if (currentTime - stopStartTime > min_stop_duration) {
        waitingForStop = false;
        stopStartTime = 0; // Reset timer
        // Serial.println("Stop");
      }
    } else {
      // If not stable, reset stop timer
      stopStartTime = 0;
    }
  }
  if (currentTime - lastUpTime > end_stop_duration) {
    lastUpTime = currentTime;
    if (numUps > 0) {
      int result = numUps;
      numUps = 0;
      return result;
    }
  }
  return 0;
}


int checkIMU() {
  gyro.read();
  compass.read();
  // SerialBT.println("IMU " + String(gyro.g.x) + "," + String(gyro.g.y) + "," + String(gyro.g.z));

  return processIMUData(
        (float)gyro.g.x, (float)gyro.g.y, (float)gyro.g.z,
        (float)compass.a.x, (float)compass.a.y, (float)compass.a.z
    );
}

class SimpleCircularDetector {
private:
    // Thresholds
    const int32_t Y_THRESHOLD = 5000;  // Threshold for Y movement detection
    const unsigned long DEBOUNCE_TIME = 500; // Time in ms before allowing new detection
    
    // State tracking
    bool isTracking = false;
    unsigned long lastDetectionTime = 0;
    int lastDirection = 0;

public:
    SimpleCircularDetector() {}
    
    void reset() {
        isTracking = false;
        lastDirection = 0;
    }
    
    int detect(int16_t gx, int16_t gy, int16_t gz) {
        unsigned long currentTime = millis();
        
        // Debounce check
        if (currentTime - lastDetectionTime < DEBOUNCE_TIME) {
            return 0;
        }
        
        // Check Y axis movement
        if (abs(gy) > Y_THRESHOLD) {
            lastDetectionTime = currentTime;

            // gy > 0 is clockwise, gy < 0 is counter-clockwise
            return (gy > 0) ? 1 : -1;
        }
        
        return 0;
    }
};


// Global detector instance
SimpleCircularDetector circularDetector;

int processIMUData(float gx, float gy, float gz, float ax, float ay, float az) {
    // Detect circular motion
    int direction = circularDetector.detect(gyro.g.x, gyro.g.y, gyro.g.z);

    return direction;
}


int checkAuxButton() {
  // returns clicks the bytton add in an inteval of 3 seconds
  int clicks = 0;
  // unsigned long startTime = millis();
  // if (digitalRead(AUX_BTN_PIN) == HIGH) {
  //   while (millis() - startTime < 3000) {
  //     if (digitalRead(AUX_BTN_PIN) == HIGH) {
  //       clicks++;
  //     }
  //   }
  // }
  return clicks;
}





//=============================================
//  MULTI-CLICK:  One Button, Multiple Events
//=============================================

// Button timing variables
int debounce = 20;       // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250;         // max ms between clicks for a double click event
int holdTime = 700;      // ms hold period: how long to wait for press+hold event
int longHoldTime = 3000; // ms long hold period: how long to wait for press+hold event

// Button variables
boolean buttonVal = HIGH;          // value read from button
boolean buttonLast = HIGH;         // buffered value of the button's previous state
boolean DCwaiting = false;         // whether we're waiting for a double click (down)
boolean DConUp = false;            // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;           // whether it's OK to do a single click
long downTime = -1;                // time the button was pressed down
long upTime = -1;                  // time the button was released
boolean ignoreUp = false;          // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;         // when held, whether to wait for the up event
boolean holdEventPast = false;     // whether or not the hold event happened already
boolean longHoldEventPast = false; // whether or not the long hold event happened already

int checkButton()
{
  int event = 0;
  buttonVal = HIGH - digitalRead(BTN_PIN);
  // Button pressed down
  if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce)
  {
    downTime = millis();
    ignoreUp = false;
    waitForUp = false;
    singleOK = true;
    holdEventPast = false;
    longHoldEventPast = false;
    if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true)
      DConUp = true;
    else
      DConUp = false;
    DCwaiting = false;
  }
  // Button released
  else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce)
  {
    if (not ignoreUp)
    {
      upTime = millis();
      if (DConUp == false)
        DCwaiting = true;
      else
      {
        event = 2;
        DConUp = false;
        DCwaiting = false;
        singleOK = false;
      }
    }
    holdEventPast = false;
  }
  // Test for normal click event: DCgap expired
  if (buttonVal == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
  {
    event = 1;
    DCwaiting = false;
  }
  // Test for hold
  if (buttonVal == LOW && (millis() - downTime) >= holdTime)
  {
    // Trigger "normal" hold

    event = 3;
    waitForUp = true;
    ignoreUp = true;
    DConUp = false;
    DCwaiting = false;
    // downTime = millis();
    holdEventPast = true;
  }
  buttonLast = buttonVal;
  return event;
}

/* 4-Way Button:  Click, Double-Click, Press+Hold, and Press+Long-Hold Test Sketch

By Jeff Saltzman
Oct. 13, 2009

To keep a physical interface as simple as possible, this sketch demonstrates generating four output events from a single push-button.
1) Click:  rapid press and release
2) Double-Click:  two clicks in quick succession
3) Press and Hold:  holding the button down
4) Long Press and Hold:  holding the button for a long time
*/