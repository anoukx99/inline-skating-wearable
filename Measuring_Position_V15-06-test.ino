// Code to measure the delta between two inertial measurement units
// And to send the data to a mobile phone using Bluetooth.
// Adapted by Anouk de Graaf

#include<Wire.h>

#include "esp32-hal-ledc.h"

#define TIMER_WIDTH 10


#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

const int MPU_addr = 0x68;               // I2C address of the MPU-6050
const int numberOfSamples = 20;         // the accelerometer values are an average of a number of samples

const int AD00 = 21;                     // digital output to control 2 accelerometers
const int AD01 = 17;                     // dito, set them alternating to address 0x68
const int AD02 = 18;
const int AD03 = 19;
const int AD04 = 5;
const int AD05 = 4;

const int sampleFrequency = 60;          // how many accelerometer samples per second in working mode

const int durationCalibration = 10000;   // period that the calibration phase takes

int timerCalibration = 10000;            // timer for calibration
int timerPosture = 0;                    // to measure time between poor posture detection and feedback
int sensorCounter = 0;

int32_t angle90_l = 0;
int32_t angle180_l = 0;
int32_t angle90_r = 0;
int32_t angle180_r = 0;
int32_t desiredAngle_l = 0;
int32_t desiredAngle_r = 0;

int32_t minPosture = 0;                  // minimal difference in the position for the accelerometers during calibration
int32_t maxPosture = 0;                  // maximal difference in the position for the accelerometers during calibration
int32_t threshold = 0;

int32_t angle_l;
int32_t angle_r;

char supportLeg_new;
char supportLeg_old;

bool cal_1 = false;
bool paused = false;
bool cal_2 = false;

int pushOffTimer_l = 0;
int pushOffTimer_r = 0;
int setdownTimer_l = 0;
int setdownTimer_r = 0;

struct Delta {
  int32_t l;
  int32_t r;
};

struct Angle {
  int32_t l;
  int32_t r;
};

Angle angle_new {
  0,
  0,
};

Angle angle_old {
  0,
  0,
};

// the main part of the program is organized as a state machine with the states defined here
enum class ProgramState {
  Initialization,
  Calibration,
  Working,
  Feedback,
};

void sleep();

ProgramState phase = ProgramState::Initialization;   // code is organized as a state machine
void setup() {

  digitalWrite(SDA, HIGH);                // activate internal pullup resistor for I2C
  digitalWrite(SCL, HIGH);                // activate internal pullup resistor for I2C

  pinMode(AD00, OUTPUT);                  // these pins determine to which address the accelerometers write
  pinMode(AD01, OUTPUT);                  // they are alternating put to HIGH/LOW to write to the same address
  pinMode(AD02, OUTPUT);                  // these pins determine to which address the accelerometers write
  pinMode(AD03, OUTPUT);                  // they are alternating put to HIGH/LOW to write to the same
  pinMode(AD04, OUTPUT);
  pinMode(AD05, OUTPUT);

  Wire.begin();
  for (int i = 0; i < 4; i++) {
    initMpu(i);                           // waking up the accelerometers
  }

  Serial.begin(9600);
  SerialBT.begin("ESP32"); //Bluetooth device name

  timerCalibration = millis();            // initialize timer for calibration phase
}

void loop() {
  int N;

  // this first part is a while loop that triggers while touch is detected and decides whether it
  // was a long touch or a short touch, changing the booleans shortTouchDetected or longTouchDetected

  switch (phase) {
    case ProgramState::Initialization: {
        Serial.println("phase 0: initialize ");

        // initialize variables for calibration
        angle90_l = angle180_l = 0;
        angle90_r = angle180_r = 0;

        phase = ProgramState::Calibration;
        Serial.print("phase 1: calibration: ");
        Serial.println("We gaan calibreren. Maak een hoek van 90 graden. Klaar voor? Typ: start_1");
        break;
      }

    case ProgramState::Calibration: {
        //Calibration 90 degree angle
        int N = 0;
        while (millis() - timerCalibration < durationCalibration) {
          struct Delta d;
          d = getDelta(60);
          angle90_l = angle90_l + d.l;
          angle90_r = angle90_r + d.r;

          N = N + 1;
        }

        angle90_l = angle90_l / N;
        angle90_r = angle90_r / N;

        Serial.print("Average delta_r: "); Serial.print(angle90_l);
        Serial.print(" Average delta_l: "); Serial.println(angle90_r);
        timerCalibration = millis();

        //Break between calibration phases
        while (millis() - timerCalibration < 5000) {
          int tijdOver = 5000 - (millis() - timerCalibration);
          Serial.print("De eerste calibratie is klaar! Maak nu een hoek van 180 graden. We gaan verder over zoveel seconden:"); Serial.println(tijdOver);
        }

        timerCalibration = millis();
        N = 0;

        //Calibration for angle of 180 degrees
        while (millis() - timerCalibration < durationCalibration) {
          struct Delta d;
          d = getDelta(60);
          angle180_l = angle180_l + d.l;
          angle180_r = angle180_r + d.r;
          N = N + 1;
        }

        angle180_l = angle180_l / N;
        angle180_r = angle180_r / N;
        Serial.print("Average delta_r: "); Serial.print(angle180_l);
        Serial.print(" Average delta_l: "); Serial.println(angle180_r);


        timerCalibration = millis();

        //Break between calibration phases
        while (millis() - timerCalibration < 5000) {
          int tijdOver = 5000 - (millis() - timerCalibration);
          Serial.print("De eerste calibratie is klaar! Maak nu de hoek die je tijdens het skeeleren wilt vasthouden. We gaan verder over zoveel seconden:"); Serial.println(tijdOver);
        }

        timerCalibration = millis();
        N = 0;

        while (millis() - timerCalibration < durationCalibration) {
          struct Delta d;
          d = getDelta(60);
          desiredAngle_l = desiredAngle_l + d.l;
          desiredAngle_r = desiredAngle_r + d.r;
          N = N + 1;
        }

        desiredAngle_l = desiredAngle_l / N;
        desiredAngle_r = desiredAngle_r / N;
        Serial.print("Average delta_r: "); Serial.print(desiredAngle_l);
        Serial.print(" Average delta_l: "); Serial.println(desiredAngle_r);

        desiredAngle_l = map(desiredAngle_l, angle90_l, angle180_l, 90, 175);
        desiredAngle_r = map(desiredAngle_r, angle90_r, angle180_r, 90, 175);

        Serial.print("Desired Angle Links: "); Serial.print(desiredAngle_l);
        Serial.print(" Desired Angle Rechts: "); Serial.println(desiredAngle_r);

        Serial.println("We zijn klaar met calibreren!!");
        phase = ProgramState::Working;
        break;
      }

    case ProgramState::Working: {
        //Serial.println("phase 2: working ");

        // get the difference of accelerometers from a series of samples
        struct Delta d;

        d = getDelta(sampleFrequency);
        angle_old = angle_new;
        angle_new.r =  map(d.r, angle90_r, angle180_r, 90, 175);
        angle_new.l =  map(d.l, angle90_l, angle180_l, 90, 175);

        //Check which leg is skated on
        giveFeedback();

        byte incoming = 0;
        if (SerialBT.available()) {
          incoming = SerialBT.read();
          Serial.println(incoming);
        }

        if (incoming == 'r') {
          phase = ProgramState::Initialization;
          incoming = 0;
        }

        break;
      }
  }
}

/////////////////////////////////////////////////////////////////////
/// Function for feedback
/////////////////////////////////////////////////////////////////////

void giveFeedback() {

  //Dit feedback gedeelte werkt nog niet.
  //Morgen opnieuw naar kijken.

  supportLeg_old = supportLeg_new;
  supportLeg_new = checkSupportLeg();

  if (supportLeg_new != supportLeg_old) {
    Serial.print("Support Leg: "); Serial.println(supportLeg_new);
  }

  if (supportLeg_new == 'l') {
    //Check if last five values are above desired angle
    for (int i = 0; i < 5; i++) {
      if (angle_new.l > desiredAngle_l) {
        sensorCounter += 1;
      } else {
        sensorCounter = 0;
        break;
      }
    }
  }

  if (supportLeg_new == 'r') {
    for (int i = 0; i < 5; i++) {
      if (angle_new.r > desiredAngle_r) {
        sensorCounter += 1;
      } else {
        sensorCounter = 0;
        break;
      }
    }

  }

  if (sensorCounter > 4) {
    //Serial.println("Kniehoek te hoog, feedback geven!");
    sensorCounter = 0;
  }
}

char checkSupportLeg() {
  bool print_ = false;
  if (millis() - setdownTimer_r > 200) {
    if ((angle_new.r - angle_old.r) > 50) {
      Serial.print("Afzet Delta Rechts");
      SerialBT.print("Afzet Delta Rechts");
      pushOffTimer_r = millis();
      print_ = true;
      supportLeg_new = 'l';
    }
  }
  if (millis() - setdownTimer_l > 200) {
    if ((angle_new.l - angle_old.l) > 50) {
      Serial.print(" | Afzet delta links");
      SerialBT.print(" | Afzet delta links");
      pushOffTimer_l = millis();
      print_ = true;
      supportLeg_new = 'r';
    }
  }
  if (millis() - pushOffTimer_r > 200) {
    if ((angle_old.r - angle_new.r > 50)) {
      Serial.print(" | Delta neerzetten rechts");
      SerialBT.print(" | Delta neerzetten rechts");
      print_ = true;
      supportLeg_new = 'r';
      setdownTimer_r = millis();
    }
  }
  if (millis() - pushOffTimer_l > 200) {
    if ((angle_old.l - angle_new.l > 50)) {
      Serial.print(" | Delta neerzetten links");
      SerialBT.print(" | Delta neerzetten links");
      print_ = true;
      supportLeg_new = 'l';
      setdownTimer_l = millis();
    }
  }
  if (((angle_new.r - angle_old.r) > 50) && (angle_old.l - angle_new.l > 50))  {
    Serial.print(" | Dubbele check standbeen rechts ");
    SerialBT.print(" | Dubbele check sf tandbeen rechts ");
    print_ = true;
    supportLeg_new = 'r';
  }
  if (((angle_new.l - angle_old.l) > 50) && (angle_old.r - angle_new.r > 50))  {
    Serial.print(" | Dubbele check standbeen links ");
    SerialBT.print(" | Dubbele check standbeen links ");
    supportLeg_new = 'l';
    print_ = true;
  }
  if (angle_new.l > 180) {
    Serial.print(" | Afzet threshold links");
    SerialBT.print(" | Afzet threshold links");
    supportLeg_new = 'r';
    pushOffTimer_l = millis();
    print_ = true;
  }
  if (angle_new.r > 180) {
    Serial.print(" | Afzet threshold rechts");
    SerialBT.print(" | Afzet threshold rechts");
    pushOffTimer_r = millis();
    supportLeg_new = 'l';
    print_ = true;
  }
  if (print_) {
    Serial.println("");
    SerialBT.println("");
  }
  return supportLeg_new;
}

/////////////////////////////////////////////////////////////////////////
// accelerometer business functions
/////////////////////////////////////////////////////////////////////////

void initMpu(int mpuIndex) { // NOTE: also wakes MPU
  writeByteToMpu(mpuIndex, 0x6B, 0x1 << 1); // set CLKSEL to 1 (use x-axis gyro reference)
  writeByteToMpu(mpuIndex, 0x1B, 0x0 << 3); // set FS_SEL to 0 (use full scale gyro)
  writeByteToMpu(mpuIndex, 0x1C, 0x0 << 3); // set AFS_SEL to 0 (use full scale accel)
}

void resetMpu(int mpuIndex) {
  Serial.println("Resetting mpu...");
  writeByteToMpu(mpuIndex, 0x6B, 0x1 << 7); // set DEVICE_RESET bit of PWR_MGMT_1 register to 1
  delay(100);
  writeByteToMpu(mpuIndex, 0x68, 0x1 | 0x2 | 0x4); // set GYRO_RESET, ACCEL_RESET and TEMP_RESET bits of SIGNAL_PATH_RESET register to 1
  delay(100);
  initMpu(mpuIndex);
}


void selectMpu(int mpuIndex) {
  digitalWrite(AD00, HIGH);
  digitalWrite(AD01, HIGH);
  digitalWrite(AD02, HIGH);
  digitalWrite(AD03, HIGH);
  digitalWrite(AD04, HIGH);
  digitalWrite(AD05, HIGH);

  if (mpuIndex == 0) {
    digitalWrite(AD00, LOW);
  }
  if (mpuIndex == 1) {
    digitalWrite(AD01, LOW);
  }
  if (mpuIndex == 2) {
    digitalWrite(AD02, LOW);
  }
  if (mpuIndex == 3) {
    digitalWrite(AD03, LOW);
  }
  if (mpuIndex == 4) {
    digitalWrite(AD04, LOW);
  }
  if (mpuIndex == 5) {
    digitalWrite(AD05, LOW);
  }

}

void writeByteToMpu(int mpuIndex, int addr, int value) {
  selectMpu(mpuIndex);

  Wire.beginTransmission(MPU_addr);
  Wire.write(addr);
  Wire.write(value);
  Wire.endTransmission(true);
}

uint8_t readByteFromMpu(int mpuIndex, int addr) {
  selectMpu(mpuIndex);
  Wire.beginTransmission(MPU_addr);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 1, true);
  return Wire.read();
}

int16_t readAcc(int mpuIndex) {
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  selectMpu(mpuIndex);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_addr, 1 * 2, true); // request a total of 14 registers

  auto const readShort = []() {
    return Wire.read() << 8 | Wire.read();
  };

  AcX = readShort(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  //  AcY = readShort(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  //  AcZ = readShort(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //  Tmp = readShort(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  //  GyX = readShort(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  //  GyY = readShort(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  //  GyZ = readShort(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  return AcX;
}

struct Delta getDelta(int sampleFrequency) {
  // different sample frequencies for calibration (fast) and normal working (slower)

  int32_t delta_l, delta_r, delta_b;
  int32_t average0, average1, average2, average3, average4, average5;


  //Neater to put into for loops.

  average0 = 0;
  average1 = 0;
  average2 = 0;
  average3 = 0;
  average4 = 0;
  average5 = 0;


  for (int i = 0; i < numberOfSamples; i++) {
    average0 += (int32_t) readAcc(0);
    average1 += (int32_t) readAcc(1);
    average2 += (int32_t) readAcc(2);
    average3 += (int32_t) readAcc(3);
    average4 += (int32_t) readAcc(3);
    average5 += (int32_t) readAcc(3);
    //delay(1000 / sampleFrequency);              // how many samples per second
  }

  average0 = average0 / numberOfSamples;
  average1 = average1 / numberOfSamples;
  average2 = average2 / numberOfSamples;
  average3 = average3 / numberOfSamples;
  average4 = average4 / numberOfSamples;
  average5 = average5 / numberOfSamples;

  delta_l = (average0 - average1);
  delta_r = (average2 - average3);
  delta_b = (average5 - average4);

  angle_l = map(delta_l, angle90_l, angle180_l, 90, 175);
  angle_r = map(delta_r, angle90_r, angle180_r, 90, 175);

  Serial.print(" Delta left Leg = "); Serial.print(delta_l);
  Serial.print(" Delta right Leg = "); Serial.print(delta_r);
  Serial.print(" Delta back = "); Serial.print(delta_b);
  Serial.print(" Ac0: "); Serial.print(average0);
  Serial.print(" Ac1: "); Serial.print(average1);
  Serial.print(" Ac2: "); Serial.print(average2);
  Serial.print(" Ac3: "); Serial.print(average3);
  Serial.print(" Ac4: "); Serial.print(average4);
  Serial.print(" Ac5: "); Serial.print(average5);
  Serial.print(" IMU Angle R Leg = "); Serial.print(angle_r);
  Serial.print(" IMU Angle L Leg = "); Serial.println(angle_l);

  SerialBT.print(" Ac0 = "); SerialBT.print(average0);
  SerialBT.print(" Ac1 = "); SerialBT.print(average1);
  SerialBT.print(" Ac2 = "); SerialBT.print(average2);
  SerialBT.print(" Ac3 = "); SerialBT.print(average3);
  SerialBT.print(" Delta left Leg = "); SerialBT.print(delta_l);
  SerialBT.print(" Delta right Leg = "); SerialBT.print(delta_r);
  SerialBT.print(" IMU Angle R Leg = "); SerialBT.print(angle_r);
  SerialBT.print(" IMU Angle L Leg = "); SerialBT.println(angle_l);


  if ((average0 == 0) || (average0 == -1) || (average1 == 0) || (average1 == -1)) {
    for (int i = 0; i < 4; i++) {
      resetMpu(i); // sometimes I2C fails, then restart accelerometers
    }
  }

  Delta delta;
  delta.l = delta_l;
  delta.r = delta_r;

  return delta;
}
