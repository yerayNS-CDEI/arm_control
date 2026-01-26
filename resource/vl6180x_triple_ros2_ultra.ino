#include <Adafruit_VL6180X.h>
#include <NewPing.h>

#define SONAR_NUM 3
#define MAX_DISTANCE 60

NewPing sonar[SONAR_NUM] = {
  NewPing(13, 12, MAX_DISTANCE),  // (trigger, echo, max_dist)
  NewPing(11, 10, MAX_DISTANCE),
  NewPing(9, 8, MAX_DISTANCE)
};

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 5
#define SHT_LOX2 4
#define SHT_LOX3 3

#define TIMING_PIN 7

// objects for the VL6180X
Adafruit_VL6180X lox1 = Adafruit_VL6180X();
Adafruit_VL6180X lox2 = Adafruit_VL6180X();
Adafruit_VL6180X lox3 = Adafruit_VL6180X();

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
*/
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(100);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(100);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if (!lox1.begin()) {
    Serial.println(F("Failed to boot first VL6180X"));
    while (1);
  }
  lox1.setAddress(LOX1_ADDRESS);
  delay(100);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(100);

  //initing LOX2
  if (!lox2.begin()) {
    Serial.println(F("Failed to boot second VL6180X"));
    while (1);
  }
  lox2.setAddress(LOX2_ADDRESS);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(100);

  //initing LOX3
  if (!lox3.begin()) {
    Serial.println(F("Failed to boot third VL6180X"));
    while (1);
  }
  lox3.setAddress(LOX3_ADDRESS);
  Serial.println(F("Sensors initialized"));
}

void read_sensors() {

  uint8_t r1 = lox1.readRange();
  uint8_t s1 = lox1.readRangeStatus();

  uint8_t r2 = lox2.readRange();
  uint8_t s2 = lox2.readRangeStatus();

  uint8_t r3 = lox3.readRange();
  uint8_t s3 = lox3.readRangeStatus();

  Serial.print("S1:");
  Serial.print((s1 == VL6180X_ERROR_NONE) ? r1 : 255);  // 255 as invalid value (if error raised)
  Serial.print("mm ");
  Serial.print("S2:");
  Serial.print((s2 == VL6180X_ERROR_NONE) ? r2 : 255);
  Serial.print("mm ");
  Serial.print("S3:");
  Serial.print((s3 == VL6180X_ERROR_NONE) ? r3 : 255);
  Serial.println("mm");
}

void read_ultrasonic_sensors() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    delay(30); // Small delay to avoid interferences
    Serial.print("U");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(sonar[i].ping_cm());
    Serial.print("cm ");
  }
}

//===============================================================
// Setup
//===============================================================
void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  // Enable timing pin so easy to see when pass starts and ends
  pinMode(TIMING_PIN, OUTPUT);

#ifdef GPIO_LOX1
  // If we defined GPIO pins, enable them as PULL UP
  pinMode(GPIO_LOX1, INPUT_PULLUP);
  pinMode(GPIO_LOX2, INPUT_PULLUP);
  pinMode(GPIO_LOX3, INPUT_PULLUP);
#endif

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(TIMING_PIN, LOW);
  Serial.println("All in reset mode...(pins are low)");


  Serial.println("Starting...");
  setID();

}

//===============================================================
// Loop
//===============================================================
void loop() {
  read_ultrasonic_sensors();
  read_sensors();
  
  delay(100);
}
