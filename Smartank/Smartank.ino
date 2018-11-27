#include <SoftwareSerial.h>
#include <OneWire.h>

//PIN Assignment
#define PH_SENSOR A0            //pH meter
#define DIRTY_SENSOR A1       // Dirty sensor
#define TEMPEATURE_SENSOR 4

#define BTrx 2
#define BTtx 3

#define MORTOR_IN_1 7                          // motor in 1
#define MORTOR_IN_2 6                          // motor in 2

#define LED 13

#define Offset 0.00            //deviation compensate for ph meter

#define samplingInterval 20
#define notifyInterval 800

OneWire ds(TEMPEATURE_SENSOR);
SoftwareSerial BTSerial(BTrx,BTtx);

void setup(){
  BTSerial.begin(9600);
  Serial.begin(9600);
  //TODO: OUTPUT PIN?
  pinMode(TEMPEATURE_SENSOR, OUTPUT); //Pin, Only Sensing Read Setup
  pinMode(MORTOR_IN_1,OUTPUT);              // motor input 1
  pinMode(MORTOR_IN_2,OUTPUT);              // motor input 2
}

void loop(void)
{
  static float pHValue, dirtyVoltage, temperature;
  static unsigned long samplingTime = 0;
  static unsigned long notifyTime = 0;

  unsigned long currentTime = millis();

  if(currentTime  > samplingTime + samplingInterval)
  {
     pHValue = getPH();
     dirtyVoltage= getDirty();     // dirty sensor's voltage (A1)
     temperature = getTemp();                //ds18b20
     samplingTime = currentTime;
  }

  if(currentTime > notifyTime + notifyInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    notify(pHValue, dirtyVoltage, temperature);
    notifyTime = currentTime;
    delay(1000);
  }

  //TODO: configuration for boundary value
  if (((temperature >= 28 || pHValue <= 6.0) || dirtyVoltage <= 1.0))    // operating condition for motor
    motorA_Rotation();
}

void notify(float pHValue, float dirtyVoltage, float temperature) {
  Serial.println(pHValue,2);
  Serial.println(dirtyVoltage,2);
  Serial.println(temperature,2);
  digitalWrite(LED,digitalRead(LED)^1);
  BTSerial.println(pHValue);
  BTSerial.println(dirtyVoltage);
  BTSerial.println(temperature);
}

float getDirty() {
  int sensorValue = analogRead(DIRTY_SENSOR);             // dirty sensor's value (A1)
  return sensorValue*(5.0/1024.0);     // dirty sensor's voltage (A1)
}

float getPH() {
  float voltage = getAveragePH() * 5.0 / 1024;
  return (3.5 * voltage) + Offset;
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      Serial.println("Cannot Search addr");
      ds.reset_search();
      return -1001;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1002;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1003;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum; //해당 값을 반환한다. - 즉 해당 값이 출력으로 나온다.
}

// DS18B20

#define ArrayLenth  40    //times of collection
double getAveragePH(){
  static int arr[ArrayLenth];   //Store the average value of the sensor feedback
  static int pHArrayIndex = 0;
  static bool fullySampled = false;
  int i;
  double avg;
  long sum = 0;
  int maxPH, minPH;
  int numberOfSamples = ArrayLenth;
  int currentPH = analogRead(PH_SENSOR);

  //TODO: int value?
  arr[pHArrayIndex++] = currentPH;
  if (pHArrayIndex == ArrayLenth) {
    pHArrayIndex = 0;
    fullySampled = true;
  }

  if (!fullySampled)
    numberOfSamples = pHArrayIndex;

  if (numberOfSamples < 5) {   //less than 5, calculated directly statistics
    for(i = 0; i < numberOfSamples; i++)
      sum += arr[i];
    avg = sum/numberOfSamples;
    return avg;
  }

  minPH = min(arr[0], arr[1]);
  maxPH = max(arr[0], arr[1]);

  for (i = 2; i < numberOfSamples; i++) {
    if (minPH <= arr[i] && arr[i] <= maxPH) {
      sum += arr[i];
      continue;
    }
    if ( arr[i] < minPH) {
      sum += minPH;     //arr<minPH
      minPH = arr[i];
    } else {
      sum += maxPH;    //arr>maxPH
      maxPH = arr[i];
    }
  }//for
  avg = (double)sum / (numberOfSamples - 2);
  return avg;
}

void motorA_Rotation()
{
    digitalWrite(MORTOR_IN_1,HIGH);
    digitalWrite(MORTOR_IN_2,LOW);
}
