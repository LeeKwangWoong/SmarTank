#include <SoftwareSerial.h>
#include <OneWire.h>
#define BTtx 3
#define BTrx 2
#define SensorPin A0            //pH meter
#define DirtySensorPin A1       // Dirty sensor
#define Offset 0.00            //deviation compensate for ph meter
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
const int Temp_SensingPin = 4;
OneWire ds(Temp_SensingPin);
SoftwareSerial BTSerial(BTrx,BTtx);
int IN1=7;                          // motor in 1
int IN2=6;                          // motor in 2

void setup(){
  BTSerial.begin(9600);
  Serial.begin(9600);
  pinMode(Temp_SensingPin, OUTPUT); //Pin, Only Sensing Read Setup
  pinMode(IN1,OUTPUT);              // motor input 1
  pinMode(IN2,OUTPUT);              // motor input 2
}

void loop(void)
{
  int sensorValue=analogRead(DirtySensorPin);             // dirty sensor's value (A1)
  float dirtyVoltage=sensorValue*(5.0/1024.0);     // dirty sensor's voltage (A1)
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  float temperature = getTemp();                //ds18b20

  if (((temperature >= 28 || pHValue <= 6.0) || dirtyVoltage <= 1.0))    // operating condition for motor
    motorA_Rotation();

  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {

        Serial.println(pHValue,2);
        Serial.println(dirtyVoltage,2);
        Serial.println(temperature,2);
        digitalWrite(LED,digitalRead(LED)^1);
        printTime=millis();
        BTSerial.println(pHValue);
        BTSerial.println(dirtyVoltage);
        BTSerial.println(temperature);
        delay(1000);
        }
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

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

void motorA_Rotation()
{
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
}

