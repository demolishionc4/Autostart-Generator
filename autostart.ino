/*
 * Have a generator autostart if power loss is detected
 * Existing electric start generator Harbor Freight Predator 8500 known by many as a 
 * typical chonda (chinese Honda knockoff)
 * Expecting to be able to still manually start the generator along with automatically by
 * tapping into existing generator wiring system, not a complete rewire the generator ignition.
 * We have a manual transfer swtich/box that the generator is hooked into.
 * This creates the need for a safe guard in it to require a button to be pushed by a human
 * to indicate someone is home and flipped the switch transfer box to generator power and back.
 * Also when power comes back on it will wait 5 minutes after button push before it turns off the 
 * generator.
 * The 5 minutes is to provide a cool down session for the generator along with validate the pole 
 * power is stable as often we see during power outages a few flickers of power or be back on and 
 * then a minute later go right back off.
 * When the LED is off it indicates the human has swtich back to pole power.
 * We want to test start the generator once a month and let it run the 30 mintues. This will keep
 * the start battery fully conditioned and fresh fuel flowing through the carb along test the start.
 * There is also an enclosure my generator is in so if the temp reaches 90F or higher and generator is
 * running to turn on the vent fan.
 * We are using relays and sensors purchased from Amazon as I had no interest of becoming electrical 
 * engineer and make my own circuits. I want to keep things as simple as possible with buy it.
 * Items:
 * Uno/Nano (I have a nano but it is the same as Uno but with a few extra pins)
 * 4 relay module (we will only use 3 but cheaper to buy this than 3 individual replays and easier to wire)
 * LC Technologies AC voltage sensor (Emon library from open energy monitor works with this)
 * Waterproof 20kg servo to control choke
 * 12v solonoid valve for gas on/off
 * button
 * hall sensor 3 wire NO
 * 1" LCD SPI display
 * DS3231 Real Time Clock
 * x2 TMP36 temperature sensors
 * green LED to indicate generator is running
 * red LED to indicate if safe to stop generator
 * yellow LED to indicate gas on
 * blue LED to indicate ignition on
 * white LED to indicate Starter on
 * 2 watertight boxes to house the electronics
 * IMPORTANT INFO:
 * 4 way relay I got has NormOpen side be actually HIGH and to enable set to LOW
 * This way if the board or a wire becomes disconnected then you dont have relays enabled
 * and burning out say your starter.
 * This also saves wear on your relay module since you dont need to energize your relays to
 * open the circuit.
 * So set to HIGH to disengage relay (no power) and LOW to engage relay (with power).
 */
#include "EmonLib.h" //Include Emon Library for the AC line voltage sensor
#include <Servo.h> //Choke Servo Library
#include "SSD1306Ascii.h" //github/bbkbarbar/Arduino-SSD1306Ascii library
#include "SSD1306AsciiAvrI2c.h" //same small ssd1306Ascii library set
#include "DS3231.h" //github.com/NorthernWidget/DS3231
#include <Wire.h> //used with ds3231.h for the i2c
#define I2C_ADDRESS 0x3c //My LCD i2c address and fairly common one
EnergyMonitor emon1; //Create an instance
int FANTEMP = A0; //Fan Temp sensor is on pin A0
//int AC = A1; //This is not needed but makes it human readable to know AC sensor is on pin A1
int ENGTEMP = A2; //Engine Temp sensor is on pin A3
//int SDA = A4; //This is not needed but makes it human readable to know the SDA i2c channel is A4
//int SCL = A5; //This is not needed but makes it human readable to know the SCL i2c channel is A5
int FAN = 2; //Generator enclosure vent fan pin
int GEN = 3; //generator RPM pin
//int Choke = 4; //This is not needed but makes it human readable to know choke servo is on pin 4
int GAS = 5; //Gas Pin
int IGN = 6; //Ignition Pin
int SW = 7; //Starter Pin
int PUSHBUTTON = 8; //Button Pin
int StopLed = 9; //Safe to stop generator LED
int GasLed = 10; //Gas on LED
int IgnLed = 11; //Ignition on LED
int StartLed = 12; //Starter on LED
int GenLed = 13; //Generator run LED
Servo ChokeServ;
SSD1306AsciiAvrI2c oled;
uint32_t Start_Time = 10000; //10 seconds to allow starter to run per attempt
uint32_t Wait_Time = 30000; //30 seconds to wait between start attempts to allow starter to cool
const unsigned long sampleTime = 1000;
int long Gen_Shutdown_Time = 0;
int long Gen_Test_Time = 0;
int long Now_Test_Time = 0;
int long Now_Stop_Time = 0;
int long Stop_Time = 0;
int Choke_Time = 0;
int Current_Time = 0;
int Time_Left = 0;
int ButtonTimeLeft = 0;
int IGN_Attempt = 0;
int GasState = 0;
int SwState = 0;
int IgnState = 0;
int GenRpm = 0;
int GenState = 0;
int ButtonState = 0;
int StopState = 0;
int TestState = 0;
int Volts = 0;
int readIndex = 0;
int total = 0;
int average = 0;
int AutoState = LOW; //this is to indicate if generator manual started or automated.
const int numReadings = 5; //Number of times to read AC volts to create the average and add to array
int readings[numReadings]; //readings array to handle volt average
DateTime now;
RTClib RTC;

void setup() {
  Serial.begin(115200); //enable serial connection for debug. Switch any oled.print to serial.print to send to serial
  pinMode(GEN,INPUT);
  pinMode(GAS,OUTPUT);
  pinMode(IGN,OUTPUT);
  pinMode(SW,OUTPUT);
  pinMode(GenLed,OUTPUT);
  pinMode(StopLed,OUTPUT);
  pinMode(GasLed,OUTPUT);
  pinMode(IgnLed,OUTPUT);
  pinMode(StartLed,OUTPUT);
  pinMode(PUSHBUTTON,INPUT);
  digitalWrite(GAS,HIGH); //Gas off
  digitalWrite(IGN,HIGH); //Starter off
  digitalWrite(SW,HIGH); //Ignition off
  digitalWrite(GenLed,LOW); //LED off
  digitalWrite(StopLed,LOW); //LED off
  digitalWrite(GasLed,LOW); //LED off
  digitalWrite(IgnLed,LOW); //LED off
  digitalWrite(StartLed,LOW); //LED off
  ChokeServ.attach(4); //Choke servo on pin 4
  Wire.begin();
  emon1.voltage(1, 110, 1.7); //voltage: input pin on Analog2, calibration, phase_shift
  for (int thisReading = 0; thisReading < numReadings; thisReading++) { //initialize readings to 0
    readings[thisReading] = 0;
  }
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(font5x7);
  oled.clear();
  #if INCLUDE_SCROLLING == 0
  #error INCLUDE_SCROLLING must be non-zero. Edit SSD1306Ascii.h
  #elif INCLUDE_SCROLLING == 1
  // Scrolling is not enable by default for INCLUDE_SCROLLING set to one.
  oled.setScroll(true);
  #else // INCLUDE_SCROLLING
  // Scrolling is enable by default for INCLUDE_SCROLLING greater than one.
  #endif
}

void gen_status() { //generator loop to look at current generator state
  GenRpm = getRPM();
  Serial.print("RPM");
  Serial.println(GenRpm);
  oled.print("RPM ");
  oled.println(GenRpm);
  if(GenRpm>=500) { //Check the RPMs of the generator and 500 or higher is running
    digitalWrite(GenLed,HIGH); //turn on LED if generator is running
    GenState = HIGH; //Set Gen state to high indicating generator is running
  }
  else {
    digitalWrite(GenLed,LOW); //turn off LED if generator is not running
    GenState = LOW; //Set GenState to low indicating generator is not running
  } 
}

void volt_status() { //check on voltage to see if pole power is valid
  emon1.calcVI(10,1000); //Calculate all. No of half wavelengths (crossings), time-out
  float Vrms = (emon1.Vrms - 3);
  total = total - readings[readIndex]; //subtract the last reading
  readings[readIndex] = Vrms; //input the Vrms value
  total = total + readings[readIndex]; //add the resding to the total
  readIndex = readIndex + 1; //advance to the next position in the array
  if (readIndex >= numReadings) { //if we ae at the end of the array wrap around to the begining
    readIndex = 0;
  }
  average = total / numReadings; //calculate the average
  if (average < 45) { Vrms = 0.0; } //Set 90 volts being low
  Volts = average * 2; //Double Vrms to get actual voltage
}

void choke() { //choke control using engine block temp with a sensor glued to side of block
 int reading = analogRead(ENGTEMP); //getting the voltage reading from the temperature sensor
 float voltage = reading * 5.0; // converting that reading to voltage (for 3.3v arduino use 3.3 and not 5.0)
 voltage /= 1024.0;
 float temperatureC = (voltage - 0.5) * 100 ; //converting from 10 mv per degree wit 500 mV offset to temp in C
 float EngTempF = (temperatureC * 9.0 / 5.0) + 32.0; // now convert to Fahrenheit for us Americans
 Serial.print("Engine temp ");
 Serial.println(EngTempF);
 if(EngTempF < 70) { //If generator block temp is less than 70F full choke
  ChokeServ.write(10); //degrees for full choke
  Serial.print("Full choke ");
  Serial.print(EngTempF);
  Serial.println(" F");
  oled.print("Full choke ");
  oled.print(EngTempF);
  oled.println(" F");
 }
 if(EngTempF >= 70 && EngTempF <= 110) { //If generator block temp is between 70F and 110F half choke
   ChokeServ.write(45); //degrees for half choke
   Serial.print("half choke ");
   Serial.print(EngTempF);
   Serial.println(" F");
   oled.print("half choke ");
   oled.print(EngTempF);
   oled.println(" F");
 }
 if(EngTempF > 110) { //If generator block temp is higher than 110F open choke
  ChokeServ.write(90); //degrees for open choke and servo defaults to 90
 }
}

void button() {
  DateTime now = RTC.now();
  if(GenState==1 && Volts<=10 && StopState==0) { //Press the button to indicate confirmation of manual switchover
    if(ButtonState==HIGH) { //Button pushed to indicate transfer began
      Serial.println("Verified");
      oled.println("Verified");
      digitalWrite(StopLed,HIGH);
    }
    else {
      Serial.println("Hold button");
      oled.println("Hold button");
    }
  }
  if(GenState==1 && Volts>=10 && StopState==1) { //If generator stop LED is off we can initiate shutdown
    if(ButtonState==HIGH) { //Button pushed to start shutdown
      Now_Stop_Time = now.unixtime();
      Stop_Time = Now_Stop_Time + 300L; //Set stop time now.unixtime + 5 minutes
      Serial.println("Allow shutdown");
      oled.println("Allow shutdown");
      digitalWrite(StopLed,LOW);
    }
    else {
      Serial.println("Hold button");
      oled.println("Hold button");
    }
  }
}

void genstart() { //Generator start sequence
  if(IGN_Attempt<3 && GenState==LOW) { //Try this for 3 attempts with Generator at low and less than as starts with attempt 0
    IGN_Attempt++; //add one count to start attempts
    Serial.print(IGN_Attempt);
    Serial.println(" of 3 attempts");
    Serial.println("Gen not running");
    Serial.println("Gas on");
    oled.print(IGN_Attempt);
    oled.println(" of 3 attempts");
    oled.println("Gen not running");
    oled.println("Gas on");
    digitalWrite(GAS,LOW); //Gas on
    digitalWrite(GasLed,HIGH); //Gas LED on
    delay(5000); //wait 5 seconds for gas to start flow
    Serial.println("Full choke");
    oled.println("Full choke");
    ChokeServ.write(10); //tell servo to go to position 10 degrees
    delay(200); //wait 200ms to let choke get to set
    Serial.println("Ignition on");
    oled.println("Ignition on");
    digitalWrite(IGN,LOW); //Ignition On
    digitalWrite(IgnLed,HIGH); //Ignition LED on
    delay(2000); //wait 2 second before starter attempt
    Serial.println("Starter on");
    oled.println("Starter on");
    for( uint32_t timer = millis(); (millis()-timer) < Start_Time; ) { //start a timer in milliseconds called timer and compare to time set against Start_Time
      digitalWrite(SW,LOW); //Starter On
      digitalWrite(StartLed,HIGH); //Starter LED on
      gen_status();
      if(GenState==HIGH) { //generator running
        Serial.println("Starter off");
        oled.println("Starter off");
        digitalWrite(SW,HIGH); //Starter off
        digitalWrite(StartLed,LOW); //Starter LED off
        Serial.println("Generator running");
        oled.println("Generator running");
        AutoState = 1;
        Choke_Time = 0;
        break;
      }
      delay(1);
    }
    Serial.println("Starter off");
    oled.println("Starter off");
    digitalWrite(SW,HIGH); //Starter Off
    digitalWrite(StartLed,LOW); //Starter LED off
    gen_status();
    //if Start loop complete and genrator not running wait 30 sec before next attempt but if 3rd attempt no reason to delay since wont try start attempt again
    if(GenState==LOW && Volts<=10 && IGN_Attempt < 3) {
      Serial.println("Tried for 10 seconds");
      Serial.println("Failed to start");
      Serial.println("Wait 30 seconds");
      oled.println("Tried for 10 seconds");
      oled.println("Failed to start");
      oled.println("Wait 30 seconds");
      for( uint32_t timer2 = millis(); (millis()-timer2) < Wait_Time; ) {
      }
    }      
  }
  else {
    if(GenState==HIGH) { //change to RPM to indicate generator running
      Serial.println("Generator running");
      oled.println("Generator running");
      choke();
      IGN_Attempt=0;
    }
    else { //Generator did not start so shutdown everything
      genshutdown();
      Serial.println("Failed to start!");
      Serial.println("Please check");
      Serial.println("generator!");
      oled.println("Failed to start!");
      oled.println("Please check");
      oled.println("generator!");
    }
  }
}

void genshutdown() { //Generator shutdown sequence
  Serial.print("Generator");
  Serial.println("shutdown");
  oled.print("Generator");
  oled.println("shutdown");
  digitalWrite(IGN,HIGH);
  digitalWrite(IgnLed,LOW); //Ignition LED off
  digitalWrite(GAS,HIGH);
  digitalWrite(GasLed,LOW); //Gas LED off
  delay(1);
}

void testrun() {
  DateTime now = RTC.now();
  if(now.day()==15 && now.hour()==12 && now.minute()==00 && TestState==LOW && GenState==LOW) { //Test the generator by starting at specified time and date
    Serial.println("Running test");
    oled.println("Running test");
    TestState = HIGH; //set the Test State to High
    genstart();
    Now_Test_Time = now.unixtime(); //set the variable for now unix time in long format
    Gen_Test_Time = Now_Test_Time + 1800L; //Set test to run 30 minutes which is 1800 seconds
  }
}

void boxtemp() {
 int reading = analogRead(FANTEMP); //getting the voltage reading from the temperature sensor
 float voltage = reading * 5.0; // converting that reading to voltage (for 3.3v arduino use 3.3 and not 5.0)
 voltage /= 1024.0;
 float temperatureC = (voltage - 0.5) * 100 ; //converting from 10 mv per degree wit 500 mV offset to temp in C
 float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0; // now convert to Fahrenheit for us Americans
 Serial.print("Box Temp ");
 Serial.println(temperatureF);
 if(GenState==HIGH && temperatureF >= 90) { //If generator is running and enclosure box is at 90F or higher
  digitalWrite(FAN,LOW); //start the fan
 }
 else {
  digitalWrite(FAN,HIGH); //Stop the fan
 }
}

int getRPM() {
  int count = 0;
  boolean countFlag = LOW;
  unsigned long currentTime = 0;
  unsigned long startTime = millis();
  while (currentTime <= sampleTime)
  {
    if (digitalRead(GEN) == HIGH) {
      countFlag = HIGH;
    }
    if (digitalRead(GEN) == LOW && countFlag == HIGH) {
      count++;
      countFlag=LOW;
    }
    currentTime = millis() - startTime; 
  }
  int countRpm = int(60000/float(sampleTime))*count;
  return countRpm;
}

void loop() {
  DateTime now = RTC.now();
  volt_status(); //get ac pole voltage info
  gen_status(); //get generator run status
  button(); //check button
  GenRpm = getRPM(); //check RPM on generator
  boxtemp(); //enclosure box fan control
  GasState = digitalRead(GAS); //current GAS relay state
  SwState = digitalRead(SW); //current SW relay state
  IgnState = digitalRead(IGN); //current IGN relay state
  ButtonState = digitalRead(PUSHBUTTON); //current button state
  StopState = digitalRead(StopLed); //current state of the system safety LED
  testrun(); //check to see if we should run the monthly test run
  if(Volts<=10) { //This indicates the AC voltage sensor on pole side of power has been lost or brown out and 10 helps with initial start
    oled.println("Power lost");
    genstart();
  }     
  else { //if AC power from the pole is available
    Serial.print("Good ");
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.println(now.second(), DEC);
    oled.print("Good ");
    oled.print(now.month(), DEC);
    oled.print('/');
    oled.print(now.day(), DEC);
    oled.print(' ');
    oled.print(now.hour(), DEC);
    oled.print(':');
    oled.print(now.minute(), DEC);
    oled.print(':');
    oled.println(now.second(), DEC);
    if(GenState==HIGH && AutoState==HIGH) { //if generator is running in an auto fashion
      if(TestState == HIGH) { //check if running generator test
        if(Gen_Test_Time <= now.unixtime()) { //use generator test time to decide if we shutdown
          genshutdown();
          TestState = LOW;
          AutoState = LOW;
          delay(1);
        }
        else {
          Serial.print("Run test for ");
          Serial.print(Gen_Test_Time - now.unixtime());
          Serial.println(" s");
          oled.print("Run test for ");
          oled.print(Gen_Test_Time - now.unixtime());
          oled.println(" s");
        }  
      }
      if(TestState == LOW) { //not running generator test
        if(Stop_Time <= now.unixtime() && StopState==LOW) { //5 minute count down for cool down before begin shutdown and Red LED is off
          genshutdown();
          AutoState = LOW;
          delay(1);
        }
        else {
          Gen_Shutdown_Time = Stop_Time - now.unixtime();
          Serial.print("Shutdown in ");
          Serial.print(Gen_Shutdown_Time);
          Serial.println(" s");
          oled.print("Shutdown in ");
          oled.print(Gen_Shutdown_Time);
          oled.println(" s");
          delay(1);
        }
      }
    }
    if(GenState==HIGH && AutoState==LOW) {
      Serial.println("Generator manual run");
      oled.println("Generator manual run");
    }
    delay(800);
    IGN_Attempt=0; //reset ignition attempts
  }
}

