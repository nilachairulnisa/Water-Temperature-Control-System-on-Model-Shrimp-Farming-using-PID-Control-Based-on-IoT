#include <LiquidCrystal_I2C.h>

//#include <analogWrite.h>

#include <OneWire.h>
#include <DallasTemperature.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

//--------------------------------------PID
double Kp = 108;
double Ki = 108;
double Kd = 27;
float error_suhu_min;
float error_suhu_max;
float integral_error_min;
float integral_error_max;
float derivatif_error_min;
float derivatif_error_max;
float last_error_min = 0;
float last_error_max = 0;
float PID_kincir_min;
float PID_kincir_max;

//-------------------------------------Arduino
//#define ONE_WIRE_BUS  A0
//#define IN1  3
//#define IN2  4
//#define ENC  A5


//-------------------------------------ESP32
#define ONE_WIRE_BUS  7
#define IN1  6 //13
#define IN2  4 //12
#define IN3  5 
#define IN4  2 
#define IN5  3 // kincir  

//#define ENC  27 //digital ESP32 untuk pengaturan kecepatan PID
OneWire oneWire (ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//-----------------------------------Setting Suhu
float suhu = 0.0;
float suhu_max = 31;
float suhu_min = 28;

//-------------------------------------Milis
unsigned long Waktuawalwrite = 0; //startcounting timer write
unsigned long intervalwrite = 1000; //looping timer write
unsigned long Counterwaktu;



void setup() {
  Serial.begin(9600);
  lcd.begin();
  // Menyalakan lampu latar lCD
  lcd.backlight();
  //Clear LCD
  lcd.clear();
  sensors.begin();
  pinMode (ONE_WIRE_BUS, INPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (IN5, OUTPUT);
  Counterwaktu = millis();

}



void loop() {

  //analogWriteResolution (10); 
  Counterwaktu = millis();
  unsigned long elapsedTime = (double)(Counterwaktu - Waktuawalwrite);

  analogWrite (IN5, 75);
 
 //-----------------------------Pembacaan Suhu
  sensors.setResolution(12);
  sensors.requestTemperatures();
  float suhu = sensors.getTempCByIndex(0);
  Serial.print("Suhu : ");
  Serial.print(suhu_max);
  Serial.print(" ");
  Serial.print(suhu_min);
  Serial.print(" ");
  Serial.println(suhu);
  lcd.setCursor(0, 0);
  lcd.print(suhu);
  delay(1000);
  
  //---------------------------------------PID Suhu Min
  error_suhu_min = suhu_min - suhu;
  integral_error_min += error_suhu_min * elapsedTime;
  derivatif_error_min = (error_suhu_min - last_error_min) / elapsedTime;
  last_error_min = error_suhu_min;

  PID_kincir_min = (Kp * error_suhu_min) + (Ki * integral_error_min) + (Kd * derivatif_error_min);
  if(PID_kincir_min <= 0)
  {
    PID_kincir_min = 0;
  }
  if(PID_kincir_min >= 255)
  {
    PID_kincir_min = 255;
  }
  else
  //Serial.println (PID_kincir_min, 2);

  //--------------------------------------PID Suhu Max
  error_suhu_max = suhu - suhu_max;
  integral_error_max += error_suhu_max * elapsedTime;
  derivatif_error_max = (error_suhu_max - last_error_max) / elapsedTime;
  last_error_max = error_suhu_max;

  PID_kincir_max = (Kp * error_suhu_max) + (Ki * integral_error_max) + (Kd * derivatif_error_max);
  if(PID_kincir_max <= 0)
  {
    PID_kincir_max = 0;
  }
  if(PID_kincir_max >= 255)
  {
    PID_kincir_max = 255;
  }
  else
  //Serial.println (PID_kincir_max, 3);
  delay (1000);
  
  

  //--------------------------------------Deklarasi Keadaan
  if (Counterwaktu - Waktuawalwrite > intervalwrite)
   {
    Waktuawalwrite = Counterwaktu ;
   
    if (suhu < suhu_min)
    {
    analogWrite(IN1, PID_kincir_min);
    analogWrite(IN2, 0);
    analogWrite(IN3, PID_kincir_min);
    analogWrite(IN4, 0);
    
    } 

    else

    if (suhu > suhu_max)
    {
    analogWrite(IN1, PID_kincir_max);
    analogWrite(IN2, 0);
    analogWrite(IN3, PID_kincir_max);
    analogWrite(IN4, 0);
    
    }

    else
    {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
    
    }
 } 
 
}
