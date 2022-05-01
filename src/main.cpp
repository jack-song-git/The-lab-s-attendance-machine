#include<Arduino.h>
#include<ESP32Servo.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0,14,27,26);
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
// All Boards without Reset of the Display


#define TrigPin 18
#define EchoPin 19
#define Srevo_left405 32
#define Servo_right410 33
#define Body_sensor 12


Servo left405,right410;                    //创建舵机对象
char Servo_Name;                           //servo Serial Name
int pos=0;                                   //servo Servo Pos
int a=analogRead(Body_sensor);
float Value_cm;

float Ultrasonic_module(void)
{
    digitalWrite(TrigPin, LOW); //低高低电平发一个短时间脉冲去TrigPin
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);
    Value_cm = float( pulseIn(EchoPin, HIGH) * 17 ) / 1000; 
    //读取一个引脚的脉冲（HIGH或LOW）。例如，如果value是HIGH，pulseIn()会等待引脚变为HIGH，开始计时，再等待引脚变为LOW并停止计时。
    //接收到的高电平的时间(us)*340m/s/2=接收到高电平的时间(us)*17000cm/1000000us = 接收到高电平的时间*17/1000(cm) 
    Serial.print(Value_cm);
    Serial.println("cm");
    delay(1000);
    return Value_cm;
    
}

void setup()
{
    ESP32PWM::allocateTimer(0);             //The Sevor use timer
    left405.attach(Srevo_left405);
    right410.attach(Servo_right410);
    pinMode(Body_sensor,INPUT);
    pinMode(TrigPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    u8g2.begin();
    Serial.begin(115200);
    
}

void loop() 
{
    if(Serial.available())
    {   
           
        Servo_Name=Serial.read();    
        pos=Serial.parseInt();   
        switch(Servo_Name)
        {
        case 'a':        
        Serial.print("Srevo_left405= ");
        Serial.println(pos);
        left405.write(pos);
        break;
        case 'b':        
        Serial.print("Srevo_right410= ");
        Serial.println(pos);
        right410.write(pos);
        break;

        }
    }
    Ultrasonic_module();
    u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,24,"Hello World!");
  } while ( u8g2.nextPage() );
}



