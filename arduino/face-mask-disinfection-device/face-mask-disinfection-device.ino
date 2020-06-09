/*
 * Author: Jean Noel Lefebvre - http://www.needlab.org/ - june 3th 2020
 * 
 * */
//http://wiki.seeedstudio.com/Grove-Light_Sensor/
//https://github.com/thomasfredericks/Metro-Arduino-Wiring
//http://emery.claude.free.fr/arduino-lcd-1602.html
//https://wiki.seeedstudio.com/Grove-16x2_LCD_Series/
//https://www.arduino.cc/en/Reference/LiquidCrystal
//https://projetsdiy.fr/mesure-humidite-temperature-capteur-dht11-dht22-arduino-raspberry/#Identification_des_broches
//https://learn.adafruit.com/dht

#define LCD_1602
//#define LCD_GROVE_16x2

#include "DHT.h"   
#include <Wire.h>
#include <Metro.h> 
#include <EEPROM.h>

#ifdef LCD_1602
  #include <LiquidCrystal.h>
#endif
#ifdef LCD_GROVE_16x2
  #include "rgb_lcd.h"
#endif

#define BUZZER 13
#define BP_MODE 8
#define BP_START 9
#define DHTPIN 7
#define RELAY_HEAT 6
#define RELAY_UV 10
#define LIGHT_SENSOR A0

#define DEBOUNCE 12
#define DHTTYPE DHT22 // DHT 22 (AM2302)
#define TEMPERATURE 70
#define SEUIL_TEMP TEMPERATURE-5
#define HIGH_TEMP TEMPERATURE+10
#define SEUIL_LIGHT 60
#define TIMER 30

//*********************************************************
class Input
{
  public:
  Input();
  void Compute(bool);
  bool Value; 
  bool Pulse;
  private:
  int Counter;
};
Input::Input()
{
  Value=false;
  Pulse=false;
  Counter=0;
};
void Input::Compute(bool Val)
{
  if(Val) Counter++;
  else 
    {
      Counter=0;
      Value=false;
    }
  if (Counter==DEBOUNCE-1) Pulse=true;
  else Pulse=false;
  
  if (Counter > DEBOUNCE) 
  {
    Counter=DEBOUNCE;
    Value=true;
  }
};
//*********************************************************
#ifdef LCD_1602
  // Définition des broches RS, E, et Data (DB4 à DB7)
  LiquidCrystal lcd(12, 11, 5, 4, 3, 2); 
#endif

#ifdef LCD_GROVE_16x2
  rgb_lcd lcd;
  const int colorR = 200;
  const int colorG = 128;
  const int colorB = 128;
#endif

DHT dht(DHTPIN, DHTTYPE);  
enum Modes {HEAT_UVC, HEAT, UVC};
enum States{INIT, COUNT, END, ERR};
struct _FiniteStateMachine
{
  int State;
  int PreviousState;
  long Cycles;
  bool Change;
} FSM;

Input BP_Mode, BP_StartStop;
int Mode;
float Humidity;
float Temperature;
int Light;
int Timer=TIMER;
int Minute=0;
bool PointSeconde;

Metro Tick1s = Metro(1000); 
Metro Tick10s = Metro(5000); 

bool pidPower = false;
bool LightOK=false;
bool TemperatureOK=false;
String StatusLine="";

//**********************************************************************************
void setup() 
{
  pinMode(BP_MODE, INPUT_PULLUP);
  pinMode(BP_START, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY_HEAT, OUTPUT);
  pinMode(RELAY_UV, OUTPUT);
  Serial.begin(9600);
  //analogReference(INTERNAL);//set the refenrence voltage 1.1V
  
  BP_Mode = Input();
  BP_StartStop = Input();
  tone(BUZZER, 6000, 500);

  EEPROM.get(0, Mode);
  if(Mode > UVC) 
  {
    Mode=UVC;
    EEPROM.put(0, Mode);
  }
  if(Mode < HEAT_UVC) 
  {
    Mode=HEAT_UVC;
    EEPROM.put(0, Mode);
  }
  
  lcd.begin(16, 2);
  lcd.clear();
  DisplayMode();
  
#ifdef LCD_GROVE_16x2
  lcd.setRGB(colorR, colorG, colorB);
#endif
 
  dht.begin();
  FSM_init();
}

void loop() 
{
  Humidity = dht.readHumidity();
  Temperature = dht.readTemperature();
  PIDcompute(Temperature);
  
  if (Temperature >= SEUIL_TEMP) TemperatureOK=true;
  else TemperatureOK=false; 
  if(Mode==UVC) TemperatureOK=true;
  
  BP_Mode.Compute(!digitalRead(BP_MODE));
  BP_StartStop.Compute(!digitalRead(BP_START));
  
  Light=analogRead(LIGHT_SENSOR)/10;
  if(Light >= SEUIL_LIGHT) LightOK=true;
  else LightOK=false;

  if(Temperature >= HIGH_TEMP)  
      {
        tone(BUZZER, 3000, 200);
        StatusLine="High Temp.";
        delay(1000);
        FSM.State=ERR;
      }

  if ((Tick10s.check() == 1) )
  {
      Serial.print(0);
      Serial.print(" ");
      Serial.print(Temperature);
      Serial.print(" ");
      Serial.print(HIGH_TEMP);
      Serial.print(" ");
      Serial.print(TEMPERATURE);
      Serial.println();
  }
    
  if ((Tick1s.check() == 1) )
  {
      if(FSM.State==COUNT)
      {
        if(PointSeconde) PointSeconde=false;
        else PointSeconde=true;
        Minute--;
        if(Minute < 0) 
        {
          Minute=59;
          if(Timer) Timer--;
        }
        tone(BUZZER, 60, 20);
      }
      Display();
  }
  
  FSM_compute();
  
  if(pidPower && (Temperature < HIGH_TEMP)) digitalWrite(RELAY_HEAT, HIGH);
  else digitalWrite(RELAY_HEAT, LOW);
}

//************************************************
void FSM_init()
{
  FSM.State = 0;
  FSM.PreviousState = -1;
  FSM.Cycles = 0;
}
int FSM_compute()
{
  if (FSM.Cycles < 4000000000) FSM.Cycles++;
  if (FSM.State != FSM.PreviousState) FSM.Change = true;
  else FSM.Change = false;
  FSM.PreviousState = FSM.State;
  if (FSM.Change)
  {
    FSM.Cycles = 0;
  }
  
  switch (FSM.State)
  {
    case INIT: //---------------------------------------------------
      if (FSM.Change) //actions sur apparition de l'état
      {
        //Serial.println("INIT");
        DisplayMode(); 
      }
      Timer=TIMER;
      Minute=0;
      digitalWrite(RELAY_UV, LOW);

      if(BP_Mode.Pulse) 
      {
        ////Serial.print("BP_MODE / ");
        Mode++;
        if(Mode > UVC) Mode=HEAT_UVC;
        EEPROM.put(0, Mode);
        //Serial.println(Mode);
        tone(BUZZER, 1000, 50);        
        DisplayMode();
       }
    
      if(BP_StartStop.Pulse) 
      {
        switch(Mode)
        {
          case HEAT_UVC:
          case HEAT:
            if(TemperatureOK)
            {
              tone(BUZZER, 1000, 200);
              FSM.State=COUNT;
            }
            else
            {
              tone(BUZZER, 200, 1000);
              DispMessage("Low Temperature");
            }
          break;
          case UVC:
              tone(BUZZER, 1000, 200);
              FSM.State=COUNT;
          break;
        }
      }     
      break;
      
    case COUNT: //---------------------------------------------------
      if (FSM.Change) //actions sur apparition de l'état
      {
        //Serial.println("COUNT");
      }
      switch(Mode)
      {
        case HEAT_UVC:
          digitalWrite(RELAY_UV, HIGH);
          if(!LightOK && (Timer<=TIMER-2)) FSM.State=ERR;
            if(!TemperatureOK) FSM.State=ERR; 
        break;
        case HEAT:
          digitalWrite(RELAY_UV, LOW);
            if(!TemperatureOK) FSM.State=ERR; 
        break;
        case UVC:
          digitalWrite(RELAY_UV, HIGH);
          if(!LightOK && (Timer<=TIMER-2)) FSM.State=ERR;
        break;
      }
      if(Temperature >= HIGH_TEMP)  FSM.State=ERR;
      if((Timer== 0)&&(Minute==0)) FSM.State=END;
      
      if(BP_StartStop.Pulse) 
      {
           tone(BUZZER, 1000, 200);
           FSM.State=INIT;
       }
      break;
      
    case END: //---------------------------------------------------
      if (FSM.Change) //actions sur apparition de l'état
      {
        //Serial.println("END");
        StatusLine="END";
        Display();
        for(int i=0; i<6; i++)
        {
           tone(BUZZER, 300, 200);
            delay(500);
        }
      }
      digitalWrite(RELAY_UV, LOW);
      
      if(BP_StartStop.Value) 
      {
           tone(BUZZER, 1000, 200);
           FSM.State=INIT;
       }
      break;
      
    case ERR: //---------------------------------------------------
      if (FSM.Change) //actions sur apparition de l'état
      {
        //Serial.println("ERR");
        StatusLine="ERROR";
        Display();
        tone(BUZZER, 300, 3000);
      }
      digitalWrite(RELAY_UV, LOW);
      
      if(BP_StartStop.Pulse) 
      {
           tone(BUZZER, 1000, 200);
           FSM.State=INIT;
       }
      break;
  }
  return (FSM.State);
}

//*************************************************************
void PIDcompute(float Temperature)
{
  static bool Previous;
  Previous=pidPower;
  
  if((Mode != UVC)&& (Temperature < HIGH_TEMP))
  {
    if(Temperature < TEMPERATURE) pidPower=true;
    else pidPower=false;
  }
  else 
  {
    pidPower=false;
  }
  if (Previous != pidPower) Display();
}
//*************************************************************
void DispMessage(String Text)
{
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(Text);
  delay(3000);
}

void Display()
{
  lcd.begin(16, 2);
  lcd.clear();
  //first Line ************************************************************

  lcd.setCursor(0, 0); // Position du curseur sur la colonne 0, ligne 0
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(StatusLine);
  lcd.setCursor(10, 0);
  lcd.print(Timer);
  lcd.print(":");
  if(Minute<10) lcd.print(0);
  lcd.print(Minute);

 //second Line ************************************************************
 lcd.setCursor(0, 1);
 lcd.print("                ");
 lcd.setCursor(0, 1);
 if(pidPower) lcd.print("* ");
 else lcd.print("  ");
 lcd.print(Temperature);
 lcd.setCursor(6, 1);
 lcd.print("c ");
 lcd.print(int(Humidity));
 lcd.print("% ");
 lcd.print(Light);
 lcd.print("i");
}
//****************************************************
void DisplayMode()
{
  switch(Mode)
        {
          case HEAT_UVC:
            StatusLine="Heat & UV";
          break;
          case HEAT:
            StatusLine="Heat      ";
          break;
          case UVC:
            StatusLine="       UV ";
          break;
        }
        Display();
}
