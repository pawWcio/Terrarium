#include "DHT.h"                                    //importuj bibliotekę czujnika DHT
#include "Wire.h"                                   //importuj bibliotekę interfejsu OneWire
#include "TimeLib.h"                                //importuj bibliotekę odczytu czasu
#include "DS1307RTC.h"                              //importuj bibliotekę zegara RTC
#include "LiquidCrystal_I2C.h"                      //importuj bibliotekę wyświetlacza LCD
#define PIR 2                                       //deklaracja czujnika ruchu PIR na pinie 2
#define PWMPIN 4                                    //deklaracja odczytu PWM wentylatora na pinie 3
#define diodaPIN 3
#define SENSORDHT21PIN 5                            //deklaracja czujnika na pinie 5
#define LEDPIN 6                                    //deklaracja diody na pinie 6
#define TXFRAMESIZE 8                               //deklaracja rozmiaru ramki
#define SERIALSPEED 9600                            //deklaracja prędkości transmisji portu szeregowego
#define dataPin 7
#define clockPin A1
#define latchPin A0

int wypelnienie = 0;
int zmiana = 5;
int pwmVal= 1;                                       
unsigned long time;
unsigned int rpm;
String stringRPM;
DHT sensor(SENSORDHT21PIN, DHT21);                  //utwórz instancję dla czujnika DHT21
LiquidCrystal_I2C lcd(0x3F,16,2);                   //utwórz instancję dla wyświetlacza LCD

struct data {                                       //struktura danych
  float humidity;                                   //zmienna wilgotności
  float temperature;                                //zmienna temperatury
};

int counter = 0;                                    //zmienna licznika

tmElements_t tm;                                    //definicja klasy czasu

const char *monthName[12] = {                       //definicja tablicy miesięcy
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
        
struct bytes {                                      //struktura adresów 
  uint8_t tab_bytes[TXFRAMESIZE];
};

union send_frame {                                  //definicja unii wysyłanej ramki danych
  data values ;                                     //dane
  bytes bytes_to_send;                              //bajty do wysyłania
};

union send_frame TxFrame;                           //ramka transmiji danych


void hc595(byte value){
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST , value);
  digitalWrite(latchPin, HIGH);
}



byte setBit(byte &number, byte n, byte value){  // zwracamy liczbę przez referencję i zmieniamy bit
  return number |= number ^= (-value ^ number) & (1UL << n);
}




 
void setup() 
{
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  hc595(254);

  
  Serial.begin(SERIALSPEED);       //uruchom UART z predkoscia 9600 baud
  set_time();                      //ustawienie czasu
  lcd.init();                      //inicjalizacja wyswitlacza LCD 
  lcd.backlight();                 //podświetlenie wyswietlacza LCD
  read_time();                     //odczyt czasu

  pinMode(LEDPIN,OUTPUT);
  pinMode(PIR, INPUT); //PIR jako wejście
  sensor.begin();                                             //uruchom modul DHT
  read_SensorDHT21();
//  TxFrame.values.temperature = sensor.readTemperature();      //przypisz zmiennej temperatura odczyt temperatury powietrza
//  TxFrame.values.humidity = sensor.readHumidity();            //przypisz zmiennej wilgotność odczyt wilgotnosci powietrza
   pinMode(PWMPIN, OUTPUT);   
   TCCR2A = 0x23;     // COM2B1, WGM21, WGM20 
   // Set prescaler  
   TCCR2B = 0x0A;   // WGM21, Prescaler = /8
   // Set TOP and initialize duty cycle to zero(0)
   OCR2A = 79;    // TOP DO NOT CHANGE, SETS PWM PULSE RATE
   OCR2B = 0;    // duty cycle for Pin 3 (0-79) generates 1 500nS pulse even when 0 :
//   digitalWrite(2, HIGH);   // Starts reading
   Serial.begin(9600);

}




void set_time(){
  bool parse=false;
  bool config=false;

  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

  Serial.begin(9600);
  while (!Serial) ; // wait for serial
  delay(200);
  Serial.println("DS1307RTC Read Test");
  Serial.println("-------------------");
}






void read_time(){
  bool parse=false;
  bool config=false;

  while (!Serial) ; // wait for Arduino Serial Monitor
  delay(200);
  if (parse && config) {
    Serial.print("DS1307 configured Time=");
    Serial.print(__TIME__);
    Serial.print(", Date=");
    Serial.println(__DATE__);
  } else if (parse) {
    Serial.println("DS1307 Communication Error :-{");
    Serial.println("Please check your circuitry");
  } else {
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }
}






bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}





bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}






void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}





void setup_TxFrame_data(){
//  TxFrame.values.start_code=0x40;
//  TxFrame.values.code=0x20;
//  TxFrame.values.length_frame=TXFRAMESIZE;
//  TxFrame.values.stop_code=0x80;
  
}





void send_Data(){
  Serial.write(TxFrame.bytes_to_send.tab_bytes, TXFRAMESIZE);
}





void read_SensorDHT21(){
         TxFrame.values.temperature = sensor.readTemperature();     //przypisz zmiennej temperatura odczyt temperatury powietrza
         TxFrame.values.humidity = sensor.readHumidity();           //przypisz zmiennej wilgotnosc odczyt wilgotnosci powietrza
}






void fan()
{
 unsigned int x;
  //ramp up fan speed by increasing duty cycle every 200mS, takes 16 seconds
 for(x = 0; x < 80; x++) {
   OCR2B = x;    // set duty cycle
   getRPMS();
   Serial.println(x);
   delay(200);
 }
 while (Serial.available() == 0);
 int val = Serial.parseInt();
 if (val > 0 && val < 80) {
     OCR2B = val;
 }
}

void led(){
  analogWrite(diodaPIN, wypelnienie); //Generujemy sygnał o zadanym wypełnieniu
 
 if (wypelnienie < 255) { //Jeśli wypełnienie mniejsze od 100%
 wypelnienie = wypelnienie + zmiana; //Zwiększamy wypełnienie
 } else {
 wypelnienie = 0; //Jeśli wypełnienie większe od 100%, to wracamy na początek
 }
 
 delay(50); //Małe opóźnienie, aby efekt był widoczny
}




void set_fan_speed(uint8_t duty_cycle){
 if (duty_cycle > 0 && duty_cycle < 80) {
     OCR2B = duty_cycle;
 }
}





char getRPMS() {
 time = pulseIn(2, HIGH);
  //  Serial.println(rpm, DEC);
 rpm = (1000000 * 60) / (time * 4);
 stringRPM = String(rpm);
 if (stringRPM.length() < 5) {
   Serial.println(rpm, DEC);
 }
}






void loop() {
  led();
 if (RTC.read(tm)) {
    Serial.print("Ok, Time = ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.println();
      
      char buff [16];
        sprintf(buff, "%d/%d/%d", tm.Day, tm.Month, 1970+tm.Year);
        lcd.setCursor(0,0);
        lcd.print(buff );
        lcd.setCursor(0,1);
        int i_hour = sprintf(buff, "%d:", tm.Hour);
        if (i_hour<3){
        i_hour = sprintf(buff, "0%d:", tm.Hour);
        }
        lcd.print(buff);
              int i_minute = sprintf(buff, "%d:", tm.Minute);
              if (i_minute<3){
              i_hour = sprintf(buff, "0%d:", tm.Minute);
              }
              lcd.print(buff);
                        int i_second = sprintf(buff, "%d", tm.Second);
                        if (i_second<2){
                        i_hour = sprintf(buff, "0%d", tm.Second);
                        }
                        lcd.print(buff);
 }
                else {
                if (RTC.chipPresent()) {
                  Serial.println("The DS1307 is stopped.  Please run the SetTime");
                  Serial.println("example to initialize the time and begin running.");
                  Serial.println();
                } else {
                  Serial.println("DS1307 read error!  Please check the circuitry.");
                  Serial.println();
                }
                delay(9000);
                }
                delay(1000);

  
  if(counter > 10){ 
         read_SensorDHT21();
         send_Data();
       //  fan();
         counter = 0;
  }

  if (Serial.available()>0){
    uint8_t serial_read_data = Serial.read();
    set_fan_speed(serial_read_data);
  }                       

  counter ++;
  delay(10);                                           //odczekaj 10 milisekund między pomiarami
  
}
