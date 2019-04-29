#include "DHT.h"                                    //importuj bibliotekę czujnika DHT
#include "Wire.h"                                   //importuj bibliotekę interfejsu OneWire
#include "TimeLib.h"                                //importuj bibliotekę odczytu czasu
#include "DS1307RTC.h"                              //importuj bibliotekę zegara RTC
#include "LiquidCrystal_I2C.h"                      //importuj bibliotekę wyświetlacza LCD

#define PIRPIN            2                         //deklaracja czujnika ruchu PIR
#define LEDPIN            3                         //deklaracja taśm led
#define PWMPIN            5                         //deklaracja pinu sterujacego PWM wentylatora
#define DATAPIN           7                         //deklaracja pinu ustawiającego dane rejestru przesuwnego
#define LATCHPIN          A0                        //deklaracja pinu zatrzasku rejestru przesuwnego
#define CLOCKPIN          A1                        //deklaracja pinu przesuwającego rejestru przesuwnego
#define SENSORDHT21PIN    A2                        //deklaracja czujnika temperatury i wilgotnosci

#define BULBRELAY         7                         //deklaracja numeru bitu odpowiedzialnego za zarowke
#define TXFRAMESIZE       8                         //deklaracja rozmiaru ramki
#define SERIALSPEED       9600                      //deklaracja prędkości transmisji portu szeregowego

int pulsewidth = 0;                                 //wypelnienie sygnalu regulujacego jasnosc swiecenia ledow
int ledtime = 100;                                  //okres zmian jasnosci swiecenia ledow
int change = 5;                                     //zmiana poziomu swiecenia ledow
int maxledbright = 255;                             //maksymalna jasnosc swiecenia ledow
byte relay = 255;                                   //domyslna wartosc przekaznikow (255 wszystkie wylaczone)                                         
unsigned long time;
unsigned int rpm;
String stringRPM;

DHT sensor(SENSORDHT21PIN, DHT21);                  //utwórz instancję dla czujnika DHT21
LiquidCrystal_I2C lcd(0x3F,16,2);                   //utwórz instancję dla wyświetlacza LCD

struct data {                                       //struktura danych z czunika DHT21
  float humidity;                                   //zmienna wilgotności
  float temperature;                                //zmienna temperatury
};

tmElements_t tm;                                    //definicja klasy czasu zegara

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



 
void setup() 
{
  pinMode(DATAPIN,  OUTPUT);
  pinMode(CLOCKPIN, OUTPUT);
  pinMode(LATCHPIN, OUTPUT);
  pinMode(LEDPIN,   OUTPUT);
  pinMode(PWMPIN,   OUTPUT);  
  
  pinMode(PIRPIN,      INPUT); //PIR jako wejście
  
  hc595(relay);

  
  Serial.begin(SERIALSPEED);       //uruchom UART z predkoscia 9600 baud
  set_time();                      //ustawienie czasu
  lcd.init();                      //inicjalizacja wyswitlacza LCD 
  lcd.backlight();                 //podświetlenie wyswietlacza LCD


  sensor.begin();                                             //uruchom modul DHT
  read_SensorDHT21();
  
   TCCR2A = 0x23;     // COM2B1, WGM21, WGM20 
   // Set prescaler  
   TCCR2B = 0x0A;   // WGM21, Prescaler = /8
   // Set TOP and initialize duty cycle to zero(0)
   OCR2A = 79;    // TOP DO NOT CHANGE, SETS PWM PULSE RATE
   OCR2B = 0;    // duty cycle for Pin 3 (0-79) generates 1 500nS pulse even when 0 :
//   digitalWrite(2, HIGH);   // Starts reading
   Serial.begin(9600);

}




void hc595(byte value){                             //funkcja sterowania rejestrem przesuwnym
  digitalWrite(LATCHPIN, LOW);
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST , value);
  digitalWrite(LATCHPIN, HIGH);
}



byte setBit(byte &number, byte n, byte value){      //zwracamy liczbę przez referencję i zmieniamy bit
  return number |= number ^= (-value ^ number) & (1UL << n);
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




void ledfade(){
  analogWrite(LEDPIN, pulsewidth); //Generujemy sygnał o zadanym wypełnieniu
 
 if (pulsewidth > 0) { //Jeśli wypełnienie wieksze od 0%
 pulsewidth = pulsewidth - change; //Zmniejszamy wypełnienie
 } else {
 pulsewidth = 0; //Jeśli wypełnienie rowne 0% to zatrzymaj zmiany
 }
 
 delay(ledtime); //Małe opóźnienie, aby efekt był widoczny
}



void ledbright(){
  analogWrite(LEDPIN, pulsewidth); //Generujemy sygnał o zadanym wypełnieniu
 
 if (pulsewidth < maxledbright) { //Jeśli wypełnienie mniejsze od 100%
 pulsewidth = pulsewidth + change; //Zwiększamy wypełnienie
 } else {
 pulsewidth = 0; //Jeśli wypełnienie większe od 100%, to wracamy na początek
 }
 
 delay(ledtime); //Małe opóźnienie, aby efekt był widoczny
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




void printsensor(){
Serial.print(TxFrame.values.temperature);
Serial.print(" ");
Serial.println(TxFrame.values.humidity);
}




void printclock(){
bool parse=false;
bool config=false;
       
if (parse && config) {
  Serial.print("DS1307 configured Time=");
  Serial.print(tm.Hour);
  Serial.print(":");
  Serial.print(tm.Minute);
  Serial.print(":");
  Serial.print(tm.Second);
  
  Serial.print(", Date=");
  Serial.print(tm.Day);
  Serial.print(".");
  Serial.print(tm.Month);
  Serial.print(".");
  Serial.print(1970+tm.Year);
} 
else if (parse) {
  Serial.println("DS1307 Communication Error :-{");
  Serial.println("Please check your circuitry");
} 
else {
  Serial.print("Time=\"");
  Serial.print(tm.Hour);
  Serial.print(":");
  Serial.print(tm.Minute);
  Serial.print(":");
  Serial.print(tm.Second);
  
  Serial.print("\", Date=\"");
  Serial.print(tm.Day);
  Serial.print(".");
  Serial.print(tm.Month);
  Serial.print(".");
  Serial.print(1970+tm.Year);
  Serial.println("\"");
}
}



void readserial(){
byte incomingByte=0;
incomingByte = Serial.read(); 
    
if (incomingByte == '1') {
  read_SensorDHT21();
  printsensor();
}

if (incomingByte == '2') {
  RTC.read(tm);
  printclock();

}

if (incomingByte == '4') {
zapalzarowke();

}

if (incomingByte == '5') {
zgaszarowke();

}


if (incomingByte == '6') {
zmienstanzarowki();

}

}



void lcdread(){
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




void setbyte(int n, byte &number){      //ustawianie n-tego bitu w zmiennej number, referencja aby działać na zmienionej liczbie a nie jej kopii
  number |= 1UL << n;
}


void clearbyte(int n, byte &number){    //czyszczenie n-tego bitu w zmiennej number
  number &= ~(1UL << n);
}


void togglebyte(int n, byte &number){    //zamiana (negowanie) n-tego bitu w zmiennej number 0->1,1->0
  number ^= 1UL << n;
}


bool checkbyte(int n, byte number){    //sprawdzanie n-tego bitu w zmiennej number
  return (number >> n) & 1U;             //zwrocenie wartosci n-tego bitu zmiennej number 
}


void zapalzarowke(){
  clearbyte(BULBRELAY, relay);          //stan niski na bicie
  hc595(relay);
}

void zgaszarowke(){                  
  setbyte(BULBRELAY, relay);            //stan wysoki na bicie
  hc595(relay);
}

void zmienstanzarowki(){                  
  togglebyte(BULBRELAY, relay);            //stan wysoki na bicie
  hc595(relay);
}


void loop() {
  RTC.read(tm);
  lcdread();
  ledbright();
  if (Serial.available()>0) readserial();
}
