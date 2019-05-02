//*************************************          LIBRARIES          **************************************

#include "DHT.h"                                    //import biblioteki czujnika DHT
#include "Wire.h"                                   //import biblioteki interfejsu OneWire
#include "TimeLib.h"                                //import biblioteki odczytu czasu
#include "DS1307RTC.h"                              //import biblioteki zegara RTC
#include "LiquidCrystal_I2C.h"                      //import biblioteki wyświetlacza LCD


//*************************************          DEFINITIONS          **************************************

#define PIRPIN            2                         //deklaracja pinu czujnika ruchu PIR
#define LEDPIN            5                         //deklaracja pinu taśm LED
#define RPMPIN            4                         //deklaracja pinu odczytu predkosci wentylatora   
#define PWMPIN            3                         //deklaracja pinu sterujacego PWM wentylatora
#define DATAPIN           7                         //deklaracja pinu ustawiającego dane rejestru przesuwnego
#define LATCHPIN          A0                        //deklaracja pinu zatrzasku rejestru przesuwnego
#define CLOCKPIN          A1                        //deklaracja pinu przesuwającego rejestru przesuwnego
#define SENSORDHT21PIN    A2                        //deklaracja pinu czujnika temperatury i wilgotnosci

#define FANRELAY          0                         //deklaracja numeru bitu odpowiedzialnego za wentylator
#define BULBRELAY         7                         //deklaracja numeru bitu odpowiedzialnego za zarowke
#define TXFRAMESIZE       8                         //deklaracja rozmiaru ramki
#define SERIALSPEED       9600                      //deklaracja prędkości transmisji portu szeregowego


//*************************************          VARIABLES         **************************************

int pulsewidth = 0;                                 //wypelnienie sygnalu regulujacego jasnosc swiecenia ledow
int ledtime = 100;                                  //okres zmian jasnosci swiecenia tasmy LED
int change = 5;                                     //zmiana poziomu swiecenia tasmy LED
int maxledbright = 255;                             //maksymalna jasnosc swiecenia tasmy LED
byte relay = 255;                                   //domyslna wartosc przekaznikow (255 wszystkie wylaczone)                                         
unsigned long time;                                 //zmienna okresu wentylatora
unsigned int rpmfan;                                //zmienna predkosci obrotow wentylatora
String stringRPM;                                   //predkosc obrotow wentylatora zapisana w ciagu znakow

int pwmPin     = 3; // digital PWM pin 9
int pwmVal     = 1; // The PWM Value
int speeddutycycle = 0;                             //wypelnienie sygnalu wentylatora (w zakresie od 0-80)



//*************************************          STRUCTURES          **************************************

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


//*************************************          FUNCTIONS          **************************************
 
void setup()                                        //funkcja konfiguracyjna
{
  pinMode(DATAPIN,  OUTPUT);                        //ustawienie pinu danych rejestru jako wyjscie
  pinMode(CLOCKPIN, OUTPUT);                        //ustawienie pinu zegarowego rejestru jako wyjscie
  pinMode(LATCHPIN, OUTPUT);                        //ustawienie pinu zatrzasku rejestru jako wyjscie
  pinMode(LEDPIN,   OUTPUT);                        //ustawienie pinu tasmy LED jako wyjscie
  pinMode(PWMPIN,   OUTPUT);                        //ustawienie pinu sterowania PWM wentylatora jako wyjscie
  
  pinMode(PIRPIN,   INPUT);                         //ustawienie pinu czujnika PIR jako wejscie
  pinMode(RPMPIN,   INPUT);                         //ustawienie pinu odczytu predkosci wentylatora jako wejscie
  
  hc595(relay);                                     //ustawienie wyjsc rejestru przesuwnego na zadeklarowana wczesniej wartosc

  Serial.begin(SERIALSPEED);                        //uruchomienie UART z zadeklarowana wczesniej wartoscia
  set_time();                                       //ustawienie czasu
  lcd.init();                                       //inicjalizacja wyswitlacza LCD 
  lcd.backlight();                                  //podświetlenie wyswietlacza LCD
  sensor.begin();                                   //uruchomienie czujnika DHT21
  read_SensorDHT21();                               //odczytanie wartosci zmierzonych danych czujnika DHT21
  
  TCCR2A = 0x23;                                    //ustawienie prescalera (COM2B1, WGM21, WGM20)  
  TCCR2B = 0x0A;                                    //ustawienie prescalera (WGM21, Prescaler =/8)
  OCR2A = 79;                                       //ustawienie najwyzszej wartosci, od ktorej bedzie odliczane w dol (ustawia wypelnienie PWM- nie zmieniac!)
  OCR2B = 0;                                        //ustawienie wartosci PWM (0-79)
// digitalWrite(RPMPIN, HIGH);                      //odczyt predkosci wentylatora



}

//*************************************          SHIFT REGISTER          **************************************

void hc595(byte value)                              //funkcja sterowania rejestrem przesuwnym
{
  digitalWrite(LATCHPIN, LOW);
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST , value);
  digitalWrite(LATCHPIN, HIGH);
}


//*************************************          TRANSMISION          **************************************

void setup_TxFrame_data()
{
//  TxFrame.values.start_code=0x40;
//  TxFrame.values.code=0x20;
//  TxFrame.values.length_frame=TXFRAMESIZE;
//  TxFrame.values.stop_code=0x80; 
}

void send_Data()
{
 Serial.write(TxFrame.bytes_to_send.tab_bytes, TXFRAMESIZE);
}



//*************************************          BYTES          **************************************

byte setBit(byte &number, byte n, byte value)      //zwracamy liczbę przez referencję i zmieniamy bit
{
  return number |= number ^= (-value ^ number) & (1UL << n);
}

void setbyte(int n, byte &number)      //ustawianie n-tego bitu w zmiennej number, referencja aby działać na zmienionej liczbie a nie jej kopii
{
  number |= 1UL << n;
}


void clearbyte(int n, byte &number)    //czyszczenie n-tego bitu w zmiennej number
{
  number &= ~(1UL << n);
}


void togglebyte(int n, byte &number)    //zamiana (negowanie) n-tego bitu w zmiennej number 0->1,1->0
{
  number ^= 1UL << n;
}


bool checkbyte(int n, byte number)    //sprawdzanie n-tego bitu w zmiennej number
{
  return (number >> n) & 1U;             //zwrocenie wartosci n-tego bitu zmiennej number 
}


//*************************************          REAL TIME CLOCK          **************************************

void set_time()
{
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


//*************************************          LCD SCREEN         **************************************

void lcdscreen()
{
 char buff [16];
 lcd.setCursor(0,0);
 int i_hour = sprintf(buff, "%d:", tm.Hour);
   if (i_hour<3){
   i_hour = sprintf(buff, "0%d:", tm.Hour);
   }
 lcd.print(buff);
      int i_minute = sprintf(buff, "%d", tm.Minute);
         if (i_minute<2){
         i_hour = sprintf(buff, "0%d", tm.Minute);
         }
      lcd.print(buff);
      
 lcd.setCursor(0,1);
 sprintf(buff, "%d/%d/%d", tm.Day, tm.Month, tm.Year-30);
 int i_day = sprintf(buff, "%d/", tm.Day);
   if (i_day<3){
   i_hour = sprintf(buff, "0%d/", tm.Day);
   }
 lcd.print(buff);
     int i_month = sprintf(buff, "%d/", tm.Month);
        if (i_month<3){
        i_hour = sprintf(buff, "0%d/", tm.Month);
        }
     lcd.print(buff);
           sprintf(buff, "%d", tm.Year-30);
           lcd.print(buff );
           
lcd.setCursor(10,0);
lcd.print(sensor.readTemperature(),1);
lcd.setCursor(14, 0);
lcd.print((char)223);
lcd.setCursor(15, 0);
lcd.print("C");
lcd.setCursor(10,1);
lcd.print(sensor.readHumidity(),1);
lcd.setCursor(15, 1);
lcd.print("%");     
}


//*************************************          DHT21 SENSOR          **************************************

void read_SensorDHT21()
{
 TxFrame.values.temperature = sensor.readTemperature();     //przypisz zmiennej temperatura odczyt temperatury powietrza
 TxFrame.values.humidity = sensor.readHumidity();           //przypisz zmiennej wilgotnosc odczyt wilgotnosci powietrza
}


//*************************************          LED STRIPES          **************************************

void ledfade()
{
 analogWrite(LEDPIN, pulsewidth); //Generujemy sygnał o zadanym wypełnieniu
 
 if (pulsewidth > 0) { //Jeśli wypełnienie wieksze od 0%
 pulsewidth = pulsewidth - change; //Zmniejszamy wypełnienie
 } else {
 pulsewidth = 0; //Jeśli wypełnienie rowne 0% to zatrzymaj zmiany
 }
 delay(ledtime); //Małe opóźnienie, aby efekt był widoczny
}

void ledbright()
{
 analogWrite(LEDPIN, pulsewidth); //Generujemy sygnał o zadanym wypełnieniu
 
 if (pulsewidth < maxledbright) { //Jeśli wypełnienie mniejsze od 100%
 pulsewidth = pulsewidth + change; //Zwiększamy wypełnienie
 } else {
 pulsewidth = 0; //Jeśli wypełnienie większe od 100%, to wracamy na początek
 }
 delay(ledtime); //Małe opóźnienie, aby efekt był widoczny
}


//*************************************          PWM FAN          **************************************

void set_fan_speed(uint8_t duty_cycle)
{
 if (duty_cycle > 0 && duty_cycle < 80) {
     OCR2B = duty_cycle;
 }
}

char getRPMS() 
{
 time = pulseIn(RPMPIN, HIGH);
 rpmfan = (1000000 * 60) / (time * 4);
 stringRPM = String(rpmfan);
 //if (stringRPM.length() < 5) {
   Serial.println(rpmfan, DEC);
// }
}

void fanaccelerate()
{
 speeddutycycle+=10;
 if (speeddutycycle > 0 && speeddutycycle < 80) {
     OCR2B = speeddutycycle;  
 }
}

void fanslowdown()
{
 speeddutycycle-=10;
 if (speeddutycycle > 0 && speeddutycycle < 80) {
     OCR2B = speeddutycycle;  
 }
}

void togglefan()
{                  
  togglebyte(FANRELAY, relay);            //stan wysoki na bicie
  hc595(relay);
}
 


//*************************************          SERIAL MONITOR          **************************************

void printsensor()
{
Serial.print(TxFrame.values.temperature);
Serial.print(" ");
Serial.println(TxFrame.values.humidity);
}

void printclock()
{
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

void readserial()
{
byte incomingByte=0;
incomingByte = Serial.read(); 
    
if (incomingByte == '1') {
  read_SensorDHT21();
  Serial.print("Odczyt czujnika DHT: ");
  printsensor();
}

if (incomingByte == '2') {
  RTC.read(tm);
  Serial.print("Odczyt zegara: ");
  printclock();
}

if (incomingByte == '3') {
togglebulb();
Serial.println("Zmieniono stan pracy zarowki");
}

if (incomingByte == '4') {
togglefan();
Serial.println("Zmieniono stan pracy wiatraka");
}

if (incomingByte == '5') {
fanaccelerate();
Serial.println("Zwiekszono obroty wiatraka");
}

if (incomingByte == '6') {
fanslowdown();
Serial.println("Zmniejszono obroty wiatraka");
}

}


//*************************************          HEAT BULB          **************************************

void lightonbulb()
{
  clearbyte(BULBRELAY, relay);          //stan niski na bicie
  hc595(relay);
}

void lightoffbulb()
{                  
  setbyte(BULBRELAY, relay);            //stan wysoki na bicie
  hc595(relay);
}

void togglebulb()
{                  
  togglebyte(BULBRELAY, relay);            //stan wysoki na bicie
  hc595(relay);
}


//*************************************          MAIN LOOP          **************************************

void loop() 
{
  RTC.read(tm);
  lcdscreen();
  ledbright();
  if (Serial.available()>0) readserial();
}
