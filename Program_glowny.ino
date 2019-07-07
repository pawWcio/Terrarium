//*************************************          LIBRARIES          **************************************

#include "DHT.h"                                    //import biblioteki czujnika DHT
#include "Wire.h"                                   //import biblioteki interfejsu OneWire
#include "TimeLib.h"                                //import biblioteki odczytu czasu
#include "DS1307RTC.h"                              //import biblioteki zegara RTC
#include "LiquidCrystal_I2C.h"                      //import biblioteki wyświetlacza LCD


//*************************************          DEFINITIONS          **************************************

#define SENSORDHT21PIN    2                         //deklaracja pinu czujnika temperatury i wilgotnosci
#define PWMPIN            3                         //deklaracja pinu sterujacego PWM wentylatora
#define RPMPIN            4                         //deklaracja pinu odczytu predkosci wentylatora  
#define LEDPIN            5                         //deklaracja pinu taśm LED 
#define BUTTONPIN         6                         //deklaracja pinu przycisku włączającego generator mgły manualnie
#define DATAPIN           7                         //deklaracja pinu ustawiającego dane rejestru przesuwnego
#define DIODEFOGGERPIN    8                         //deklaracja pinu diody sygnalizujacej prace generatora mgly
#define RGBDIODEREDPIN    9                         //deklaracja pinu koloru czerwonego diody RGB
#define RGBDIODEGREENPIN  10                        //deklaracja pinu koloru zielonego diody RGB
#define RGBDIODEBLUEPIN   11                        //deklaracja pinu koloru niebieskiego diody RGB
#define DIODEBULBPIN      12                        //deklaracja pinu diody sygnalizujacej prace zarowki
#define PIRPIN            13                        //deklaracja pinu czujnika ruchu PIR
#define LATCHPIN          A0                        //deklaracja pinu zatrzasku rejestru przesuwnego
#define CLOCKPIN          A1                        //deklaracja pinu przesuwającego rejestru przesuwnego
#define LIGHTSENSORPIN    A5                        //deklaracja pinu czujnika natezenia swiatla

#define FOGGERFANRELAY    7                         //deklaracja numeru bitu odpowiedzialnego za wentylator
#define FOGGERRELAY       6                         //deklaracja numeru bitu odpowiedzialnego za generator mgly
#define NIGHTLEDRELAY     5                         //deklaracja numeru bitu odpowiedzialnego za oswietlenie nocne
#define BULBRELAY         4                         //deklaracja numeru bitu odpowiedzialnego za zarowke
#define TXFRAMESIZE       8                         //deklaracja rozmiaru ramki
#define SERIALSPEED       9600                      //deklaracja prędkości transmisji portu szeregowego


//*************************************          VARIABLES         **************************************

int pulsewidth = 0;                                 //wypelnienie sygnalu regulujacego jasnosc swiecenia ledow
int ledtime = 100;                                  //okres zmian jasnosci swiecenia tasmy LED
int change = 25;                                    //zmiana poziomu swiecenia tasmy LED
int maxledbright = 255;                             //maksymalna jasnosc swiecenia tasmy LED
int lightsensorvalue = 900;                         //odczyt wartosci natezenia swiatla
int lightintensity = 100;                           //zdeklarowany poziom wlaczenia oswietlenia nocnego
byte relay = 255;                                   //domyslna wartosc przekaznikow (255 wszystkie wylaczone)                                         
unsigned long time;                                 //zmienna okresu wentylatora
unsigned int rpmfan;                                //zmienna predkosci obrotow wentylatora
String stringRPM;                                   //predkosc obrotow wentylatora zapisana w ciagu znakow

int pwmPin = 3; // digital PWM pin 9
int pwmVal = 1; // The PWM Value
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
  pinMode(DATAPIN,          OUTPUT);                //ustawienie pinu danych rejestru jako wyjscie
  pinMode(CLOCKPIN,         OUTPUT);                //ustawienie pinu zegarowego rejestru jako wyjscie
  pinMode(LATCHPIN,         OUTPUT);                //ustawienie pinu zatrzasku rejestru jako wyjscie
  pinMode(LEDPIN,           OUTPUT);                //ustawienie pinu tasmy LED jako wyjscie
  pinMode(PWMPIN,           OUTPUT);                //ustawienie pinu sterowania PWM wentylatora jako wyjscie    
  pinMode(RGBDIODEREDPIN,   OUTPUT);                //ustawienie pinu koloru czerwonego diody RGB jako wyjscie
  pinMode(RGBDIODEGREENPIN, OUTPUT);                //ustawienie pinu koloru zielonego diody RGB jako wyjscie
  pinMode(RGBDIODEBLUEPIN,  OUTPUT);                //ustawienie pinu koloru niebieskiego diody RGB jako wyjscie
  pinMode(DIODEFOGGERPIN,   OUTPUT);                //ustawienie pinu diody generatora mgly jako wyjscie
  pinMode(DIODEBULBPIN,     OUTPUT);                //ustawienie pinu diody zarowki grzewczej jako wyjscie  

  pinMode(LIGHTSENSORPIN,   INPUT);                 //ustawienie pinu czujnika natezenia swiatla jako wejscie
  pinMode(PIRPIN,           INPUT);                 //ustawienie pinu czujnika PIR jako wejscie
  pinMode(BUTTONPIN,        INPUT);                 //ustawienie pinu przycisku jako wejscie
  pinMode(RPMPIN,           INPUT);                 //ustawienie pinu odczytu predkosci wentylatora jako wejscie
  
  hc595(relay);                                     //ustawienie wyjsc rejestru przesuwnego na zadeklarowana wczesniej wartosc

  Serial.begin(SERIALSPEED);                        //uruchomienie UART z zadeklarowana wczesniej wartoscia
  set_time();                                       //ustawienie czasu
  lcd.init();                                       //inicjalizacja wyswitlacza LCD 
  lcd.backlight();                                  //podświetlenie wyswietlacza LCD
  sensor.begin();                                   //uruchomienie czujnika DHT21
  read_SensorDHT21();                               //odczytanie wartosci zmierzonych danych czujnika DHT21
  
  TCCR2A = 0x23;                                    //ustawienie prescalera (COM2B1, WGM21, WGM20)  
  TCCR2B = 0x0A;                                    //ustawienie prescalera (WGM21, Prescaler =/8)
  OCR2A = 79;                                       //ustawienie najwyzszej wartosci wypelnienia PWM, od ktorej bedzie odliczane w dol
  OCR2B = 0;                                        //ustawienie najnizszej wartosci wypelnienia PWM, od ktorej bedzie odliczane w gore
  analogWrite(RGBDIODEREDPIN, 0);  
  analogWrite(RGBDIODEGREENPIN, 0);
  analogWrite(RGBDIODEBLUEPIN, 0);
// digitalWrite(RPMPIN, HIGH);                      //odczyt predkosci wentylatora



}

//*************************************          SHIFT REGISTER          **************************************

void hc595(byte value)                              //funkcja sterowania rejestrem przesuwnym
{
  digitalWrite(LATCHPIN, LOW);                      //zatrzask ustawiony w stan niski (zablokowanie wyjsc rejestru)
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST , value);    //pojedynczy przesuw bajtow danych, zaczynajac od najbardziej znaczacego
  digitalWrite(LATCHPIN, HIGH);                     //zatrzask ustawiony w stan wysoki (odblokowanie wyjsc rejestru)
}


//*************************************          TRANSMISION          **************************************

void setup_TxFrame_data()                           //funkcja deklarujaca ramke transmisji danych
{
//  TxFrame.values.start_code=0x40;                 //kod poczatku
//  TxFrame.values.code=0x20;                       //kod funkcji
//  TxFrame.values.length_frame=TXFRAMESIZE;        //kod dlugosci ramki
//  TxFrame.values.stop_code=0x80;                  //kod konca 
}

void send_Data()                                    //funkcja przesylu ramki danych
{
 Serial.write(TxFrame.bytes_to_send.tab_bytes, TXFRAMESIZE);
}



//*************************************          BYTES          **************************************

byte set_Bit(byte &number, byte n, byte value)      //funkcja zwracajaca liczbe przez referencje i zmieniajaca bit
{
  return number |= number ^= (-value ^ number) & (1UL << n);
}

void set_Byte(int n, byte &number)                  //ustawianie n-tego bitu w zmiennej number, referencja aby działać na zmienionej liczbie a nie jej kopii
{
  number |= 1UL << n;
}


void clear_Byte(int n, byte &number)                //czyszczenie n-tego bitu w zmiennej number
{
  number &= ~(1UL << n);
}


void toggle_Byte(int n, byte &number)               //zamiana (negowanie) n-tego bitu w zmiennej number 0->1,1->0
{
  number ^= 1UL << n;
}


bool check_Byte(int n, byte number)                 //sprawdzanie n-tego bitu w zmiennej number
{
  return (number >> n) & 1U;                        //zwrocenie wartosci n-tego bitu zmiennej number 
}


//*************************************          REAL TIME CLOCK          **************************************

void set_time()                                     //funkcja konfigurujaca czas zegara
{
  bool parse=false;
  bool config=false;

  if (getDate(__DATE__) && getTime(__TIME__)) {     //sprawdzenie poprawnosci konfiguracji zegara
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

//  Serial.begin(9600);                               //wlaczenie portu szeregowego
//  while (!Serial) ; // wait for serial
//  delay(200);

  Serial.println("DS1307RTC Start");            //wyswietlenie informacji o poprawnej konfiguracji zegara
  Serial.println("-------------------");
}

bool getTime(const char *str)                       //funkcja ustawiajaca czas rzeczywisty zegara
{
  int Hour, Min, Sec;                               //deklaracja zmiennych czasu

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;                                   //przypisanie wartosci do zmiennych
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)                       //funkcja ustawiajaca date zegara
{
  char Month[12];                                   //zmienne daty
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;                                      //przypisanie wartosci do zmiennych
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}


//*************************************          LCD SCREEN         **************************************

void lcdDisplay()                                    //funkcja wyswietlajaca dane na wyswietlaczu LCD
{
 char buff [16];                                     //stworzenie bufora znakow
 lcd.setCursor(0,0);                                 //ustawienie kursora na wyswietlaczu
 int i_hour = sprintf(buff, "%d:", tm.Hour);         //sprawdzanie warunkow wyswietlania
   if (i_hour<3){
   i_hour = sprintf(buff, "0%d:", tm.Hour);
   }
 lcd.print(buff);                                    
      int i_minute = sprintf(buff, "%d", tm.Minute);
         if (i_minute<2){
         i_hour = sprintf(buff, "0%d", tm.Minute);
         }
      lcd.print(buff);                               //wyswietlanie aktualnej godziny na wyswietlaczu
      
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
           lcd.print(buff );                          //wyswietlanie aktualnej daty na wyswietlaczu
           
lcd.setCursor(10,0);
lcd.print(sensor.readTemperature(),1);                //wyswietlanie aktualnej temperatury na wyswietlaczu
lcd.setCursor(14, 0);
lcd.print((char)223);                                 //wyswietlanie znaku stopni na wyswietlaczu
lcd.setCursor(15, 0);
lcd.print("C");
lcd.setCursor(10,1);
lcd.print(sensor.readHumidity(),1);                   //wyswietlanie aktualnej wilgotnosci na wyswietlaczu
lcd.setCursor(15, 1);
lcd.print("%");     
}


//*************************************          DHT21 SENSOR          **************************************

void read_SensorDHT21()                               //funkcja odczytu danych z czujnika DHT21
{
 TxFrame.values.temperature = sensor.readTemperature();     //przypisz zmiennej temperatura odczyt temperatury powietrza
 TxFrame.values.humidity = sensor.readHumidity();           //przypisz zmiennej wilgotnosc odczyt wilgotnosci powietrza
}

//*************************************          NIGHT LIGHTNING          **************************************

void turnOn_NightLight()                              //funkcja kontrolowania oswietlenia nocnego
{
  lightsensorvalue = analogRead(LIGHTSENSORPIN);      //odczyt natezenia swiatla z fotorezystora              
  
  if (lightsensorvalue < lightintensity && digitalRead(PIRPIN) == LOW) {
       set_Byte(NIGHTLEDRELAY, relay);                //wlaczenie oswietlenia nocnego   
       hc595(relay); 
  } 
  else {
       clear_Byte(NIGHTLEDRELAY, relay);              //wylaczenie oswietlenia nocnego 
       hc595(relay);      
  }
}

void toggle_NightLight()                              //funkcja wlaczania/wylaczania oswietlenia nocnego
{                  
  toggle_Byte(NIGHTLEDRELAY, relay);                  //zmiana stanu na bicie odpowiedzialnym za oswietlenie nocne
  hc595(relay);                                       //ustawienie rejestru przesuwnego
}



//*************************************          BULB DIODE          **************************************

void toggle_BulbDiode()                               //funkcja sygnalizacji stanu pracy zarowki grzewczej
{
  byte bulbstatus = check_Byte(BULBRELAY, relay);     //przypisanie bitu odpowiedzialnego za prace zarowki do zmiennej 
  if (bulbstatus == 0) {                              //sprawdzenie warunku pracy
     digitalWrite(DIODEBULBPIN, HIGH);                //wlaczenie diody    
  } 
  else {
     digitalWrite(DIODEBULBPIN, LOW);                 //wylaczenie diody      
  }
}

//*************************************          FOGGER DIODE          **************************************

void toggle_FoggerDiode()                             //funkcja sygnalizacji stanu pracy zarowki grzewczej
{
  byte foggerstatus = check_Byte(FOGGERRELAY, relay); //przypisanie bitu odpowiedzialnego za prace zarowki do zmiennej 
  if (foggerstatus == 0) {                            //sprawdzenie warunku pracy
     digitalWrite(DIODEFOGGERPIN, HIGH);              //wlaczenie diody    
  } 
  else {
     digitalWrite(DIODEFOGGERPIN, LOW);               //wylaczenie diody      
  }
}

//*************************************          RGB DIODE          **************************************
    
void rgb_RedColor(){
    analogWrite(RGBDIODEREDPIN, 255);  
    analogWrite(RGBDIODEGREENPIN, 0);
    analogWrite(RGBDIODEBLUEPIN, 0);
}

void rgb_OrangeColor(){
    analogWrite(RGBDIODEREDPIN, 255);  
    analogWrite(RGBDIODEGREENPIN, 165);
    analogWrite(RGBDIODEBLUEPIN, 0);
}

void rgb_YellowColor(){
    analogWrite(RGBDIODEREDPIN, 255);  
    analogWrite(RGBDIODEGREENPIN, 255);
    analogWrite(RGBDIODEBLUEPIN, 0);
}

void rgb_GreenColor(){
    analogWrite(RGBDIODEREDPIN, 0);  
    analogWrite(RGBDIODEGREENPIN, 255);
    analogWrite(RGBDIODEBLUEPIN, 0);
}

void rgb_BlueColor(){
    analogWrite(RGBDIODEREDPIN, 0);  
    analogWrite(RGBDIODEGREENPIN, 0);
    analogWrite(RGBDIODEBLUEPIN, 255);
}

void rgb_PurpleColor(){
    analogWrite(RGBDIODEREDPIN, 255);  
    analogWrite(RGBDIODEGREENPIN, 0);
    analogWrite(RGBDIODEBLUEPIN, 255);
}

void rgb_TurquoiseColor(){
    analogWrite(RGBDIODEREDPIN, 0);  
    analogWrite(RGBDIODEGREENPIN, 255);
    analogWrite(RGBDIODEBLUEPIN, 255);
}


//*************************************          LED STRIPES          **************************************

void led_Fade()                                       //funkcja symulowania zmierzchu za pomoca oswietlenia 
{
 analogWrite(LEDPIN, pulsewidth);                     //generowanie sygnalu o zadanym wypelnieniu
 
 if (pulsewidth > 0) {                                //jesli wypelnienie wieksze od 0%
 pulsewidth = pulsewidth - change;                    //zmniejszamy wypelnienie
 } else {
 pulsewidth = 0;                                      //jesli wypelnienie rowne 0% to zatrzymaj zmiany
 }
 delay(ledtime);                                      //zadeklarowany okres zmian jasnosci
}

void led_Bright()                                     //funkcja symulowania switu za pomoca oswietlenia
{
 analogWrite(LEDPIN, pulsewidth);                     //generowanie sygnalu o zadanym wypelnieniu
 
 if (pulsewidth < maxledbright) {                     //jesli wypelnienie mniejsze od maksymalnego zadeklarowanego
 pulsewidth = pulsewidth + change;                    //zwiekszamy wypelnienie
 } else {
 pulsewidth = maxledbright;                           //jesli wypelnienie wieksze od zadeklarowanego to pozostaw wartosc maksymalna
 }
 delay(ledtime);                                      //zadeklarowany okres zmian jasnosci
}

void led_Bright_Manual()                             //reczne rozjasnianie ledow     
{
 analogWrite(LEDPIN, pulsewidth);                    
 
 if (pulsewidth < maxledbright) {                     
 pulsewidth += change;                                
 } else {
 pulsewidth = maxledbright;                           
 }
}

void led_Fade_Manual()                             //reczne sciemnianie ledow     
{
 analogWrite(LEDPIN, pulsewidth);                    
 
 if (pulsewidth > 0) {                     
 pulsewidth -= change;                                
 } else {
 pulsewidth = 0;                           
 }
}



//*************************************          PWM FAN          **************************************

void set_Fan_Speed(uint8_t duty_cycle)                //funkcja ustawiania wypelnienia sygnalu wentylatora
{
 if (duty_cycle > 0 && duty_cycle < 80) {
     OCR2B = duty_cycle;
 }
}

char get_RPMS()                                       //funkcja odczytu predkosci wentylatora
{
 time = pulseIn(RPMPIN, HIGH);                        //odczyt okresu tetnienia
 rpmfan = (1000000 * 60) / (time * 4);                //zamiana odczytu na predkosc wentylatora
 stringRPM = String(rpmfan);                          //zapis predkosci do zmiennej tekstowej
 Serial.println(rpmfan, DEC);                         //wyswietlenie predkosci wentylatora na monitorze portu szeregowego
}

void fan_Accelerate()                                 //funkcja zwiekszania predkosci obrotow wentylatora
{
 speeddutycycle+=10;                                  //zwiekszenie wypelnienia
 if (speeddutycycle > 0 && speeddutycycle < 80) {
     OCR2B = speeddutycycle;  
 }
}

void fan_SlowDown()                                   //funkcja zmniejszania predkosci obrotow wentylatora
{
 speeddutycycle-=10;                                  //zmniejszenie wypelnienia
 if (speeddutycycle > 0 && speeddutycycle < 80) {
     OCR2B = speeddutycycle;  
 }
}

void toggle_FoggerFan()                               //funkcja wlaczania/wylaczania wentylatora
{                  
  toggle_Byte(FOGGERFANRELAY, relay);                 //zmiana stanu na bicie odpowiedzialnym za wentylator
  hc595(relay);                                       //ustawienie rejestru przesuwnego
}

 
//*************************************          HEAT BULB          **************************************

void lightOn_Bulb()                                   //funkcja wlaczania zarowki grzewczej
{
  clear_Byte(BULBRELAY, relay);                       //stan niski na bicie odpowiedzialnym za zarowke
  hc595(relay);
}

void lightOff_Bulb()                                  //funkcja wylaczania zarowki grzewczej
{                  
  set_Byte(BULBRELAY, relay);                         //stan wysoki na bicie odpowiedzialnym za zarowke
  hc595(relay);
}

void toggle_Bulb()                                    //funkcja wlaczania/wylaczania zarowki grzewczej
{                  
  toggle_Byte(BULBRELAY, relay);                      //zmiana stanu na bicie odpowiedzialnym za zarowke
  hc595(relay);
}

//*************************************          FOGGER          **************************************

void lightOn_Fogger()                                   //funkcja wlaczania generatora mgly
{
  clear_Byte(FOGGERRELAY, relay);                       //stan niski na bicie odpowiedzialnym za generator mgly
  hc595(relay);
}

void lightOff_Fogger()                                  //funkcja wylaczania generatora mgly
{                  
  set_Byte(FOGGERRELAY, relay);                         //stan wysoki na bicie odpowiedzialnym za generator mgly
  hc595(relay);
}

void toggle_Fogger()                                    //funkcja wlaczania/wylaczania generatora mgly
{                  
  toggle_Byte(FOGGERRELAY, relay);                      //zmiana stanu na bicie odpowiedzialnym za generator mgly
  hc595(relay);
}


//*************************************          SERIAL MONITOR          **************************************

void print_Sensor()                                   //funkcja wyswietlajaca odczyt czujnika DHT21 na monitorze portu szeregowego
{
Serial.print(TxFrame.values.temperature);             //wyswietl temperature
Serial.print(" ");
Serial.println(TxFrame.values.humidity);              //wyswietl wilgotnosc
} 

void print_Clock()                                    //funkcja wyswietlajaca czas i date zegara na monitorze portu szeregowego
{
bool parse=false;
bool config=false;
       
if (true /*parse && config*/) {                                //sprawdzenie poprawnosci konfiguracji zegara
  Serial.print("DS1307 configured Time=");
  Serial.print(tm.Hour);                              //wyswietlanie godziny konfiguracji zegara
  Serial.print(":");
  Serial.print(tm.Minute);
  Serial.print(":");
  Serial.print(tm.Second);
  
  Serial.print(", Date=");                            //wyswietlanie daty konfiguracji zegara
  Serial.print(tm.Day);
  Serial.print(".");
  Serial.print(tm.Month);
  Serial.print(".");
  Serial.print(1970+tm.Year);
} 
else if (parse) {
  Serial.println("DS1307 Communication Error :-{");   //wyswietlanie komunikatu w przypadku niepoprawnej konfiguracji zegara
  Serial.println("Please check your circuitry");
} 
else {
  Serial.print("Time=\"");                            //wyswietlanie akualnej godziny
  Serial.print(tm.Hour);
  Serial.print(":");
  Serial.print(tm.Minute);
  Serial.print(":");
  Serial.print(tm.Second);
  
  Serial.print("\", Date=\"");                        //wyswietlanie aktualnej daty
  Serial.print(tm.Day);
  Serial.print(".");
  Serial.print(tm.Month);
  Serial.print(".");
  Serial.print(1970+tm.Year);
  Serial.println("\"");
}
}

void read_Serial()                                    //funkcja sterowania odczytami monitora portu szeregoego
{
byte incomingByte=0;                                  //zmienna wysylanych danych do portu
incomingByte = Serial.read();                         //przypisanie odczytu portu do zmiennej
    
if (incomingByte == '1') {                            //jezeli wyslano 1 to wyswietl odczyt czujnika DHT21
  read_SensorDHT21();
  Serial.print("Odczyt czujnika DHT: ");
  print_Sensor();
}

if (incomingByte == '2') {                            //jezeli wyslano 2 to wyswietl aktualna godzine i date z zegara
  RTC.read(tm);
  Serial.print("Odczyt zegara: ");
  print_Clock();
}

if (incomingByte == '3') {                            //jezeli wyslano 3 to zmien stan pracy zarowki grzewczej
toggle_Bulb();
Serial.println("Zmieniono stan pracy zarowki");
}

if (incomingByte == '4') {                            //jezeli wyslano 4 to zmien stan pracy wiatraka    
toggle_FoggerFan();
Serial.println("Zmieniono stan pracy wiatraka");
}

if (incomingByte == '5') {                            //jezeli wyslano 5 to zmien stan pracy osiwetlenia nocnego  
toggle_NightLight();
Serial.println("Zmieniono stan pracy oswietlenia nocnego");
}

if (incomingByte == '6') {                             //jezeli wyslano 6 to zwieksz obroty wiatraka
fan_Accelerate();
Serial.println("Zwiekszono obroty wiatraka");
}

if (incomingByte == '7') {                             //jezeli wyslano 7 to zmniejsz obroty wiatraka
fan_SlowDown();
Serial.println("Zmniejszono obroty wiatraka");
}

if (incomingByte == '8') {                             //jezeli wyslano 8 to zwieksz jasnosc ledow
led_Bright_Manual();
Serial.println("Zwiekszono jasnosc ledow");
}

if (incomingByte == '9') {                             //jezeli wyslano 9 to zmniejsz jasnosc ledow
led_Fade_Manual();
Serial.println("Zmniejszono jasnosc ledow");
}

if (incomingByte == '10') {                             //jezeli wyslano 10 to podaj odczyty stanów
Serial.println(digitalRead(BUTTONPIN));
Serial.println(digitalRead(PIRPIN));
Serial.println(analogRead(LIGHTSENSORPIN));
}

if (incomingByte == 'b') {                             
rgb_BlueColor();
Serial.println("Kolor niebieski");
}

if (incomingByte == 'p') {                             
rgb_PurpleColor();
Serial.println("Kolor fioletowy");
}

if (incomingByte == 't') {                             
rgb_TurquoiseColor();
Serial.println("Kolor turkusowy");
}

}


//*************************************          MAIN LOOP          **************************************

void loop()                                            //petla glowna programu
{
  RTC.read(tm);
  read_Serial();
  hc595(relay);
  lcdDisplay();
  toggle_BulbDiode();
  toggle_FoggerDiode();
  if (Serial.available()>0) read_Serial();
}
