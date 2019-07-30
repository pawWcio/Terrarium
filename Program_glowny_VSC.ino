

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

#define BULBRELAY         4                         //deklaracja numeru bitu odpowiedzialnego za zarowke
#define NIGHTLEDRELAY     5                         //deklaracja numeru bitu odpowiedzialnego za oswietlenie nocne
#define FOGGERRELAY       6                         //deklaracja numeru bitu odpowiedzialnego za generator mgly
#define FOGGERFANRELAY    7                         //deklaracja numeru bitu odpowiedzialnego za wentylator
#define TXFRAMESIZE       8                         //deklaracja rozmiaru ramki
#define SERIALSPEED       9600                      //deklaracja prędkości transmisji portu szeregowego


//*************************************          VARIABLES         **************************************

float day_temperature = 24.0;                         //zadana wartosc temperatury dziennej
float night_temperature = 20.0;                       //zadana wartosc temperatury nocnej 
float setpoint_temperature; 
float setpoint_temperature_hysteresis = setpoint_temperature + 0.5;  //histereza temperatury
float setpoint_humidity;                       //zadana wartosc wilgotnosci
float setpoint_humidity_hysteresis = setpoint_humidity + 5;  //histereza wilgotnosci           
int pulsewidth = 0;                                 //wypelnienie sygnalu regulujacego jasnosc swiecenia ledow
int ledtime = 60000;                                //okres zmian jasnosci swiecenia tasmy LED
int change = 25;                                    //zmiana poziomu swiecenia tasmy LED
int maxledbright = 250;                             //maksymalna jasnosc swiecenia tasmy LED
int morning_hour = 7;                               //godzina rozpoczecia switu
int night_hour = 22;                                //godzina rozpoczecie zmierzchu
int lightsensorvalue = 0;                           //odczyt wartosci natezenia swiatla
int lightintensity = 1023;                          //zdeklarowany poziom wlaczenia oswietlenia nocneg
int fandelaytime = 5000;                            //czas po rozpoczeciu generowania mgly, po ktorym wlaczy sie wentylator
int fan_scaler = 10;                                //skalowanie predkosci wentylatora
int speeddutycycle = 0;                             //wypelnienie sygnalu wentylatora (w zakresie od 0-80)
unsigned long turnOnFoggerMillis = 0;               //czas od wlaczenia mgly
unsigned long turnOnFoggerFanMillis = 0;            //czas od wlaczenia wiatraka
unsigned long turnOnButtonMillis = 0;
unsigned long previousLedMillis = 0;                //czas od ostatniego wlaczenia ledow
const long foggingtime = 10000;                     //zadeklarowany czas generowania mgly po wlaczeniu manualnym
byte relay = 255;                                   //domyslna wartosc przekaznikow (255 wszystkie wylaczone)                                         
unsigned long time;                                 //zmienna okresu wentylatora
unsigned int rpmfan;                                //zmienna predkosci obrotow wentylatora
String stringRPM;                                   //predkosc obrotow wentylatora zapisana w ciagu znakow
unsigned long currentMillis = millis();             //czas od uruchomienia programu
bool check_status=false;
bool button_status=false;
bool fogger_status=false;                              //sprawdzanie dzialania foggera
bool bulb_status=false;                               //sprawdzanie dzialania zarowki
const int TX_FRAME_SIZE = 14;
const int RX_FRAME_SIZE = 14;
const int START_CODE = 0x40;
const int END_CODE = 0x80;
float temp_temperature=0.0;

int pwmPin = 3; // digital PWM pin 9
int pwmVal = 1; // The PWM Value


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
        
//struct bytes {                                      //struktura adresów 
//  uint8_t tab_bytes[TXFRAMESIZE];
//};


struct dataTX
{
  uint8_t start_code;
  float value_temperature;
  float value_humidity;
  uint8_t value_fan;
  uint8_t value_time;
  uint8_t value_bulb;
  uint8_t value_fogger;
  uint8_t end_code;
} __attribute__((__packed__));

union frameTX_u {
  uint8_t *bytes;
  struct dataTX *data;
} frameTX;

struct dataRX
{
  uint8_t start_code;
  float receive_temperature;
  float receive_humidity;
  uint8_t receive_fan;
  uint8_t receive_time;
  uint8_t receive_bulb;
  uint8_t receive_fogger;
  uint8_t end_code;
} __attribute__((__packed__));

union frameRX_u {
  uint8_t *bytes;
  struct dataRX *data;
} frameRX;




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
  pinMode(RPMPIN,           INPUT);                 //ustawienie pinu odczytu predkosci wentylatora jako wejscie
  pinMode(BUTTONPIN,        INPUT_PULLUP);          //ustawienie pinu przycisku jako wejscie z rezystorem podciagajacym
  
  hc595(relay);                                     //ustawienie wyjsc rejestru przesuwnego na zadeklarowana wczesniej wartosc
  turnOff_FoggerFan();

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
  currentMillis = millis(); 

  frameTX.bytes = new uint8_t[TX_FRAME_SIZE];
  frameRX.bytes = new uint8_t[RX_FRAME_SIZE];
  frameTX.data->value_temperature = sensor.readTemperature();
  frameTX.data->value_humidity = sensor.readHumidity();
  frameTX.data->value_fan = OCR2B;
  frameTX.data->value_time = tm.Hour;
  frameTX.data->value_bulb = bulb_status;
  frameTX.data->value_fogger = fogger_status;
  frameTX.data->start_code = START_CODE;
  frameTX.data->end_code = END_CODE;
  
}

//*************************************          SHIFT REGISTER          **************************************

void hc595(byte value)                              //funkcja sterowania rejestrem przesuwnym
{
  digitalWrite(LATCHPIN, LOW);                      //zatrzask ustawiony w stan niski (zablokowanie wyjsc rejestru)
  shiftOut(DATAPIN, CLOCKPIN, MSBFIRST , value);    //pojedynczy przesuw bajtow danych, zaczynajac od najbardziej znaczacego
  digitalWrite(LATCHPIN, HIGH);                     //zatrzask ustawiony w stan wysoki (odblokowanie wyjsc rejestru)
}


//*************************************          TRANSMISION          **************************************

void transmision(){
  
  frameTX.data->value_temperature = sensor.readTemperature();
  frameTX.data->value_humidity = sensor.readHumidity();
  frameTX.data->value_fan = OCR2B;
  frameTX.data->value_time = tm.Hour;
  frameTX.data->value_bulb = bulb_status;
  frameTX.data->value_fogger=fogger_status;

//String crc= "1011";                          //kod crc
//String encoded= "00000";                          //zakodowane bity
//int crc_size= sizeof(crc);                  //dlugosc kodu crc
//int encoded_size= sizeof(encoded);          //dlugosc zdekodowanych bitow

  Serial.write(frameTX.bytes, TX_FRAME_SIZE);
  
  if (Serial.available() >= RX_FRAME_SIZE)
  {
   frameRX.bytes[0]= Serial.read();
     if(frameRX.data->start_code == START_CODE ){
      for (byte i = 1; i < RX_FRAME_SIZE; i++)
      {
        frameRX.bytes[i] = Serial.read();
      }
      if(frameRX.data->end_code==END_CODE){

        //encoded += frameRX.bytes[];
            //for(int i=1; i<=crc_size-1; i++){             //dodajemy wyzerowane bity
            //  encoded+='0';  
            //for(int j=0; j<= encoded_size-crc_size; j++ )
                   //while(frameRX.bytes[i]!=0){
                   //}
              //encoded[i+j]=encoded[j]^crc[j];

//String encoded_crc = encoded.substring((encoded_size-_crc_size), encoded_size+1); //wyodrebniony kod crc
//String encoded_frame = frame+encoded_crc;                                         //wyodrebniona ramka z dodanym kodem crc
              
//temp_temperature = frameRX.data->receive_temperature;
//Serial.write(frameRX.bytes, RX_FRAME_SIZE);
      }
    }
  }
//      frameTX.data->value_temperature = frameRX.data->receive_temperature;
//      frameTX.data->value_humidity = frameRX.data->receive_humidity;
//      frameTX.data->value_fan = frameRX.data->receive_fan;
//      frameTX.data->value_time = frameRX.data->receive_time;
//      frameTX.data->value_bulb = frameRX.data->receive_bulb;
//      frameTX.data->value_fogger = frameRX.data->receive_fogger;
  
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
//lcd.print(temp_temperature,1);                      //temperatura do testow transmisji
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

void read_SensorDHT21()                                      //funkcja odczytu danych z czujnika DHT21
{
// TxFrame.values.temperature = sensor.readTemperature();     //przypisz zmiennej odczyt temperatury powietrza
// TxFrame.values.humidity = sensor.readHumidity();           //przypisz zmiennej odczyt wilgotnosci powietrza
 //TxFrame.values.fan = OCR2B;                                //przypisz zmiennej odczyt predkosci wiatraka
 //TxFrame.values.time = tm.Hour;                             //przypisz zmiennej odczyt aktualnej godziny
 //TxFrame.values.bulb = bulb_status;                         //przypisz zmiennej odczyt stanu zarowki
 //TxFrame.values.fogger = fogger_status;                     //przypisz zmiennej odczyt stanu foggera
}

//*************************************          NIGHT LIGHTNING          **************************************

void turnOn_NightLight()                              //funkcja wlaczania oswietlenia nocnego
{                  
  clear_Byte(NIGHTLEDRELAY, relay);                   //zmiana stanu na bicie odpowiedzialnym za oswietlenie nocne
  hc595(relay);                                       //ustawienie rejestru przesuwnego
}

void turnOff_NightLight()                             //funkcja wylaczania oswietlenia nocnego
{                  
  set_Byte(NIGHTLEDRELAY, relay);                     //zmiana stanu na bicie odpowiedzialnym za oswietlenie nocne
  hc595(relay);                                       //ustawienie rejestru przesuwnego
}

void toggle_NightLight()                              //funkcja wlaczania/wylaczania oswietlenia nocnego
{                  
  toggle_Byte(NIGHTLEDRELAY, relay);                  //zmiana stanu na bicie odpowiedzialnym za oswietlenie nocne
  hc595(relay);                                       //ustawienie rejestru przesuwnego
}

void night_lightning()                                //funkcja kontrolowania oswietlenia nocnego
{
  lightsensorvalue = analogRead(LIGHTSENSORPIN);      //odczyt natezenia swiatla z fotorezystora              
  
  if (lightsensorvalue >= lightintensity && digitalRead(PIRPIN) == LOW) {  //jezeli wykryto ruch w ciemnym pomieszczeniu
    if (tm.Hour >= night_hour || tm.Hour < morning_hour){  //jezeli jest noc
      turnOn_NightLight();                             //wlacz oswietlenie nocne
    }
  } 
  else {
    turnOff_NightLight();                              //wylacz oswietlenie nocne
  }
}


//*************************************          BULB DIODE          **************************************

void lightOn_BulbDiode()
{
 digitalWrite(DIODEBULBPIN, HIGH);  
}

void lightOff_BulbDiode()
{
 digitalWrite(DIODEBULBPIN, LOW);  
}

//*************************************          FOGGER DIODE          **************************************

void lightOn_FoggerDiode()
{
 digitalWrite(DIODEFOGGERPIN, HIGH);  
}

void lightOff_FoggerDiode()
{
 digitalWrite(DIODEFOGGERPIN, LOW);  
}

//*************************************          RGB DIODE          **************************************

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

void rgb_Control(int duty_cycle){
  if (duty_cycle <= 30)
  { rgb_TurquoiseColor();
  }

  else if (duty_cycle > 30 && duty_cycle <=60)
  {
    rgb_BlueColor();
  }

  else{
    rgb_PurpleColor();
  }
}

//*************************************          LED STRIPES          **************************************

void led_Fade()                                            //funkcja symulowania zmierzchu za pomoca oswietlenia 
{
 analogWrite(LEDPIN, pulsewidth);                          //generowanie sygnalu o zadanym wypelnieniu
 
 if (pulsewidth > 0) {                                     //jesli wypelnienie wieksze od 0%
      if (currentMillis - previousLedMillis >= ledtime) {  //jezeli uplynal czas zmian oswietlenia
        previousLedMillis = currentMillis;
        pulsewidth = pulsewidth - change;                  //zmniejszamy wypelnienie
      }
 }
 else {
 pulsewidth = 0;                                           //jesli wypelnienie rowne 0% to zatrzymaj zmiany
 }
}

void led_Bright()                                          //funkcja symulowania switu za pomoca oswietlenia
{
 analogWrite(LEDPIN, pulsewidth);                          //generowanie sygnalu o zadanym wypelnieniu
 
 if (pulsewidth < maxledbright) {                          //jesli wypelnienie mniejsze od maksymalnego zadeklarowanego
       if (currentMillis - previousLedMillis >= ledtime) { //jezeli uplynal czas zmian oswietlenia
        previousLedMillis = currentMillis;
        pulsewidth = pulsewidth + change;                   //zwiekszamy wypelnienie
       }
 }       
 else {
 pulsewidth = maxledbright;                                //jesli wypelnienie wieksze od zadeklarowanego to pozostaw max
 }
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

void day_lightning()                                     //funkcja sterujaca oswietleniem dziennym
{
  if(tm.Hour >= morning_hour && tm.Hour < night_hour){
  led_Bright();                                          //wlacz rozjasnianie rano 
  }
  else if(tm.Hour >= night_hour){
  led_Fade() ;                                           //wlacz sciemnianie wieczorem            
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

 
//*************************************          HEAT BULB          **************************************

void lightOn_Bulb()                                   //funkcja wlaczania zarowki grzewczej
{
  clear_Byte(BULBRELAY, relay);                       //stan niski na bicie odpowiedzialnym za zarowke
  lightOn_BulbDiode();                                //wlaczenie diody zarowki
  bulb_status==true;                                  //informacja ze zarowka jest wlaczona
  hc595(relay);
}

void lightOff_Bulb()                                  //funkcja wylaczania zarowki grzewczej
{                  
  set_Byte(BULBRELAY, relay);                         //stan wysoki na bicie odpowiedzialnym za zarowke
  lightOff_BulbDiode();                               //wylaczenie diody zarowki
  bulb_status=false;                                 //informacja ze zarowka jest wylaczona
  hc595(relay);
}

void toggle_Bulb()                                    //funkcja wlaczania/wylaczania zarowki grzewczej
{                  
  toggle_Byte(BULBRELAY, relay);                      //zmiana stanu na bicie odpowiedzialnym za zarowke
  hc595(relay);
}


//*************************************          FOGGER FAN       **************************************

void turnOn_FoggerFan()                                  
{
  set_Byte(FOGGERFANRELAY, relay);                   
  hc595(relay);
}

void turnOff_FoggerFan()                                  
{                  
  clear_Byte(FOGGERFANRELAY, relay);                         
  hc595(relay);
}

void toggle_FoggerFan()                                    
{                  
  toggle_Byte(FOGGERFANRELAY, relay);                      
  hc595(relay);
}

void check_fogger_fan(){

      if (check_status==true){
        if (currentMillis >=  (turnOnFoggerMillis + fandelaytime)) { //jezeli uplynal czas zamglawiania
        turnOn_FoggerFan();                                  //wlaczenie wentylatora
        check_status=false;
        }
      }
}


//*************************************          FOGGER          **************************************


void turnOn_Fogger()                                   //funkcja wlaczania generatora mgly
{
  clear_Byte(FOGGERRELAY, relay);                      //stan niski na bicie odpowiedzialnym za generator mgly
  turnOnFoggerMillis=currentMillis;
  check_status=true;
  fogger_status=true;                                 //informacja ze fogger jest wlaczony
  hc595(relay);
  lightOn_FoggerDiode();                             //wlaczenie diody generatora
}

void turnOff_Fogger()                                  //funkcja wylaczania generatora mgly
{                  
  set_Byte(FOGGERRELAY, relay);                        //stan wysoki na bicie odpowiedzialnym za generator mgly
  turnOnFoggerMillis = millis();
  fogger_status=false;                                //informacja ze fogger jest wylaczony
  turnOff_FoggerFan();                                 //wylaczenie wentylatora 
  lightOff_FoggerDiode();                              //wylaczenie diody generatora
  hc595(relay);
}

void toggle_Fogger()                                    //funkcja wlaczania/wylaczania generatora mgly
{                  
  toggle_Byte(FOGGERRELAY, relay);                      //zmiana stanu na bicie odpowiedzialnym za generator mgly
  hc595(relay);
}

void push_Button()                                      //funkcja wlaczajaca zamglawiacz manualnie za pomoca przycisku
{
  if (digitalRead(BUTTONPIN) == HIGH){                    //jezeli wcisnieto przycisk
    button_status=true;
    turnOn_Fogger();                                      //wlacz zamglawiacz
    turnOnButtonMillis=currentMillis;
  }
}

void check_button(){
  
      if (button_status==true){
        if (currentMillis >=  (turnOnButtonMillis + foggingtime)) { //jezeli uplynal czas zamglawiania
        turnOff_Fogger();                                  //wlaczenie wentylatora
        button_status=false;
        }
      }
}


//*************************************          SERIAL MONITOR          **************************************

//void print_Sensor()                                   //funkcja wyswietlajaca odczyt czujnika DHT21 na monitorze portu szeregowego
//{
//Serial.print(TxFrame.values.temperature);             //wyswietl temperature
//Serial.print(" ");
//Serial.println(TxFrame.values.humidity);              //wyswietl wilgotnosc
//} 

void print_Clock()                                    //funkcja wyswietlajaca czas i date zegara na monitorze portu szeregowego
{
bool parse=false;
bool config=false;
       
if (true /*parse && config*/) {                       //sprawdzenie poprawnosci konfiguracji zegara
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
//  print_Sensor();
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

if (incomingByte == 's') {                             //jezeli wyslano s to podaj odczyty stanów
  Serial.print("P: ");
Serial.println(digitalRead(BUTTONPIN));
 Serial.print("Pir: ");
Serial.println(digitalRead(PIRPIN));
 Serial.print("L: ");
Serial.println(analogRead(LIGHTSENSORPIN));
}

}


void check(){                                           //funkcja sprawdzajaca statusy
  check_button();
  check_fogger_fan();
}

//*************************************       CONTROL ALGORITM      **************************************

void control(){                                                     
                 
float actual_temperature = sensor.readTemperature();     

if(tm.Hour >= morning_hour && tm.Hour < night_hour){
setpoint_temperature = day_temperature;
}
else{
setpoint_temperature = night_temperature;
}

float temperature_difference = actual_temperature - setpoint_temperature;  //roznica temperatury  
float fan_speed = fan_scaler*temperature_difference;                 //predkosc wentylatora                           
                                                                    
if(setpoint_temperature >= actual_temperature){                      //jezeli temperatura nizsza od zadanej
  lightOn_Bulb();                                                    //wlaczenie zarowki grzewczej
}

 if(setpoint_temperature_hysteresis <= actual_temperature){          //histereza
  lightOff_Bulb();                                                   //wylaczenie zarowki grzewczej
  set_Fan_Speed(fan_speed);                                           //ustawienie predkosci wentylatora
}
           
float actual_humidity = sensor.readHumidity();                                                                     
                                                                     
if(setpoint_humidity >= actual_humidity){                            //jezeli wilgotnosc nizsza od zadanej
  turnOn_Fogger();                                                   //wlaczenie generatora mgly
}

 if(setpoint_humidity_hysteresis <= actual_humidity){                //histereza
  turnOff_Fogger();                                                  //wylaczenie generatora
}
}


//*************************************          MAIN LOOP          **************************************

void loop()                                          //petla glowna programu
{
  currentMillis = millis();     
  check();     
  transmision();                      
  RTC.read(tm);
  hc595(relay);
//  read_Serial();
  lcdDisplay();
  rgb_Control(OCR2B);
//  control();
  push_Button(); 
  day_lightning();
  night_lightning();
 // if (Serial.available()>0) read_Serial();
}
