#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

#define PERIOD_SIZE 10    // liczba okreœlaj¹ca maksymaln¹ iloœæ odczytów z sensora któr¹ uznajemy za pomiar
#define MEASURES    10    // liczba pomiarów branych pod uwagê przy obliczaniu têtna
#define SAMPLE_SIZE 4     // liczba pomiarów u¿ywanych do wyg³adzenia krzywej sygna³u
#define RISE        4     // od tej wartoœci zale¿y co interpretujemy jako uderzenie serca 
//(jeœli ta wartoœæ bêdzie za ma³a, to uznawane bêd¹ sygna³y niew³aœciwe, je¿eli za du¿a, to nie bêd¹ uznane wartoœci, które powinniœmy przyj¹æ;
//eksperymentalnie zauwa¿y³em, ¿e wartoœæ 3 dzia³a najlepiej) 

// Piny do silnika krokowego
#define STEPPER_PIN_1 8
#define STEPPER_PIN_2 9 
#define STEPPER_PIN_3 10  
#define STEPPER_PIN_4 11  

bool kierunek = 0;
int step_number = 0;

int sensor = A0;     // pin od fotodiody
int RED = 3;         // pin kontroluj¹cy czerwon¹ diodê LED
int IR = 4;          // pin kontroluj¹cy emiter podczerwieni
int T = 20;          // czas u¿ywany do wyliczenia œredniej wartoœci z sensora (aby zminimalizowaæ b³êdy pomiarowe)

float odczytyIR[SAMPLE_SIZE];   // odczytyIR - tablica ostatnich [SAMPLE_SIZE] odczytów IR
float odczytyRED[SAMPLE_SIZE];
float sumaIR;
float sumaRED;
float vIR;      // vIR - ostateczny wynik pomiaru IR
float vRED;     
float czyt;     // zmienna u¿ywana do zapisu odczytu z sensora
float start;

// tablica odczytów u¿ywana do wyg³adzania krzywej sygna³u
float odczytyIRMM[PERIOD_SIZE], odczytyREDMM[PERIOD_SIZE];

int ptr;
int probki;
int samplesCounter = 0;
int ptrMM = 0;
float maxIR=0;
float minIR=0;
float maxRED=0;
float minRED=0;
double R=0;
float pomiaryR[MEASURES];
int czasyPomiarow[MEASURES];
int pom = 0;
int n;
bool wzrost;
int rise_count;   // do liczenia rosn¹cych pomiarów
long int last_beat;
float prevIR; // od previous IR - zmienna z poprzedni¹ wartoœci¹ IR - przydatne do oceniania czy sygna³ jest rosn¹cy
uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0}; // symbol serduszka
uint8_t eogonek[8] = {0x0,0x0,0xe,0x11,0x1f,0x10,0xe,0x2};  // polski znak "ê"

LiquidCrystal_I2C lcd(0x27, 20, 4);   // wyœwietlacz LCD 

void makeStep(bool dir){
  if(dir){
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    } 
  }
  else{
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
    }
  }
  step_number += 1;
  step_number %= 4;
}

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, heart);
  lcd.createChar(1, eogonek);

  lcd.setCursor(4,1);
  lcd.print("T");lcd.printByte(1);lcd.print("tno:   BPM");  //Têtno
  lcd.setCursor(0,0);
  lcd.print("Saturacja:   %");
  
  Serial.begin(9600);
  Serial.flush();
  pinMode(sensor,INPUT);
  pinMode(RED,OUTPUT);
  pinMode(IR,OUTPUT);
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);

  digitalWrite(RED,LOW);
  digitalWrite(IR,LOW);
  probki=0;
  for (int i = 0; i < PERIOD_SIZE; i++){
    odczytyIRMM[i]  = 0;
    odczytyREDMM[i] = 0;
  }
  for (int i = 0; i < MEASURES; i++) { czasyPomiarow[i]=0; pomiaryR[i]=0; }
   

  for (int i = 0; i < SAMPLE_SIZE; i++) { odczytyIR[i] = 0; odczytyRED[i]=0; }
  sumaIR = 0; sumaRED=0; 
  ptr = 0; 
}

void loop ()
{
  // Gasimy czerwony LED, aby nie wp³ywa³ na odczyty
  digitalWrite(RED,LOW);
  // Uruchamiamy emiter podczerwieni
  digitalWrite(IR,HIGH);
  
  // nale¿y wyliczyæ œredni¹ wartoœæ z odczytów
  // w okresie T w celu zminimalizowania b³êdów pomiarowych
  n = 0;
  start = millis();
  czyt = 0.0;
  do
  {
    czyt += analogRead (sensor); //dodajemy do siebie odczyty z fotodiody
    n++;
  }
  while (millis() < start + T);  
  czyt /= n;  // i dzielimy je przez iloœæ tych odczytów
  
  // Do sumaIR dodajemy powy¿sz¹ wartoœæ
  // i usuwamy najstarszy odczyt w tablicy (o indeksie równym wartoœci ptr)
  // by uzyskaæ sumê ostatnich [SAMPLE_SIZE] odczytów
  sumaIR += czyt;
  sumaIR -= odczytyIR[ptr];
  odczytyIR[ptr] = czyt;
  vIR = sumaIR / SAMPLE_SIZE;   //wyliczamy œredni¹

  // Gasimy emiter podczerwieni ¿eby nie wp³ywa³odczyty
  digitalWrite(IR,LOW);
  // Uruchamiamy czerwony LED i powtarzamy powy¿sze czynnoœci
  digitalWrite(RED,HIGH);

  // nale¿y wyliczyæ œredni¹ wartoœæ z odczytów
  // w okresie T w celu zminimalizowania b³êdów pomiarowych
  n = 0;
  start = millis();
  czyt = 0.;
  do
  {
    czyt += analogRead (sensor); //dodajemy do siebie odczyty z fotodiody
    n++;
  }
  while (millis() < start + T);  
  czyt /= n;  // i dzielimy je przez iloœæ tych odczytów
  
  // Do sumaRED dodajemy powy¿sz¹ wartoœæ
  // i usuwamy najstarszy odczyt w tablicy (o indeksie równym wartoœci ptr)
  // by uzyskaæ sumê ostatnich [SAMPLE_SIZE] odczytów
  sumaRED += czyt;
  sumaRED -= odczytyRED[ptr];
  odczytyRED[ptr] = czyt;
  vRED = sumaRED / SAMPLE_SIZE;   //wyliczamy œredni¹
                                    
  // Obliczanie wspó³czynnika R (potrzebnego do wyliczenia saturacji)
  // zapisujemy uzyskane wy¿ej œrednie
  odczytyIRMM[ptrMM]=vIR;
  odczytyREDMM[ptrMM]=vRED;
  ptrMM++;
  ptrMM %= PERIOD_SIZE;
  samplesCounter++;

  if(samplesCounter>=probki){
    samplesCounter =0;
    maxIR = 0;
    minIR = 1500;
    maxRED = 0; 
    minRED = 1500;
    for(int i=0;i<PERIOD_SIZE;i++) {
      //maksimum
      if( odczytyIRMM[i] > maxIR) 
        maxIR = odczytyIRMM[i];
      if( odczytyREDMM[i] > maxRED) 
        maxRED = odczytyREDMM[i];

      // minimum (wykluczamy wartoœci poni¿ej zera, poniewa¿ takie wartoœci musz¹ 
      // byæ b³êdne i mog³y by spowodowaæ ujemn¹ du¿¹ wartoœæ parametru R)
      if( odczytyIRMM[i] > 0 && odczytyIRMM[i] < minIR )
        minIR = odczytyIRMM[i];
      if( odczytyREDMM[i] > 0 && odczytyREDMM[i] < minRED ) 
        minRED = odczytyREDMM[i];
        
      odczytyIRMM[i] = 0;
      odczytyREDMM[i] = 0;
    }
    R =  ( (maxRED-minRED) / minRED) / ( (maxIR-minIR) / minIR ) ;
  }
  
  int avgBPM=0;      //zmienna na uœrednion¹ wartoœæ pulsu w uderzeniach na minutê 
  
  // sprawdzamy czy wartoœæ wzros³a wzglêdem wczeœniejszej iteracji pêtli (implikuje to uderzenie serca)
  if (vIR > prevIR)
  {
    
    rise_count++; // iloœæ wystêpuj¹cych kolejno po sobie rosn¹cych pomiarów (zmienna porównywana z wartoœci¹ [RISE])
    if (!wzrost && rise_count > RISE)
    {
      lcd.setCursor(3,1);
      lcd.printByte(0);
      
      wzrost = true;    // Ten bool uniemo¿liwia wykrycie jednego uderzenia dwa razy
      
      pomiaryR[pom] = R;  // wartoœæ pom to index obecnego pomiaru
      czasyPomiarow[pom] = millis() - last_beat;  // sprawdzamy ile czasu minê³o od ostatniego pomiaru i zapisujemy go
      
      if(czasyPomiarow[pom] > 200){
        for(int i = 0; i<60; i++){
          makeStep(kierunek);
          delay(5);
        }
        kierunek = !kierunek;
      }
      
      last_beat = millis(); // sprawdzamy obecny czas (aby potem wyliczyæ okres pomiêdzy pomiarami)
      int temp = 0;         // tymczasowa zmienna do obliczenia œredniego okresu miêdzy pomiarami
      
      for(int i =0; i<MEASURES; i++) 
        temp += czasyPomiarow[i];
      
      temp = temp / MEASURES;
      probki = temp / (2*T);  // iloœæ próbek (potrzebna do obliczenia wartoœci parametru R)
        
      int avgPeriod = 0;

      int c = 0;  // zmienna oceniaj¹ca jakoœæ ostatnich MEASURES pomiarów
                  // jeœli nastêpuje têtno przyspieszy³o lub zwolni³o w trakcie 
                  // jednego pomiaru o wiêcej ni¿ 10%, to uznajemy to za b³êdny pomiar
                    
      for(int i =1; i<MEASURES; i++) {
        if ((czasyPomiarow[i] > czasyPomiarow[i-1] / 1.1) && (czasyPomiarow[i] < czasyPomiarow[i-1] * 1.1)){
          c += 1;
          avgPeriod += czasyPomiarow[i];
        }
      }
          
      pom += 1;
      pom %= MEASURES;

      // w celu obliczenia têtna bierzemy pod uwagê conajmniej 5 odczytów
      // i liczymy z nich œredni¹
      avgBPM = 60000 / ( avgPeriod / c) ;
      
      if(c > 3) {
        if(avgBPM < 300){
          if(avgBPM > 99) lcd.setCursor(10,1);
          else{
            lcd.setCursor(10,1);
            lcd.print(" ");   
          }
          lcd.print(String(avgBPM));
        }
        // Saturacja obliczana jest ze wzoru:
        // SpO2 = k*R + m
        // zmienne k i m nale¿y obliczyæ u¿ywaj¹c innego pulsoksymetru
        int SpO2 = -21 * R + 99;
        if(SpO2 > 100) SpO2 = 100;
        
        if(SpO2 < 100) lcd.setCursor(11,0);
        else lcd.setCursor(10,0);
        lcd.print(String(SpO2));
      }
      if(c == 0){
        lcd.setCursor(10,0);
        lcd.print("   ");
        lcd.setCursor(10,1);
        lcd.print("   ");
      }
    }
  }
  else{
    lcd.setCursor(3,1);
    lcd.print(" ");
    wzrost = false;
    rise_count = 0;
  }
  // Przydatne do sprawdzenia nastêpnego pomiaru
  prevIR = vIR;

  // Przeœlij dane do wyjœcia Serial (pozwala min. utworzyæ wykres)
  Serial.print(vIR);
  Serial.print(",");
  Serial.print(vRED);
  Serial.println();

  // zwiêksz wartoœæ indeksu ptr    
  ptr++;
  ptr %= SAMPLE_SIZE;
}