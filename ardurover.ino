/*
   rover controller
   Arduino = uno, board ARDBOT1C (TB6612 + 5v regulator untuk servo)
   receiver bluetooth di serial RX, 9600 baud
   output motor = 2-3, 5-6, PWM 8 bit timer 2 interrupt
   motor kanan 1 (2-3), kiri 2 (5-6)
   servo steer di pin D8
   LED di 8,9,10,11 active low
   Digital pin
   2  3  4  5  6  7  8  9  10 11  12  13  A0  A1  A2  A3  A4  A5
   AVR port
   D2 D3 D4 D5 D6 D7 B0 B1 B2 B3  B4  B5  C0  C1  C2  C3  C4  C5
                             MOSIMISO SCK                SDA  SCL
*/
#include<EEPROM.h>
#include<Servo.h>
//#define LED1BIT 0x04
//#define LED2BIT 0x08
#define LED1PIN 8
#define LED2PIN 9
#define LED3PIN 10
#define LED4PIN 11
#define STEERPIN 8
#define MAXPWM 128
#define TIMEOUTVAL 200
//pin 2-3, 5-6 di port D
//2-3:
#define M1ABIT 0x02
#define M1BBIT 0x04
//5-6:
#define M2ABIT 0x20
#define M2BBIT 0x40
//port B
#define T1BIT 0x01
#define T2BIT 0x02
#define T3BIT 0x04
#define T4BIT 0x08
//EEPROM
#define MAGIC 0xbd
#define ADR_SETTINGS 0x02

//=================variabel-variabel=================
Servo steer;
volatile long t, t0, tlastcommand;
//data yang dikirim dari remote nilainya antara 0-255, titik netral 128
int xdata, ydata, x2data, y2data;
//nilai power output dari -255 sampai +255
volatile int cpwm = 0, m1pb = 0, m2pb = 0;//, m1pba, m2pba;
uint8_t pbuf, rxbuf[8];
uint8_t newcommandflag = 0;
struct savedvars {
  byte magic;
  float m1f, m2f;
  int steerpin, steerinvert, steermax, steeroffset;
  int drivemode;  //0=tank, 1 = car (ackermann), 2 = dc steer atau synchro drive
  //mode 0: output motor kiri kanan bisa beda, menentukan maju lurus atau belok
  //mode 1: kedua motor outputnya selalu sama, servo steer aktif
  //mode 2: motor 1 untuk maju, motor 2 untuk steer atau putar
} vars1;
enum states {
  AWAL,
  WAITBYTES,
  COMMANDCODE
} state;
void defaultsettings() {
  vars1.magic = MAGIC;
  vars1.m1f = 1;
  vars1.m2f = 1;
  vars1.steerpin = 8;
  vars1.steerinvert = 0;
  vars1.steermax = 45;
  vars1.steeroffset = 0;
  vars1.steerinvert = 0;
  vars1.drivemode = 0;
}
void savesettings() {
  EEPROM.put(ADR_SETTINGS, vars1);
}
void loadsettings() {
  EEPROM.get(ADR_SETTINGS, vars1);
  if (vars1.magic != MAGIC) {
    defaultsettings();
  }
}
inline void led1on() {
  digitalWrite(LED1PIN, HIGH);
  //todo: ganti dengan port bit set/clear
}
inline void led1off() {
  digitalWrite(LED1PIN, LOW);
}
inline void led2on() {
  digitalWrite(LED2PIN, HIGH);
}
inline void led2off() {
  digitalWrite(LED2PIN, LOW);
}
inline void led3on() {
  digitalWrite(LED3PIN, HIGH);
}
inline void led3off() {
  digitalWrite(LED3PIN, LOW);
}
inline void led4on() {
  digitalWrite(LED4PIN, HIGH);
}
inline void led4off() {
  digitalWrite(LED4PIN, LOW);
}
inline void motorenable() {
  DDRD |= (M1ABIT | M1BBIT | M2ABIT | M2BBIT);
}
inline void motordisable() {
  DDRD &= ~(M1ABIT | M1BBIT | M2ABIT | M2BBIT);
}
//motor 1 di port B dan D, motor 2 di port D
inline void m1fwd() {
  PORTD |= M1ABIT;
  PORTD &= ~M1BBIT;
}
inline void m1rev() {
  PORTD |= M1BBIT;
  PORTD &= ~M1ABIT;
}
inline void m1off() {
  PORTD &= ~M1ABIT;
  PORTD &= ~M1BBIT;
}
inline void m2fwd() {
  PORTD |= M2ABIT;
  PORTD &= ~M2BBIT;
}
inline void m2rev() {
  PORTD |= M2BBIT;
  PORTD &= ~M2ABIT;
}
inline void m2off() {
  PORTD &= ~(M2ABIT | M2BBIT);
}

inline void m1intenable() { //interrupt enable
  TIFR2 |= 0x02; //clear flag
  TIMSK2 |= 0x02;
}
inline void m1intdisable() {
  TIMSK2 &= ~0x02;
}
inline void m2intenable() {
  TIFR2 |= 0x04;
  TIMSK2 |= 0x04;
}
inline void m2intdisable() {
  TIMSK2 &= ~0x04;
}
void setpower(int p1, int p2) {
  m1pb = p1;
  m2pb = p2;
  if (m1pb >= 254) m1pb = 255;
  else if (m1pb < -254) m1pb = -255;
  if (m2pb >= 254) m2pb = 255;
  else if (m2pb < -254) m2pb = -255;
}
inline void setupinterrupt() {
  //timer 2
  OCR2A = 0;
  OCR2B = 0;
  TIMSK2 = 0x01; //interrupt overflow
  TCCR2A = 0;
  TCCR2B = 0x03; //prescaler /32 = 500 kHz / 256 = sekitar 2 kHz
}

ISR(TIMER2_OVF_vect) {
  if (m1pb > 0) { //power positif
    m1fwd(); //led1on();
    //digitalWrite(10,LOW);
    if (m1pb < 255) {
      OCR2A = m1pb;
      m1intenable(); //aktifkan PWM
    }
    else {
      m1intdisable();//matikan PWM, nyala nonstop saja
    }
  }
  else if (m1pb < 0) { //power negatif
    m1rev(); //led1on();
    if (m1pb > -255) {
      OCR2A = -m1pb;
      m1intenable();
    }
    else {
      m1intdisable();
    }
  }
  else { // power 0
    m1off(); //led1off();
    m1intdisable();
    OCR2A = 0;
  }
  //motor 2
  if (m2pb > 0) {
    m2fwd(); //led2on();
    if (m2pb < 255) {
      OCR2B = m2pb;
      m2intenable();
    }
    else {
      m2intdisable();
    }
  }
  else if (m2pb < 0) {
    m2rev(); //led2on();
    if (m2pb > -255) {
      OCR2B = -m2pb;
      m2intenable();
    }
    else {
      m2intdisable();
    }
  }
  else { //0
    m2off(); //led2off();
    m2intdisable();
    OCR2B = 0;
  }
}
ISR(TIMER2_COMPA_vect) {
  m1off();
  //digitalWrite(10,HIGH);
}
ISR(TIMER2_COMPB_vect) {
  m2off();
}

void steerto(int a) {
  if (a > vars1.steermax) {
    a = vars1.steermax;
  }
  else if (a < -vars1.steermax) {
    a = -vars1.steermax;
  }
  steer.write(90 + a + vars1.steeroffset);
}
//==========================command handler========================
void getfromto(Stream &S, int &i) {
  i = S.parseInt();
}
void getfromto(Stream &S, float &f) {
  f = S.parseFloat();
}
int bytesExpected = 0;
void prosesdata() {
  //urutan data: magic byte, x,y,x2,y2,b
  float f;
  char c;
  int j;
  switch (state) {
    case AWAL:  //terima magic byte awal
      c = Serial.read();
      //Serial.println(c);
      if (c == MAGIC) {
        //terima 5 byte
        bytesExpected = 5;
        pbuf = 0;
        state = WAITBYTES;
      }
      else if (c == '#') {
        //huruf berikutnya adalah kode apa yang diubah
        state = COMMANDCODE;
      }
      break;
    case WAITBYTES:
      rxbuf[pbuf++] = Serial.read();
      //bytesExpected--;
      if (--bytesExpected == 0) {
        xdata = rxbuf[0];
        ydata = rxbuf[1];
        x2data = rxbuf[2];
        y2data = rxbuf[3];
        newcommandflag = 1;
        state = AWAL;
      }
    case COMMANDCODE:
      c = Serial.read();
      switch (c) {
        case '?':  //help
          Serial.println(F("s=save r=read\nd=drivemode L=leftscale R=rightscale\nP=servopin M=steermax O=steeroffset V=invertsteer\n!=factoryreset"));
          break;
        case 's':  //save settings
          savesettings();
          Serial.println(F("settings saved."));
          break;
        case 'r':
          Serial.print(F("drivemode=")); Serial.println(vars1.drivemode);
          Serial.print(F("Lscale=")); Serial.print(vars1.m1f); Serial.print(F(" Rscale=")); Serial.println(vars1.m2f);
          Serial.print(F("steerpin=")); Serial.print(vars1.steerpin);
          Serial.print(F(" max=")); Serial.print(vars1.steermax);
          Serial.print(F(" offset=")); Serial.print(vars1.steeroffset);
          Serial.print(F(" invert=")); Serial.println(vars1.steerinvert);
          break;

        case 'L':  //set faktor skala motor 1
          getfromto(Serial, vars1.m1f);
          break;
        case 'R':  //set faktor skala motor 2
          getfromto(Serial, vars1.m2f);
          break;
        case 'P':  //servo pin
          getfromto(Serial, vars1.steerpin);
          steer.detach();
          steer.attach(vars1.steerpin);
          steerto(0);
          break;
        case 'M':  //servo max
          getfromto(Serial, vars1.steermax);
          break;
        case 'O':  //servo offset
          getfromto(Serial, vars1.steeroffset);
          steer.write(90 + vars1.steeroffset);
          break;
        case 'V':  //servo invert
          getfromto(Serial, vars1.steerinvert);
          break;
        case 'd': //drive mode
          getfromto(Serial, vars1.drivemode);
          break;
        case '!':  //factory reset
          defaultsettings();
          savesettings();
          //Serial.println(F("Factory reset!!"));
          led2on(); led3on(); led4on();
          delay(100);
          led2off(); led3off(); led3off();
          break;
      }
      state = AWAL;
  }  //tutup switch(state)
}
void prosesoutput() {
  //rentang nilai xdata, ydata, x2data, y2data antara 0-255, titik netral = 128
  int xx = (xdata - 128) * 2; //convert jadi -255 sampai +255
  int xx2 = (x2data - 128) * 2;
  int yy = (ydata - 128) * 2;  //analogwrite maksimumnya 255
  float powerkiri, powerkanan, powermaju;
  int deltamotor;
  if (abs(xx) >= abs(xx2)) {
    deltamotor = xx;
  } else {
    deltamotor = xx2;
  }
  //powermaju = yy;

  if (vars1.drivemode == 1) {  //car
    int steerangle = map(deltamotor, -255, 255, -vars1.steermax, vars1.steermax);
    //Serial.print(" sa=");
    //Serial.print(steerangle);
    if (vars1.steerinvert == 1) {
      steerangle = - steerangle;
    }
    steerto(steerangle);
  }
  if (vars1.drivemode == 0) {  //tank
    powerkiri = (yy + deltamotor) * vars1.m1f;
    powerkanan = (yy - deltamotor) * vars1.m2f;
  } else if (vars1.drivemode == 1) {  //car
    powerkiri = yy * vars1.m1f;
    powerkanan = yy * vars1.m2f;
  } else if (vars1.drivemode == 2) { //synchro drive
    powerkiri = powermaju * vars1.m1f; //satu motor untuk maju
    powerkanan = deltamotor * vars1.m2f; //motor kedua untuk mengubah arah hadap
  }

  if (powerkiri > 255) powerkiri = 255;
  else if (powerkiri < -255) powerkiri = -255;
  if (powerkanan > 255) powerkanan = 255;
  else if (powerkanan < -255) powerkanan = -255;
  setpower(powerkiri, powerkanan);
}
//==============================awal program=======================
void setup() {
  Serial.begin(9600); //bluetooth
  //Serial.begin(57600); //usb arduino leonardo
  loadsettings();
  pinMode(LED1PIN, OUTPUT);
  pinMode(LED2PIN, OUTPUT);
  pinMode(LED3PIN, OUTPUT);
  pinMode(LED4PIN, OUTPUT);
  led1off(); led2off(); led3off(); led4off();
  led1on(); delay(100);
  led2on(); delay(100);
  led3on(); delay(100);
  led4on(); delay(100);
  led1off(); led2off(); led3off(); led4off();
  steer.attach(vars1.steerpin);
  steerto(0);
  //test motor
  setupinterrupt();
  motorenable(); m1off(); m2off();
  setpower(80, 80);
  delay(100);
  setpower(-80, -80);
  delay(100);
  setpower(0, 0);
  delay(100);
  //test servo
  steerto(vars1.steermax);
  delay(200);
  steerto(-vars1.steermax);
  delay(200);
  steerto(0);
}
void loop() {
  t = millis();
  while (Serial.available()) {
    prosesdata();
  }
  if (newcommandflag) {
    tlastcommand = t;
    newcommandflag = 0;
    //ubah x, y, x2, y2 jadi m1power dan m2power
    prosesoutput();
  }
  if (t - tlastcommand > TIMEOUTVAL) {
    setpower(0, 0);
  }
}
