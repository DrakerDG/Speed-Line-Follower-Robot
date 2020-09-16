/* 8xCNY70 Sensor Array Test Code
As Analog Signal

By DrakerDG (c)

https://www.youtube.com/user/DrakerDG
*/

// Number of sensors to use
#define IR 8

// Sensor factor
#define KS 1000
  
// Overlap factor
#define OL 200

const byte pSen[IR] = {21, 20, 19, 18, 17, 16, 15, 14};

// LEDs Pins
const byte pLED[3] = {4, 3, 2};
int SenX[IR];
int MinX[IR];
int MaxX[IR];
unsigned long PosX = 0;
unsigned long PosM = (IR*KS-KS)/2;
int PosH = 0;
bool detLe = false;

unsigned long Tm0 = 0;
unsigned long Tm1 = 0;

void CalSnX(void);
void BlinkX(void);
void EstSnX(void);
void PosLED(void);

void setup(){
  Serial.begin(9600);
  
  for(byte i=0;i<3;i++){
    pinMode(pLED[i], OUTPUT);
    digitalWrite(pLED[i], LOW);
  }
  delay(1500); 
  // Calibration Init
  digitalWrite(pLED[1], HIGH);
  CalSnX();
  digitalWrite(pLED[1], LOW);
  // Calibration End
  delay(500);  
}

void loop(){
  EstSnX();
  PosLED();
}

void CalSnX(){
  Tm0 = millis();
  Tm1 = Tm0;
  unsigned long TmL;
  for(byte i=0; i<IR; i++){
    SenX[i]=analogRead(pSen[i]);
    MinX[i]=SenX[i];
    MaxX[i]=SenX[i];
  }
  while((millis()-Tm0)<=10000){
    for(byte i=0; i<IR; i++){
      SenX[i]=analogRead(pSen[i]);
      if(SenX[i]<MinX[i]) MinX[i]=SenX[i];
      if(SenX[i]>MaxX[i]) MaxX[i]=SenX[i];
    }
    TmL = millis();
    if ((TmL-Tm1)>=100){
      BlinkX();
      Tm1 = TmL;
    }
  }
/*  
  for(byte i=0; i<IR; i++){
    Serial.print(MinX[i]);
    Serial.print("  ");
  }
  Serial.println();
  for(byte i=0; i<IR; i++){
    Serial.print(MaxX[i]);
    Serial.print("  ");
  }
*/
}

void BlinkX(){
  for(byte i=0;i<3;i++){
    digitalWrite(pLED[i], !digitalRead(pLED[i]));
  }
}

void EstSnX(){
  unsigned long Tm1 = millis();
  if ((Tm1-Tm0)>50){
    detLe = false;
    unsigned long avgS = 0;
    unsigned int sumS = 0;
    
    char DataX[100];
    char sPosH[6];
   
    for(byte i=0; i<IR; i++){
      SenX[i] = analogRead(pSen[i]);
      SenX[i] = map(SenX[i], MinX[i], MaxX[i], KS, 0);
      SenX[i] = constrain(SenX[i], 0, KS);
      if(SenX[i]>200)detLe = true;
      if(SenX[i]>50){
        avgS += (long)SenX[i]*(i*KS);
        sumS += SenX[i];
      }
    }
    if(detLe)PosX = avgS/sumS;
    else if(PosX < PosM) PosX = 0;
    else PosX = PosM*2;
    PosH = PosX-PosM;
  
    sprintf(sPosH,"%5d", PosH);
    sprintf(DataX,"%4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d", SenX[0], SenX[1], SenX[2], SenX[3], SenX[4], SenX[5], SenX[6], SenX[7], PosX);
    Serial.print(DataX);
    Serial.println(sPosH);
    Tm0 = Tm1;
  }
}

void PosLED(){
  unsigned long TmL = millis();
  if((PosX>(PosM-KS/2))&&(PosX<(PosM+KS/2))) digitalWrite(pLED[1], HIGH);
  else digitalWrite(pLED[1], LOW);
  
  if(detLe){
    if(PosX<(PosM-OL)) digitalWrite(pLED[0], HIGH);
    else digitalWrite(pLED[0], LOW);
    if(PosX>(PosM+OL)) digitalWrite(pLED[2], HIGH);
    else digitalWrite(pLED[2], LOW);
  }
  else{
    if((PosX<(PosM-OL))&&((TmL-Tm1)>100)){
      digitalWrite(pLED[0], !digitalRead(pLED[0]));
      Tm1 = TmL;
    }
    if((PosX>(PosM+OL))&&((TmL-Tm1)>100)){
      digitalWrite(pLED[2], !digitalRead(pLED[2]));
      Tm1 = TmL;
    } 
  }
}
