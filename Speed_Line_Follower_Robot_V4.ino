/* Speed Line Follower Robot V4
    ____             __             ____  ______
   / __ \_________ _/ /_____  _____/ __ \/ ____/
  / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
 / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
/_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
                                                
By DrakerDG (c)

https://www.youtube.com/user/DrakerDG
*/

// Number of sensors to use
#define IR 8

// Sensor factor
#define KS 1000  

// Overlap factor
#define OL 200

// Base Speed Motors 
#define SpBSE 140 //140

// Calibration Speed Motors
#define SpBSB 40 //40

// Break Speed Motors
#define SpFWD 140 //140
#define SpREV 190 //190

// PID constants
float Kp = 0.020; // 0.020;
float Ki = 0.001; // 0.001;
float Kd = 0.010; // 0.010;
long P=0, I=0, D=0, PIDv=0, pErr=0;

// Sensor Pins
const byte pSen[IR] = {21, 20, 19, 18, 17, 16, 15, 14};

// LEDs Pins
const byte pLED[3] = {4, 7, 8};
int SenX[IR];
int MinX[IR];
int MaxX[IR];

// Sensor Position
unsigned long PosX = 0;
unsigned long PosM = (IR*KS-KS)/2;
int PosH = 0;

// Detection Line
bool detLe = false;

// Running Status
bool OnRun = false;

// Time Counters
unsigned long Tm0 = 0;
unsigned long Tm1 = 0;

// Motor-A
int IN1 = 5;
int IN2 = 11;

// Motor-B
int IN3 = 10;
int IN4 = 6;

// Speed Motors
int SMoA = 0;
int SMoB = 0;

// Switch Pin
const byte pSW = 13;

void CalSnX(void);
void BlinkX(void);
void EstSnX(void);
void PosLED(void);
void CalPID(void);
void MoCTRL(int x);

void setup(){
// Only to sensors test
//  Serial.begin(9600);
  
// Set all the motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Motor A
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  // Motor B
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Start SW
  pinMode(pSW, INPUT);  
      
  for(byte i=0;i<3;i++){
    pinMode(pLED[i], OUTPUT);
    digitalWrite(pLED[i], LOW);
  }
  delay(1500);
  
  // Calibration Init
  digitalWrite(pLED[1], HIGH);
  CalSnX();
  digitalWrite(pLED[1], LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  // Calibration End
  
  delay(500);  
}

void loop(){
  if(digitalRead(pSW)) OnRun=true;
  EstSnX();
  PosLED();
  CalPID();
  MoCTRL(OnRun);
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
    analogWrite(IN2, SpBSB);
    analogWrite(IN4, SpBSB+10);
  }
}

void BlinkX(){
  for(byte i=0;i<3;i++){
    digitalWrite(pLED[i], !digitalRead(pLED[i]));
  }
}

void EstSnX(){
  detLe = false;
  unsigned long avgS = 0;
  unsigned int sumS = 0;
   
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

/*
  // Only to sensors test //
  char DataX[100];
  char sPosH[6];
  sprintf(sPosH,"%5d", PosH);
  sprintf(DataX,"%4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d", SenX[0], SenX[1], SenX[2], SenX[3], SenX[4], SenX[5], SenX[6], SenX[7], PosX);
  Serial.print(DataX);
  Serial.println(sPosH);
*/
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

void CalPID(){
  P = PosX - PosM;
  I = P + pErr;
  D = P - pErr;
  PIDv = (Kp*P) + (Ki*I) + (Kd*D);
  pErr = P;
}

void MoCTRL(int x){
  int MoSpL = 0;
  int MoSpR = 0;
  if(detLe){
    if(x){
      MoSpL = SpBSE + PIDv;
      MoSpR = SpBSE - PIDv;
    
      MoSpL = constrain(MoSpL, 0, 255);
      MoSpR = constrain(MoSpR, 0, 255);
    
      // Motor A
      analogWrite(IN1, 0);
      analogWrite(IN2, MoSpL);  
      // Motor B
      analogWrite(IN3, MoSpR);
      analogWrite(IN4, 0);          
    }
    else{
      if(P<-OL){
        MoSpL = -PIDv;
        MoSpR = MoSpL;
        // Motor A
        analogWrite(IN1, MoSpL);
        analogWrite(IN2, 0);  
        // Motor B
        analogWrite(IN3, MoSpR);
        analogWrite(IN4, 0);
      }
      else if(P>OL){
        MoSpL = PIDv;
        MoSpR = MoSpL;
        // Motor A
        analogWrite(IN1, 0);
        analogWrite(IN2, MoSpL);  
        // Motor B
        analogWrite(IN3, 0);
        analogWrite(IN4, MoSpR);
      }
      else{
        // Motor A
        analogWrite(IN1, 0);
        analogWrite(IN2, 0);  
        // Motor B
        analogWrite(IN3, 0);
        analogWrite(IN4, 0); 
      }
    }
  }
  else{
    if(P==-PosM){
      MoSpL = SpFWD;
      MoSpR = SpREV;
      // Motor A
      analogWrite(IN1, MoSpL);
      analogWrite(IN2, 0);  
      // Motor B
      analogWrite(IN3, MoSpR);
      analogWrite(IN4, 0);     
    }
    else if(P==PosM){
      MoSpL = SpREV;
      MoSpR = SpFWD;
      // Motor A
      analogWrite(IN1, 0);
      analogWrite(IN2, MoSpL);  
      // Motor B
      analogWrite(IN3, 0);
      analogWrite(IN4, MoSpR);
    }
  }
}
