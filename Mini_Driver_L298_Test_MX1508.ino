/*  
Mini Driver L298 Test MX1508
with joystick

By DrakerDG

https://www.youtube.com/user/DrakerDG
*/
  
// Motor-A

int IN1 = 5;
int IN2 = 11;

// Motor-B

int IN3 = 10;
int IN4 = 6;

// Pot Input

int Jk1 = A0;
int Jk2 = A1;

int SMoA = 0;
int SMoB = 0;

int eJk1 = 0;
int eJk2 = 0;

int JkV;
int JkH;

void readPo(void);
void ctrlMo(void);

void setup()

{

  Serial.begin(9600);
  
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
  
}

void loop() {
  readPo();
  
  ctrlMo();
  Serial.print("JkV: ");
  Serial.print(eJk1);
  Serial.print("  JkH: ");
  Serial.println(eJk2);
  delay(100);
}


void readPo(){
  JkV = analogRead(Jk1);
  if(JkV>612) eJk1 = 1;
  else if(JkV<412) eJk1 = -1;
  else eJk1 = 0;
  JkH = analogRead(Jk2);
  if(JkH>632) eJk2 = 1;
  else if(JkH<392) eJk2 = -1;
  else eJk2 = 0;
}

void ctrlMo(){
  if(eJk1==1){
    SMoA = map(JkV,0,612,0,255);
    SMoB = SMoA;
    if(eJk2==1) SMoB=0;
    else if(eJk2==-1) SMoA=0;
    // Motor A
    analogWrite(IN1, SMoA);
    analogWrite(IN2, 0);  
    // Motor B
    analogWrite(IN3, 0);
    analogWrite(IN4, SMoB);
  }
  else if(eJk1==-1){
    SMoA = map(JkV,0,412,255,0);
    SMoB = SMoA;
    if(eJk2==1) SMoB=0;
    else if(eJk2==-1) SMoA=0;
    // Motor A
    analogWrite(IN1, 0);
    analogWrite(IN2, SMoA);  
    // Motor B
    analogWrite(IN3, SMoB);
    analogWrite(IN4, 0);
  }
  else {
    // Motor A
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);  
    // Motor B
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}