#define NOT_DRIVEPIN 9
#define DRIVEPIN 6

#define HIGH_IR 3
#define LOW_IR 10

float theta_d, theta, dtheta, err;
int t1, t2;
int tick_count, tick_d;
int highIR, lowIR;

char from_serial[6];
float kp = 4;

void setV(float v){
  if(v<127){
    analogWrite(DRIVEPIN,0);
    analogWrite(NOT_DRIVEPIN,(127-v)*2);
  }else{
    analogWrite(NOT_DRIVEPIN,0);
    analogWrite(DRIVEPIN,(127-v)*2);
  }
}

void refreshTime(){
  t1 = millis();
  if(t1-t2>=20){
    float p = kp*(theta_d-theta);
    setV(p+127);
    t2=t1;
  }
}

void tick(){
  highIR = digitalRead(HIGH_IR);
  lowIR = digitalRead(LOW_IR);
  if(highIR == lowIR)
    tick_count++;
  else
    tick_count--;
}

void setup() {
  Serial.begin(9600);
  pinMode(DRIVEPIN, OUTPUT);
  pinMode(NOT_DRIVEPIN, OUTPUT);
  pinMode(HIGH_IR, INPUT_PULLUP);
  pinMode(LOW_IR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HIGH_IR), tick, CHANGE);
  theta = 0;
  t1 = millis();
}

void loop() {
  while(true){
    if(Serial.available() >= 6){
      for(char i = 0; i < 7; i++){
        from_serial[i] = Serial.read();
      }
      if (from_serial[0] == char(0xFF) && from_serial[1] == char(0xFF)){
        char chk = ((from_serial[2]+from_serial[3]+from_serial[4])%256);
        if(chk == from_serial[5]){
          theta_d = (from_serial[4]*256)+(unsigned char)from_serial[3];
          if(0 == from_serial[2])
            theta_d *= -1;
          Serial.println(theta_d);
        }else{
          Serial.write("checksum err");
        }
      }else{
        Serial.write("header err");
      }
    }
    if(theta_d > 720)
      theta_d = 720;
    if(theta_d <-720)
      theta_d = -720;
    theta = float(tick_count)*1.187;
    refreshTime();
  }

}
