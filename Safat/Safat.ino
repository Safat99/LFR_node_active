//Safat
char c='0';

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pin_setup();
}

void loop() {
  if(Serial1.available()){
    c=Serial1.read();
  }
  if(c!='S'){
    Serial.println(c);
  }
  if(c=='F'){
    motor(1, 1);
  }
  else if(c=='B'){
    motor(-1, -1);
  }
  else if(c=='L'){
    motor(-1, 1);
  }
  else if(c=='R'){
    motor(1, -1);
  }
  else if(c=='G'){
    motor(0, -1);
  }
  else if(c=='I'){
    motor(-1, 0);
  }
  else if(c=='H'){
    motor(0, -1);
  }
  else if(c=='J'){
    motor(-1, 0);
  }
  else{
    motor(0, 0);
  }
}

void pin_setup(){
  for(int i=22; i<=33; i++){
    pinMode(i, OUTPUT);
  }
  for(int i=2; i<=7; i++){
    pinMode(i, OUTPUT);
  }
}

void motor(int left, int right){
  //LEFT MOTOR
  if(left==1){
    for(int i=22; i<=26; i+=2){
      digitalWrite(i, 1);
    }
    for(int i=23; i<=27; i+=2){
      digitalWrite(i, 0);
    }
    for(int i=2; i<=4; i++){
      digitalWrite(i, 1);
    }
  }
  else if(left==-1){
    for(int i=22; i<=26; i+=2){
      digitalWrite(i, 0);
    }
    for(int i=23; i<=27; i+=2){
      digitalWrite(i, 1);
    }
    for(int i=2; i<=4; i++){
      digitalWrite(i, 1);
    }
  }
  else{
    for(int i=22; i<=27; i++){
      digitalWrite(i, 0);
    }
    for(int i=2; i<=4; i++){
      digitalWrite(i, 0);
    }
  }
  //RIGHT MOTOR
  if(left==1){
    for(int i=28; i<=32; i+=2){
      digitalWrite(i, 1);
    }
    for(int i=29; i<=33; i+=2){
      digitalWrite(i, 0);
    }
    for(int i=5; i<=7; i++){
      digitalWrite(i, 1);
    }
  }
  else if(left==-1){
    for(int i=28; i<=32; i+=2){
      digitalWrite(i, 0);
    }
    for(int i=29; i<=33; i+=2){
      digitalWrite(i, 1);
    }
    for(int i=5; i<=7; i++){
      digitalWrite(i, 1);
    }
  }
  else{
    for(int i=28; i<=33; i++){
      digitalWrite(i, 0);
    }
    for(int i=5; i<=7; i++){
      digitalWrite(i, 0);
    }
  }
}

