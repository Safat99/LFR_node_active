//KOPAYALA RETURNS
//ROBOMECHATRONICS ASSOCIATION BANGLADESH
//CHITTAGONG UNIVERSITY OF ENGINEERING & TECHNOLOGY

#include <EEPROM.h>
#include <flash_stm32.h>
#include <LiquidCrystal.h>


#define left_basespeed 100    //100   //130
#define right_basespeed 115   //115   //145

//int count=0;
float lfr_kp=4.5, lfr_kd=1.2;   //kp=4.5 & kd=1.2 value for LFR
float maze_kp=0.0, maze_kd=0.0;   //kp & kd value for MAZE
unsigned long int count=0;

int eeprom_address[10];
int sensor[7]={0, 0, 0, 0, 0, 0, 0}, max_value[7]={0, 0, 0, 0, 0, 0, 0}, min_value[7]={4095, 4095, 4095, 4095, 4095, 4095, 4095}, REFF[7]={2000, 2000, 2000, 2000, 2000, 2000, 2000};
int readkey, currentposition=0;
int left_speed=left_basespeed, right_speed=right_basespeed;
int sum=0, error=0, total_error=0, previous_error=0;

char sensor_pin[7]={PA0, PA1, PA2, PA3, PA4, PA5, PA6};
char motor_pin[7]={ PB8, PB3, PB4, PB5, PB6, PB7, PB9};
char lcd_pin[6]={PB12, PB13, PB14, PB15, PA8, PA15};     //RS=PB12   E=PB13    D4=PB14   D5=PB15   D6=PA8    D7=PA15
char button='0';

boolean item[5]={0, 0, 0, 0, 0};
boolean line[2]={0, 0};
boolean ir[7]={0, 0, 0, 0, 0, 0, 0};
boolean STOP=0, right_flag=0, left_flag=0, plus=0, plus1=0, plus2=0, T=0, RF=0, LF=0, R=0, L=0, NODER=0, NODEL=0;

byte positionarrow[8] = {
  B11000, //   **
  B11100, //   ***
  B11110, //   ****
  B11111, //   *****
  B11110, //   ****
  B11100, //   ***
  B11000, //   **
  B00000, //
};
byte uparrow[8] = {
  B00100, //    *
  B01110, //   ***
  B11111, //  *****
  B00100, //    *
  B00100, //    *
  B00100, //    *
  B00100, //    *
  B00000, //
};
byte downarrow[8] = {
  B00100, //    *
  B00100, //    *
  B00100, //    *
  B00100, //    *
  B11111, //  *****
  B01110, //   ***
  B00100, //    *
  B00000, //
};



LiquidCrystal lcd(lcd_pin[0], lcd_pin[1], lcd_pin[2], lcd_pin[3], lcd_pin[4], lcd_pin[5]);



void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  lcd.begin(16, 2);
  lcd.createChar(0, positionarrow);
  lcd.createChar(1, uparrow);
  lcd.createChar(2, downarrow);
  pin_setup();
  delay(200);
//  showlogo();
  menu_selection();
  sensor_read();
  while(!(ir[2]==1 || ir[3]==1 || ir[4]==1)){
    motor(100, 100);
    sensor_read();
  }
  
}

void loop() {
//  digitalWrite(PB0, 1);
//  check_sensor();
  sensor_read();
  node_check();
  TURN();
  LFR_LINE_TRACK();
//  motor(0, 0);
}

void showlogo(){
  lcd.setCursor(0, 0);
  lcd.print("KOPAYALA RETURNS");
  lcd.setCursor(3, 1);
  lcd.print("RMA  CUET");
  delay(2000);
  lcd.clear();
}

void pin_setup(){
  for(int i=1; i<7; i++){
    pinMode(motor_pin[i], OUTPUT);
  }
  pinMode(PB0, OUTPUT);
}

void check_button(){
  while(1){
    readkey=analogRead(PA7);
    if(readkey>3900){
      button='Y';
//      Serial.println(button);
      delay(300);
      break;
    }
    else if(readkey>2800 && readkey<3100){
      button='N';
//      Serial.println(button);
      delay(300);
      break;
    }
    else if(readkey>1800 && readkey<2100){
      button='D';
//      Serial.println(button);
      delay(300);
      break;
    }
    else if(readkey>800 && readkey<1100){
      button='U';
//      Serial.println(button);
      delay(300);
      break;
    }
    else {
      button='0';
    }
  }
}

void menu_selection(){
  mainwindow:
    delay(100);
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("START");
    lcd.setCursor(1, 1);
    lcd.print("SETTINGS");
    lcd.setCursor(0, currentposition);
    lcd.write(byte(0));
    check_button();
    if(button=='Y'){
        line[currentposition]=1;
    }
    else if(button=='U'){
      if(currentposition==1){
        currentposition=0;
        lcd.setCursor(0, 1);
        lcd.write(' ');
        lcd.setCursor(0, 0);
        lcd.write(byte(0));
      }
    }
    else if(button=='D'){
      if(currentposition==0){
        currentposition=1;
        lcd.setCursor(0, 0);
        lcd.write(' ');
        lcd.setCursor(0, 1);
        lcd.write(byte(0));
      }
    }
    if(line[0]==1){
      line[0]=0;
      goto startnow;
    }
    else if(line[1]==1){
      line[1]=0;
      goto submenuwindow0;
    }
    else{
      goto mainwindow;
    }
  submenuwindow0:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("SETTINGS");
    lcd.setCursor(1, 1);
    lcd.print("CALIBRATE");
    lcd.setCursor(0, 1);
    lcd.write(byte(0));
    lcd.setCursor(15, 1);
    lcd.write(byte(2));
    check_button();
    if(button=='Y'){
      item[0]=1;
      goto confirm;
    }
    else if(button=='N'){
      goto mainwindow;
    }
    else if(button=='U'){
      goto submenuwindow0;
    }
    else if(button=='D'){
      goto submenuwindow1;
    }
  submenuwindow1:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("SETTINGS");
    lcd.setCursor(15, 0);
    lcd.write(byte(1));
    lcd.setCursor(1, 1);
    lcd.print("SENSOR READ");
    lcd.setCursor(0, 1);
    lcd.write(byte(0));
    lcd.setCursor(15, 1);
    lcd.write(byte(2));
    check_button();
    if(button=='Y'){
      item[1]=1;
      goto confirm;
    }
    else if(button=='N'){
      goto mainwindow;
    }
    else if(button=='U'){
      goto submenuwindow0;
    }
    else if(button=='D'){
      goto submenuwindow2;
    }
  submenuwindow2:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("SETTINGS");
    lcd.setCursor(15, 0);
    lcd.write(byte(1));
    lcd.setCursor(1, 1);
    lcd.print("REFF VALUE");
    lcd.setCursor(0, 1);
    lcd.write(byte(0));
    lcd.setCursor(15, 1);
    lcd.write(byte(2));
    check_button();
    if(button=='Y'){
      item[2]=1;
      goto confirm;
    }
    else if(button=='N'){
      goto mainwindow;
    }
    else if(button=='U'){
      goto submenuwindow1;
    }
    else if(button=='D'){
      goto submenuwindow3;
    }
  submenuwindow3:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("SETTINGS");
    lcd.setCursor(15, 0);
    lcd.write(byte(1));
    lcd.setCursor(1, 1);
    lcd.print("LFR");
    lcd.setCursor(0, 1);
    lcd.write(byte(0));
    lcd.setCursor(15, 1);
    lcd.write(byte(2));
    check_button();
    if(button=='Y'){
      item[3]=1;
      goto confirm;
    }
    else if(button=='N'){
      goto mainwindow;
    }
    else if(button=='U'){
      goto submenuwindow2;
    }
    else if(button=='D'){
      goto submenuwindow4;
    }
  submenuwindow4:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("SETTINGS");
    lcd.setCursor(15, 0);
    lcd.write(byte(1));
    lcd.setCursor(1, 1);
    lcd.print("MAZE");
    lcd.setCursor(0, 1);
    lcd.write(byte(0));
    check_button();
    if(button=='Y'){
      item[4]=1;
      goto confirm;
    }
    else if(button=='N'){
      goto mainwindow;
    }
    else if(button=='U'){
      goto submenuwindow3;
    }
    else if(button=='D'){
      goto submenuwindow4;
    }
    
  confirm:
    if(item[0]==1){
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("CALIBRATE");
      lcd.setCursor(1, 1);
      lcd.print("CONFIRM: Y/N");
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      check_button();
      if(button=='Y'){
        calibration();
        item[0]=0;
        goto submenuwindow0;
      }
      else if(button=='N'){
        item[0]=0;
        goto submenuwindow0;
      }
      else{
        goto confirm;
      }
    }
    else if(item[1]==1){
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("SENSOR READ");
      lcd.setCursor(1, 1);
      lcd.print("CONFIRM: Y/N");
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      check_button();
      if(button=='Y'){
        lcd_check_sensor();
        item[1]=0;
        goto submenuwindow1;
      }
      else if(button=='N'){
        item[1]=0;
        goto submenuwindow1;
      }
      else{
        goto confirm;
      }
    }
    else if(item[2]==1){
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("REFF VALUE");
      lcd.setCursor(1, 1);
      lcd.print("CONFIRM: Y/N");
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      check_button();
      if(button=='Y'){
        check_reff_value();
        item[2]=0;
        goto submenuwindow2;
      }
      else if(button=='N'){
        item[2]=0;
        goto submenuwindow2;
      }
      else{
        goto confirm;
      }
    }
    else if(item[3]==1){
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("LFR SETUP");
      lcd.setCursor(1, 1);
      lcd.print("CONFIRM: Y/N");
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      check_button();
      if(button=='Y'){
        lfr_setup();
        item[3]=0;
        goto submenuwindow3;
      }
      else if(button=='N'){
        item[3]=0;
        goto submenuwindow3;
      }
      else{
        goto confirm;
      }
    }
    else if(item[4]==1){
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("MAZE SETUP");
      lcd.setCursor(1, 1);
      lcd.print("CONFIRM: Y/N");
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      check_button();
      if(button=='Y'){
        maze_setup();
        item[4]=0;
        goto submenuwindow4;
      }
      else if(button=='N'){
        item[4]=0;
        goto submenuwindow4;
      }
      else{
        goto confirm;
      }
    }

  startnow:
  start_bot();
  check_reff_value();
//  goto mainwindow;
}


void start_bot(){
  for(int i=0; i<5; i++){
//    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("STARTING BOT");
    delay(200);
    lcd.clear();
    delay(100);
  }
}

void lfr_setup(){
  for(int i=0; i<5; i++){
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("SETUP LFR");
    delay(200);
    lcd.clear();
    delay(200);
  }
  lcd.print("LFR SETUP DONE");
  delay(1000);
}

void maze_setup(){
  for(int i=0; i<5; i++){
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("SETUP MAZE");
    delay(200);
    lcd.clear();
    delay(200);
  }
  lcd.print("MAZE SETUP DONE");
  delay(1000);
}


void check_sensor(){
  for(int i=0; i<7; i++){
    Serial.print(analogRead(sensor_pin[i]));
    Serial.print("    ");
  }
  Serial.println();
  delay(200);
}

void lcd_check_sensor(){
  while(1){
    lcd.clear();
    for(int i=0; i<7; i++){
      ir[i]=analogRead(sensor_pin[i])<REFF[i];
      lcd.setCursor(i+4, 0);
      lcd.print(ir[i]);
      delay(20);
    }
    delay(300);
    readkey=analogRead(PA7);
    if(readkey>2800){
      while(analogRead(PA7)>2800){}
      delay(100);
      lcd.clear();
      break;
    }
  }
}

void sensor_read(){
  for(int i=0; i<7; i++){
    ir[i]=analogRead(sensor_pin[i])>REFF[i];
  }
}

void calibration(){
  lcd_cal_start();
  count=millis()+3000;
  while(millis()<=count){
    motor(-left_basespeed, right_basespeed);
    for(int i=0; i<7; i++){
      sensor[i]=analogRead(sensor_pin[i]);
      if(sensor[i]>max_value[i]){
        max_value[i]=sensor[i];
      }
      if(sensor[i]<min_value[i]){
        min_value[i]=sensor[i];
      }
    }
  }
  motor(0, 0);
  delay(50);
  count=millis()+3000;
  while(millis()<=count){
    motor(left_basespeed, -right_basespeed);
    for(int i=0; i<7; i++){
      sensor[i]=analogRead(sensor_pin[i]);
      if(sensor[i]>max_value[i]){
        max_value[i]=sensor[i];
      }
      if(sensor[i]<min_value[i]){
        min_value[i]=sensor[i];
      }
    }
  }
  motor(0, 0);
  delay(50);
  for(int i=0; i<7; i++){
    REFF[i]=(max_value[i]+min_value[i])/2;
  }
  delay(100);
  eeprom_update();
  lcd_cal_stop();
}

void eeprom_update(){
  delay(200);
  for(int i=0; i<7; i++){
    EEPROM.write(eeprom_address[i], REFF[i]);
    delay(50);
  }
  delay(100);
}


void check_reff_value(){
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("READING EEPROM");
  delay(500);
  for(int i=0; i<7; i++){
    REFF[i]=EEPROM.read(eeprom_address[i]);
    Serial.print(REFF[i]);
    Serial.print("\t\t");
    delay(20);
  }
  Serial.println();
  delay(100);
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("READING DONE");
  delay(1000);
  lcd.clear();
}


void lcd_cal_start(){
  lcd.clear();
  for(int i=0; i<5; i++){
  lcd.setCursor(2, 0);
  lcd.print("CALIBRATION");
  lcd.setCursor(6, 1);
  lcd.print("!!!!");
  delay(200);
  lcd.clear();
  delay(100);
  }
  lcd.setCursor(2, 0);
  lcd.print("CALIBRATION");
  lcd.setCursor(4, 1);
  lcd.print("RUNNING");
}

void lcd_cal_stop(){
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("CALIBRATION");
  lcd.setCursor(4, 1);
  lcd.print("DONE");
  delay(2000);
  lcd.clear();
}


void motor(int left_pwm, int right_pwm){
  if(left_pwm==0 && right_pwm==0){
    digitalWrite(motor_pin[3], 0);
  }
  else{
    digitalWrite(motor_pin[3], 1);
  }
  if(left_pwm>=0){
    digitalWrite(motor_pin[5], 1);
    digitalWrite(motor_pin[4], 0);
    analogWrite(motor_pin[6], left_pwm);
  }
  else{
    digitalWrite(motor_pin[5], 0);
    digitalWrite(motor_pin[4], 1);
    analogWrite(motor_pin[6], left_pwm);
  }
  if(right_pwm>=0){
    digitalWrite(motor_pin[1], 1);
    digitalWrite(motor_pin[2], 0);
    analogWrite(motor_pin[0], right_pwm);
  }
  else{
    digitalWrite(motor_pin[1], 0);
    digitalWrite(motor_pin[2], 1);
    analogWrite(motor_pin[0], right_pwm);
  }
}

void LFR_LINE_TRACK(){
  sum= ir[1]+ir[2]+ir[3]+ir[4]+ir[5];
  if(sum){
    error= (ir[1]*-20+ir[2]*-10+ir[3]*0+ir[4]*10+ir[5]*20)/sum;
    total_error= lfr_kp*error + lfr_kd*(error-previous_error);
    previous_error=error;
    if(left_speed>255){
      left_speed=255;
    }
    if(right_speed>255){
      right_speed=255;
    }
    if(total_error==0){
      left_speed=left_basespeed+50;
      right_speed=right_basespeed+50;
    }
    else{
      left_speed=left_basespeed-total_error;
      right_speed=right_basespeed+total_error;
      motor(left_speed, right_speed);
    }
  }
  else{
    motor(left_basespeed, right_basespeed);
  }
}


void node_check(){
  if(ir[1]==1 && ir[2]==1 && ir[3]==1 && ir[4]==1 && ir[5]==1 && ir[6]==1 && ir[0]==1){
    STOP=1;
  }
    else{
          if(ir[1]==1 && ir[3]==1){
            if(NODER==0){
              right_flag=1;
              NODER=1;
              count=millis()+500;
            }
          }
          if(ir[3]==1 && ir[5]==1){
            if(NODEL==0){
              left_flag=1;
              NODEL=1;
              count=millis()+500;
            }
          }
        if(millis()<count){
            if(ir[0]==1 || ir[6]==1){
            if(right_flag==1 && left_flag==1){
              if(ir[2]==1 || ir[3]==1 || ir[4]==1){
                plus=1;
              }
            }
            else if(right_flag==1){
              if(ir[2]==1 || ir[3]==1 || ir[4]==1){
                RF=1;
              }
              else{
                R=1;
              }
            }
            else if(left_flag==1){
              if(ir[2]==1 || ir[3]==1 || ir[4]==1){
                LF=1;
              }
              else{
                L=1;
              }
           }
          } 
        }
        else{
          false_all_node();
        }
  }
}

void false_all_node(){
 STOP=0; right_flag=0; left_flag=0; plus=0; T=0, RF=0; LF=0; R=0; L=0; NODER=0; NODEL=0;
}

void TURN(){
  if(STOP==1){
    motor(-left_basespeed, -right_basespeed);
    delay(50);
    motor(0, 0);
    digitalWrite(PB0, 1);
    delay(200);
    digitalWrite(PB0, 0);
    delay(200);
    digitalWrite(PB0, 1);
    delay(200);
    digitalWrite(PB0, 0);
    delay(200);
    digitalWrite(PB0, 1);
    delay(200);
    digitalWrite(PB0, 0);
    delay(2000);
    false_all_node();
  }
  else if(plus==1){
    if(plus1==0){
      sensor_read(); 
      while(ir[2]==1 || ir[3]==1 || ir[4]==1){
        motor(left_basespeed, -right_basespeed);
        sensor_read();
      }
    motor(left_basespeed, -right_basespeed);
    delay(20);
    sensor_read();
    while(!(ir[2]==0 && ir[3]==1 && ir[4]==0)){
      motor(left_basespeed, -right_basespeed);
      sensor_read();
    }
    motor(-left_basespeed, right_basespeed);
    delay(20);
    motor(0, 0);
    plus1=1;
    false_all_node();
    delay(50);
    }
    else{
      sensor_read(); 
      while(ir[2]==1 || ir[3]==1 || ir[4]==1){
        motor(-left_basespeed, right_basespeed);
        sensor_read();
      }
    motor(-left_basespeed, right_basespeed);
    delay(20);
    sensor_read();
    while(!(ir[2]==0 && ir[3]==1 && ir[4]==0)){
      motor(-left_basespeed, right_basespeed);
      sensor_read();
    }
    motor(left_basespeed, -right_basespeed);
    delay(20);
    motor(0, 0);
    plus1=0;
    false_all_node();
    delay(50);
    }
  }
  else if(RF==1){
    digitalWrite(PB0, 1);
//    motor(left_basespeed, right_basespeed);
    motor(0, 0);
    delay(50);
    sensor_read();
    while(ir[2]==1 || ir[3]==1 || ir[4]==1){
      motor((left_basespeed-40), (right_basespeed-40));
      sensor_read();
    }
    motor(left_basespeed, right_basespeed);
    delay(50);
    sensor_read();
    while(!(ir[2]==0 && ir[3]==1 && ir[4]==0)){
      motor(left_basespeed, -right_basespeed);
      sensor_read();
    }
    motor(-left_basespeed, right_basespeed);
    delay(20);
    motor(0, 0);
    false_all_node();
    delay(50);
    digitalWrite(PB0, 0);
  }
  else if(LF==1){
    digitalWrite(PB0, 1);
//    motor(left_basespeed, right_basespeed);
    motor(0, 0);
    delay(50);
    sensor_read();
    while(ir[2]==1 || ir[3]==1 || ir[4]==1){
      motor((left_basespeed-40), (right_basespeed-40));
      sensor_read();
      }
    motor(left_basespeed, right_basespeed);
    delay(50);
    sensor_read();
    while(!(ir[2]==0 && ir[3]==1 && ir[4]==0)){
      motor(-left_basespeed, right_basespeed);
      sensor_read();
    }
    motor(left_basespeed, -right_basespeed);
    delay(20);
    motor(0, 0);
    false_all_node();
    delay(50);
    digitalWrite(PB0, 0);
  }
  else if(R==1){
//    digitalWrite(PB0, 1);
    motor(left_basespeed, right_basespeed);
    delay(50);
    sensor_read();
    while(!(ir[2]==0 && ir[3]==1 && ir[4]==0)){
      motor(left_basespeed, -right_basespeed);
    sensor_read();
    }
    motor(-left_basespeed, right_basespeed);
    delay(20);
    motor(0, 0);
    false_all_node();
    delay(50);
//    digitalWrite(PB0, 0);
  }
  else if(L==1){
//    digitalWrite(PB0, 1);
    motor(left_basespeed, right_basespeed);
    delay(50);
    sensor_read();
    while(!(ir[2]==0 && ir[3]==1 && ir[4]==0)){
      motor(-left_basespeed, right_basespeed);
      sensor_read();
    }
    motor(left_basespeed, -right_basespeed);
    delay(20);
    motor(0, 0);
    false_all_node();
    delay(50);
//    digitalWrite(PB0, 0);
  }
}
