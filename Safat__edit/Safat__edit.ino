#include <EEPROM.h>
#include <flash_stm32.h>


//Motor Pin
#define LM1 PB4     //here>>portion A is left....portion B is right
#define LM2 PB3
#define RM1 PB7
#define RM2 PB6
#define LMPWM PB9
#define RMPWM PB8
#define STBY PB5

//LED pin
#define lft_led PB15
#define rht_led PA8

//
#define Kp 35
#define Kd 15


int sensor_pin[10]= {PA6, PB1, PB0, PA7, PA4, PA3, PA2, PA0, PA1, PA5};
int sensor[10];
int sum;
boolean d_sensor[10];

int max_value[10]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, min_value[10]={4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096}, avg_value[10];
int avg_value_2[10]={3650, 3650, 3500, 3500, 3400, 3600, 3500, 3700, 3750, 3500 } ;

int avg_value_3[10];
int avg_value_3_f(){
for (int i=0;i<10;i++){
  avg_value_3[i]=avg_value_2[i] -100;
   Serial.println(avg_value_3[i]);
}
}
boolean lft_flag, b_lft, rht_flag, b_rht;
boolean plus, T, LT, RT, U;

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_NONE);
  Serial.begin(9600);
  pin_declare();
  digitalWrite(STBY, HIGH);
//avg_value_3_f();
  delay(1000);
  callibration();
}

void loop() {
// sensor_read();
 d_sensor_read();
  detect_node();
//  turn();
  if(d_sensor[0]==1 || d_sensor[1]==1 || d_sensor[7]==1 || d_sensor[8]==1){
    motor_control(90, 90);
  }
  else{
    PD();
  }
}

void pin_declare(){
//Motor Pin
  pinMode(LM1,OUTPUT);
  pinMode(LM2,OUTPUT);
  pinMode(RM1,OUTPUT);
  pinMode(RM2,OUTPUT);
  pinMode(LMPWM,OUTPUT);
  pinMode(RMPWM,OUTPUT);
  pinMode(STBY,OUTPUT);
//LED Pin
  pinMode(lft_led,OUTPUT);
  pinMode(rht_led,OUTPUT);
}
void d_sensor_read(){
  sum=0;
  for(int i=0; i<10; i++){
    d_sensor[i]= analogRead(sensor_pin[i]) > avg_value[i];
    if(d_sensor[i]){
      sum++;
    }
    Serial.print(d_sensor[i]);
    Serial.print("    ");
  }
  Serial.println();
 // delay(100);

}

void sensor_read(){
  for(int i=0; i<10; i++){
    sensor[i]= analogRead(sensor_pin[i]);
    //d_sensor[i]= analogRead(sensor_pin[i]) > avg_value_2[i];
    Serial.print(sensor[i]);
    Serial.print("    ");
  }
  Serial.println();
  delay(100);

//if (d_sensor[0]==1 || d_sensor[1]==1) digitalWrite(lft_led,1); else digitalWrite(lft_led,0); 
//if (d_sensor[8]==1 || d_sensor[7]==1) digitalWrite(rht_led,1); else digitalWrite(rht_led,0);
}

void motor_control(int lft_speed, int rht_speed){
  if(lft_speed>=0){
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  analogWrite(LMPWM, lft_speed);  
  }
  else{
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  analogWrite(LMPWM, -1*lft_speed);    
  }

  if(rht_speed>=0){
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  analogWrite(RMPWM, rht_speed);  
  }
  else{
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
  analogWrite(RMPWM, -1*rht_speed);    
  }  
}

void callibration(){
  unsigned long time_count=millis()+3000;
  motor_control(150, -150);// eita loop er baire thaka sotteo kaaz kortase kmne??
  while(millis()<time_count){
    for(int i=0; i<10; i++){
      sensor[i]=analogRead(sensor_pin[i]);
      if(sensor[i]<min_value[i]){
        min_value[i]=sensor[i];
      }
      if(sensor[i]>max_value[i]){
        max_value[i]=sensor[i];
      }
    }
  }
      
  time_count=millis()+3000;
  motor_control(-150, 150);
  while(millis()<time_count){
    for(int i=0; i<10; i++){
      sensor[i]=analogRead(sensor_pin[i]);
      if(sensor[i]<min_value[i]){
        min_value[i]=sensor[i];
      }
      if(sensor[i]>max_value[i]){
        max_value[i]=sensor[i];
      }
    }
  }

  motor_control(0, 0);
  delay(100);
  for(int i=0; i<10; i++){
    avg_value[i]= ((min_value[i] + max_value[i])/2) +100 ;
    Serial.print(avg_value[i]);
    Serial.print("    ");
  }
  Serial.println("\n\n");
}

void detect_node(){
  if(d_sensor[0]==1){
    lft_flag=1;
  }
  if(d_sensor[8]==1){
    rht_flag=1;
  }

  if(lft_flag==1 && d_sensor[1]==1){
    b_lft=1;
  }
  if(rht_flag==1 && d_sensor[7]==1){
    b_rht=1;
  }

  if(sum==0){
    motor_control(100, 100);
    delay(30);
    motor_control(0, 0);
    d_sensor_read();
    U=1;
    while(d_sensor[9]==0){
      motor_control(-100, 100);
      d_sensor_read();
    }
    motor_control(-100, 100);
    delay(30);
    motor_control(0, 0);
    delay(100);
    clear_all();
  }

  if(lft_flag==1 && rht_flag==1){
    if(b_lft==1 && b_rht==1){
      if(d_sensor[1]==0 && d_sensor[7]==0){
          motor_control(100, 100);
          delay(30);
          motor_control(0, 0);
//          delay(1000);
        d_sensor_read();
        if(d_sensor[3]==1 || d_sensor[4]==1 || d_sensor[5]==1 || d_sensor[9]==1){
          plus=1;
//          motor_control(0, 0);
          Serial.println("PLUS \n\n");
          digitalWrite(lft_led, HIGH);
          digitalWrite(rht_led, HIGH);
          delay(2000);
          clear_all();
        }
        else{
          T=1;
          motor_control(0, 0);
          Serial.println("TT \n\n");
          unsigned long start_time=millis()+2000;
          while(millis()<start_time){
            digitalWrite(lft_led, HIGH);
            digitalWrite(rht_led, HIGH);
            delay(100);
            digitalWrite(lft_led, LOW);
            digitalWrite(rht_led, LOW);
            delay(100);
          }
//          digitalWrite(lft_led, HIGH);
//          delay(3000);
          clear_all();
        }
      }
    }
  }
  
  else if(lft_flag==1 && rht_flag==0){
    if(b_lft==1 && b_rht==0){
      if(d_sensor[1]==0 && d_sensor[7]==0){
        motor_control(100, 100);
        delay(30);
        motor_control(0, 0);
//        delay(1000);
        d_sensor_read();
        if(d_sensor[9]==1 || d_sensor[3]==1 ||  d_sensor[4]==1 ||  d_sensor[5]==1){
          LT=1;
          Serial.println("LT \n\n");
          digitalWrite(lft_led, HIGH);
//          digitalWrite(rht_led, HIGH);
          delay(2000);
          clear_all();
        }
        else{
          d_sensor_read();
          while(d_sensor[9]==0){
            motor_control(-100, 100);
            d_sensor_read();
          }
          motor_control(-100,100);
          delay(30);
          motor_control(0,0);
          delay(100);
          clear_all();
        }
      }
    }
  }
  else if(lft_flag==0 && rht_flag==1){
    if(b_lft==0 && b_rht==1){ 
      if(d_sensor[1]==0 && d_sensor[7]==0){
      motor_control(100, 100);
      delay(30);
      motor_control(0, 0);
//      delay(1000);
      d_sensor_read();
      if(d_sensor[9]==1 || d_sensor[3]==1 ||  d_sensor[4]==1 ||  d_sensor[5]==1 ){
          RT=1;
          Serial.println("RT \n\n");
//          digitalWrite(lft_led, HIGH);
          digitalWrite(rht_led, HIGH);
          delay(2000);
          clear_all();
        }
        else {
          d_sensor_read();
          while(d_sensor[9]==0){
            motor_control(100, -100);
            d_sensor_read();
          }
          motor_control(100,-100);
          delay(30);
          motor_control(0,0);
          delay(100);
          clear_all();
        }
     }
   }
  }
}

void clear_all(){
  lft_flag=0; rht_flag=0; b_lft=0; b_rht=0;
  plus=0; T=0; LT=0; RT=0; U=0;
  digitalWrite(lft_led, LOW);
  digitalWrite(rht_led, LOW);
}

int koyta_kalo(){
   int count = 0;
   for(int i=2; i<6; i++){
   if (d_sensor[i]) count++;
   }
   return count;
}

float error_count(){
  float b = koyta_kalo();
  float positions = (d_sensor[2]* -2 +d_sensor[3]* -1 +d_sensor[4]* 0 + d_sensor[5]*1+d_sensor[6]*2)/b;
  float error = positions- 0;
  return error;
}

void PD(){
  float error = error_count();
 int left_speed, right_speed;
  int prev_error;
  left_speed = 100 + (Kp * error +Kd*(error-prev_error)) ;
  right_speed = 100 - (Kp * error +Kd*(error-prev_error)) ;
  prev_error=error;
//  if (error<-1){
//    right_speed+=60;
//    left_speed-=60;
//  }
  // Kd*(error-prev_error)
  if (left_speed > 255)left_speed = 250;
  if (right_speed > 255 )right_speed = 250;
  if (left_speed < -255)left_speed = -250;
  if (right_speed < -255)right_speed = -250;
  motor_control(left_speed, right_speed);

  Serial.print(left_speed);
  Serial.print("\t");
  Serial.println(right_speed);
}
