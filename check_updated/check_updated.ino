int sensor_pin[7] = { PB1,PB0,PA7,PA4,PA3,PA2,PA0 };
int sensor[7]; 
#define top_left PA6
#define top_middle PA5
#define top_right PA1

int t_lft,t_mdl,b_lft,b_rgt,t_rgt;
bool h_t_lft,h_t_rgt,h_b_lft,h_b_rgt;

#define LM2 PB4//here>>portion A is left....portion B is right
#define LM1 PB3
#define RM2 PB7
#define RM1 PB6
#define LMPWM PB9
#define RMPWM PB8
#define STBY PB5
#define Kp 40//33.33>>5 sensor e 40 kp aar kd 130 motamuti khushi
#define Kd 130//100
#define lft_led PB15
#define rgt_led PA8

void  analogSerialRead();
void dS_Read();
void SensorRead();
int koyta_kalo();
float  PD();
void  motor_control();

void setup()
{
   afio_cfg_debug_ports(AFIO_DEBUG_NONE);
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(LM1,OUTPUT);
  pinMode(LM2,OUTPUT);
  pinMode(RM1,OUTPUT);
  pinMode(RM2,OUTPUT);
  pinMode(LMPWM,OUTPUT);
  pinMode(RMPWM,OUTPUT);
  pinMode(STBY,OUTPUT);
  //pinMode(PC13,OUTPUT);
  }
void speed_control(int lft, int right)
{
  if (lft >= 0)
        {
        digitalWrite(LM1, HIGH);
        digitalWrite(LM2, LOW);
        analogWrite(LMPWM, lft);
        }
  else 
      {     
        digitalWrite(LM2, HIGH);
        digitalWrite(LM1, LOW);
        analogWrite(LMPWM, -1*lft);  
        }
  if (right >= 0)
           {
             digitalWrite(RM1, HIGH);
             digitalWrite(RM2, LOW);
             analogWrite(RMPWM, right);
            }
                
  else {
          digitalWrite(RM2, HIGH);
          digitalWrite(RM1, LOW);
          analogWrite(RMPWM, -1*right);
        }
}
void stopp()
{
  speed_control(0,0);
}

void analogSerialRead()
{
  int a,b,c;
  a=analogRead(top_left);
  b=analogRead(top_middle);
  c=analogRead(top_right);
  Serial.print(a);
  Serial.print("\t\t\t");
  Serial.print(b);
  Serial.print("\t\t\t");
  Serial.print(c);
  Serial.println();
for(int i=0;i<7;i++)
   {
   sensor[i]= analogRead(sensor_pin[i]);
  Serial.print(sensor[i]);
  Serial.print("\t");    
  
}
Serial.println(" ");
}

void dS_Read()
{
int a,b,c;
  a=analogRead(top_left);
  b=analogRead(top_middle);
  c=analogRead(top_right);
    if ( a<3300 ) a=0; else a=1;
  if ( b<3300 ) b=0; else b=1;
  if ( c<3300 ) c=0; else c=1;
  Serial.print(a);
  Serial.print("  ");
  Serial.print(b);
  Serial.print("  ");
  Serial.print(c);
  Serial.println();
      
 for(int i=0;i<7;i++)
   {
    sensor[i]=analogRead(sensor_pin[i]);

   if (sensor[i]<= 3000)  sensor[i]=0;
  else sensor[i]=1;
 
//  Serial.print("Sensor[");
//  Serial.print(i);
//  Serial.print("]=");
  Serial.print(sensor[i]);
  //Serial.print("\t");
  }
  Serial.println();
 
}
 
  
void SensorRead()
{
   int a,b,c,d,e;   
  a=analogRead(top_left);
  b=analogRead(top_middle);
  c=analogRead(top_right);
  if ( a<3300 ) a=0; else a=1;
  if ( b<2800 ) b=0; else b=1;
  if ( c<3300 ) c=0; else c=1;
d=analogRead(sensor_pin[0]);
e=analogRead(sensor_pin[6]);
   if ( d<3200 ) d=0; else d=1;
   if ( e<3200 ) e=0; else e=1;
   
t_lft=a;
t_mdl=b;
t_rgt=c;
b_lft=d;
b_rgt=e; 
   
   for(int i=1;i<6;i++)
   {
    sensor[i]=analogRead(sensor_pin[i]);

   if (sensor[i]<= 3000)  sensor[i]=0;
   else sensor[i]=1;
  } 

}


int koyta_kalo()
{
  int count = 0; 
   for( int i=1; i<6; i++)
   {
    sensor[i]= analogRead(sensor_pin[i]);
  
    if (sensor[i]<= 3200)  
      sensor[i]=0;
    else 
      sensor[i]=1;
     if (sensor[i]==1) count++;   
      Serial.print(sensor[i]);
   }
    
   Serial.print("\t");
 
      Serial.print(count);
    Serial.println(" ");
   
  return count;
}


  float PD() 
{
  
  float b = koyta_kalo();
  if (b==0) return 99;
//  if ((b==7) && top_middle==1) return 100;
  float positions = (   sensor[1] * (-2) + sensor[2] * (-1) + sensor[3] * 0 + sensor[4] * 1 + sensor[5] *2 ) / b;
  float error = positions - 0;
  int P = error;
  Serial.println(error);
  return error;
}

void motor_control()
{
SensorRead();
// int a,b,c,d,e;   
//  a=analogRead(top_left);
//  b=analogRead(top_middle);
//  c=analogRead(top_right);
//  if ( a<3300 ) a=0; else a=1;
//  if ( b<2800 ) b=0; else b=1;
//  if ( c<3300 ) c=0; else c=1;
//d=analogRead(sensor_pin[0]);
//e=analogRead(sensor_pin[6]);
//   if ( d<3200 ) d=0; else d=1;
//   if ( e<3200 ) e=0; else e=1;
//   
//t_lft=a;
//t_mdl=b;
//t_rgt=c;
//b_lft=d;
//b_rgt=e; 

     if (t_lft==1) h_t_lft=1;
     if (t_rgt==1) h_t_rgt=1;
     if (b_lft==1) h_b_lft=1;
     if (b_rgt==1) h_b_rgt=1;

if (( (h_t_lft==1 && h_b_lft == 1)  || (h_t_rgt==1 && h_b_rgt==1)) && ( t_lft==0 && b_lft == 0  && t_rgt==0 && b_rgt==0 ) ) 
{
 speed_control(-50,-50);
 delay(50);
 stopp();
 delay(1000);
 SensorRead();delay(1000);
 if (h_t_lft==1 && h_b_lft==1 && h_t_rgt==1 && h_b_rgt==1)
    {
      if (t_mdl==1 || sensor[2]==1 || sensor[3]==1 || sensor[4]==1)
      { 
        Serial.println("Plus"); digitalWrite(lft_led,HIGH);delay(50);
      }
      else{
        Serial.println("TTTTT");digitalWrite(rgt_led,HIGH);delay(50);
      }
    }
    h_t_lft=0;  h_b_lft=0; h_t_rgt=0; h_b_rgt=0;
  }

  float error = PD();
  if (error == 99) 
  {
   int x= koyta_kalo();
    while(x==0)
         { stopp();
            x=koyta_kalo();
         }
  }  
  
  int left_speed, right_speed;
  int prev_error;
  left_speed = 130 + (Kp * error +Kd*(error-prev_error)) ;
  right_speed = 130 - (Kp * error +Kd*(error-prev_error)) ;
  prev_error=error;
  // Kd*(error-prev_error)
  if (left_speed > 255)left_speed = 250;
  if (right_speed > 255 )right_speed = 250;
  if (left_speed < -255)left_speed = -250;
  if (right_speed < -255)right_speed = -250;
  speed_control(left_speed, right_speed);

  Serial.print(left_speed);
  Serial.print("\t");
  Serial.println(right_speed);
}

void loop()
{
 // if (Serial1.available() > 0)  
//Serial1.println("hii");
  digitalWrite(STBY,HIGH);
// analogSerialRead();
 //dS_Read();digitalWrite(rgt_led,HIGH);delay(50);
// SensorRead();
  //PD();
  motor_control();
 //koyta_kalo();
//speed_control(-200,-200);
//delay(50);
  }
