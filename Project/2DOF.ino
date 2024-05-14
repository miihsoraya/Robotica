/*
  Cinematica Inversa em 2DOF
*/

#include <Servo.h>
#include <math.h>

Servo servo_9;  
Servo servo_10;


bool exec = 1; 
float l1 = 6, l2 = 6;	//juntas
float x =-12, y =0;	//localização desejada
float theta_1,theta_2, alf_1, alf_2, bet_3, L=0;

void math(){
  
  L = sqrt(pow(x,2)+pow(y,2));	
  alf_1 = acos((pow(l2,2)-pow(l1,2)-pow(L,2))/(-2*l1*L)); //lei dos cossenos -- ALPHA
  alf_2 = atan2(y, x); //medida do ag em rad  //pi

  theta_1 = degrees(alf_2-alf_1) ; //conversao ang p radianos
  
  bet_3 = acos((pow(L,2)-pow(l1,2)-pow(l2,2))/(-2*l1*l2));	//Beta
  theta_2 = degrees(bet_3);
  

  theta_2 = 270 - theta_2;	//com base na localização dos servos
}

void setup() {
  Serial.begin(9600);
  servo_9.attach(9, 500, 2500);
  servo_10.attach(10, 500, 2500);
  
}
  
void loop() {
  math();
   
  if(exec){
    if(isnan(theta_2) || (y<0)){
      Serial.println("-----LIMITE EXCEDIDO------");
      exec=false;	
    
    }else{ 
      delay(900);
      servo_9.write(theta_1);
      delay(3000); 
      servo_10.write(theta_2);
      exec = false;
      
      Serial.println("tetha1------------------------");
      Serial.println(theta_1);
      Serial.println("tetha2------------------------");
      Serial.println(theta_2);
    }   
  }  
}

//https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
