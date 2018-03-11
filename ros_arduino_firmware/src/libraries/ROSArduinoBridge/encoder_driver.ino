#include "motor_driver.h"
#include "commands.h"

/* encode */
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
int left_rotate = 0;
int right_rotate = 0;

void initEncoders(){
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(19, INPUT);
  pinMode(18, INPUT);
  attachInterrupt(0, encoderLeftISR, CHANGE);  
  attachInterrupt(1, encoderLeftISR,  CHANGE);  
  attachInterrupt(4, encoderRightISR, CHANGE);
  attachInterrupt(5, encoderRightISR, CHANGE);
}

void encoderLeftISR(){
    if(direction(LEFT) == BACKWARDS){
        left_enc_pos--;
    }else{
        left_enc_pos++;
    }
}

void encoderRightISR(){
    if(direction(RIGHT) == BACKWARDS){
      right_enc_pos--;
    }else{
      right_enc_pos++;
    }
}

  long readEncoder(int i) {
      long encVal = 0L;
  if (i == LEFT)  {
    noInterrupts();
    encVal = left_enc_pos;
    interrupts();
  }
  else {
    noInterrupts();
    encVal = right_enc_pos;
    interrupts();
  }
  return encVal;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      noInterrupts();
      left_enc_pos=0L;
      interrupts();
      return;
    } else { 
      noInterrupts();
      right_enc_pos=0L;
      interrupts();
      return;
    }
  }

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}



