#include <Arduino.h>
#include "MillisTimer.h"
#include "generateMove.h"

// Control for Cliffhanger game.  Made by David Parker nlightn0@gmail.com 04/01/2022

/*
*******************************Overview of button behaviors*************************************
1)Start button: Random goat move 1-12 spaces along the track. Each move different. 
Beginning yodel to start, then Traveling music track plays during movement. When the figure stops, a
bell “DING”
2)Go directly to space #24. Beginning yodel to start, then plays music while it travels and 
when the figure stops a bell “Ding”
3)Manual run. Beginning yodel to start, character moves until host takes finger off button then the stop DING
4)Sound track win..play sound ding ding ding ding and winning music
5)Sound track lose…loosing sound, and music
6)Idle music.
7)Resets game to start position moves goat to starting position. No sound.
********************************Electromagnetic triggers***************************************
An electromagnet creates a “start of game” sound when he crosses position one.
An electromagnet at the top of the mountain would trigger a sound effect - falling yodel sound
Perhaps another electromagnet could trigger red flashing lighting 
    and a sound effect that warns that the goat is going to fall off when it passes.
*********************************Pinouts for Sound Board***************************************
T01HOLDL.ogg                            Travel Music
T02.ogg                                 Losing (Falling Yodel) Sound
T03.ogg                                 Winning Sound
T04.ogg                                 1 Ding
T05.ogg                                 Danger Sound
T06HOLDL.ogg                            Idle Music
T07.ogg                                 Buzz
T08.ogg                                 Reset Game Music
*/
/*Serial1 pins 19(RX), 18(TX)*/

const int stepperDelay = 2000;// 4*1000 (DEFAULT)
const int resetStepperDelay = 400;
const int resetTimeout = 15000;
const uint32_t stepsPerInterval = 284;  //7100/25
//const uint32_t stepsPerInterval = 284;  //7100/25
//steps to end: 7096
//steps to danger: 6332
#define MOVE_24 24
//const uint32_t stepsPerInterval = 254; //6100/24
//********************************SENSORS AND MOTOR CONTROL***********************************
const int resetPin  = 24;                    //Start of game location
const int dangerLocationPin = 25;            //Play Danger Sound
const int fallPin = 26;                      //Play Lose Sound
const int stepPin = 22;                      //Stepper Motor Control
const int dirPin  = 23;
const int stepper_enable_pin = 3;
const int max845_enable = 2;

typedef enum stepper_direction_t {
  FORWARD = 1,
  REVERSE = -1
} stepper_direction_t;

//************************CUSTOM TYPES FOR MESSAGING AND COMMUNICATION
typedef enum message_status_t {
  ACK = 0xAA,
  NACK = 0xCC
} message_status_t;

typedef enum message_send_state_t {
  TRANSMIT,
  RECEIVE
} message_send_state_t;

typedef enum lectern_state_t {
  RESET = 0,
  RANDOM_MOVE = 1,
  SPACE24 = 2,
  MANUAL = 3,
  NONE = 4,
  NO_MESSAGE = 5,
  MOVE_1 = 6
} lectern_state_t;

typedef struct master_message_t {
  uint8_t startByte;
  lectern_state_t currentState;
  uint8_t endByte;
} master_message_t;

typedef struct response_message_t {
  uint8_t startByte;
  uint8_t warningState;
  uint8_t endState;
  uint8_t travelState;
  message_status_t messageStatus;
  uint8_t endByte;
} response_message_t;


uint8_t startByte = 0x81;
uint8_t endByte = 0x7E;
lectern_state_t newState = NONE;
lectern_state_t currentState = NONE;
lectern_state_t previousState = NONE;

message_send_state_t messageSendState = RECEIVE;
message_status_t messageStatus = NACK;


MillisTimer messageReceiveTimer = {500};
MillisTimer messageErrorIndicatorTimer = {1000};

bool ledState = false;
bool errorFlag = false;
bool messageAck = false;

bool atStart = false;
bool moveToStart = false;
bool moveDistance = false;
bool jogForward = false;
bool isTravelling = false;

bool stepperEnabled = false;
bool resetEnabled = true;
int distance = 0;
uint32_t currentPosition = 0;
uint32_t newPosition = 0;
uint8_t warningState = 0;
uint8_t endState = 0;

void setup() {
  init_rail_inputs();
  init_rail_outputs();
  
  Serial.begin(115200);
  Serial1.begin(115200);
  randomSeed(analogRead(A0));
 
  //timeBetweenGates();
  //simulateGame();
  //testOutcomes();

}


void timeBetweenGates() {
  bool travel = true;
  unsigned long startTime, endTime;

  while(travel) {
    takeStep(REVERSE, resetStepperDelay);
    if(digitalRead(resetPin) == LOW) {
      travel = false;
      startTime = millis();
    }
  }
  travel = true;
  int count = 0;
  while(travel) {
    takeStep(FORWARD, stepperDelay);
    count++;
    if(digitalRead(dangerLocationPin) == LOW) {
      travel = false;
      endTime = millis();
    }
  }
  float interval1 = (endTime - startTime)/1000;
  int count1 = count;

  travel = true;
  count = 0;
  startTime = endTime;
  while(travel) {
    takeStep(FORWARD, stepperDelay);
    count++;
    if(digitalRead(fallPin) == LOW) {
      travel = false;
      endTime = millis();
    }
  }
  Serial.println();
  Serial.println("Start to danger:");
  Serial.print("Time: ");
  Serial.print(interval1);
  Serial.print("s steps: ");
  Serial.println(count1);
  Serial.print(" spaces between start and danger: ");
  Serial.print(count1/(float)stepsPerInterval);
  Serial.println();
  Serial.println("danger to fall:");
  Serial.print("Time: ");
  Serial.print((endTime - startTime)/1000);
  Serial.print("s steps: ");
  Serial.println(count);
  Serial.print("spaces between danger and fall: ");
  Serial.print(count/(float)stepsPerInterval);
  Serial.println();
  delay(6000);
  travel = true;
  while(travel) {
    takeStep(REVERSE, resetStepperDelay);
    if(digitalRead(resetPin) == LOW) {
      travel = false;
      startTime = millis();
    }
  }
  
}


void loop() {
  read_inputs();
  handle_messages();
  display_error();
  handle_states();
  move_stepper();
}

/*
  RESET,
  RANDOM_MOVE,
  SPACE24,
  MANUAL,
  NONE,
  MOVE_1
*/
uint32_t getNewPosition(uint32_t start, uint32_t distance) {
  return start + distance * stepsPerInterval;
}

void move_stepper() {
  static MillisTimer resetTimer = {resetTimeout};
  static MillisTimer resetEnableTimer = {resetTimeout + 5000};
  static bool movingToStart = false;
  if(jogForward) {
    takeStep(FORWARD, stepperDelay);
    currentPosition++;
  } else if (moveDistance) {
    if(currentPosition < newPosition) {
      takeStep(FORWARD, stepperDelay);
      currentPosition++;
    } else {
      moveDistance = false;
      Serial.println();
      Serial.print("current position: ");
      Serial.println(currentPosition);
    }
  } else if (moveToStart) {
    if(!movingToStart) {
      movingToStart = true;
      resetTimer.reset();
      resetEnableTimer.reset();
      Serial.print("resetting from position: ");
      Serial.println(currentPosition);
    } else {
      if(atStart || resetTimer.timeUp()) {
        movingToStart = false;
        currentPosition = 0;
        moveToStart = false;
        newState = NONE;
        //clear buffer
        Serial.print("current position: ");
        Serial.println(currentPosition);
      } else {
        takeStep(FORWARD, resetStepperDelay);
        currentPosition++;
      }
    }




    while(Serial1.available()) {
      Serial1.read();
    }

  }
  if(resetEnableTimer.timeUp() && !resetEnabled) {
    resetEnabled = true;
  }
}

uint8_t getRandomDistance() {
  static uint8_t lastDistance = -1;
  uint8_t newDistance = -1;
  do {
    newDistance = random(1,13);
    //loop u    
  } while(newDistance != lastDistance && newDistance == 12);
  Serial.print("new distance: ");
  Serial.println(newDistance);
  return newDistance;
}

void handle_states() {
  switch(newState) {
    case RESET:
      if(resetEnabled){
        moveToStart = true;
        Serial.println("moving to start");
        resetEnabled = false;
        //reset counter to turn 0
        resetGame();
      }

      break;
    case RANDOM_MOVE:
      if(moveDistance == false) {
        moveDistance = true;
        distance = getNextMove();
        newPosition = getNewPosition(currentPosition, distance);
        Serial.print("random move - moving ");
        Serial.print(distance);
        Serial.print(" blocks, from ");
        Serial.print(currentPosition);
        Serial.print(" to ");
        Serial.println(newPosition);
      }
      break;
    case SPACE24:
      if(moveDistance == false) {
        moveDistance = true;
        newPosition = getNewPosition(0, MOVE_24);
        Serial.print("space 24 - moving 24 blocks, from ");
        Serial.print(currentPosition);
        Serial.print(" to ");
        Serial.println(newPosition);
      }

      break;
    case MOVE_1:
      if(moveDistance == false) {
        moveDistance = true;
        newPosition = getNewPosition(currentPosition, 1);
        Serial.print("moving 1 space - moving 1 block");
        Serial.print(currentPosition);
        Serial.print(" to ");
        Serial.println(newPosition);
      }

      break;
    case MANUAL:
      //if(moveDistance == false)
      Serial.println("jog forward");
      jogForward = true;
      break;
    case NONE:
      if(jogForward == true) {
        Serial.print("stop jogging, current position: ");
        Serial.println(currentPosition);
        jogForward = false;
      }
      break;
    default:
      messageStatus= NACK;
      errorFlag = true;
  }
  newState = NO_MESSAGE;
}

void read_inputs() {
  if(!moveToStart) {
    if(digitalRead(dangerLocationPin) == LOW) {
      warningState = 1;
      Serial.println("danger location triggered");
    }
    if(digitalRead(fallPin) == LOW) {
      if(endState == 0) {
        moveDistance = false;
        Serial.println("fall triggered");
      }
      endState = 1;
      
    }
  }
  if(digitalRead(resetPin) == LOW) {
    if(!atStart) {
      Serial.println("at start");
    }
    atStart = true;
    
  } else {
    atStart = false;
  }
}

//blinks led when communication is broken
void display_error() {
  static bool ledState = LOW;
  if (errorFlag) {
    if (messageErrorIndicatorTimer.timeUp()) {
      messageErrorIndicatorTimer.reset();
      //toggle led
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
    }
  } else if(ledState == HIGH) {
    ledState = LOW;
    digitalWrite(LED_BUILTIN, LOW);
    //if led on, turn off
  }
}


void takeStep(stepper_direction_t direction, uint16_t stepDelay) {
  isTravelling = true;
  if(!stepperEnabled) {
    digitalWrite(stepper_enable_pin, LOW);
    stepperEnabled = true;
  }
  switch(direction) {
    case FORWARD:
      digitalWrite(dirPin,HIGH); // Enables the belt to move forward
      digitalWrite(stepPin,HIGH);  
      delayMicroseconds(stepDelay);

      digitalWrite(stepPin,LOW);
      delayMicroseconds(stepDelay);
      Serial.print(".");
      break;
    case REVERSE:
      digitalWrite(dirPin,LOW); // Enables the belt to move forward
      digitalWrite(stepPin,HIGH);

      delayMicroseconds(stepDelay); 
      digitalWrite(stepPin,LOW); 

      delayMicroseconds(stepDelay);
      break;
    default:
      Serial.println("invalid direction");
      break;
  }
}

void init_rail_inputs() {
  pinMode(resetPin, INPUT_PULLUP);            //optical sensor - home position
  pinMode(dangerLocationPin, INPUT_PULLUP);   //optical sensor - triggers DANGER sound
  pinMode(fallPin, INPUT_PULLUP);             //optical sensor - triggers LOSE sound

  pinMode(LED_BUILTIN, OUTPUT);               //enable built in led -- 13
}

void init_rail_outputs() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepper_enable_pin, OUTPUT);
  pinMode(max845_enable, OUTPUT);
  digitalWrite(max845_enable, LOW);
  //disable stepper
  digitalWrite(stepper_enable_pin, HIGH);
}

void handle_messages() {
  static message_status_t messageStatus = NACK;
  master_message_t messageIn = {0, NO_MESSAGE, 0};
  response_message_t response = {startByte, warningState, endState, isTravelling, ACK, endByte};
  switch (messageSendState) {
    case RECEIVE:
      if (messageReceiveTimer.timeUp()) {

        errorFlag = true;
        Serial.println("message timeout ");
        messageSendState = TRANSMIT;
        messageStatus= NACK;
        
        //transmit message
      } else {

        if ((unsigned)Serial1.available() >= sizeof(master_message_t)) {
          //Serial.print("message receieved -> current position: ");
          //Serial.println(currentPosition);
          messageSendState = TRANSMIT;
          Serial1.readBytes((byte *)&messageIn, sizeof(master_message_t));
          if(messageIn.startByte == startByte && messageIn.endByte == endByte) {
            messageStatus = ACK;
            //Serial.print("ACK ");
            //Serial.print(messageIn.startByte);
            //Serial.print(" ");
            //Serial.print(messageIn.currentState);
            //Serial.print(" ");
            //Serial.println(messageIn.endByte);
            newState = messageIn.currentState;
            
            if(newState != NONE) {
              Serial.print(messageIn.startByte);
              Serial.print(" ");
              Serial.print(messageIn.currentState);
              Serial.print(" ");
              Serial.println(messageIn.endByte);
            }
            
            errorFlag = false;
          } else {
            Serial.print("NACK: ");
            Serial.print(messageIn.startByte,HEX);
            Serial.print(" ");
            Serial.print(messageIn.currentState,HEX);
            Serial.print(" ");
            Serial.print(messageIn.endByte,HEX);
            while(Serial1.available()) {
              Serial.print(" ");
              Serial.print(Serial1.read(),HEX);
            }
            Serial.println();
            messageStatus= NACK;
          }
          //give time for lectern to switch to receive mode
          delay(2);
        }
      }

      break;
    case TRANSMIT:
      //reset states
      warningState = 0;
      endState = 0;
      response.messageStatus = messageStatus;
      
      digitalWrite(max845_enable, HIGH);
      delayMicroseconds(600);
      Serial1.write((byte *)&response, sizeof(response_message_t));
      delayMicroseconds(600);
      digitalWrite(max845_enable, LOW);
      //Serial.println("response sent");
      isTravelling = false;
      messageReceiveTimer.reset();
      messageSendState = RECEIVE;
      break;
  }
}

