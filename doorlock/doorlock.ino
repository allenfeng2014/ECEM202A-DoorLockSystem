#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_TFTLCD.h>      // Hardware-specific TFT library
#include <TouchScreen.h>
#include <EEPROM.h>               // EEPROM library
#include <Servo.h>                // Servo Motor library
#include <Adafruit_Fingerprint.h> // fingerprint sensor library

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

#define TS_MINX 120
#define TS_MINY 75
#define TS_MAXX 920
#define TS_MAXY 940
#define MINPRESSURE 300
#define MAXPRESSURE 500

// Assign meaningful names to color values
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// screen UI parameters
#define PNODERADIUS 18
#define BUTTONWIDTH4 80
#define BUTTONHEIGHT 40
#define BUTTONWIDTHsm 60
#define BUTTONHEIGHTsm 30

// FSM states
#define STATE_Off 0
#define STATE_VisitorLcked 1
#define STATE_VisitorUnlcked 2
#define STATE_OwnerVerify 3
#define STATE_PWEnter 4
#define STATE_BioEnter 5
#define STATE_LockDoor 6
#define STATE_UnlockDoor 7
#define STATE_KeyManager 8
#define STATE_BioManager 9
#define STATE_PWManager 10
#define STATE_KeyDelete 11
#define STATE_PWInvalid 12

// EEPROM addresses
#define ADDR_FirstTimeUse 0
#define ADDR_DoorLocked 1
#define ADDR_PWSet0 2     // 0 means PWData0 not set
#define ADDR_PWLength0 3  // length of PWData0, (1-12)
#define ADDR_PWData0 4    // PWData0, valid if PWSet0 = 1
#define ADDR_PWSet1 16
#define ADDR_PWLength1 17
#define ADDR_PWData1 18
#define ADDR_PWSet2 30
#define ADDR_PWLength2 31
#define ADDR_PWData2 32
#define ADDR_LastKeyType 44   // last key type (bio 0/pw 1) used for unlock
#define ADDR_LastKeyEntry 45  // last key entry (0/1/2) used for unlock

// declare touch screen and LCD object
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// declare servo motor object
int servoPin = 12;
Servo doorMotor;

// declare fingerprint sensor object
SoftwareSerial mySerial(10, 11);
Adafruit_Fingerprint bioSensor = Adafruit_Fingerprint(&mySerial);

// tft varaibles
int tftWidth = tft.width();
int tftHeight = tft.height();
// FSM variables
int curState, nexState, prevState;
// PW variables
int pwNodesX[12], pwNodesY[12];
bool pwenterReset;
int nodePressed, nodeCount, prevNodePressed;
int pwSequence[12];
int addrsPW[3] = {ADDR_PWSet0, ADDR_PWSet1, ADDR_PWSet2};
int addrPWTarget;
int pwCount;
// bioSensor variables
int bioDataReceived;
bool fingerRemoved;
uint16_t idBioTarget;
int bioCount;

// last key used variables
String lastKeyType;
uint16_t lastKeyEntry, thisKeyEntry;

void setup()
{
  // serial monitor
  Serial.begin(9600);
  Serial.println(F("\ndoor lock system"));
  
  // initialize TFT LCD
  tft.reset();
  uint16_t identifier = tft.readID();
  tft.begin(identifier);

  // initialize servo motor
  doorMotor.attach(servoPin);
  
  // initialize fingerprint sensor
  bioSensor.begin(57600);
  
  // variable init
  pwenterReset = true;
  bioDataReceived = 0;
  fingerRemoved = true;
  
  // pwEnterUI node array init
  int x_step = tftWidth/3;
  int y_step = tftHeight/4 - 10;
  for (int row=0; row<4; row++) {
    for (int col=0; col<3; col++) {
      pwNodesX[row * 3 + col] = 37 + x_step * col;
      pwNodesY[row * 3 + col] = 30 + y_step * row;
    }
  }
  
  bool firstTime=true;
  pwCount = 0;
  bioCount = 0;
  // check for manual reset from pin 13
  pinMode(13, INPUT);
  if (digitalRead(13)==HIGH) {
    Serial.println("reseting");
    // reset door motor to 0 degree
    doorMotor.write(10);
    EEPROM.update(ADDR_DoorLocked, 0);
    // clear all key entries
    for (int i=0; i<3; i++) {
      EEPROM.update(addrsPW[i], 0);
    }
    bioSensor.emptyDatabase();
    curState = STATE_KeyManager;
  }
  // else check if valid key exists
  else {
    for (int i=0; i<3; i++) {
      if (EEPROM.read(addrsPW[i])==1) {
        firstTime = false;
        pwCount ++;
      }
    }
    for (uint16_t i=1; i<4; i++) {
      if (bioSensor.loadModel(i)==FINGERPRINT_OK) {
        firstTime = false;
        bioCount ++;
      }
    }
    if (firstTime) {
      doorMotor.write(10);
      EEPROM.update(ADDR_DoorLocked, 0);
      curState = STATE_KeyManager;
      nexState = STATE_KeyManager;
      Serial.println("nexState is keymanager");
    }
    else {
      doorMotor.write(10+160*EEPROM.read(ADDR_DoorLocked));
      curState = STATE_Off;
      nexState = STATE_Off;
      Serial.println("nexState is off");
    }
  }
  updateScreen(curState);
}

void loop()
{
  TSPoint p = ts.getPoint();
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  // check for touchscreen activity
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
    // scale from 0->1023 to tft.width
    p.x = map(p.x, TS_MINX, TS_MAXX, 0, tftWidth);
    p.y = map(p.y, TS_MINY, TS_MAXY, tftHeight, 0);
    updateState(p.x, p.y);
  }
  // else check for biosensor activity
  else if (curState==STATE_BioEnter && bioSensor.getImage()==FINGERPRINT_OK) {
    updateState(-1, -1); // -1 means finger pressed on bioSensor 
  }
  else {
    // finger not pressed on bioSensor
    fingerRemoved = true;
  }
  if (nexState!=curState) {
      Serial.print("\nNext State: ");
      Serial.print(nexState);
      updateScreen(nexState);
      prevState = curState;
      curState = nexState;
    }
  else {
    maintainState(p.x, p.y);
  }
}

void updateState(int x, int y) {
  // check if a button is pressed, 0 means none
  int buttonPressed = buttonCheck(x, y);
  switch (curState) {
    case STATE_Off:
      if (EEPROM.read(ADDR_DoorLocked))
        nexState = STATE_VisitorLcked;
      else
        nexState = STATE_VisitorUnlcked;
      break;
    case STATE_VisitorLcked:
      if (buttonPressed) {
        // top left button (bio)
        if (buttonPressed==1) {
          nexState = STATE_BioEnter;
        }
        // top right button (PW)
        else if (buttonPressed==2) {
          nexState = STATE_PWEnter;
        }
      }
      break;
    case STATE_VisitorUnlcked:
      if (buttonPressed) {
        // top left button (edit)
        if (buttonPressed==1) {
          nexState = STATE_OwnerVerify;
        }
        // top right button (lock)
        else if (buttonPressed==2) {
          doorMotor.write(170);
          EEPROM.update(ADDR_DoorLocked, 1);
          nexState = STATE_LockDoor;
        }
      }
      break;
    case STATE_OwnerVerify:
      if (buttonPressed) {
        // top left button (back)
        if (buttonPressed==1) {
          nexState = STATE_VisitorUnlcked;
        }
        // bottom left button (bio)
        else if (buttonPressed==3) {
          nexState = STATE_BioEnter;
        }
        // bottom right button (PW)
        else if (buttonPressed==4) {
          nexState = STATE_PWEnter;
        }
      }
      break;
    case STATE_PWEnter:
      if (buttonPressed) {
        // top left button (back)
        if (buttonPressed==1) {
          pwenterReset = true;
          switch (prevState) {
            case STATE_OwnerVerify:
              curState = STATE_VisitorUnlcked;
              break;
            case STATE_PWManager:
              curState = STATE_KeyManager;
              break;
          }
          nexState = prevState;
        }
        // top right button (done)
        else if (buttonPressed==2) {
          pwenterReset = true;
          switch (prevState) {
            case STATE_VisitorLcked:
              // if pw valid, unlock door
              if (pwVerify(pwSequence, nodeCount)) {
                doorMotor.write(10);
                EEPROM.update(ADDR_DoorLocked, 0);
                nexState = STATE_UnlockDoor;
                // store entry key info
                EEPROM.get(ADDR_LastKeyEntry, lastKeyEntry);
                if (!EEPROM.read(ADDR_LastKeyType)) {
                  lastKeyType = "bio"; // 0 means bio entry
                  lastKeyEntry --;
                }
                else {
                  lastKeyType = "pw"; // 1 means pw entry
                }
                EEPROM.update(ADDR_LastKeyType, 1);
                EEPROM.put(ADDR_LastKeyEntry, thisKeyEntry);
              }
              // if pw invalid, go to pw invalid page
              else {
                nexState = STATE_PWInvalid;
              }
              break;
            case STATE_OwnerVerify:
              // if pw valid, go to key manager
              if (pwVerify(pwSequence, nodeCount)) {
                nexState = STATE_KeyManager;
              }
              // if pw invalid, go to pw invalid page
              else {
                curState = STATE_OwnerVerify;
                nexState = STATE_PWInvalid;
              }
              break;
            case STATE_PWManager:
              // update pwSet, pwLength, pwData
              EEPROM.update(addrPWTarget, 1);
              EEPROM.update(addrPWTarget+1, nodeCount);
              for (int i=0; i<nodeCount; i++) {
                EEPROM.update(addrPWTarget+2+i, pwSequence[i]);
              }
              clearScreen();
              printText("Password saved", 10, 100, 3);
              pwCount ++;
              delay(1000);
              nexState = STATE_PWManager;
              break;
          }
        }
      }
      break;
    case STATE_BioEnter:
      if (buttonPressed) {
        // top left button (back)
        if (buttonPressed==1) {
          switch (prevState) {
            case STATE_OwnerVerify:
              curState = STATE_VisitorUnlcked;
              break;
            case STATE_PWManager:
              curState = STATE_KeyManager;
              break;
            case STATE_BioManager:
              bioDataReceived = 0;
              break;
          }
          nexState = prevState;
        }
      }
      // fingerprint data from bioSensor
      else if (x == -1 && y == -1) {
        bioSensor.image2Tz();
        switch (prevState) {
          case STATE_VisitorLcked:
            updateBioMsg("Reading fingerprint...");
            delay(500);
            // if bio valid, unlock door
            if (bioSensor.fingerFastSearch()==FINGERPRINT_OK) {
              doorMotor.write(10);
              EEPROM.update(ADDR_DoorLocked, 0);
              nexState = STATE_UnlockDoor;
              // store unlock key info
              EEPROM.get(ADDR_LastKeyEntry, lastKeyEntry);
              if (!EEPROM.read(ADDR_LastKeyType)) {
                lastKeyType = "bio"; // 0 means bio entry
                lastKeyEntry --;
              }
              else {
                lastKeyType = "pw"; // 1 means pw entry
              }
              EEPROM.update(ADDR_LastKeyType, 0);
              EEPROM.put(ADDR_LastKeyEntry, bioSensor.fingerID);
            }
            // else update screen message "retry"
            else {
              updateBioMsg("Fingerprint invalid. Please retry...");
              delay(1500);
            }
            break;
          case STATE_OwnerVerify:
            updateBioMsg("Reading fingerprint...");
            delay(500);
            // if bio valid, go to key manager
            if (bioSensor.fingerFastSearch()==FINGERPRINT_OK) {
              updateBioMsg("Fingerprint matches");
              delay(1500);
              nexState = STATE_KeyManager;
            }
            // else update screen message "retry"
            else {
              updateBioMsg("Fingerprint invalid. Please retry...");
              delay(1500);
            }
            break;
          case STATE_BioManager:
            // read and process current fingerprint
            if (fingerRemoved && bioSensor.getImage()==FINGERPRINT_OK) {
              updateBioMsg("Reading fingerprint...");
              // wait until finger stable
              delay(500);
              updateBioMsg("Fingerprint received. Processing data...");
              delay(2000);
              // we need 2 fingerprint samples to create a model
              // check number of samples received so far
              // if first sample
              if (!bioDataReceived) {
                // if sample ok, store into slot 1
                if (bioSensor.image2Tz(1)==FINGERPRINT_OK) {
                  updateBioMsg("Sample 1 stored. Remove and press again...");
                  bioDataReceived = 1;
                }
                // else update screen message "retry"
                else {
                  updateBioMsg("Fingerprint unclear. Remove and press again...");
                }
              }
              // if second sample
              else if (bioDataReceived==1) {
                // if sample ok, store into slot 2
                if (bioSensor.image2Tz(2)==FINGERPRINT_OK) {
                  updateBioMsg("Sample 2 stored. Saving fingerprint...");
                  delay(2000);
                  // create fingerprint model using 2 samples
                  bioSensor.createModel();
                  if (bioSensor.storeModel(idBioTarget)==FINGERPRINT_OK) {
                    updateBioMsg("Fingerprint saved");
                    bioCount ++;
                    delay(2000);
                    nexState = STATE_BioManager;
                  }
                  bioDataReceived = 0;
                }
              }
              fingerRemoved = false;
            }
            break;
        }
      }
      break;
    case STATE_LockDoor:
      nexState = STATE_Off;
      break;
    case STATE_UnlockDoor:
      nexState = STATE_Off;
      break;
    case STATE_KeyManager:
      if (buttonPressed) {
        // top right button (done)
        if (buttonPressed==2) {
          nexState = STATE_Off;
        }
        // bottom left button (bio)
        else if (buttonPressed==3) {
          nexState = STATE_BioManager;
        }
        // bottome right button (PW)
        else if (buttonPressed==4) {
          nexState = STATE_PWManager;
        }
      }
      break;
    case STATE_BioManager:
      if (buttonPressed) {
        // top right button (done)
        if (buttonPressed==2) {
          nexState = STATE_KeyManager;
        }
      }
      else {
        buttonPressed = buttonMidCheck(x, y);
        if (buttonPressed) {
          uint16_t i = ceil((float(buttonPressed) / 2));
          if (bioSensor.loadModel(i)==FINGERPRINT_OK) {
            idBioTarget = i;
            // left button (del)
            if (buttonPressed%2) {
              nexState = STATE_KeyDelete;
            }
            // right button (edit)
            else {
              nexState = STATE_BioEnter;
            }
          }
          // for invalid entry, only right button (add) available
          else {
            // right button (add)
            if (!(buttonPressed%2)) {
              idBioTarget = i;
              nexState = STATE_BioEnter;
            }
          }
        }
      }
      break;
    case STATE_PWManager:
      if (buttonPressed) {
        // top right button (done)
        if (buttonPressed==2) {
          nexState = STATE_KeyManager;
        }
      }
      else {
        buttonPressed = buttonMidCheck(x, y);
        if (buttonPressed) {
          int i = floor((float(buttonPressed -1) / 2));
          Serial.print("\nbutton ");
          Serial.print(buttonPressed);
          Serial.print(" pressed");
          // for valid entry, two buttons (del, edit) available
          if (EEPROM.read(addrsPW[i])==1) {
            addrPWTarget = addrsPW[i];
            // left button (del)
            if (buttonPressed%2) {
              nexState = STATE_KeyDelete;
            }
            // right button (edit)
            else {
              nexState = STATE_PWEnter;
            }
          }
          // for invalid entry, only right button (add) available
          else {
            // right button (add)
            if (!(buttonPressed%2)) {
              addrPWTarget = addrsPW[i];
              nexState = STATE_PWEnter;
            }
          }
        }
      }
      break;
    case STATE_KeyDelete:
      if (buttonPressed) {
        // bottom left button (NO)
        if (buttonPressed==3) {
          curState = STATE_KeyManager;
          nexState = prevState;
        }
        // bottom right button (YES)
        else if (buttonPressed==4) {
          // delete pw entry if from PWManager
          if (prevState==STATE_PWManager) {
            EEPROM.write(addrPWTarget, 0);
            pwCount --;
          }
          // delete bio entry if from BioManager
          else {
            bioSensor.deleteModel(idBioTarget);
            bioCount --;
          }
          curState = STATE_KeyManager;
          nexState = prevState;
          if (pwCount + bioCount == 0) {
            clearScreen();
            printText("No valid key in memory. Please reset system.", 10, 100, 2);
            while(1) {
              delay(10);
            }
          }
        }
      }
      break;
    case STATE_PWInvalid:
      if (buttonPressed) {
        // bottom right button (OK)
        if (buttonPressed==4) {
          curState = prevState;
          nexState = STATE_PWEnter;
        }
      }
      break;
  }
}

void maintainState(int x, int y) {
  switch (curState) {
    case STATE_PWEnter:
      if (pwenterReset) {
        Serial.print("pwenter reset\n");
        prevNodePressed = 0;
        nodeCount = 0;
        pwenterReset = false;
      }
      // check which node is pressed, 0 means none
      nodePressed = 0;
      float distance;
      for (int i=0; i<12; i++) {
        distance = sqrt(pow(pwNodesX[i]-x, 2) + pow(pwNodesY[i]-y, 2));
        if (distance < float(PNODERADIUS)) {
          nodePressed = i + 1;
          break;
        }
      }
      if (nodePressed && nodePressed != prevNodePressed && nodeCount < 12) {
        tft.fillCircle(pwNodesX[nodePressed-1], pwNodesY[nodePressed-1], PNODERADIUS, RED);
        pwSequence[nodeCount] = nodePressed;
        nodeCount ++;
        if (nodeCount > 1) {
          int p0 = prevNodePressed - 1;
          int p1 = nodePressed - 1;
          tft.drawLine(pwNodesX[p0], pwNodesY[p0], pwNodesX[p1], pwNodesY[p1], RED);
        }
        prevNodePressed = nodePressed;
      }
      break;
  }
}

void updateScreen(int state) {
  switch (state) {
    case STATE_Off:
      clearScreen();
      break;
    case STATE_VisitorLcked:
      clearScreen();
      printText("Visitor Lcked", 10, 100, 3);
      drawButton("bio", 1);
      drawButton(" PW", 2);
      break;
    case STATE_VisitorUnlcked:
      clearScreen();
      printText("Visitor Unlcked", 10, 100, 3);
      drawButton("edit", 1);
      drawButton("lock", 2);
      break;
    case STATE_OwnerVerify:
      clearScreen();
      printText("Owner Verify", 10, 100, 3);
      drawButton("back", 1);
      drawButton("bio", 3);
      drawButton(" PW", 4);
      break;
    case STATE_PWEnter:
      clearScreen();
      drawButton("back", 1);
      drawButton("done", 2);
      // draw nodes for pw enter
      for (int i=0; i<12; i++) {
        tft.fillCircle(pwNodesX[i], pwNodesY[i], PNODERADIUS, MAGENTA);
      }
      break;
    case STATE_BioEnter:
      clearScreen();
      printText("Enter Fingerprint", 10, 100, 3);
      drawButton("back", 1);
      break;
    case STATE_LockDoor:
      clearScreen();
      printText("Door Locked", 10, 100, 3);
      break;
    case STATE_UnlockDoor:
      clearScreen();
      printText("Welcome Back", 10, 100, 3);
      printText("Last key used: ", 10, 170, 2);
      printText(lastKeyType, 100, 200, 2);
      printText(String(lastKeyEntry), 150, 200, 2);
      break;
    case STATE_KeyManager:
      clearScreen();
      printText("key manager", 20, 80, 3);
      drawButton("done", 2);
      drawButton("bio", 3);
      drawButton(" PW", 4);
      break;
    case STATE_BioManager:
      clearScreen();
      drawButton("done", 2);
      bioSensor.loadModel(0);
      for (uint16_t i=1; i<4; i++) {
        printText("entry " + String(i-1), 20, 50+(i-1)*80, 2);
        if (bioSensor.loadModel(i)==FINGERPRINT_OK) {
          Serial.println("model valid");
          printText(": valid", 100, 50+(i-1)*80, 2);
          drawButtonMid("del", 1+(i-1)*2);
          drawButtonMid("edit", 2+(i-1)*2);
        }
        else {
          printText(": invalid", 100, 50+(i-1)*80, 2);
          drawButtonMid("add", 2+(i-1)*2);
        }
      }
      break;
    case STATE_PWManager:
      clearScreen();
      drawButton("done", 2);
      for (int i=0; i<3; i++) {
        printText("entry " + String(i), 20, 50+i*80, 2);
        if (EEPROM.read(addrsPW[i])==1) {
          printText(": valid", 100, 50+i*80, 2);
          drawButtonMid("del", 1+i*2);
          drawButtonMid("edit", 2+i*2);
        }
        else {
          printText(": invalid", 100, 50+i*80, 2);
          drawButtonMid("add", 2+i*2);
        }
      }
      break;
    case STATE_KeyDelete:
      clearScreen();
      printText("Delete this key entry ?", 10, 100, 2);
      drawButton(" NO", 3);
      drawButton("YES", 4);
      break;
    case STATE_PWInvalid:
      clearScreen();
      printText("Invalid PW, please retry", 10, 100, 2);
      drawButton(" OK", 4);
      break;
  }
}

void updateBioMsg(String bioMsg) {
  tft.fillRect(0, 100, tftWidth, 90, BLUE);
  printText(bioMsg, 10, 150, 2);
}

void clearScreen() {
  tft.fillRect(0, 0, tftWidth, tftHeight, BLACK);
}

void printText(String text, int x, int y, int textSize) {
  tft.setRotation(2);
  tft.setTextSize(textSize);
  tft.setCursor(x, y);
  tft.print(text);
  tft.setRotation(0);
}

void drawButton(String buttonName, int buttonLoc) {
  switch (buttonLoc) {
    case 1: // top left
      tft.drawRect(tftWidth-BUTTONWIDTH4, tftHeight-BUTTONHEIGHT, BUTTONWIDTH4, BUTTONHEIGHT, WHITE);
      printText(buttonName, 5, 5, 3);
      break;
    case 2: // top right
      tft.drawRect(0, tftHeight-BUTTONHEIGHT, BUTTONWIDTH4, BUTTONHEIGHT, WHITE);
      printText(buttonName, 165, 5, 3);
      break;
    case 3: // bottom left
      tft.drawRect(tftWidth-BUTTONWIDTH4, 0, BUTTONWIDTH4, BUTTONHEIGHT, WHITE);
      printText(buttonName, 5, 290, 3);
      break;
    case 4: // bottome right
      tft.drawRect(0, 0, BUTTONWIDTH4, BUTTONHEIGHT, WHITE);
      printText(buttonName, 165, 290, 3);
      break;
  }
}

int buttonCheck(int x, int y) {
  int buttonNum = 0;
  // top buttons
  if (y > tftHeight-BUTTONHEIGHT && y < tftHeight) {
    // top left button
    if (x > tftWidth-BUTTONWIDTH4 && x < tftWidth)
      buttonNum = 1;
    // top right button
    else if (x > 0 && x < BUTTONWIDTH4)
      buttonNum = 2;
  }
  // bottom buttons
  else if (y > 0 && y < BUTTONHEIGHT) {
    // bottom left button
    if (x > tftWidth-BUTTONWIDTH4 && x < tftWidth)
      buttonNum = 3;
    // bottom right button
    else if (x > 0 && x < BUTTONWIDTH4)
      buttonNum = 4;
  }
  
  return buttonNum;
}

void drawButtonMid(String buttonName, int buttonLoc) {
  switch (buttonLoc) {
    case 1: // first line left
      tft.drawRect(90, 210, BUTTONWIDTHsm, BUTTONHEIGHTsm, WHITE);
      printText(buttonName, 100, 90, 2);
      break;
    case 2: // first line right
      tft.drawRect(10, 210, BUTTONWIDTHsm, BUTTONHEIGHTsm, WHITE);
      printText(buttonName, 175, 90, 2);
      break;
    case 3: // second line left
      tft.drawRect(90, 130, BUTTONWIDTHsm, BUTTONHEIGHTsm, WHITE);
      printText(buttonName, 100, 170, 2);
      break;
    case 4: // second line right
      tft.drawRect(10, 130, BUTTONWIDTHsm, BUTTONHEIGHTsm, WHITE);
      printText(buttonName, 175, 170, 2);
      break;
    case 5: // third line left
      tft.drawRect(90, 50, BUTTONWIDTHsm, BUTTONHEIGHTsm, WHITE);
      printText(buttonName, 100, 250, 2);
      break;
    case 6: // third line right
      tft.drawRect(10, 50, BUTTONWIDTHsm, BUTTONHEIGHTsm, WHITE);
      printText(buttonName, 175, 250, 2);
      break;
  }
}

int buttonMidCheck(int x, int y) {
  int buttonNum = 0;
  // first line buttons
  if (y > 210 && y < 210 + BUTTONHEIGHTsm) {
    // first line left button
    if (x > 90 && x < 90 + BUTTONWIDTHsm) {
      buttonNum = 1;
    }
    // first line right button
    else if (x > 10 && x < 10 + BUTTONWIDTHsm) {
      buttonNum = 2;
    }
  }
  // second line buttons
  else if (y > 130 && y < 130 + BUTTONHEIGHTsm) {
    // second line left button
    if (x > 90 && x < 90 + BUTTONWIDTHsm) {
      buttonNum = 3;
    }
    // second line right button
    else if (x > 10 && x < 10 + BUTTONWIDTHsm) {
      buttonNum = 4;
    }
  }
  // third line buttons
  else if (y > 50 && y < 50 + BUTTONHEIGHTsm) {
    // third line left button
    if (x > 90 && x < 90 + BUTTONWIDTHsm) {
      buttonNum = 5;
    }
    // third line right button
    else if (x > 10 && x < 10 + BUTTONWIDTHsm) {
      buttonNum = 6;
    }
  }

  return buttonNum;
}

int pwVerify(int pwSequence[12], int length) {
  int pwMatch = 0;
  for (int i=0; i<3; i++) {
    if (EEPROM.read(addrsPW[i])==1 && EEPROM.read(addrsPW[i]+1)==length) {
      int entryMatch = 1;
      for (int j=0; j<length; j++) {
        if (EEPROM.read(addrsPW[i]+2+j)!=pwSequence[j]) {
          entryMatch = 0;
          break;
        }
      }
      if (entryMatch) {
        pwMatch = 1;
        thisKeyEntry = i;
        break;
      }
    }
  }

  return pwMatch;
}
