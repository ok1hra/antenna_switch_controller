//#include <Arduino.h>

/*

  6 x 2 Antenna control
  ----------------------
  http://remoteqth.com/6x2-antenna-controler.php
  2016-12 by OK1HRA
  rev 0.3

  ___               _        ___ _____ _  _
  | _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
  |   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
  |_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Features:
  INPUTS
    - Four TRX
    - Automatic four bit BCD - may be activate LOW or HIGH, must be configure on pcb and firmware
    - Manual menu with light button and rotary encoder
    - Manually through the web interface
    - Free configure matrix input table
  OUTPUTS
    - 6 antennas
    - 2x6 gpio outputs (12V or GND output dependancy to use IO driver)
    - serialline
    - web page
  BLOCKED
    - more inpunts in same time = select only one antenna for each TRX
    - colision between TRX and same antenna
    - switching during PTT ON, separately for each TRX
    - interruption PTT path if colision detected
  SHOW on LCD/WEB
    - active number outputs
    - colision
    - PTT ON
    - name of the antenna
    - power voltage
  ETHERNET (with optional IP module)
    - DHCP
    - fixed IP
    - Show IP on LCD during start up
  - refresh input < 25 ms two TRX, < 55 ms four TRX (arduino Nano)
  - all mounted on compact pcb - useable without enclosure
  - output connector included power for switch - all in one cable
  - Ethernet hardware support
  - support two line LCD, one for each TRX
  - measure input voltage

  Changelog:
  2017-01 HTML color bug fix
  2016-12 change to rev 0.3 pinout

Dhcp.h change from
  int beginWithDHCP(uint8_t *, unsigned long timeout = 60000, unsigned long responseTimeout = 5000);
to
  int beginWithDHCP(uint8_t *, unsigned long timeout = 6000, unsigned long responseTimeout = 5000);



*/
//=====[ Settings ]===================================================
char* ant[] = {
  "All OUT off",  // <-- do not change this line
  "160m Vert.",
  " 80m Dipole",
  " 40m Moxon",
  " 30m Dipole",
  " 20m 2x 5el",
  " 10m 4x 6el",
  "M-off->BCD",  // <-- do not change this line
};
#define Inputs      6      // number of antenna used ** not implemented **
#define Ports       2      // number of - IN/OUT pair devices and LCD lines (support from 2 to 4)
#define LCDculumn  16      //
#define inputHigh          // enable input High level (default)
//#define serialECHO       // enable TX echo on serial port
#define SERBAUD    9600    // [baud] Serial port baudrate
//#define EthModule        // enable Ethernet module
//#define __USE_DHCP__       // Uncoment to Enable DHCP
//====================================================================
#if defined(EthModule)
//  #include <util.h>
  #include <Ethernet2.h>
  #include <Dhcp.h>
  #include <EthernetServer.h>
  #include <SPI.h>
#endif
#include "Wire.h"
#include <LiquidCrystal.h>
LiquidCrystal lcd(A0, A1, 7, 6, 5, 4);     // rev. 0.3
#if defined(EthModule)
  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
  IPAddress ip(192, 168, 1, 220);         // IP
  IPAddress gateway(192, 168, 1, 200);    // GATE
  IPAddress subnet(255, 255, 255, 0);     // MASK
  IPAddress myDns(8, 8, 8, 8);            // DNS (google pub)
  EthernetServer server(80);              // server PORT
  String HTTP_req;
#endif
int BCDmatrixINOUT[2][16] = { /*
---------------------------------------------------------------------------------------
BCD Band # input         0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
(Yaesu BCD)                 160 80  40  30  20  17  15  12  10  6m
---------------------------------------------------------------------------------------
                         |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
                         V   V   V   V   V   V   V   V   V   V   V   V   V   V   V   V
                    */ { 0,  1,  2,  3,  4,  5,  6,  0,  0,  0,  0,  0,  0,  0,  0,  0 }, /* --> ANT-1 OUTPUT - use only value from 0 to 6 !
                    */ { 0,  1,  2,  3,  4,  5,  6,  0,  0,  0,  0,  0,  0,  0,  0,  0 }, /* --> ANT-2 OUTPUT - use only value from 0 to 6 !
*/ };
//========================================================================================
byte a = 0;
byte b = 0;
unsigned int ab;
//long Loops=0;
int i = 0;
int j = 0;
int c = 0;
int val;
int e = 0;
const int enc0PinA = 2;
const int enc0PinB = 3;
const int sw = 9;
const int swLED = 8;
int enc0Pos = 0;
byte enc0PinALast = HIGH;
int n = HIGH;
boolean menu1state = false;
boolean menu2state = false;
long Timeout[5][2] = {
  {0, 100},
  {0, 500},
  {0, 1000},
  {0, 5000},
  {0, 3000},
};
boolean buttonActive = false;
boolean longPressActive = false;
byte ERR[8] = {0b11111, 0b11011, 0b11011, 0b11011, 0b11011, 0b11111, 0b11011, 0b11111};
byte PTT[8] = {0b11111, 0b10011, 0b10101, 0b10101, 0b10011, 0b10111, 0b10111, 0b11111};
byte MAN[8] = {0b11111, 0b01110, 0b00100, 0b01010, 0b01110, 0b01110, 0b01110, 0b11111};
byte Cursor[8] = {0b00000, 0b00000, 0b11000, 0b11000, 0b11000, 0b00000, 0b00000, 0b00000};
byte mCursor[8] = {0b00011, 0b00010, 0b11010, 0b11010, 0b11010, 0b00010, 0b00011, 0b00000};
byte m[8] = {0b00011, 0b00010, 0b00010, 0b00010, 0b00010, 0b00010, 0b00011, 0b00000};
String Note;
int port[8][6] = {
  //  adr   # ptt Err Manual part
  { 0x21, 0, 0,  0, 0, 1 }, // port1 IN
  { 0x21, 0, 0,  0, 0, 2 }, // port2 IN
  { 0x23, 0, 0,  0, 0, 1 }, // port3 IN
  { 0x23, 0, 0,  0, 0, 2 }, // port4 IN
  { 0x20, 0, 0,  0, 0, 1 }, // port1 OUT
  { 0x20, 0, 0,  0, 0, 2 }, // port2 OUT
  { 0x22, 0, 0,  0, 0, 1 }, // port3 OUT
  { 0x22, 0, 0,  0, 0, 2 }, // port4 OUT
};

//=================================================================
void setup()
{
  Wire.begin();
  for (i = 0; i < Ports; i++) {
    Wire.beginTransmission(port[i + 4][0]);
    Wire.write((byte)0x00);
    Wire.write((byte)0x00);
    Wire.endTransmission();
    Wire.beginTransmission(port[i + 4][0]);
    Wire.write((byte)0x01);
    Wire.write((byte)0x00);
    Wire.endTransmission();
  }
  lcd.begin(16, Ports);
  lcd.setCursor(1, Ports / 2 - 1);
  lcd.print(Inputs);
  lcd.setCursor(2, Ports / 2 - 1);
  lcd.print(F("x  ANT control"));
  lcd.setCursor(3, Ports / 2 - 1);
  lcd.print(Ports);
  lcd.setCursor(0, Ports / 2);
  lcd.print(F("***RemoteQTH.com"));
  delay(2000);      //5S
  lcd.clear();
  lcd.createChar(0, ERR);
  lcd.createChar(1, PTT);
  lcd.createChar(2, MAN);
  lcd.createChar(3, Cursor);
  lcd.createChar(4, mCursor);
  lcd.createChar(5, m);
  pinMode(swLED, OUTPUT);
  digitalWrite (swLED, LOW);
  pinMode(sw, INPUT);
  digitalWrite (sw, HIGH);
  pinMode (enc0PinA, INPUT);
  pinMode (enc0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc0PinB), encI, FALLING);
#if defined(serialECHO) || defined(EthModule)
  Serial.begin(SERBAUD);
#endif
  lcd.setCursor(1, Ports / 2 - 1);
  lcd.print(F("Input voltage:"));
  lcd.setCursor(1, Ports / 2);
  lcd.print(volt(analogRead(A3)));
  lcd.setCursor(7, Ports / 2);
  lcd.print("V");
  delay(3000);      //5S
  lcd.clear();
#if defined(EthModule)
#if defined __USE_DHCP__
  Ethernet.begin(mac);
#else
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
#endif
  server.begin();
  Serial.print(F("server is at "));
  Serial.println(Ethernet.localIP());
  lcd.setCursor(1, Ports / 2 - 1);
  lcd.print(F("IP address:"));
  lcd.setCursor(1, Ports / 2);
  lcd.print(Ethernet.localIP());
  delay(10000);      //5S
#endif
}

void loop() {

Gpio();

  //=====[ Ethernet ]=================
#if defined(EthModule)
  EthernetClient client = server.available();
  if (client) {
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        HTTP_req += c;
        if (c == '\n' && currentLineIsBlank) {
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/html"));
          client.println(F("Connection: close"));
          client.println();
          client.println(F("<!DOCTYPE html>"));
          client.println(F("<html>"));
          client.println(F("<head>"));
          client.print(F("<title>"));
          client.print(Inputs);
          client.print(F("x"));
          client.print(Ports);
          client.println(F(" Antenna switch</title>"));
          client.print(F("<meta http-equiv=\"refresh\" content=\"10;url=http://"));
          client.print(Ethernet.localIP());
          client.println(F("\">"));
          client.println(F("<link href='http://fonts.googleapis.com/css?family=Roboto+Condensed:300italic,400italic,700italic,400,700,300&subset=latin-ext' rel='stylesheet' type='text/css'>"));
          client.println(F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">"));
          client.println(F("<meta name=\"mobile-web-app-capable\" content=\"yes\">"));
          client.println(F("<style type=\"text/css\">"));
          client.println(F("body {font-family: 'Roboto Condensed',sans-serif,Arial,Tahoma,Verdana;background: #ccc;}"));
          client.println(F("a:link  {color: #888;font-weight: bold;text-decoration: none;}"));
          client.println(F("a:visited  {color: #888;font-weight: bold;text-decoration: none;}"));
          client.println(F("a:hover  {color: #888;font-weight: bold;text-decoration: none;}"));
          client.println(F("input {border: 2px solid #ccc;background: #fff;margin: 10px 5px 0 0;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #333;}"));
          client.println(F("input:hover {border: 2px solid #080;}"));
          client.println(F("input.g {background: #080;color: #fff;}"));
          client.println(F("input.gr {background: #800;color: #fff;}"));
          client.println(F(".bcd {border: 2px solid #080;background: #ccc;margin: 10px 5px 0 10px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #000;}"));
          client.println(F(".bcdr {border: 2px solid #800;background: #ccc;margin: 10px 5px 0 10px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #000;}"));
          client.println(F(".ptt {border: 2px solid #800;background: #ccc;margin: 10px 5px 0 10px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #800;}"));
          client.println(F("</style>"));
          client.println(F("</head>"));
          client.println(F("<body>"));
          client.print(F("<p><b>"));
          client.print(Inputs);
          client.print(F("x"));
          client.print(Ports);
          client.println(F(" Antenna switch</b></p>"));
          client.println(F("<form method=\"get\">"));
          String BANK = HTTP_req.substring(7, 8);
          String GET = HTTP_req.substring(8, 10);
          switch (GET.toInt()) {
            case 0: port[BANK.toInt()-1][1] = 0; Gpio(); break;
            case 1: port[BANK.toInt()-1][1] = 1; Gpio(); break;
            case 2: port[BANK.toInt()-1][1] = 2; Gpio(); break;
            case 3: port[BANK.toInt()-1][1] = 3; Gpio(); break;
            case 4: port[BANK.toInt()-1][1] = 4; Gpio(); break;
            case 5: port[BANK.toInt()-1][1] = 5; Gpio(); break;
            case 6: port[BANK.toInt()-1][1] = 6; Gpio(); break;
            case 20: port[BANK.toInt()-1][4] = 0; Gpio(); break;
            case 21: port[BANK.toInt()-1][4] = 1; Gpio(); break;
          }

          for (i = 0; i < Ports; i++) {
              client.print(F("<span class=\"bcd"));
              if (port[i][3] == 1) {
                client.print(F("r"));
              }
              client.print(F("\">TRX"));
              client.print(i+1);
              client.print(F(" &#10148; "));
              client.print(port[i][1]);
              client.print(F("</span>"));
              if (port[i][4] == 0) {
                client.print(F("<input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("21\" value=\"Manual\"> "));
              } else {
                client.print(F("<input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("00\" value=\"-\" class=\""));
                if (port[i][1] == 0) {
                  client.print(F("g"));
                  if (port[i][1] == 0 && port[i][3] == 1) {
                    client.print(F("r"));
                  }
                }
                client.print(F("\"><input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("01\" value=\"1\" class=\""));
                if (port[i][1] == 1) {
                  client.print(F("g"));
                  if (port[i][1] == 1 && port[i][3] == 1) {
                    client.print(F("r"));
                  }
                }
                client.print(F("\"><input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("02\" value=\"2\" class=\""));
                if (port[i][1] == 2) {
                  client.print(F("g"));
                  if (port[i][1] == 2 && port[i][3] == 1) {
                    client.print(F("r"));
                  }
                }
                client.print(F("\"><input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("03\" value=\"3\" class=\""));
                if (port[i][1] == 3) {
                  client.print(F("g"));
                  if (port[i][1] == 3 && port[i][3] == 1) {
                    client.print(F("r"));
                  }
                }
                client.print(F("\"><input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("04\" value=\"4\" class=\""));
                if (port[i][1] == 4) {
                  client.print(F("g"));
                  if (port[i][1] == 4 && port[i][3] == 1) {
                    client.print(F("r"));
                  }
                }
                client.print(F("\"><input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("05\" value=\"5\" class=\""));
                if (port[i][1] == 5) {
                  client.print(F("g"));
                  if (port[i][1] == 5 && port[i][3] == 1) {
                    client.print(F("r"));
                  }
                }
                client.print(F("\"><input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("06\" value=\"6\" class=\""));
                if (port[i][1] == 6) {
                  client.print(F("g"));
                  if (port[i][1] == 6 && port[i][3] == 1) {
                    client.print(F("r"));
                  }
                }
                client.print(F("\"><input type=\"submit\" name=\"S"));
                client.print(i+1);
                client.print(F("20\" value=\"BCD-"));
                client.print(i+1);
                client.println(F("\"> "));
              }
              if (port[i][2] == 1) {
                client.print(F("<span class=\"ptt\">PTT</span>"));
              }
              client.print(F("<br>"));
          }

          client.println(F("</form>"));
          client.println(F("<br><a href=\".\" onclick=\"window.open( this.href, this.href, 'width=450,height=200,left=0,top=0,menubar=no,location=no,status=no' ); return false;\" > split&#8599;</a>"));
          client.println(F("<br><p><b>Antennas:</b><br>"));
          client.print(F("<b>1</b> - ")); client.println(ant[1]);
          client.print(F(" | <b>2</b> - ")); client.println(ant[2]);
          client.print(F("<br><b>3</b> - ")); client.println(ant[3]);
          client.print(F(" | <b>4</b> - ")); client.println(ant[4]);
          client.print(F("<br><b>5</b> - ")); client.println(ant[5]);
          client.println(F(" | <b>6</b> - ")); client.println(ant[6]);
          client.print(F("<br><b>Input power voltage: </b>"));
          client.print(volt(analogRead(A3)));
          client.println(F("V</p></body>"));
          client.println(F("</html>"));

          Serial.print(HTTP_req);
          HTTP_req = "";
          break;
        }
        if (c == '\n') {
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          currentLineIsBlank = false;
        }
      }
    }
    delay(1);
    client.stop();
  }
#endif

  //=====[ Serial ]=================
#if defined(serialECHO)
  if (millis() - Timeout[2][0] > (Timeout[2][1])) {
    for (i = 0; i < Ports; i++) {
      Serial.print(F("<"));
      Serial.print(i + 1);
      Serial.print(F(','));
      Serial.print(port[i][1]);
      Serial.print(F(','));
      Serial.print(port[i][2]);
      Serial.print(F(','));
      Serial.print(port[i][3]);
      Serial.print(F(','));
      Serial.print(port[i][4]);
      Serial.println(F(">"));
    }
    Serial.flush();
    Timeout[2][0] = millis();
  }
#endif

  //=====[ Button ]=================
  if (digitalRead(sw) == 0) {
    if (buttonActive == 0) {
      buttonActive = 1;
      Timeout[1][0] = millis();
    }
    if ((millis() - Timeout[1][0] > Timeout[1][1]) && (longPressActive == 0)) {
      longPressActive = 1;
      menu1state = !menu1state;
      digitalWrite(swLED, menu1state);
    }
  } else {
    if (buttonActive == 1) {
      if (longPressActive == 1) {
        longPressActive = 0;
      } else {
      }
      buttonActive = 0;
    }
  }
  //=====[ LCD ]=================
  if (millis() - Timeout[0][0] > (Timeout[0][1])) {
    for (j = 0; j < Ports; j++) {
      show(j);
    }
    Timeout[0][0] = millis();
    if (millis() - Timeout[4][0] > (Timeout[4][1])) {
        if(volt(analogRead(A3)) < 10) {
            lcd.clear();
            lcd.setCursor(1, Ports / 2 - 1);
            lcd.print(F("LOW voltage!"));
            lcd.setCursor(7, Ports / 2);
            lcd.print(volt(analogRead(A3)));
            delay(2000);
        }
        if(volt(analogRead(A3)) > 15) {
            lcd.clear();
            lcd.setCursor(1, Ports / 2 - 1);
            lcd.print(F("HIGH voltage!"));
            lcd.setCursor(8, Ports / 2);
            lcd.print(volt(analogRead(A3)));
            delay(2000);
        }
    Timeout[4][0] = millis();
    }

  }
}

void Gpio(){
  //=====[ GPIOs]=================
  for (i = 0; i < Ports; i++) {
    if (menu1state == 1 && i == enc0Pos) {
      if (port[i][1] != 7) {
        port[i][4] = 1;
      } else {
        port[i][4] = 0;
      }
      rx(port[i][0], i, 1, port[i][5]);
    } else if (port[i][4] == 1) {
      rx(port[i][0], i, 1, port[i][5]);
    } else if (port[i][4] == 0) {
      rx(port[i][0], i, 0, port[i][5]);
    }

    c = 0;
    for (j = 0; j < Ports; j++) {
      if (i != j && port[i][1] == port[j + 4][1]) {
        c++;
      }
    }
    if (c > 0) {
      port[i][3] = 1;
      if (port[i][2] == 0) {
        port[i + 4][1] = 0;
      }
    } else {
      port[i][3] = 0;
      if (port[i][2] == 0) {
        port[i + 4][1] = port[i][1];
      }
    }
    if (port[i + 4][5] == 2) {
      tx(port[i + 4][0], i);
    }
  }
}

//=====[ Encoder ]========================== interrupt
void encI(){
  if(digitalRead(enc0PinA) == LOW){
      e=1;  // ++
  }else{
      e=-1; // --
  }
  Timeout[3][0] = millis();

  if (menu1state == 0) {
    enc0Pos = enc2(enc0Pos, Ports-1, e);
  } else {
    port[enc0Pos][1] = enc2(port[enc0Pos][1], Inputs+1, e);
  }
}


//=====[ Encoder2 ]========================== with interrupt

int enc2(int encPos, int range, int count) {
  encPos = encPos + e;
  if(encPos>range){
    encPos = 0;
  }
  if(encPos<0){
    encPos = range;
  }
  return encPos;
}


//=====[ show one LCD line ]=================
void show(int portNR) {
  //=====[ IN/OUT number ]==========
  lcd.setCursor(0, portNR);
  if (port[portNR][1] == 7 || port[portNR][1] == 0) {
    lcd.print("  ");
  } else {
    if (port[portNR][1] < 10) {
      lcd.print(' ');
    }
    lcd.print(port[portNR][1], DEC);
  }
  //=====[ Cursor ]=================
  for (i = 0; i < Ports; i++) {
    if (i == enc0Pos) {
      lcd.setCursor(2, i);
      if (menu1state == 1) {
        lcd.write(byte(2));
      } else if (port[i][4] == 1) {
        if (millis() - Timeout[3][0] > (Timeout[3][1])) {
          lcd.write(byte(5));
        } else {
          lcd.write(byte(4));
        }
      } else {
        if (millis() - Timeout[3][0] > (Timeout[3][1])) {
          lcd.print(' ');
        } else {
          lcd.write(byte(3));
        }
      }
    } else {
      lcd.setCursor(2, i);
      if (port[i][4] == 1) {
        lcd.write(byte(5));
      } else {
        lcd.print(' ');
      }
    }
  }
  //=====[ Status ]=================
  lcd.setCursor(3, portNR);
  if (port[portNR][3] == 1 && (port[portNR][1] != 0)) {
    lcd.write(byte(0));
  } else if (port[portNR][1] == 0 || port[portNR][1] == 7) {
    lcd.print(' ');
  } else if (port[portNR][1] == port[portNR + 4][1]) {
    lcd.print('>');
  } else {
    lcd.print(' ');
  }

  lcd.setCursor(4, portNR);
  if (port[portNR][2] == 1) {
    lcd.write(byte(1));
  } else {
    lcd.print(' ');
  }
  //=====[ Note ]=================
  lcd.setCursor(5, portNR);
  if (port[portNR][3] == 1 && port[portNR][1] != 0) {
    Note = "- (used)";
  } else {
    Note = ant[port[portNR][1]];
  }
  Note.remove(LCDculumn - 5);
  while (Note.length() < LCDculumn - 5) {
    Note += " ";
  }
  lcd.print(Note);
  if (port[portNR][1] == 7) {
    lcd.setCursor(5, portNR);
    lcd.write(byte(2));
    lcd.setCursor(15, portNR);
    lcd.print(portNR + 1);
  }
}

//=====[ TX ]===================================================
void tx(byte addr, int portNR) {
  switch (port[portNR + 3][1]) {
    case 0: a = B00000000; break;
    case 1: a = B00000001; break;
    case 2: a = B00000010; break;
    case 3: a = B00000100; break;
    case 4: a = B00001000; break;
    case 5: a = B00010000; break;
    case 6: a = B00100000; break;
    case 7: a = B00000000; break;
  }
  if (port[portNR][3] == 1 && port[portNR - 1][1] != 0) {
    a = a | (1 << 7);
  }
  switch (port[portNR + 4][1]) {
    case 0: b = B00000000; break;
    case 1: b = B00000001; break;
    case 2: b = B00000010; break;
    case 3: b = B00000100; break;
    case 4: b = B00001000; break;
    case 5: b = B00010000; break;
    case 6: b = B00100000; break;
    case 7: b = B00000000; break;
  }
  if (port[portNR - 1][3] == 1 && port[portNR][1] != 0) {
    b = b | (1 << 7);
  }
  Wire.beginTransmission(port[portNR + 4][0]);
  Wire.write(0x12);
  Wire.write((byte)a);
  Wire.endTransmission();
  Wire.beginTransmission(port[portNR + 4][0]);
  Wire.write(0x13);
  Wire.write((byte)b);
  Wire.endTransmission();
}

//=====[ Volt ]===================================================
float volt(int raw) {
  float voltage = (raw * 5.0) / 1024.0 * 7.25 + 0.4;    // resistor coeficient
  #if defined(serialECHO)
    Serial.print("Input voltage ");
    Serial.println(voltage);
  #endif
  return voltage;
}




//=====[ RX ]===================================================
void rx(byte addr, int portNR, int PTTonly, int Bank) {
  Wire.beginTransmission(addr);
  Wire.write(0x12);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1);
#if defined(inputHigh)
  a = Wire.read();
#else
  a = ~Wire.read();
#endif
  Wire.beginTransmission(addr);
  Wire.write(0x13);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1);
  b = ~Wire.read();
  if (Bank == 1) {
    if (b & (1 << 1)) {
      port[portNR][2] = 1;
    } else {
      port[portNR][2] = 0;
    }
    if (PTTonly == 0) {
      if (a & (1 << 0)) {
        a = a | (1 << 7);
      } else {
        a = a & ~(1 << 7);
      };
      if (a & (1 << 1)) {
        a = a | (1 << 6);
      } else {
        a = a & ~(1 << 6);
      };
      if (a & (1 << 2)) {
        a = a | (1 << 5);
      } else {
        a = a & ~(1 << 5);
      };
      if (a & (1 << 3)) {
        a = a | (1 << 4);
      } else {
        a = a & ~(1 << 4);
      };
      a = a >> 4;

      port[portNR][1] = BCDmatrixINOUT[0][a];
    }
  } else if (Bank == 2) {
    if (b & (1 << 0)) {
      port[portNR][2] = 1;
    } else {
      port[portNR][2] = 0;
    }
    if (PTTonly == 0) {
      a = a >> 4;
      port[portNR][1] = BCDmatrixINOUT[1][a];

    }
  }
}
