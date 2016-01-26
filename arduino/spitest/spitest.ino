#include <SPI.h>


/* SPI Test
 * Allows manual interaction with an SPI interface via the serial link for testing purposes.
 * 
 * Simply enter up to 16 hexadecimal Bytes and a newline and see the response.
 */


const int PIN_nCS = 10;

SPISettings SPI_CONF(10000000, MSBFIRST, SPI_MODE0);

void setup() {
  Serial.begin(9600);
  
  SPI.begin();
  
  pinMode(PIN_nCS, OUTPUT);
  digitalWrite(PIN_nCS, HIGH);
}

void loop() {
  byte command[16];
  int byteCount;
  while (1) {
    byteCount = readCommand(command, 16);
    if (byteCount <= 0) break;
    performCommand(command, byteCount);
  }
  
  Serial.println(F("No input received. Terminating."));
  while(1);
}

int readCommand(byte * command, int maxBytes) {
  Serial.print("< ");
  int nextChar;
  int byteCount = 0;
  boolean firstDigit = false;
  do {
    int digit = -1;
    while (!Serial.available());
    nextChar = Serial.read();
    if (nextChar < 0 || nextChar == '\n') {
      break;
    } else if ('0' <= nextChar && nextChar <= '9') {
      digit = nextChar - '0';
    } else if ('a' <= nextChar && nextChar <= 'f') {
      digit = nextChar - 'a' + 10;
    } else if ('A' <= nextChar && nextChar <= 'F') {
      digit = nextChar - 'A' + 10;
    }
    if (digit >= 0) {
      Serial.print(digit, HEX);
      if (firstDigit) {
        command[byteCount] += digit;
        firstDigit = false;
        ++byteCount;
        Serial.print(" ");
      } else {
        command[byteCount] = (digit << 4);
        firstDigit = true;
      }
    }
  } while(byteCount < maxBytes);
  Serial.println();
  return byteCount;
}

void performCommand(byte * command, int byteCount) {
  Serial.print(">");
  SPI.beginTransaction(SPI_CONF);
  digitalWrite(PIN_nCS, LOW);
  for(int i = 0; i < byteCount; ++i) {
    byte result = SPI.transfer(command[i]);
    Serial.print(" ");
    Serial.print(result, HEX);
  }
  digitalWrite(PIN_nCS, HIGH);
  SPI.endTransaction();
  Serial.println();
}

