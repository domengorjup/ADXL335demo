/*
 * GY-61 (ADXL3345) test sketch
 */
#include <stdio.h>

const int xpin = A0;
const int ypin = A1;
const int zpin = A2;

int xValue;
int yValue;
int zValue;

char record[50];

void setup() {
  Serial.begin(115200);

  // accTest se zgodi samo enkrat, nato izhod iz while!
  int sezgodi = 0;
  
  while(sezgodi < 1) {
    
    delay(100);
    accTest();
    sezgodi = 1;
    
  }
}

void loop() {}

void accTest() {
  
  int start = millis();

  while(millis() - start <= 1000) {
  
    xValue = analogRead(xpin);
    yValue = analogRead(ypin);
    zValue = analogRead(zpin);
    int tStamp = millis() - start;

    sprintf(record, "%d\t%d\t%d\t%d",tStamp,xValue,yValue,zValue);
    Serial.write(record);
    Serial.println();
    
  }

  Serial.println("end");
  
  
}
