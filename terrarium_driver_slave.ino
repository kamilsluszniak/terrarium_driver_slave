
#include <avr/interrupt.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

int one_wire = 5;

OneWire oneWire(one_wire);
DallasTemperature sensors(&oneWire);

char reports_buffer[22];
char command_buffer[32];
byte i2c_rcv = 0;
volatile int counter = 0;
volatile int t = 0;
float temp0 = 0;
float temp1 = 0;
float e = 0;
float e0 = 0;

float p = 0;
float i = 0;
float d = 0;
int triac_delay = 0;
float pid = 0.0;

float p_factor = 5.0;
float i_factor = 0.05;
float d_factor = 3.0;

bool light_on = false;
float temp0set = 30.0;
float dim_factor = 0;

char buffer[5];
String temp0str;
String temp1str;

int ind1;
int ind2;
int ind3;
int ind4;
int ind5;
int ind6;

int diodaPin = A4;

/**
   set update on a high edge
*/
ISR(PCINT0_vect) {
  if (PINB & (1 << PB2)) {
    /* LOW to HIGH pin change */
    TCNT1  = 0;//initialize counter value to 0

    if (true) {
      // 1 => 0.000064 s
      OCR1A = 1;// triac_delay
      TIMSK1 |= (1 << OCIE1A);
    }
    counter++;
  }
  else {
  }
}

ISR(TIMER1_COMPA_vect){  //change the 0 to 1 for timer1 and 2 for timer2
  TIMSK1 = 0;
  digitalWrite(12, HIGH);
  for (int i = 0; i < 200; i++) {
    t++; // make sure high state is long enough - blocking delay not using ISR
  }
  digitalWrite(12, LOW);
}


void setup()   {
  cli();
  PCICR |= 0b00000001;    // turn on port b
  PCMSK0 |= 0b00000100;   // PCINT2

  DDRB &= ~(1 << PB2);         // Clear the PB2 pin - input
  PORTB |= (1 << PB2);

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  Wire.begin(2);
  Wire.onReceive(dataRcv);
  Wire.onRequest(dataRqst);
  Serial.begin(115200);
//  sensors.begin();
  pinMode(12, OUTPUT);           // set pin to input
//  pinMode(11, OUTPUT);
//  digitalWrite(11, LOW);
  sei();
}

void loop() {
  delay(2000);
    sensors.requestTemperatures();
    temp0 = sensors.getTempCByIndex(0);
    temp1 = sensors.getTempCByIndex(1);
      char buffer[5];
    temp0str = dtostrf(temp0, 1, 2, buffer);
    temp1str = dtostrf(temp1, 1, 2, buffer);
  String msg = String(temp0str) + "," + String(temp1str);
  msg.toCharArray(reports_buffer, 22); //-127.00,-127.00
  Serial.println(reports_buffer);

  if (true) { // counter >= 100
    counter = 0;
    

    Serial.println("driver:");
    if (light_on) {
//      if (num_time >= (num_off_time - 60)) {
//        float delta_temp = temp0set - temp1;
//        e = temp1 + (delta_temp * slope) - temp0;

//      }
//      else {
        e = temp0set - temp0;
        Serial.print("e: ");
        Serial.println(e);
//      }

      if (pid <= 130) {
        i += e;
      }
      d = (e - e0);
      pid = p_factor * e + i_factor * i + d_factor * d;

      if (pid > 130) {
        pid = 130;
      }

      Serial.print("pid: ");
       Serial.println(pid);
      triac_delay = int(140 - dim_factor * pid);
      Serial.println(triac_delay);


      if (triac_delay > 140) {
        triac_delay = 140;
      }
      if (triac_delay < 10) {
        triac_delay = 10;
      }

      e0 = e;
    }
    else {
      triac_delay = 140;
      pid = 0;
      i = 0;
      d = 0;
    }


    if (light_on) {
      digitalWrite(6, HIGH);
    }
    else {
      digitalWrite(6, LOW);
    }
  }
}

//received data handler function
void dataRcv(int howMany){
  String command_string;
  while(Wire.available()) { // read all bytes received
    for (int i = 0; i < howMany; i++) {
      command_buffer[i] = Wire.read();
    }
  }
  command_string = command_buffer;
  
  Serial.println();
  Serial.println("captured String is :");
  Serial.println(command_string); //prints string to serial port out
 
  ind1 = command_string.indexOf(',');  //finds location of first ,
  light_on = command_string.substring(0, ind1).toInt() == 1 ? true : false;   //captures first data String
  ind2 = command_string.indexOf(',', ind1+1 );   //finds location of second ,
  temp0set = command_string.substring(ind1+1, ind2+1).toFloat();   //captures second data String
  ind3 = command_string.indexOf(',', ind2+1 );
  dim_factor = command_string.substring(ind2+1, ind3+1).toFloat();
  ind4 = command_string.indexOf(',', ind3+1 );
  p_factor = command_string.substring(ind3+1, ind4+1).toFloat();
  ind5 = command_string.indexOf(',', ind4+1 );
  i_factor = command_string.substring(ind4+1, ind5+1).toFloat();
  ind6 = command_string.indexOf(',', ind5+1 );
  d_factor = command_string.substring(ind5+1).toFloat();

  Serial.print("light_on = ");
  Serial.println(light_on);
  Serial.print("temp0set = ");
  Serial.println(temp0set);
  Serial.print("dim_factor = ");
  Serial.println(dim_factor);
  Serial.print("p_factor = ");
  Serial.println(p_factor);
  Serial.print("i_factor = ");
  Serial.println(i_factor);
  Serial.print("d_factor = ");
  Serial.println(d_factor);
  Serial.println();
  Serial.println();
}

// requests data handler function
void dataRqst(){

  Wire.write(reports_buffer);
}
