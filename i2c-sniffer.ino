//
// i2c protocol sniffer for Arduino Uno (ATmega328)
//
// written by teknoid, 2018
//
// based on ideas from
//   Bill Grundmann's i2c sniffer (https://billgrundmann.wordpress.com/tag/i2c-sniffer/)
//   avr-freaks forum (https://www.avrfreaks.net/forum/i2c-non-intrusive-slave-monitoring-existing-address)
//

#include <Arduino.h>

// SCL  D2  00000100  0x04
// SDA  D3  00001000  0x08
// SCL+SDA  00001100  0x0c

#define SCL_MASK 0x04
#define SDA_MASK 0x08
#define I2C_MASK 0x0c

#define SCL (byte) (PIND & SCL_MASK)
#define SDA (byte) ((PIND >> 3) & 0x01)
#define I2C (byte) (PIND & I2C_MASK)

// how many START / STOP cycles we want to sample before dumping to console
#define MAX_SAMPLES         8

// how many samples we want for wire (00 01 10 11) debugging
#define MAX_WIRE_SAMPLES    255

// the sample buffer
char *b;

// determine largest area available
uint16_t availableMemory() {
  int size = 2048; // start with the most
  char *buf;
  while ((buf = (char *) malloc(--size)) == NULL) ;
  free(buf);
  return size;
}

void dump_queue(char *from, char *to) {
  int length = (int)to - (int)b;
  Serial.println();
  Serial.print("Queue was filled: ");
  Serial.println(length);
  char *ptr = from;
  while (ptr < to) {
    printhex(*ptr++);
    if (*ptr == '-') {
      Serial.write(*ptr++);
      Serial.println();
    }
    if (*ptr == '+') {
      Serial.write(*ptr++);
      Serial.print(" ");
    }
  }
  Serial.println();
  Serial.flush();
}

void printnibble(byte d) {
  if (d < 10) Serial.write(d + '0');
  else Serial.write(d - 10 + 'A');
}

void printhex(byte d) {
  printnibble(d >> 4);
  printnibble(d & 0xf);
}

void printbin(byte d) {
  Serial.write(' ');
  for (byte mask = 0b10000000; mask > 0; mask >>= 1) {
    if (d & mask) {
      Serial.write('1');
    } else {
      Serial.write('0');
    }
  }
  Serial.flush();
}

void sample_wire(char *from) {
  for (byte i = 0; i < MAX_WIRE_SAMPLES; i++) {
    *from++ = I2C;
  }
}

void dump_wire(char *from) {
  Serial.println();
  Serial.println("I2C wire samples");
  char data;
  for (byte i = 0; i < MAX_WIRE_SAMPLES; i++) {
    data = *from++;
    switch (data) {
      case SCL_MASK: Serial.print("01 "); break;
      case SDA_MASK: Serial.print("10 "); break;
      case I2C_MASK: Serial.print("11 "); break;
      default:       Serial.print("00 "); break;
    }
  }
  Serial.println();
  Serial.flush();
}

void debug() {
  byte d;
  char *p = b;
  byte count = MAX_SAMPLES;

i2c_idle:
  while (I2C != I2C_MASK);
  while (I2C == I2C_MASK);
  if (!SCL) goto i2c_idle;

  sample_wire(p);
  dump_wire(p);
}


void sample() {
  byte d, x;
  char *p = b;
  byte count = MAX_SAMPLES;

i2c_sync:
  if (I2C != I2C_MASK) goto i2c_sync;
i2c_idle:
  if (I2C == I2C_MASK) goto i2c_idle;
  if (SDA) goto i2c_sync; // not a START condition
  while (SCL);

i2c_start:
  // Slave Address
  while (!SCL); d  = SDA; d = d << 1; while (SCL);  // MSB
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; *p++ = d;   while (SCL);  // LSB
  while (!SCL);
  if (!SDA) {
    *p++ = '+';     // ACK
  } else {
    *p++ = '-';     // NACK
    goto i2c_sync;  // nobody there or busy
  }
  while (SCL);

  // Slave Register / Data
  while (!SCL); d  = SDA; d = d << 1; while (SCL);  // MSB
i2c_next:
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; d = d << 1; while (SCL);
  while (!SCL); d |= SDA; *p++ = d;   while (SCL);  // LSB
  while (!SCL);
  if (!SDA) {
    *p++ = '+';     // ACK
  } else {
    *p++ = '-';     // NACK
  }
  while (SCL);

  while (!SCL) x = SDA;
  d = SDA;
  if (!x) {
    // SDA was low on SCL rise --> STOP or continue
    x = I2C;
    while (x == I2C);
    if (SDA) {
      if (--count == 0) goto i2c_dump;
      goto i2c_idle; // SDA rises too --> STOP
    }
  } else {
    // SDA was high on SCL rise --> RESTART or continue
    x = I2C;
    while (x == I2C);
    if (SCL) {
      while (SCL);
      goto i2c_start;  // SCL rises too --> RESTART
    }
  }

  // continue
  d = d << 1;
  while (SCL);
  goto i2c_next;

i2c_dump:
  dump_queue(b, p);
  p = b;
  count = MAX_SAMPLES;
  goto i2c_idle;

i2c_debug:
  sample_wire(p);
  dump_queue(b, p);
  dump_wire(p);
  p = b;
  count = MAX_SAMPLES;
  goto i2c_idle;
}

void setup() {
  int memory_size = availableMemory();
  int queue_size = memory_size - 128;
  b = (char *) malloc(queue_size);

  // report various sizes
  Serial.begin(115200);
  Serial.println();
  Serial.print("Available memory size: ");
  Serial.println(memory_size);
  Serial.print("Queue size: ");
  Serial.println(queue_size);
  Serial.flush();

  noInterrupts();
}

void loop() {
  // debug();
  sample();
}
