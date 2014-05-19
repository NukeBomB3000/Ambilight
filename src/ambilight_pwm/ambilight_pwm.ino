// This is based on Adafruits Adalight: https://github.com/adafruit/Adalight

// Author: Norman Steinmeier: http://nukebomb3000.myftp.org/
// Feel free to contact me

// This is an Arduino Sketch for Communication between the PC Software "Prismatik",
// an Arduino Duemilanove and 6 seperate PWM-driven LEDs.

// This is used to define all 6 PWM Ports
#define rlPin 3
#define glPin 5
#define blPin 6
#define rrPin 9
#define grPin 10
#define brPin 11

static const uint8_t magic[] = {'A','d','a'};
#define MAGICSIZE  sizeof(magic)
#define HEADERSIZE (MAGICSIZE + 3)

#define MODE_HEADER 0
#define MODE_HOLD   1
#define MODE_DATA   2

static const unsigned long serialTimeout = 60000; // seconds

// These can be used to even out any brightness differences
float redMult = 1.0;
float greenMult = 1.0;
float blueMult = 1.0;

// The number of separate LED Channels
byte ledCount = 6;

void setup(){
  
  // Setting up all pinModes
  pinMode(rlPin, OUTPUT);
  pinMode(glPin, OUTPUT);
  pinMode(blPin, OUTPUT);
  pinMode(rrPin, OUTPUT);
  pinMode(grPin, OUTPUT);
  pinMode(brPin, OUTPUT);

  // Dirty trick: the circular buffer for serial data is 256 bytes,
  // and the "in" and "out" indices are unsigned 8-bit types -- this
  // much simplifies the cases where in/out need to "wrap around" the
  // beginning/end of the buffer.  Otherwise there'd be a ton of bit-
  // masking and/or conditional code every time one of these indices
  // needs to change, slowing things down tremendously.
  uint8_t
    buffer[256],
    indexIn       = 0,
    indexOut      = 0,
    mode          = MODE_HEADER,
    hi, lo, chk, i, spiFlag;
  int16_t
    bytesBuffered = 0,
    hold          = 0,
    c;
  int32_t
    bytesRemaining;
  unsigned long
    startTime,
    lastByteTime,
    lastAckTime,
    t;

  Serial.begin(115200); // Teensy/32u4 disregards baud rate; is OK!

  Serial.print("Ada\n"); // Send ACK string to host

  startTime    = micros();
  lastByteTime = lastAckTime = millis();

  for(;;) {

    // Implementation is a simple finite-state machine.
    // Regardless of mode, check for serial input each time:
    t = millis();
    if((bytesBuffered < 256) && ((c = Serial.read()) >= 0)) {
      
      buffer[indexIn++] = c;
      bytesBuffered++;
      lastByteTime = lastAckTime = t; // Reset timeout counters
    } else {
      // No data received.  If this persists, send an ACK packet
      // to host once every second to alert it to our presence.
      if((t - lastAckTime) > 1000) {
        Serial.print("Ada\n"); // Send ACK string to host
        lastAckTime = t; // Reset counter
      }
      // If no data received for an extended time, turn off all LEDs.
      if((t - lastByteTime) > serialTimeout) {
        for(c=0; c<32767; c++) {
          
          // This is what happens when the system reaches it's pause mode
          analogWrite(rlPin, 0);
          analogWrite(glPin, 0);
          analogWrite(blPin, 0);
          analogWrite(rrPin, 0);
          analogWrite(grPin, 0);
          analogWrite(brPin, 0);
		  
        }
        delay(1); // One millisecond pause = latch
        lastByteTime = t; // Reset counter
      }
    }

    switch(mode) {

     case MODE_HEADER:

      // In header-seeking mode.  Is there enough data to check?
      if(bytesBuffered >= HEADERSIZE) {
        // Indeed.  Check for a 'magic word' match.
        for(i=0; (i<MAGICSIZE) && (buffer[indexOut++] == magic[i++]););
        if(i == MAGICSIZE) {
          // Magic word matches.  Now how about the checksum?
          hi  = buffer[indexOut++];
          lo  = buffer[indexOut++];
          chk = buffer[indexOut++];
          if(chk == (hi ^ lo ^ 0x55)) {
            // Checksum looks valid.  Get 16-bit LED count, add 1
            // (# LEDs is always > 0) and multiply by 3 for R,G,B.
            bytesRemaining = 3L * (256L * (long)hi + (long)lo + 1L);
            bytesBuffered -= 3;
            spiFlag        = 0;         // No data out yet
            mode           = MODE_HOLD; // Proceed to latch wait mode
          } else {
            // Checksum didn't match; search resumes after magic word.
            indexOut  -= 3; // Rewind
          }
        } // else no header match.  Resume at first mismatched byte.
        bytesBuffered -= i;
      }
      break;

     case MODE_HOLD:

      // Ostensibly "waiting for the latch from the prior frame
      // to complete" mode, but may also revert to this mode when
      // underrun prevention necessitates a delay.

      if((micros() - startTime) < hold) break; // Still holding; keep buffering

      // Latch/delay complete.  Advance to data-issuing mode...
      mode      = MODE_DATA; // ...and fall through (no break):

     case MODE_DATA:

//      while(spiFlag); // Wait for prior byte
      if(bytesRemaining > 0) {
        if(bytesBuffered > 0) {
          if(bytesRemaining <= ledCount){
            switch(bytesRemaining){
			
			// This is where you would need to add more cases if you are using more then 6 LEDs
            
            case 1:
              analogWrite(brPin, buffer[indexOut++]*blueMult);
              break;
            case 2:
              analogWrite(grPin, buffer[indexOut++]*greenMult);
              break;
            case 3:
              analogWrite(rrPin, buffer[indexOut++]*redMult);
              break;
            case 4:
              analogWrite(blPin, buffer[indexOut++]*blueMult);
              break;
            case 5:
              analogWrite(glPin, buffer[indexOut++]*greenMult);
              break;
            case 6:
              analogWrite(rlPin, buffer[indexOut++]*redMult);
              break;
            }
          }
          bytesBuffered--;
          bytesRemaining--;
          spiFlag = 1;
        }
        // If serial buffer is threatening to underrun, start
        // introducing progressively longer pauses to allow more
        // data to arrive (up to a point).
        if((bytesBuffered < 32) && (bytesRemaining > bytesBuffered)) {
          startTime = micros();
          hold      = 100 + (32 - bytesBuffered) * 10;
          mode      = MODE_HOLD;
	}
      } else {
        // End of data -- issue latch:
        startTime  = micros();
        hold       = 1000;        // Latch duration = 1000 uS
        mode       = MODE_HEADER; // Begin next header search
      }
    } // end switch
  } // end for(;;)
}

void loop() {
  //
}
