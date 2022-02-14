#include <SoftwareSerial.h>
#include <MIDI.h>
#include <pt.h>

enum MorphDir {
  inc = 0,
  dec = 1
};

class OpenMorph : public pt {
  public:
    bool stopRequested = false;
    bool inProgress = false;
    uint8_t controllerId = 0;
    MorphDir dir;
    unsigned long morphTime;    
};

class Switch3Hold : public pt {
  public:
    bool stopRequested = false;
    bool inProgress = false;
};

uint8_t midiInByte;
uint8_t midiBufferIn[16];
uint8_t midiInPos = 0;

#define HX_MORPH_0_SET_INC_TIME_CC 15
#define HX_MORPH_0_SET_DEC_TIME_CC 16
#define HX_MORPH_0_CONTROL_CC 18
uint16_t HX_MORPH_0_MORPH_INC_TIME_MS = 0;
uint16_t HX_MORPH_0_MORPH_DEC_TIME_MS = 0;

#define HX_MORPH_1_SET_INC_TIME_CC 20
#define HX_MORPH_1_SET_DEC_TIME_CC 21
#define HX_MORPH_1_CONTROL_CC 23
uint16_t HX_MORPH_1_MORPH_INC_TIME_MS = 0;
uint16_t HX_MORPH_1_MORPH_DEC_TIME_MS = 0;

#define HX_MORPH_2_SET_INC_TIME_CC 25
#define HX_MORPH_2_SET_DEC_TIME_CC 26
#define HX_MORPH_2_CONTROL_CC 28
uint16_t HX_MORPH_2_MORPH_INC_TIME_MS = 0;
uint16_t HX_MORPH_2_MORPH_DEC_TIME_MS = 2000;

#define HX_STOMP_MIDI_CHANNEL 3 // HX Stomp says it is 4 but sends 3
#define HX_STOMP_TOGGLE_TUNER_CC 68

#define BANDHELPER_NOTE_B_m1 11
#define BAND_HELPER_MIDI_CHANNEL 3

#define C0 12
#define C1 24
#define C2 36
#define PROGRAM_CHANGE_NOTE_C_m1 0

#define MIDI_NOTE_OFF 0x80
#define MIDI_NOTE_ON 0x90
#define MIDI_PC 0xc0
#define MIDI_CC 0xb0

SoftwareSerial midiSerial(0, 1);

static struct OpenMorph morphThread;
static struct Switch3Hold switchHoldThread;

// void onMorphBegin(uint8_t controlId, MorphDir dir, uint8_t startValue, uint8_t stopValue) {}

void sendProgramChange(byte channel, byte number) {
  midiSerial.write(MIDI_PC + channel);
  midiSerial.write(number);
}

void sendControlChange(byte channel, byte number, byte value) {
  midiSerial.write(MIDI_CC + channel);
  midiSerial.write(number);
  midiSerial.write(value);
}

void onMorphUpdate(uint8_t controlId, uint8_t currentValue) {
  if(currentValue%2 == 0 && currentValue != 0) // reduce message quantity for hx stomp
      return;

  switch (controlId) {
    case (0):
      sendControlChange(HX_STOMP_MIDI_CHANNEL, HX_MORPH_0_CONTROL_CC, currentValue);
      break;
    case (1):
      sendControlChange(HX_STOMP_MIDI_CHANNEL, HX_MORPH_1_CONTROL_CC, currentValue);
      break;
    case (2):
      sendControlChange(HX_STOMP_MIDI_CHANNEL, HX_MORPH_2_CONTROL_CC, currentValue);
      break;
    default:
      break;
  }
}

// void onMorphEnd(uint8_t controlId, MorphDir dir) {}

void onControlChange(byte channel, byte number, byte value) {
  if (channel != HX_STOMP_MIDI_CHANNEL) {
    return;
  }

  switch (number) {
    case(HX_MORPH_2_SET_INC_TIME_CC):
      HX_MORPH_2_MORPH_INC_TIME_MS = value * 100;
      break;
    case(HX_MORPH_2_SET_DEC_TIME_CC):
      HX_MORPH_2_MORPH_DEC_TIME_MS = value * 100;
      break;
      
    case(HX_MORPH_1_SET_INC_TIME_CC):
      HX_MORPH_1_MORPH_INC_TIME_MS = value * 100;
      break;
    case(HX_MORPH_1_SET_DEC_TIME_CC):
      HX_MORPH_1_MORPH_DEC_TIME_MS = value * 100;
      break;

    case(HX_MORPH_0_SET_INC_TIME_CC):
      HX_MORPH_0_MORPH_INC_TIME_MS = value * 100;
      break;
    case(HX_MORPH_0_SET_DEC_TIME_CC):
      HX_MORPH_0_MORPH_DEC_TIME_MS = value * 100;
      break;
    
    default:
      break;
  }
}

void onProgramChange(byte channel, byte number) {
  morphReset();
  if(switchHoldThread.inProgress) {
    switchHoldThread.stopRequested = true;
  }
}

void onNoteOn(byte channel, byte note, byte velocity) {    
  if (channel != HX_STOMP_MIDI_CHANNEL) {
    return;
  }

  switch (note) {
    case (PROGRAM_CHANGE_NOTE_C_m1):
      sendProgramChange(HX_STOMP_MIDI_CHANNEL, velocity);
      break;
    case (C2):
      morphThread.controllerId = 2;
      morphExec(MorphDir::inc, HX_MORPH_2_MORPH_INC_TIME_MS);
      break;
    case (C1):
      morphThread.controllerId = 1;
      morphExec(MorphDir::inc, HX_MORPH_1_MORPH_INC_TIME_MS);
      break;
    case (C0):
      morphThread.controllerId = 0;
      morphExec(MorphDir::inc, HX_MORPH_0_MORPH_INC_TIME_MS);
      break;
    case(BANDHELPER_NOTE_B_m1):
      switchHoldThread.inProgress = true;
      break;
    default:
      break;
  }
}

void onNoteOff(byte channel, byte note, byte velocity) {

  if (channel != HX_STOMP_MIDI_CHANNEL)
    return;

  switch (note) {
    case (C2):
      morphThread.controllerId = 2;
      if (morphThread.dir == MorphDir::inc) {
        morphExec(MorphDir::dec, HX_MORPH_2_MORPH_DEC_TIME_MS);
      }
      break;
    case (C1):
      morphThread.controllerId = 1;
      if (morphThread.dir == MorphDir::inc) {
        morphExec(MorphDir::dec, HX_MORPH_1_MORPH_DEC_TIME_MS);
      }
      break;
    case (C0):
      morphThread.controllerId = 0;
      if (morphThread.dir == MorphDir::inc) {
        morphExec(MorphDir::dec, HX_MORPH_0_MORPH_DEC_TIME_MS);
      }
      break;
    case(BANDHELPER_NOTE_B_m1):
      if(switchHoldThread.inProgress) {
        switchHoldThread.stopRequested = true;
        sendControlChange(BAND_HELPER_MIDI_CHANNEL, 83, 0x7f);
      }
      break;
    default:
      break;
  }
}

void midiRead() {
  while(midiSerial.available()) {

    midiInByte = midiSerial.read();
  
    // check for command
    switch(midiInByte & 0xf0) {
      case(MIDI_NOTE_OFF):
      case(MIDI_NOTE_ON):
      case(MIDI_CC):
      case(MIDI_PC):
        midiInPos = 0;
        midiBufferIn[midiInPos++] = midiInByte;
        break;
  
      default:
        midiBufferIn[midiInPos++] = midiInByte;
        break;
    }
    midiInPos %= 16;
  
    if(midiInPos == 2 && (midiBufferIn[0] & 0xf0) == MIDI_PC) {
      onProgramChange(midiBufferIn[0] & 0x0f, midiBufferIn[1]);
      midiInPos = 0;
    }
    else if(midiInPos == 3) {
      switch(midiBufferIn[0] & 0xf0) {
        case(MIDI_NOTE_OFF):
          onNoteOff((midiBufferIn[0] & 0x0f), midiBufferIn[1], midiBufferIn[2]);
          break;
        case(MIDI_NOTE_ON):
          if(midiBufferIn[2] == 0 && midiBufferIn[1] != PROGRAM_CHANGE_NOTE_C_m1)
            // Special case: Usually, a NoteOn on with velocity=0 is a Note Off
            // But I need NoteOn with velocity=0 when switching to preset 1
            onNoteOff((midiBufferIn[0] & 0x0f), midiBufferIn[1], midiBufferIn[2]);
          else
            onNoteOn((midiBufferIn[0] & 0x0f), midiBufferIn[1], midiBufferIn[2]);
          break;
        case(MIDI_CC):
          onControlChange((midiBufferIn[0] & 0x0f), midiBufferIn[1], midiBufferIn[2]);
          break;
      }
      midiInPos = 0;
    }
  }
}

void setup() {
  midiSerial.begin(31250);
  PT_INIT(&morphThread);
  PT_INIT(&switchHoldThread);
}

void loop() {
  midiRead();
  onMorph(&morphThread);
  onSwitchHold(&switchHoldThread);
}

void onSwitchHold(struct Switch3Hold *pt) {
  static unsigned long lastMeasurePoint = 0;
  static unsigned long holdBegin = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, true == pt->inProgress);

    holdBegin = millis();
    PT_WAIT_UNTIL(pt, (millis() > (holdBegin + 2000)) || pt->stopRequested);
    
    if (false == pt->stopRequested) {
      sendControlChange(HX_STOMP_MIDI_CHANNEL, HX_STOMP_TOGGLE_TUNER_CC, 0x01);
    }
    pt->stopRequested = false;
    pt->inProgress = false;
  }
  PT_END(pt);
}

void onMorph(struct OpenMorph *pt) {
  static unsigned long delayTime;
  static unsigned long lastMorphUpdate = 0;

  static uint8_t start_value = 0;
  static uint8_t stop_value = 127;
  static uint8_t current_value;
  static int8_t increment = 1;

  // static unsigned long morphTime = pt->morphTime;
  PT_BEGIN(pt);

  while (1) {
    PT_WAIT_UNTIL(pt, true == pt->inProgress);

    if (pt->dir == MorphDir::dec) {
      start_value = 127;
      stop_value = 0;
      increment = -1;
    }
    else {
      start_value = 0;
      stop_value = 127;
      increment = 1;
    }
    // morphTime = pt->morphTime;

    delayTime = pt->morphTime / 128.0;

    if (pt->morphTime == 0) {
      onMorphUpdate(pt->controllerId, stop_value);
      pt->inProgress = false;
    }
    else {
      current_value = start_value;

      while (current_value != stop_value) {
        if (pt->stopRequested) {
          onMorphUpdate(pt->controllerId, stop_value);
          pt->stopRequested = false;
          break;
        }
        lastMorphUpdate = millis();
        onMorphUpdate(pt->controllerId, current_value);

        PT_WAIT_UNTIL(pt, millis() - lastMorphUpdate > delayTime);
        current_value += increment;
      }
      onMorphUpdate(pt->controllerId, stop_value);
    }
    pt->inProgress = false;
  }
  PT_END(pt);
}

void morphReset() {
  if(morphThread.inProgress) {
    morphThread.stopRequested = true;
    delay(1.5 * (morphThread.morphTime / 128.0));
    morphThread.inProgress = false;
  }
  morphThread.dir = MorphDir::dec;
}

void morphExec(MorphDir dir, unsigned long morph_time) {

  if(morphThread.dir != MorphDir::inc && morphThread.dir != MorphDir::dec)
    return;
  
  morphReset();
  
  morphThread.dir = dir;
  morphThread.morphTime = morph_time;
  
  morphThread.inProgress = true; // activates the thread
}
