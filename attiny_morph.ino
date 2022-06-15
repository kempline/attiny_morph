// #include <SoftwareSerial.h>
#include <MIDI.h>
#include <pt.h>

#include <NeoSWSerial.h>

//#define WITH_FBV
#define WITH_MIDI


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

// SoftwareSerial midiSerial(0, 1);
#ifdef WITH_MIDI
  NeoSWSerial midiSerial(0, 1);
#endif

#ifdef WITH_FBV

//SoftwareSerial fbvSerial(2, 3); // RX, TX
  NeoSWSerial fbvSerial(2, 3); // RX, TX
  #define PIN_RE_DE 4
#endif

static struct OpenMorph morphThread;
static struct Switch3Hold switchHoldThread;

// void onMorphBegin(uint8_t controlId, MorphDir dir, uint8_t startValue, uint8_t stopValue) {}

#ifdef WITH_MIDI
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
  // midiSerial.listen();
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
#endif

// ----------------------------------------------------------------------
// FBV
// ----------------------------------------------------------------------
namespace FBV {
  const uint8_t BANK_DOWN = 0x00;
  const uint8_t BANK_UP = 0x10;
  const uint8_t CHANNEL_A = 0x20;
  const uint8_t CHANNEL_B = 0x30;
  const uint8_t CHANNEL_C = 0x40;
  const uint8_t CHANNEL_D = 0x50;
  const uint8_t FAV_CHANNEL = 0x60;
  const uint8_t AMP1 = 0x01;
  const uint8_t AMP2 = 0x11;
  const uint8_t REVERB = 0x21;
  const uint8_t TREMOLO = 0x31;
  const uint8_t MODULATION = 0x41;
  const uint8_t DELAY = 0x51;
  const uint8_t TAP_TEMPO = 0x61;
  const uint8_t FX_LOOP = 0x02;
  const uint8_t STOMP_BOX1 = 0x12;
  const uint8_t STOMP_BOX2 = 0x22;
  const uint8_t STOMP_BOX3 = 0x32;
  const uint8_t PEDAL1_SWITCH = 0x43;
  const uint8_t PEDAL1_GREEN = 0x03;
  const uint8_t PEDAL1_RED = 0x13;
  const uint8_t PEDAL2_SWITCH = 0x53;
  const uint8_t PEDAL2_GREEN = 0x23;
  const uint8_t PEDAL2_RED = 0x33;
  const uint8_t NO_COLOR = 0x00;
}

uint8_t bufferIn[16];
uint8_t bufferPos, inByte, pedal1State;
uint8_t lastChannelSwitchId;
uint8_t currentProg = 0;

#ifdef WITH_FBV
void switchLed(uint8_t hwId, uint8_t state) {
  digitalWrite(PIN_RE_DE, HIGH); delay(1);
  if(FBV::PEDAL1_SWITCH == hwId) {
    uint8_t cmd1[5]={0xF0, 0x03, 0x04, FBV::PEDAL1_GREEN, !state};
    uint8_t cmd2[5]={0xF0, 0x03, 0x04, FBV::PEDAL1_RED, state};
    fbvSerial.write(cmd1, 5);
    fbvSerial.write(cmd2, 5);
  }
  else if(FBV::PEDAL2_SWITCH == hwId) {
    uint8_t cmd1[5]={0xF0, 0x03, 0x04, FBV::PEDAL2_GREEN, !state};
    uint8_t cmd2[5]={0xF0, 0x03, 0x04, FBV::PEDAL2_RED, state};
    fbvSerial.write(cmd1, 5);
    fbvSerial.write(cmd2, 5);
  }
  else {
    uint8_t cmd1[5]={0xF0, 0x03, 0x04, hwId, state};
    fbvSerial.write(cmd1, 5);
  }
  fbvSerial.flush(); 
  digitalWrite(PIN_RE_DE, LOW);
}

void onSwitchEvent(uint8_t switchId, uint8_t value) {
  if(switchId == FBV::TAP_TEMPO && lastChannelSwitchId == FBV::CHANNEL_D)
    switchId = FBV::CHANNEL_D;

  // if(m_activeChannel == switchId)
  //  switchId = FBV::TAP_TEMPO;
  
  switch(switchId) {
    case(FBV::CHANNEL_A):
    case(FBV::CHANNEL_B):
    case(FBV::CHANNEL_C):
    {
      if(0 == value)
        return;

      switchLed(FBV::CHANNEL_A, 0x00);
      switchLed(FBV::CHANNEL_B, 0x00);
      switchLed(FBV::CHANNEL_C, 0x00);
      switchLed(switchId, 0x01);
      
      // openHxStomp.switchProgram(m_currentProg);
      currentProg = byte(currentProg/3) * 3 + (switchId/0x10 - 2);
#ifdef WITH_MIDI
      sendProgramChange(HX_STOMP_MIDI_CHANNEL, currentProg);
#endif
      break;
    }
    case(FBV::CHANNEL_D):
      
      break;
    case(FBV::TAP_TEMPO):
      
      break;
    case(FBV::PEDAL1_SWITCH):
      if(0 == value)
        return;
      
      break;
    default:
      break;
  }
  
  //if(switchId == FBV::TAP_TEMPO)
  //  switchId = m_lastSwitchId;
}

void onPedalEvent(uint8_t hwId, uint8_t value) {
  switch(pedal1State) {
    case(0):
      // openHxStomp.sendControlChange(VOL_CC, value, HX_STOMP_MIDI_CHANNEL);
      break;
    default:
      // openHxStomp.sendControlChange(WAH_CC, value, HX_STOMP_MIDI_CHANNEL);
      break;
  }
}
  
bool fbvRead() {
  //fbvSerial.listen();
  if (fbvSerial.available()) {
    inByte = fbvSerial.read();
    
    if(0xF0 == inByte)
      bufferPos = 0;

    bufferPos %= 16;
    bufferIn[bufferPos++] = inByte;

    if(bufferPos <= 1)
      return true;

    if(bufferIn[1] == bufferPos - 2) { // minus two for the first bytes (244, x; x = len of parameters)
      // we have a complete message here

      uint8_t fctCode = bufferIn[2];
      uint8_t idx = 0;
      switch(fctCode) {
        case(0x81):
          onSwitchEvent(bufferIn[3], bufferIn[4]);
          if(bufferIn[3] != FBV::TAP_TEMPO && bufferIn[3] != FBV::AMP1) 
            lastChannelSwitchId = bufferIn[3];
          break;
        case(0x82):
          onPedalEvent(bufferIn[3], bufferIn[4]);
          break;
        case(0x30):
          break;
        case(0x90):
          // expectingKeepAliveSignal = 0;
          break;
        default:
          break;
      }
      bufferPos = 0;
    }
    return true;    
  }
  return false;
}
#endif

void setup() {
  
#ifdef WITH_FBV
  fbvSerial.begin(31250);
  pinMode(PIN_RE_DE, OUTPUT);
  digitalWrite(PIN_RE_DE, LOW);
#endif

#ifdef WITH_MIDI
  midiSerial.begin(31250);
#endif
  PT_INIT(&morphThread);
  PT_INIT(&switchHoldThread);
}

void loop() {
#ifdef WITH_MIDI
  midiRead();
  onMorph(&morphThread);
  onSwitchHold(&switchHoldThread);
#endif
#ifdef WITH_FBV
  fbvRead();
#endif

  
  /*uint8_t cmd1[5]={0xF0, 0x03, 0x04, FBV::CHANNEL_A, 0x00};
    fbvSerial.write(cmd1, 5);
  delay(1000);
  uint8_t cmd2[5]={0xF0, 0x03, 0x04, FBV::CHANNEL_A, 0x7f};
    fbvSerial.write(cmd2, 5);
  delay(1000);
  */
}
