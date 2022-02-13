#include <SoftwareSerial.h>
#include <MIDI.h>
#include <pt.h>

enum MorphDir {
  inc = 0,
  dec = 1
};


class OpenMorph : public pt {
  public:
    // OpenMorph();
    //void registerIOpenMorphCb(IOpenMorph *p_pIOpenMorphCb);
    bool stopRequested = false;
    bool inProgress = false;
    uint8_t controllerId = 0;
    MorphDir dir;
    uint16_t morphTime;    
    //void reset();
    //void exec(MorphDir dir, uint16_t morph_time);
    //static int do_morph(struct OpenMorph *pt);
};

#define HX_MORPH_0_CONTROL_CC 18
uint16_t HX_MORPH_0_MORPH_INC_TIME_MS = 0;
uint16_t HX_MORPH_0_MORPH_DEC_TIME_MS = 2000;

#define HX_MORPH_1_CONTROL_CC 23
uint16_t HX_MORPH_1_MORPH_INC_TIME_MS = 0;
uint16_t HX_MORPH_1_MORPH_DEC_TIME_MS = 0;

#define HX_STOMP_MIDI_CHANNEL 4
#define PROGRAM_CHANGE_NOTE_C_m1 0

#define C0 12
#define C1 24

SoftwareSerial midiSerial(0, 1);
MIDI_CREATE_INSTANCE(HardwareSerial, midiSerial, hxMidiInterface);

uint8_t prog_no = 0;

static struct OpenMorph morph1;



void onMorphBegin(uint8_t controlId, MorphDir dir, uint8_t startValue, uint8_t stopValue) {}

void onMorphUpdate(uint8_t controlId, uint8_t currentValue) {
  switch (controlId) {
    case (0):
      midiSerial.write(HX_STOMP_MIDI_CHANNEL + 0xB0);
      midiSerial.write(HX_MORPH_0_CONTROL_CC);
      midiSerial.write(currentValue);
      break;
    case (1):
      midiSerial.write(HX_STOMP_MIDI_CHANNEL + 0xB0);
      midiSerial.write(HX_MORPH_1_CONTROL_CC);
      midiSerial.write(currentValue);
      break;
    default:
      break;
  }
}

void onMorphEnd(uint8_t controlId, MorphDir dir) { }


void onControlChange(byte channel, byte number, byte value) {
  if (channel != HX_STOMP_MIDI_CHANNEL) {
    return;
  }

  switch (number) {
    case (68):
      morph1.controllerId = 0;
      if (morph1.dir == MorphDir::inc)
        morphExec(MorphDir::dec, HX_MORPH_0_MORPH_DEC_TIME_MS);
      else
        morphExec(MorphDir::inc, HX_MORPH_0_MORPH_INC_TIME_MS);
      break;
    case (69):
      morph1.controllerId = 1;
      if (morph1.dir == MorphDir::inc)
        morphExec(MorphDir::dec, HX_MORPH_1_MORPH_DEC_TIME_MS);
      else
        morphExec(MorphDir::inc, HX_MORPH_1_MORPH_INC_TIME_MS);
      break;
    default:
      break;
  }
}
/*
  void onProgramChange(byte channel, byte number) {
  channel++;
  if (channel >= 15)
    channel = 0;
  }
*/
void onNoteOn(byte channel, byte note, byte velocity) {
  if (channel != HX_STOMP_MIDI_CHANNEL) {
    return;
  }
  switch (note) {
    case (PROGRAM_CHANGE_NOTE_C_m1):
      midiSerial.write(channel + 0xC0);
      midiSerial.write(velocity);
      break;
    case (C0):
      //morph1.controllerId = 0;
      morphExec(MorphDir::inc, HX_MORPH_0_MORPH_INC_TIME_MS);
      break;
    case (C1):
      //morph1.controllerId = 1;
      morphExec(MorphDir::inc, HX_MORPH_1_MORPH_INC_TIME_MS);
      break;
    default:
      break;
  }
}

void onNoteOff(byte channel, byte note, byte velocity) {

  if (channel != HX_STOMP_MIDI_CHANNEL)
    return;

  switch (note) {
    case (C0):
      morph1.controllerId = 0;
      if (morph1.dir == MorphDir::inc) {
        morphExec(MorphDir::dec, HX_MORPH_0_MORPH_DEC_TIME_MS);
      }
      break;
    case (C1):
      morph1.controllerId = 1;
      if (morph1.dir == MorphDir::inc) {
        morphExec(MorphDir::dec, HX_MORPH_1_MORPH_DEC_TIME_MS);
      }
      break;
    default:
      break;
  }
}

void setup() {
  midiSerial.begin(31250);
  // hxMidiInterface.setHandleProgramChange(onProgramChange);
  hxMidiInterface.setHandleControlChange(onControlChange);
  hxMidiInterface.setHandleNoteOn(onNoteOn);
  hxMidiInterface.setHandleNoteOff(onNoteOff);

  PT_INIT(&morph1);
}

void loop() {
  hxMidiInterface.read();
  do_morph(&morph1);
}


void do_morph(struct OpenMorph *pt) {
  static uint16_t delayTime = pt->morphTime / 128.0;
  static uint16_t lastMorphUpdate = 0;
  static uint8_t i;

  static uint8_t start_value = 0;
  static uint8_t stop_value = 127;
  static uint8_t current_value;
  static int8_t increment = 1;

  static unsigned long morphTime = pt->morphTime;
  PT_BEGIN(pt);

  while (1) {
    PT_WAIT_UNTIL(pt, true == pt->inProgress);

    if (pt->dir == MorphDir::dec) {
      start_value = 127;
      stop_value = 0;
      increment = -1;
      //morphTime = HX_MORPH_0_MORPH_DEC_TIME_MS;
    }
    else {
      start_value = 0;
      stop_value = 127;
      increment = 1;
      //morphTime = HX_MORPH_0_MORPH_INC_TIME_MS;
    }
    morphTime = pt->morphTime;
    delayTime = morphTime / 128.0;

    onMorphBegin(pt->controllerId, pt->dir, start_value, stop_value);

    if (morphTime == 0) {
      onMorphUpdate(pt->controllerId, stop_value);
      pt->inProgress = false;
    }
    else {
      current_value = start_value;

      while (current_value != stop_value) {
        if (pt->stopRequested) {
          onMorphUpdate(pt->controllerId, stop_value);
          onMorphEnd(pt->controllerId, pt->dir);
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
  if(morph1.inProgress) {
    morph1.stopRequested = true;
    delay(1.5 * (morph1.morphTime / 128.0));
    morph1.inProgress = false;
  }
  morph1.dir = MorphDir::dec;
}

void morphExec(MorphDir dir, uint16_t morph_time) {

  if(morph1.dir != MorphDir::inc && morph1.dir != MorphDir::dec)
    return;
  
  morphReset();
  
  morph1.dir = dir;
  morph1.morphTime = morph_time;
  
  morph1.inProgress = true; // activates the thread
}
