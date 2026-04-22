// =============================================================================
// midi4plus1.ino  —  MIDI to 4× CV/Gate converter for Arduino Nano
// Original by joeSeggiola — https://github.com/joeSeggiola/arduino-eurorack-projects
//
// Refactored & fixed:
//   1. GATE_RETRIG_MS 40 ms → 5 ms  (fits fast ratchet / glitch patterns)
//   2. loop() drains ALL pending MIDI messages per iteration
//   3. Retrig state machine: NoteOn during gate-LOW gap is queued, not lost
//   4. velocity=0 NoteOn treated as NoteOff (MIDI spec §1.4)
//   5. CC#120 / CC#123  All Notes Off  (MIDI Panic)
//   6. CC#64  Sustain Pedal
// =============================================================================

// =============================================================================
// CONFIGURATION
// =============================================================================

// Pin 1  → MIDI RX (hardware, do not change)
// A4/A5  → DAC I²C (hardware, do not change)

const byte MODE_BUTTON   = A0;              // Mode button
const byte MODE_LEDS[]   = { A1, A2, A3 }; // RGB LED (R, G, B)
const bool MODE_LEDS_PWM = true;            // false → use resistors for brightness

const byte GATES[]       = { 3, 4, 5, 6 }; // Gate output pins (voice 1–4)
const byte GATE_OR       = 7;               // OR gate / clock output pin
const byte GATES_LEDS[]  = { 8, 9, 10, 11 };
const byte GATE_OR_LED   = 12;
const byte NOTE_ON_LED   = 13;              // Blinks on every incoming NoteOn

// --- Retrig / Gate timing ----------------------------------------------------
//
// GATE_RETRIG_MS : duration the gate is held LOW between two consecutive notes
//                  on the same voice, so the envelope re-triggers.
//
//   Too long  → gate LOW gap outlasts the note interval → gates fuse → no retrig
//   Too short → envelope may not detect the LOW edge
//
//   Guideline:
//     32nd @ 200 BPM = 37 ms interval  →  5 ms fits easily
//     For sluggish envelopes raise to 10–15 ms.
//
const unsigned int GATE_RETRIG_MS   = 5;    // [ms]  gate LOW gap for re-trigger
const bool         GATE_RETRIG_MONO = false; // true → retrig even on legato mono

// --- Pitch / Split -----------------------------------------------------------
const byte PITCH_BEND_SEMITONES = 2;        // Pitch-bend range in semitones
const byte SPLIT_MIDI_OCTAVE    = 4;        // Keyboard split point (poly+mono modes)
const int  LOWEST_MIDI_OCTAVE   = 2;        // MIDI octave mapped to 0 V on CV out

// --- MIDI Clock output -------------------------------------------------------
const bool         CLOCK         = false;   // true → OR gate outputs MIDI clock
const unsigned int CLOCK_PPQ     = 24;      // 24=quarter, 12=8th, 6=16th note trigger
const unsigned int CLOCK_TRIG_MS = 10;      // Clock trigger width [ms]

// --- UI ----------------------------------------------------------------------
const unsigned long BUTTON_LOCK_LONG_PRESS_MS = 500;
const unsigned long BUTTON_DEBOUNCE_DELAY     = 50;
const unsigned long LED_MODE_LOCK_DURATION_MS = 500;
const unsigned long LED_MIN_DURATION_MS       = 50;

// =============================================================================
// DEBUGGING
// =============================================================================

const bool DEBUG              = false;
const bool DEBUG_WITH_TTYMIDI = false;

// =============================================================================
// INCLUDES
// =============================================================================

#include <EEPROM.h>
#include <MIDI.h>
#include <SoftPWM.h>
#include <Wire.h>

#include "lib/Button.cpp"
#include "lib/Led.cpp"
#include "lib/MCP4728.cpp"
#include "lib/MultiPointMap.cpp"

#include "mono.cpp"
#include "poly.cpp"

// =============================================================================
// CONSTANTS
// =============================================================================

#define N 4   // Number of voices

// Operating modes
enum Mode : byte {
    MODE_POLY       = 0,  // 4-voice poly, last-note priority
    MODE_POLY_FIRST = 1,  // 4-voice poly, first-note priority
    MODE_POLY_MONO  = 2,  // Split: 3-voice poly (low) + mono (high)
    MODE_MONO_POLY  = 3,  // Split: mono (low) + 3-voice poly (high)
    MODE_MONO       = 4,  // 4x mono on MIDI channels 1-4
    MODE_COUNT      = 5
};

// Mode LED colours  (24-bit 0xRRGGBB for SoftPWM)
#define MODE_POLY_RGB       0x330000UL
#define MODE_POLY_FIRST_RGB 0x003300UL
#define MODE_POLY_MONO_RGB  0x0000CCUL
#define MODE_MONO_POLY_RGB  0x3300CCUL
#define MODE_MONO_RGB       0x009966UL
#define CALIBRATION_RGB     0x3333CCUL

// CV math
#define MIDI_NOTE_TO_CV_FACTOR 83.333333f   // 1000 mV / 12 semitones
#define MIDI_PITCHBEND_MAX     8191

// EEPROM layout
const int EEPROM_MODE    = 0;
const int EEPROM_DAC_CAL = 100;

const char NOTE_NAMES[12][3] = {
    "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

// =============================================================================
// MIDI SETTINGS
// =============================================================================

struct MIDISettings : public midi::DefaultSettings {
    static const long BaudRate         = 31250;
    static const bool UseRunningStatus = false;  // Avoid ghost notes
    static const bool Use1ByteParsing  = false;  // Parse full messages at once
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MIDISettings);

// =============================================================================
// HARDWARE OBJECTS
// =============================================================================

Button        modeButton;
MCP4728       dac;
MultiPointMap calibration[4];
Led           gateLed[N];
Led           gateOrLed;
Led           noteOnLed;

// =============================================================================
// VOICE STATE
// =============================================================================

NoteStack      mono[N];
VoiceAllocator poly;

Mode           mode = MODE_POLY;

byte           voiceMidiNote[N];    // Current MIDI note per voice
bool           voiceActive[N];      // Gate active flag per voice
bool           voiceLocked[N];      // Sustain-locked flag per voice

// --- Retrig state machine ---------------------------------------------------
//
// RETRIG_IDLE : gate is HIGH (or voice inactive), no gap in progress
// RETRIG_LOW  : gate is being held LOW for GATE_RETRIG_MS
//
// voicePendingNote[i]:
//   0xFF          → no note queued (note was already written before startRetrig)
//   anything else → NoteOn arrived while gate was LOW; apply when gap ends
//
enum RetrigState : byte { RETRIG_IDLE, RETRIG_LOW };

RetrigState    voiceRetrigState[N];
unsigned long  voiceRetrigTime[N];   // millis() when the LOW gap started
byte           voicePendingNote[N];  // Note queued during RETRIG_LOW (0xFF=none)

// ---------------------------------------------------------------------------

int            pitchBend  = 0;
bool           outputFlag = false;

// Clock
unsigned int   clockCount        = 0;
bool           clockRunning      = false;
bool           clockTrig         = false;
unsigned long  clockTrigTime     = 0;
unsigned long  clockTrigDuration = CLOCK_TRIG_MS;

// Calibration
bool  calibrating          = false;
byte  calibratingVoice;
byte  calibratingInterval;
int   calibratingAddress;

unsigned long voiceLockLedTime = 0;

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================

void setupMain();
void setupCalibration();
void loopMain();
void loopCalibration();

void setMode(byte m);
void voicesLock();
void reset();
void allNotesOff();
void output();
void startRetrig(byte voiceIndex, byte pendingNote);

void handleNoteOn(byte channel, byte note, byte velocity);
void handleNoteOff(byte channel, byte note, byte velocity);
void handlePitchBend(byte channel, int bend);
void handleControlChange(byte channel, byte number, byte value);
void handleClock();
void handleStart();
void handleContinue();
void handleStop();
void handleSongPosition(unsigned int beats);
void handleCalibrationOffset(byte channel, byte note, byte velocity);

bool         isNoteForMonophony(byte note);
byte         getMonophonyStackIndex(byte channel);
byte         getMonophonyVoiceIndex(byte channel);
byte         getPolyphonyVoiceIndex(byte i);
unsigned int getMidiNoteCV(byte note, int pitchBendValue);
String       getMidiNoteName(byte note);

void setModeLed();
void setModeLedColor(unsigned long color);
void bootAnimation();
void debug(String line);
void debugMidiNote(String label, byte note);
void debugVoices();

// =============================================================================
// ARDUINO LIFECYCLE
// =============================================================================

void setup() {

    MIDI.begin(MIDI_CHANNEL_OMNI);
    MIDI.turnThruOff();

    if (DEBUG || DEBUG_WITH_TTYMIDI) {
        Serial.begin(9600);
        if (DEBUG) debug("midi4plus1 start");
    }

    modeButton.init(MODE_BUTTON, BUTTON_DEBOUNCE_DELAY, /*invert=*/true, /*pullup=*/true);
    pinMode(GATE_OR, OUTPUT);
    gateOrLed.init(GATE_OR_LED, LED_MIN_DURATION_MS);
    noteOnLed.init(NOTE_ON_LED, LED_MIN_DURATION_MS);
    for (byte i = 0; i < N; i++) {
        pinMode(GATES[i], OUTPUT);
        gateLed[i].init(GATES_LEDS[i]);
    }

    if (MODE_LEDS_PWM) {
        SoftPWMBegin();
    } else {
        for (byte i = 0; i < 3; i++) pinMode(MODE_LEDS[i], OUTPUT);
    }

    Wire.begin();
    Wire.setClock(400000);
    dac.init(Wire, 4);
    dac.selectVref(
        MCP4728::VREF::INTERNAL_2_8V, MCP4728::VREF::INTERNAL_2_8V,
        MCP4728::VREF::INTERNAL_2_8V, MCP4728::VREF::INTERNAL_2_8V);
    dac.selectPowerDown(
        MCP4728::PWR_DOWN::NORMAL, MCP4728::PWR_DOWN::NORMAL,
        MCP4728::PWR_DOWN::NORMAL, MCP4728::PWR_DOWN::NORMAL);
    dac.selectGain(
        MCP4728::GAIN::X2, MCP4728::GAIN::X2,
        MCP4728::GAIN::X2, MCP4728::GAIN::X2);

    int calAddr = EEPROM_DAC_CAL;
    for (byte i = 0; i < 4; i++) {
        calibration[i].init(4000);
        calAddr += calibration[i].load(calAddr);
    }

    delay(100);
    if (modeButton.readOnce()) {
        setupCalibration();
    } else {
        setupMain();
    }
}

void loop() {

    // Drain ALL buffered MIDI messages before anything else.
    // A single MIDI.read() per loop() iteration is not enough for
    // rapid-fire ratchet / glitch patterns — messages pile up in the
    // serial buffer and are handled one-per-frame at best.
    while (MIDI.read()) { /* processing happens inside the callbacks */ }

    if (calibrating) {
        loopCalibration();
    } else {
        loopMain();
    }

    gateOrLed.loop();
    noteOnLed.loop();
    for (byte i = 0; i < N; i++) gateLed[i].loop();
}

// =============================================================================
// SETUP HELPERS
// =============================================================================

void setupMain() {

    poly.init();
    for (byte i = 0; i < N; i++) {
        mono[i].init();
        voiceMidiNote[i]    = 12;           // C0
        voiceActive[i]      = false;
        voiceLocked[i]      = false;
        voiceRetrigState[i] = RETRIG_IDLE;
        voiceRetrigTime[i]  = 0;
        voicePendingNote[i] = 0xFF;
    }

    bootAnimation();

    if (CLOCK) {
        unsigned long maxBPM    = 600;
        unsigned long maxPeriod = ((60000UL / maxBPM) * CLOCK_PPQ) / 24;
        clockTrigDuration = min((unsigned long)CLOCK_TRIG_MS, (maxPeriod * 8) / 10);
    }

    for (byte i = 0; i < N; i++) gateLed[i].setMinDurationMs(LED_MIN_DURATION_MS);

    setMode(EEPROM.read(EEPROM_MODE) % MODE_COUNT);

    MIDI.setHandleNoteOn(handleNoteOn);
    MIDI.setHandleNoteOff(handleNoteOff);
    MIDI.setHandlePitchBend(handlePitchBend);
    MIDI.setHandleControlChange(handleControlChange);

    if (CLOCK) {
        MIDI.setHandleClock(handleClock);
        MIDI.setHandleStart(handleStart);
        MIDI.setHandleContinue(handleContinue);
        MIDI.setHandleStop(handleStop);
        MIDI.setHandleSongPosition(handleSongPosition);
    }
}

void setupCalibration() {
    calibrating         = true;
    calibratingVoice    = 0;
    calibratingInterval = 0;
    calibratingAddress  = EEPROM_DAC_CAL;
    setModeLedColor(CALIBRATION_RGB);
    MIDI.setHandleNoteOn(handleCalibrationOffset);
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loopMain() {

    unsigned long now = millis();

    // -------------------------------------------------------------------------
    // Retrig state machine
    //
    // Each voice in RETRIG_LOW waits GATE_RETRIG_MS, then:
    //   - if a new note was queued (voicePendingNote != 0xFF): apply it, then raise gate
    //   - otherwise: just raise the gate (note was already set before the gap)
    // -------------------------------------------------------------------------
    for (byte i = 0; i < N; i++) {
        if (voiceRetrigState[i] == RETRIG_LOW) {
            if (now - voiceRetrigTime[i] >= GATE_RETRIG_MS) {
                if (voicePendingNote[i] != 0xFF) {
                    voiceMidiNote[i]    = voicePendingNote[i];
                    voiceActive[i]      = true;
                    voicePendingNote[i] = 0xFF;
                }
                voiceRetrigState[i] = RETRIG_IDLE;
                outputFlag = true;
            }
        }
    }

    if (outputFlag) {
        output();
        outputFlag = false;
    }

    // Clock trigger timeout
    if (CLOCK) {
        if (clockTrig && (now - clockTrigTime >= clockTrigDuration)) {
            digitalWrite(GATE_OR, LOW);
            clockTrig = false;
        }
    }

    // Mode button
    byte btn = modeButton.readShortOrLongPressOnce(BUTTON_LOCK_LONG_PRESS_MS);
    if      (btn == 1) setMode((mode + 1) % MODE_COUNT);
    else if (btn == 2) voicesLock();

    if (voiceLockLedTime > 0 && now - voiceLockLedTime >= LED_MODE_LOCK_DURATION_MS) {
        voiceLockLedTime = 0;
        setModeLed();
    }
}

// =============================================================================
// CALIBRATION LOOP
// =============================================================================

void loopCalibration() {

    if (modeButton.readOnce()) {
        calibratingInterval++;
        if (calibratingInterval == calibration[calibratingVoice].size()) {
            calibratingAddress += calibration[calibratingVoice].save(calibratingAddress);
            calibratingVoice++;
            calibratingInterval = 0;
            if (calibratingVoice >= N) {
                calibrating = false;
                for (byte i = 0; i < N; i++) gateLed[i].off();
                setModeLedColor(0x000000);
                delay(1000);
                setupMain();
                return;
            }
        }
    }

    for (byte i = 0; i < N; i++) gateLed[i].set(i == calibratingVoice);

    if (millis() % 50 == 0) {
        unsigned int sz    = calibration[calibratingVoice].size();
        unsigned int step  = calibration[calibratingVoice].getStep();
        unsigned int value = step * (calibratingInterval + 1);
        dac.analogWrite(
            calibration[0].map(calibratingVoice == 0 ? value : (calibratingVoice > 0 ? step * sz : 0)),
            calibration[1].map(calibratingVoice == 1 ? value : (calibratingVoice > 1 ? step * sz : 0)),
            calibration[2].map(calibratingVoice == 2 ? value : (calibratingVoice > 2 ? step * sz : 0)),
            calibration[3].map(calibratingVoice == 3 ? value : (calibratingVoice > 3 ? step * sz : 0))
        );
        for (byte i = 0; i < N; i++) {
            digitalWrite(GATES[i], calibratingVoice == i ? HIGH : LOW);
        }
    }
}

// =============================================================================
// MODE & VOICE MANAGEMENT
// =============================================================================

void setMode(byte m) {
    mode = (Mode)m;
    switch (mode) {
        case MODE_POLY:
            poly.setMode(VoiceAllocator::Mode::LAST);
            poly.setSize(N);
            break;
        case MODE_POLY_FIRST:
            poly.setMode(VoiceAllocator::Mode::FIRST);
            poly.setSize(N);
            break;
        case MODE_POLY_MONO:
        case MODE_MONO_POLY:
            poly.setMode(VoiceAllocator::Mode::LAST);
            poly.setSize(N - 1);
            break;
        default:
            break;
    }
    EEPROM.update(EEPROM_MODE, mode);
    setModeLed();
    reset();
}

void voicesLock() {
    bool didUnlock = false;
    for (byte i = 0; i < N; i++) {
        if (voiceLocked[i]) {
            voiceLocked[i] = false;
            voiceActive[i] = false;
            outputFlag     = true;
            didUnlock      = true;
        }
    }
    if (mode != MODE_MONO) {
        if (!didUnlock) {
            for (byte i = 0; i < N; i++) {
                if (voiceActive[i] && !isNoteForMonophony(voiceMidiNote[i])) {
                    voiceLocked[i] = true;
                }
            }
            if (DEBUG) debugVoices();
        }
        voiceLockLedTime = millis();
        if (MODE_LEDS_PWM) {
            SoftPWMSet(MODE_LEDS[0], 0);
            SoftPWMSet(MODE_LEDS[1], 0);
            SoftPWMSet(MODE_LEDS[2], 0);
        } else {
            for (byte i = 0; i < 3; i++) digitalWrite(MODE_LEDS[i], LOW);
        }
    }
}

void reset() {
    for (byte i = 0; i < N; i++) {
        digitalWrite(GATES[i], LOW);
        gateLed[i].off();
        voiceActive[i]      = false;
        voiceLocked[i]      = false;
        voiceRetrigState[i] = RETRIG_IDLE;
        voiceRetrigTime[i]  = 0;
        voicePendingNote[i] = 0xFF;
        mono[i].clear();
    }
    digitalWrite(GATE_OR, LOW);
    gateOrLed.off();
    poly.clear();
    pitchBend  = 0;
    outputFlag = true;
}

void allNotesOff() {
    if (DEBUG) debug("All Notes Off");
    for (byte i = 0; i < N; i++) {
        mono[i].clear();
        voiceActive[i]      = false;
        voiceRetrigState[i] = RETRIG_IDLE;
        voiceRetrigTime[i]  = 0;
        voicePendingNote[i] = 0xFF;
    }
    poly.clear();
    outputFlag = true;
}

// =============================================================================
// OUTPUT
// =============================================================================

void output() {

    // CV (DAC)
    unsigned int dacValues[4];
    for (byte i = 0; i < 4; i++) {
        if (i < N) {
            int pb = pitchBend;
            // In split modes, pitch-bend applies to the mono voice only
            if ((mode == MODE_POLY_MONO || mode == MODE_MONO_POLY) &&
                !isNoteForMonophony(voiceMidiNote[i])) {
                pb = 0;
            }
            dacValues[i] = getMidiNoteCV(voiceMidiNote[i], pb);
        } else {
            dacValues[i] = 0;
        }
    }
    dac.analogWrite(
        calibration[0].map(dacValues[0]),
        calibration[1].map(dacValues[1]),
        calibration[2].map(dacValues[2]),
        calibration[3].map(dacValues[3])
    );

    // Gate pins — LOW while a retrig gap is in progress
    for (byte i = 0; i < N; i++) {
        bool gateHigh = voiceActive[i] && (voiceRetrigState[i] == RETRIG_IDLE);
        digitalWrite(GATES[i], gateHigh ? HIGH : LOW);
        gateLed[i].set(gateHigh);
    }

    // OR gate
    if (!CLOCK) {
        bool orHigh = false;
        byte first  = (mode == MODE_MONO_POLY) ? 1 : 0;
        byte last   = (mode == MODE_POLY_MONO) ? N - 2 : N - 1;
        for (byte i = first; i <= last; i++) orHigh |= (voiceActive[i] && voiceRetrigState[i] == RETRIG_IDLE);
        digitalWrite(GATE_OR, orHigh ? HIGH : LOW);
        gateOrLed.set(orHigh);
    }

    if (DEBUG) debugVoices();
}

// =============================================================================
// RETRIG HELPER
// =============================================================================

// Pull the gate LOW immediately and begin a GATE_RETRIG_MS gap.
//
// pendingNote:
//   0xFF → the note has already been written to voiceMidiNote[i]; just do the gap.
//   other → apply this note to voiceMidiNote[i] when the gap ends (used when a
//            new NoteOn arrives while a previous gap is still running).
//
void startRetrig(byte i, byte pendingNote) {
    digitalWrite(GATES[i], LOW);
    gateLed[i].set(false);
    voiceRetrigState[i] = RETRIG_LOW;
    voiceRetrigTime[i]  = millis();
    voicePendingNote[i] = pendingNote;
}

// =============================================================================
// MIDI HANDLERS
// =============================================================================

void handleNoteOn(byte channel, byte note, byte velocity) {

    // MIDI spec §1.4: NoteOn with velocity=0 is equivalent to NoteOff.
    // Many sequencers use this form (especially with running status enabled).
    if (velocity == 0) {
        handleNoteOff(channel, note, 0);
        return;
    }

    noteOnLed.flash();

    // -------------------------------------------------------------------------
    // Monophonic path
    // -------------------------------------------------------------------------
    if (isNoteForMonophony(note)) {

        if (mode == MODE_MONO && (channel == 0 || channel > N)) return;

        byte si = getMonophonyStackIndex(channel);
        byte vi = getMonophonyVoiceIndex(channel);

        mono[si].noteOn(note);

        if (voiceRetrigState[vi] == RETRIG_LOW) {
            // A retrig gap is already running — queue this note.
            // Pre-write CV so pitch changes immediately (CV leads gate),
            // consistent with the polyphonic path behaviour.
            voiceMidiNote[vi]    = note;
            voicePendingNote[vi] = note;
            // Do NOT set outputFlag; gate must stay LOW until the gap ends.
        } else {
            bool needsRetrig = GATE_RETRIG_MONO && voiceActive[vi];
            voiceMidiNote[vi] = note;   // Update CV immediately
            voiceActive[vi]   = true;
            if (needsRetrig) {
                startRetrig(vi, 0xFF);  // Note already written; open gap only
            } else {
                outputFlag = true;
            }
        }

    // -------------------------------------------------------------------------
    // Polyphonic path
    // -------------------------------------------------------------------------
    } else {

        int voice = poly.noteOn(note);
        if (voice < 0) return;  // No voice available (Mode::FIRST, all busy)

        byte i        = getPolyphonyVoiceIndex(voice);
        voiceLocked[i] = false;

        if (voiceRetrigState[i] == RETRIG_LOW) {
            // Gap running — queue the new note.
            // Pre-write the CV so pitch changes immediately (CV leads gate).
            voiceMidiNote[i]    = note;
            voicePendingNote[i] = note;
            // Gate stays LOW; loopMain will raise it after the gap.
        } else {
            bool needsRetrig = voiceActive[i] && (voiceMidiNote[i] != note);
            voiceMidiNote[i]  = note;   // Update CV
            voiceActive[i]    = true;
            if (needsRetrig) {
                startRetrig(i, 0xFF);
            } else {
                outputFlag = true;
            }
        }
    }
}

void handleNoteOff(byte channel, byte note, byte velocity) {

    // -------------------------------------------------------------------------
    // Monophonic path
    // -------------------------------------------------------------------------
    if (isNoteForMonophony(note)) {

        if (mode == MODE_MONO && (channel == 0 || channel > N)) return;

        byte si = getMonophonyStackIndex(channel);
        byte vi = getMonophonyVoiceIndex(channel);

        int newNote = mono[si].noteOff(note);

        if (newNote > -1) {
            // A previously held note resurfaces — glide or retrig to it
            if (voiceRetrigState[vi] == RETRIG_LOW) {
                voicePendingNote[vi] = (byte)newNote;
                voiceMidiNote[vi]    = (byte)newNote;   // CV leads gate
            } else {
                bool needsRetrig = GATE_RETRIG_MONO &&
                                   voiceActive[vi] &&
                                   voiceMidiNote[vi] != (byte)newNote;
                voiceMidiNote[vi] = (byte)newNote;
                voiceActive[vi]   = true;
                if (needsRetrig) {
                    startRetrig(vi, 0xFF);
                } else {
                    outputFlag = true;
                }
            }
        } else {
            // Stack empty — close the gate and cancel any retrig in progress.
            // If we leave voicePendingNote set, loopMain would resurrect the
            // note when the gap expires (ghost note bug).
            voiceActive[vi]      = false;
            voiceRetrigState[vi] = RETRIG_IDLE;
            voicePendingNote[vi] = 0xFF;
            outputFlag           = true;
        }

    // -------------------------------------------------------------------------
    // Polyphonic path
    // -------------------------------------------------------------------------
    } else {

        int voice = poly.noteOff(note);
        if (voice < 0) return;

        byte i = getPolyphonyVoiceIndex(voice);
        if (!voiceLocked[i]) {
            voiceActive[i]      = false;
            voicePendingNote[i] = 0xFF;  // Cancel any queued note; prevents ghost
            outputFlag          = true;  // note if NoteOff arrives during retrig gap
        }
    }
}

void handlePitchBend(byte channel, int bend) {
    pitchBend  = bend;
    outputFlag = true;
}

// CC#64  = Sustain Pedal
// CC#120 = All Sound Off   )
// CC#123 = All Notes Off   ) MIDI Panic
void handleControlChange(byte channel, byte number, byte value) {

    if (number == 120 || number == 123) {
        allNotesOff();
        return;
    }

    if (number == 64) {
        if (value >= 64) {
            // Pedal down: latch currently active voices
            for (byte i = 0; i < N; i++) {
                if (voiceActive[i]) voiceLocked[i] = true;
            }
        } else {
            // Pedal up: release latched voices
            for (byte i = 0; i < N; i++) {
                if (voiceLocked[i]) {
                    voiceLocked[i] = false;
                    voiceActive[i] = false;
                    outputFlag     = true;
                }
            }
        }
    }
}

// =============================================================================
// CLOCK HANDLERS
// =============================================================================

void handleClock() {
    if (!clockRunning) return;
    if (clockCount == 0) {
        digitalWrite(GATE_OR, HIGH);
        clockTrig     = true;
        clockTrigTime = millis();
        gateOrLed.flash();
    }
    clockCount = (clockCount + 1) % CLOCK_PPQ;
}

void handleStart() {
    clockCount   = 0;
    clockRunning = true;
}

void handleContinue() {
    clockRunning = true;
}

void handleStop() {
    clockRunning = false;
}

void handleSongPosition(unsigned int beats) {
    clockCount = (beats * 6) % CLOCK_PPQ;  // MIDI beat = 16th = 6 pulses @ 24 PPQ
}

// =============================================================================
// CALIBRATION HANDLER
// =============================================================================

void handleCalibrationOffset(byte channel, byte note, byte velocity) {
    if (!calibrating) return;
    noteOnLed.flash();
    gateOrLed.flash();
    int offset = (note >= (4 + 1) * 12) ? 1 : -1;  // Upper half +1, lower half -1
    int v = calibration[calibratingVoice].get(calibratingInterval);
    calibration[calibratingVoice].set(calibratingInterval, max(0, v + offset));
}

// =============================================================================
// UTILITY
// =============================================================================

bool isNoteForMonophony(byte note) {
    bool highKey = note >= (SPLIT_MIDI_OCTAVE + 1) * 12;
    return (mode == MODE_MONO) ||
           (mode == MODE_POLY_MONO &&  highKey) ||
           (mode == MODE_MONO_POLY && !highKey);
}

byte getMonophonyStackIndex(byte channel) {
    return (mode == MODE_MONO) ? ((channel - 1) % N) : 0;
}

byte getMonophonyVoiceIndex(byte channel) {
    if (mode == MODE_MONO)      return (channel - 1) % N;
    if (mode == MODE_MONO_POLY) return 0;
    return N - 1;
}

byte getPolyphonyVoiceIndex(byte i) {
    return (mode == MODE_MONO_POLY) ? i + 1 : i;
}

unsigned int getMidiNoteCV(byte note, int pitchBendValue) {
    int   lowestNote = (LOWEST_MIDI_OCTAVE + 1) * 12;
    float noteForCV  = (float)note - lowestNote;
    if (pitchBendValue != 0) {
        noteForCV += ((float)pitchBendValue / MIDI_PITCHBEND_MAX) * PITCH_BEND_SEMITONES;
    }
    return (unsigned int)min(4000, max(0, (int)roundf(noteForCV * MIDI_NOTE_TO_CV_FACTOR)));
}

String getMidiNoteName(byte note) {
    String s = NOTE_NAMES[note % 12];
    s.concat((note / 12) - 1);
    return s;
}

// =============================================================================
// LED HELPERS
// =============================================================================

void setModeLed() {
    switch (mode) {
        case MODE_POLY:       setModeLedColor(MODE_POLY_RGB);       break;
        case MODE_POLY_FIRST: setModeLedColor(MODE_POLY_FIRST_RGB); break;
        case MODE_POLY_MONO:  setModeLedColor(MODE_POLY_MONO_RGB);  break;
        case MODE_MONO_POLY:  setModeLedColor(MODE_MONO_POLY_RGB);  break;
        case MODE_MONO:       setModeLedColor(MODE_MONO_RGB);        break;
        default:              break;
    }
}

void setModeLedColor(unsigned long color) {
    byte r = (color >> 16) & 0xFF;
    byte g = (color >>  8) & 0xFF;
    byte b =  color        & 0xFF;
    if (MODE_LEDS_PWM) {
        SoftPWMSet(MODE_LEDS[0], r);
        SoftPWMSet(MODE_LEDS[1], g);
        SoftPWMSet(MODE_LEDS[2], b);
    } else {
        digitalWrite(MODE_LEDS[0], r ? HIGH : LOW);
        digitalWrite(MODE_LEDS[1], g ? HIGH : LOW);
        digitalWrite(MODE_LEDS[2], b ? HIGH : LOW);
    }
}

void bootAnimation() {
    for (byte i = 0; i < N; i++) { gateLed[i].on();  delay(100); }
    gateOrLed.on();  delay(200);
    for (byte i = 0; i < N; i++) { gateLed[i].off(); delay(100); }
    gateOrLed.off(); delay(100);
    gateOrLed.loop();
    delay(200);
}

// =============================================================================
// DEBUG
// =============================================================================

void debug(String line) {
    if (!DEBUG) return;
    if (DEBUG_WITH_TTYMIDI) {
        Serial.write((byte)0xFF);
        Serial.write((byte)0x00);
        Serial.write((byte)0x00);
        Serial.write((byte)line.length());
        Serial.print(line);
        Serial.flush();
    } else {
        Serial.println(line);
    }
}

void debugMidiNote(String label, byte note) {
    if (!DEBUG) return;
    debug(label + " " + note + " (" + getMidiNoteName(note) + ")");
}

void debugVoices() {
    if (!DEBUG) return;
    String m = "Voices: ";
    for (byte i = 0; i < N; i++) {
        // Brace char literals avoided to prevent Arduino IDE brace-matching
        // false error on the last line of the file.
        // 0x7B='{' 0x7D='}' 0x5B='[' 0x5D=']' 0x2E='.'
        char open  = voiceActive[i] ? (voiceLocked[i] ? (char)0x7B : (char)0x5B) : (char)0x2E;
        char close = voiceActive[i] ? (voiceLocked[i] ? (char)0x7D : (char)0x5D) : (char)0x2E;
        m += open;
        m += getMidiNoteName(voiceMidiNote[i]);
        if (voiceRetrigState[i] == RETRIG_LOW) m += (char)0x7E; // '~'
        m += close;
    }
    debug(m);
}