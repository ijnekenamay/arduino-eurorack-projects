#ifndef poly_h
#define poly_h

#include "Arduino.h"

// Polyphonic voice allocator.
// Supports two allocation strategies:
//   Mode::LAST  — LRU (Least Recently Used) with voice stealing; priority to the last note played.
//   Mode::FIRST — First-available; priority to the first free slot; ignores new notes when full.
//
// Based on Emilie Gillet's CVpal:
// https://github.com/pichenettes/cvpal/blob/master/cvpal/voice_allocator.cc
//
// Changes vs original:
//   - Added isActive() / getNote() accessors for external state inspection
//   - Comments updated for clarity

#define MAX_VOICES 4

class VoiceAllocator {

public:

    enum class Mode { LAST, FIRST };

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    void init() {
        this->setMode(Mode::LAST);
        this->setSize(0);
        this->clear();
        for (byte i = 0; i < MAX_VOICES; i++) {
            this->note[i] = 12; // Default: C0
        }
    }

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    void setMode(Mode m) {
        this->mode = m;
    }

    /** Set the number of available voices (clamped to MAX_VOICES). */
    void setSize(byte sz) {
        this->size = min(MAX_VOICES, sz);
    }

    // -----------------------------------------------------------------------
    // Note events
    // -----------------------------------------------------------------------

    /**
     * Allocate a voice for an incoming note.
     * Returns the voice index [0, size), or -1 if no voice could be allocated.
     *
     * Mode::LAST:
     *   1. Re-use the voice already playing this note (if any).
     *   2. Use the least-recently-touched inactive voice.
     *   3. Steal the least-recently-touched active voice (voice stealing).
     *
     * Mode::FIRST:
     *   Use the first inactive voice; return -1 if all voices are active.
     */
    int noteOn(byte note) {

        if (this->size == 0) return -1;

        int voice = -1;

        if (this->mode == Mode::LAST) {

            // 1. Already playing this exact note?
            voice = this->find(note);

            // 2. Least-recently-touched inactive voice
            if (voice == -1) {
                for (int i = MAX_VOICES - 1; i >= 0; i--) {
                    byte v = this->lru[i];
                    if (v < this->size && !this->active[v]) {
                        voice = v;
                        break;
                    }
                }
            }

            // 3. Voice steal: least-recently-touched active voice
            if (voice == -1) {
                for (int i = MAX_VOICES - 1; i >= 0; i--) {
                    byte v = this->lru[i];
                    if (v < this->size) {
                        voice = v;
                        break;
                    }
                }
            }

            this->touch(voice);

        } else { // Mode::FIRST

            for (byte i = 0; i < this->size; i++) {
                if (!this->active[i]) {
                    voice = i;
                    break;
                }
            }
            if (voice == -1) return -1; // All voices busy — new note dropped
        }

        this->note[voice]   = note;
        this->active[voice] = true;
        return voice;
    }

    /**
     * Release the voice playing a note.
     * Returns the voice index, or -1 if the note was not found.
     */
    int noteOff(byte note) {
        int voice = this->find(note);
        if (voice != -1) {
            this->active[voice] = false;
            if (this->mode == Mode::LAST) this->touch(voice);
        }
        return voice;
    }

    /**
     * Mark all voices inactive and reset the LRU order.
     */
    void clear() {
        for (byte i = 0; i < MAX_VOICES; i++) {
            this->active[i] = false;
            this->lru[i]    = MAX_VOICES - i - 1; // Initial order: 3,2,1,0
        }
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    bool isActive(byte voice) const {
        return (voice < MAX_VOICES) && this->active[voice];
    }

    byte getNote(byte voice) const {
        return (voice < MAX_VOICES) ? this->note[voice] : 0;
    }

private:

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /** Return the voice index playing 'note', or -1. */
    int find(byte note) const {
        for (byte i = 0; i < this->size; i++) {
            if (this->note[i] == note) return i;
        }
        return -1;
    }

    /**
     * Move 'voice' to the most-recently-used position (front of lru[]).
     * lru[0] = most recently used; lru[MAX_VOICES-1] = least recently used.
     */
    void touch(byte voice) {
        // Shift everything except 'voice' one position to the right
        int dst = MAX_VOICES - 1;
        for (int src = MAX_VOICES - 1; src >= 0; src--) {
            if (this->lru[src] != voice) this->lru[dst--] = this->lru[src];
        }
        this->lru[0] = voice;
    }

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    Mode mode;
    byte size;                      // Number of voices currently in use
    byte note[MAX_VOICES];          // Note value assigned to each voice
    bool active[MAX_VOICES];        // True while a voice's gate is held
    byte lru[MAX_VOICES];           // Voice indices sorted by recency (MRU → LRU)
};

#endif