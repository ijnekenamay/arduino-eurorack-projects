#ifndef mono_h
#define mono_h

#include "Arduino.h"

// Monophonic note stack (last-note priority with full history).
// Based on Emilie Gillet's CVpal:
// https://github.com/pichenettes/cvpal/blob/master/cvpal/note_stack.h
//
// Fix vs original:
//   - clear() loop bound corrected: was `i < CAPACITY`, must be `i <= CAPACITY`
//     because note[] / next[] are CAPACITY+1 elements (index 0 is a sentinel).

#define CAPACITY 10
#define FREE     0xFF

class NoteStack {

public:

    void init() {
        this->clear();
    }

    /**
     * Push a note onto the top of the stack.
     * If the note is already present it is moved to the front (re-trigger).
     */
    void noteOn(byte note) {

        // Remove the note if it is already in the stack (handles re-trigger)
        this->noteOff(note);

        // If the stack is full, evict the oldest (tail) entry
        if (this->size == CAPACITY) {
            byte oldest = 0;
            for (byte i = 1; i <= CAPACITY; i++) {
                if (this->next[i] == 0) {
                    oldest = this->note[i];
                    break;
                }
            }
            this->noteOff(oldest);
        }

        // Find a free slot
        byte slot = 0;
        for (byte i = 1; i <= CAPACITY; i++) {
            if (this->note[i] == FREE) {
                slot = i;
                break;
            }
        }

        // Insert at head
        this->next[slot] = this->root;
        this->note[slot] = note;
        this->root       = slot;
        this->size++;
    }

    /**
     * Remove a note from the stack.
     * Returns the most recently played remaining note, or -1 if the stack is now empty.
     */
    int noteOff(byte note) {

        byte current  = this->root;
        byte previous = 0;

        while (current) {
            if (this->note[current] == note) break;
            previous = current;
            current  = this->next[current];
        }

        if (current) {
            // Unlink
            if (previous) {
                this->next[previous] = this->next[current];
            } else {
                this->root = this->next[current];
            }
            // Free the slot
            this->next[current] = 0;
            this->note[current] = FREE;
            this->size--;
        }

        return (this->size > 0) ? (int)this->note[this->root] : -1;
    }

    /** Return the top-of-stack note without modifying the stack, or -1 if empty. */
    int peek() const {
        return (this->size > 0) ? (int)this->note[this->root] : -1;
    }

    /** True if the stack contains no notes. */
    bool empty() const {
        return this->size == 0;
    }

    /**
     * Clear the stack.
     * Bug fix: loop must cover indices 0..CAPACITY (CAPACITY+1 elements).
     */
    void clear() {
        this->size = 0;
        this->root = 0;
        for (byte i = 0; i <= CAPACITY; i++) {   // was: i < CAPACITY (off-by-one)
            this->note[i] = FREE;
            this->next[i] = 0;
        }
    }

private:
    byte size;
    byte root;                  // Index of the head node (1-based; 0 = none)
    byte note[CAPACITY + 1];    // Note values; index 0 is a sentinel / unused
    byte next[CAPACITY + 1];    // Next-node indices; 0 = end of list
};

#endif