#include "hw_lb.h"

static void play_tone(float freq, uint32_t duration_ms) {
	if (freq > 0) {
		pwmChangePeriod(&BUZZER_PWM, BUZZER_PWM_FREQ_HZ / freq);
		BUZZER_ON();
	}
	if (duration_ms > 0) {
		chThdSleepMilliseconds(duration_ms);
	}
	BUZZER_OFF();
}

// standard BPM is 120 beats per minute
// 1 beat = 1/4 note
const uint32_t whole_note_ms = 500;

// Parse a note specification
// The note specification is a string of the form:
//   [A-G,P](#)([0-9])[/[1-9]]
// where:
//   [A-G] is the note name
//   [P] is a pause
//   [#] is an optional sharp
//   [0-9] (optional) octave number, defaults to 4
//   [/ [1-9]] (optional) is an optional duration fraction, for example /2 is a half note
//   Default is /4, i.e a quarter note.
// 
// The note is parsed and the frequency and duration are returned
// in the freq and duration_ms parameters.
// Returns 0 on success, -1 on error
int parse_note(const char *note_str, float *freq, uint32_t *duration_ms) {
	const char* ptr = note_str;

	// Parse note or pause
	if (*ptr == 'P') {
		*freq = 0;
		ptr++;
	} else if (*ptr >= 'A' && *ptr <= 'G') {
		char name = *ptr++;
		char* endptr;
		unsigned int octave = strtoul(ptr, &endptr, 10);
		if (ptr == endptr){
			octave = 4;
		}
		ptr = endptr;
		int is_sharp = (*ptr == '#') ? (ptr++, 1) : 0;
		
		int note_index = name - 'A';
		int A_offset = ((note_index - 2 + 7) % 7) - 5;
		int semitone_offset_from_A4 = 
			(2 * A_offset)                 // 2 semitones per natural note
			+ (A_offset < -2)              // Adjust for omitted E/F semitone
			+ is_sharp                     // Add 1 if the note is sharp
			+ ((octave - 4) * 12);         // Adjust for octave relative to A4

		// Calculate the frequency based on the semitone offset from A4
		*freq = 440.0 * pow(2.0, semitone_offset_from_A4 / 12.0);
	} else {
		return -1;
	};
	
	// Parse the duration
	uint8_t frac = 4; // default to quarter note  
	if (*ptr++ == '/') {
		char* endptr;
		frac = strtoul(ptr, &endptr, 10);
		ptr = endptr;
		if (frac == 0) {
			return -1;
		}
	}
	*duration_ms = whole_note_ms / frac;

	return 0;
}

int play_note(const char *note_str) {
	float freq;
	uint32_t dur;
	if (parse_note(note_str, &freq, &dur)) {
		return -1;
	}
	play_tone(freq, dur);
	return 0; 
}

// Play a melody
// The melody is a string of notes separated by spaces
// Example: D D F D  F G C5 A  A3 A3 C A3  C D G E
int play_melody(const char *melody_string) {
	const char* ptr = melody_string;
	while (*ptr) {
		if (play_note(ptr)) {
			return 0;
		}
		while (*ptr && *ptr != ' ') {
			ptr++;
		}
		if (*ptr) {
			ptr++;
		}
	}
	return 0;
}
