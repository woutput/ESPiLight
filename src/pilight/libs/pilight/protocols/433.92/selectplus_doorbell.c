/*
	Copyright (C) 2020 woutput

	This file is part of pilight.
	This Source Code Form is subject to the terms of the Mozilla Public
	License, v. 2.0. If a copy of the MPL was not distributed with this
	file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../core/pilight.h"
#include "../../core/common.h"
#include "../../core/dso.h"
#include "../../core/log.h"
#include "../protocol.h"
#include "../../core/binary.h"
#include "../../core/gc.h"
#include "selectplus_doorbell.h"

// Timing from TODO

// Increase this value to be more robust, but also create more false positives
#define PEAK_TO_PEAK_JITTER	80

// The same jitter is assumed on every pulse type (as may be expected)

// Short pulse timing
#define MIN_SHORT_PULSE_LENGTH	(AVG_SHORT_PULSE_LENGTH - 0.5 * PEAK_TO_PEAK_JITTER)
#define AVG_SHORT_PULSE_LENGTH	372
#define MAX_SHORT_PULSE_LENGTH	(AVG_SHORT_PULSE_LENGTH + 0.5 * PEAK_TO_PEAK_JITTER)

// Medium pulse timing
#define MIN_MEDIUM_PULSE_LENGTH	(AVG_MEDIUM_PULSE_LENGTH - 0.5 * PEAK_TO_PEAK_JITTER)
#define AVG_MEDIUM_PULSE_LENGTH	1094
#define MAX_MEDIUM_PULSE_LENGTH	(AVG_MEDIUM_PULSE_LENGTH + 0.5 * PEAK_TO_PEAK_JITTER)

// Long pulse timing
#define MIN_LONG_PULSE_LENGTH	(AVG_LONG_PULSE_LENGTH - 0.5 * PEAK_TO_PEAK_JITTER)
#define AVG_LONG_PULSE_LENGTH	6536
#define MAX_LONG_PULSE_LENGTH	(AVG_LONG_PULSE_LENGTH + 0.5 * PEAK_TO_PEAK_JITTER)

#define RAW_LENGTH		36
// Two pulses per bit, first pulse is header, last pulse is footer
#define BINARY_LENGTH		17

#define NORMAL_REPEATS		68

static int validate(void) {
	// Check for match in raw length
	if (selectplus_doorbell->rawlen == RAW_LENGTH) {
		// Check for match in header (short pulse) and footer (long pulse)
		if (selectplus_doorbell->raw[0] >= MIN_SHORT_PULSE_LENGTH) &&
		   (selectplus_doorbell->raw[0] <= MAX_SHORT_PULSE_LENGTH) &&
		   (selectplus_doorbell->raw[RAW_LENGTH - 1] >= MIN_LONG_PULSE_LENGTH) &&
		   (selectplus_doorbell->raw[RAW_LENGTH - 1] <= MAX_LONG_PULSE_LENGTH) {
			return 0;
		}
	}

	return -1;
}

static void createMessage(int id) {
	selectplus_doorbell->message = json_mkobject();

	json_append_member(selectplus_doorbell->message, "id", json_mknumber(id, 0));

	// There is no on/off for a doorbell; always assume "on"
	json_append_member(selectplus_doorbell->message, "state", json_mkstring("on"));

	selectplus_doorbell->txrpt = NORMAL_REPEATS;
}

static void parseCode(void) {
	int binary[BINARY_LENGTH], x, i = 0;

	for (x = 1; x < selectplus_doorbell->rawlen - 1; x += 2) {
		if ((selectplus_doorbell->raw[x] >= MIN_MEDIUM_PULSE_LENGTH) &&
		    (selectplus_doorbell->raw[x] <= MAX_MEDIUM_PULSE_LENGTH) &&
		    (selectplus_doorbell->raw[x + 1] >= MIN_SHORT_PULSE_LENGTH) &&
		    (selectplus_doorbell->raw[x + 1] <= MAX_SHORT_PULSE_LENGTH)) {
			binary[i++] = 0;
		} else if ((selectplus_doorbell->raw[x] >= MIN_SHORT_PULSE_LENGTH) &&
			   (selectplus_doorbell->raw[x] <= MAX_SHORT_PULSE_LENGTH) &&
			   (selectplus_doorbell->raw[x + 1] >= MIN_MEDIUM_PULSE_LENGTH) &&
			   (selectplus_doorbell->raw[x + 1] <= MAX_MEDIUM_PULSE_LENGTH)) {
			binary[i++] = 1;
		} else {
			return; // decoding failed, return without creating message
		}
	}

	state = 1; // on

	int id = binToDec(binary, 0, BINARY_LENGTH - 1);
	createMessage(id);
}

static void createLow(int start, int end) {
	int i;

	for (i = start; i <= end; i += 2) { // medium - short
		selectplus_doorbell->raw[i] = AVG_MEDIUM_PULSE_LENGTH;
		selectplus_doorbell->raw[i + 1] = AVG_SHORT_PULSE_LENGTH;
	}
}

static void createHigh(int start, int end) {
	int i;

	for (i = start; i <= end; i += 2) { // short - medium
		selectplus_doorbell->raw[i] = AVG_SHORT_PULSE_LENGTH;
		selectplus_doorbell->raw[i + 1] = AVG_MEDIUM_PULSE_LENGTH;
	}
}

static void createId(int id) {
	int binary[255];
	int length = 0;
	int i = 0, x = 0;

	length = decToBinRev(id, binary);
	for (i = 0; i <= length; i++) {
		if (binary[i] == 0) {
			x = i * 2;
			createLow(x, x + 1);
		} else { //so binary[i] == 1
			x = i * 2;
			createHigh(x, x + 1);
		}
	}
}

static void createHeader(void) {
	selectplus_doorbell->raw[0] = AVG_SHORT_PULSE_LENGTH;
}

static void createFooter(void) {
	selectplus_doorbell->raw[RAW_LENGTH - 1] = AVG_LONG_PULSE_LENGTH;
}

static int createCode(struct JsonNode *code) {
	int id = -1;
	int state = 1;
	double itmp = -1;

	if (json_find_number(code, "id", &itmp) == 0)
		id = (int)round(itmp);

	if (id == -1) {
		logprintf(LOG_ERR, "selectplus_doorbell: insufficient number of arguments; provide id");
		return EXIT_FAILURE;
	} else if (id > 131071 || id < 0) {
		logprintf(LOG_ERR, "selectplus_doorbell: invalid id range. id should be between 0 and 131071");
		return EXIT_FAILURE;
	} else {
		createMessage(id);
		createHeader();
		createId(id);
		createFooter();
		selectplus_doorbell->rawlen = RAW_LENGTH;
	}
	return EXIT_SUCCESS;
}


static void printHelp(void) {
	printf("\t -t --on\t\t\tring the doorbell\n");
	printf("\t -i --id=id\t\t\tcontrol a device with this id\n");
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void selectplusDoorbellInit(void) {

	protocol_register(&selectplus_doorbell);
	protocol_set_id(selectplus_doorbell, "selectplus_doorbell");
	protocol_device_add(selectplus_doorbell, "selectplus_doorbell", "SelectPlus doorbell");
	selectplus_doorbell->devtype = SWITCH;
	selectplus_doorbell->hwtype = RF433;
	selectplus_doorbell->minrawlen = RAW_LENGTH;
	selectplus_doorbell->maxrawlen = RAW_LENGTH;
	selectplus_doorbell->maxgaplen = MAX_LONG_PULSE_LENGTH;
	selectplus_doorbell->mingaplen = MIN_LONG_PULSE_LENGTH;

	options_add(&selectplus_doorbell->options, "t", "on", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	options_add(&selectplus_doorbell->options, "f", "off", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
	options_add(&selectplus_doorbell->options, "u", "unit", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([1-4])$");
	options_add(&selectplus_doorbell->options, "i", "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-9]{1}|[0-9]{2}|[0-9]{3}|[0-9]{4}|[0-9]{5}|[0-9]{6})$");
	options_add(&selectplus_doorbell->options, "a", "all", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, NULL);
	options_add(&selectplus_doorbell->options, "l", "learn", OPTION_NO_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, NULL);

	options_add(&selectplus_doorbell->options, "0", "readonly", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)0, "^[10]$");
	options_add(&selectplus_doorbell->options, "0", "confirm", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)0, "^[10]$");

	selectplus_doorbell->parseCode = &parseCode;
	selectplus_doorbell->createCode = &createCode;
	selectplus_doorbell->printHelp = &printHelp;
	selectplus_doorbell->validate = &validate;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "selectplus_doorbell";
	module->version = "1.0";
	module->reqversion = "6.0";
	module->reqcommit = "84";
}

void init(void) {
	selectplusDoorbellInit();
}
#endif
