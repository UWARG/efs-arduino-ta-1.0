#pragma once
#include "Adafruit_Sensor-master/Adafruit_Sensor.h"

float mag_data[3];

// Offsets applied to raw x/y/z mag values
const float mag_offsets[3] = {20.58,  -29.25,  -18.64};

// Soft iron error compensation matrix
const float soft_iron[3][3] = {
  { 0.616,  0.007, 0.020 },
  { 0.007,  0.607, -0.027 },
  { 0.020, -0.027,  2.627 }
};

// Mag declination at Waterloo [MUST CHANGE FOR COMPETITION]
const float mag_decl = -9.466667;

void mag_refine(const sensors_event_t &mag_event) {
    mag_data[0] = mag_event.magnetic.x - mag_offsets[0];
    mag_data[1] = mag_event.magnetic.y - mag_offsets[1];
    mag_data[2] = mag_event.magnetic.z - mag_offsets[2];

    for (uint8_t i = 0; i < 3; i++) {
        mag_data[i] =
            (soft_iron[i][0] * mag_data[0]) +
            (soft_iron[i][1] * mag_data[1]) +
            (soft_iron[i][2] * mag_data[2]);
    }
}


