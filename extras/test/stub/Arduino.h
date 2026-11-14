// Host-side stub used only to test-compile and unit-test the library on a PC.
#pragma once
#include <stdint.h>
#include <math.h>
extern unsigned long g_micros;
inline unsigned long micros() { return g_micros; }
inline unsigned long millis() { return g_micros / 1000UL; }
