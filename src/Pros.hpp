#pragma once

#include "commonuse.hpp"

void IRAM_ATTR modeJumpDetect(ContrleMode& lastMode, ContrleMode currentMode);

void IRAM_ATTR sensorUpdata();

void IRAM_ATTR controlProcess();