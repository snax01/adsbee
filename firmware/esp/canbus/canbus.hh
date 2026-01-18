#pragma once

#include "driver/MCP251XFD.h"

bool CanbusInit();
void transmit_can();
void check_can_errors();