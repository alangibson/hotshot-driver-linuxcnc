#include "tmc/helpers/API_Header.h"
#include "tmc/ic/TMC5041/TMC5041_Register.h"
#include "tmc/ic/TMC5041/TMC5041_Constants.h"
#include "tmc/ic/TMC5041/TMC5041_Fields.h"

#ifndef GLOBAL_OK
#include "global.h"
#endif

// Taken from TMC5041.h
#define TMC5041_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc5041_readInt(tdef, address), mask, shift)
#define TMC5041_FIELD_WRITE(tdef, address, mask, shift, value) \
	(tmc5041_writeInt(tdef, address, FIELD_SET(tmc5041_readInt(tdef, address), mask, shift, value)))
int32_t tmc5041_writeDatagram(joint_t * motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
int32_t tmc5041_writeInt(joint_t * motor, uint8_t address, int32_t value);
int32_t tmc5041_readInt(joint_t * motor, uint8_t address);

