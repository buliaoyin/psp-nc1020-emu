#ifndef _NC1020_H_
#define _NC1020_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

void wqxInitialize(const char*);
void wqxreset();
void wqxSetKey(uint8_t, bool);
void wqxRunTimeSlice(float, bool);
bool wqxCopyLcdBuffer(uint8_t*);
void wqxLoadNC1020();
void wqxSaveNC1020();
void wqxQuitNC1020();

#endif /* _NC1020_H_ */
