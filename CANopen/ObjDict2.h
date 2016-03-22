
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef OBJDICT2_H
#define OBJDICT2_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 ObjDict2_valueRangeTest (UNS8 typeValue, void * value);
const indextable * ObjDict2_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks);

/* Master node data struct */
extern CO_Data ObjDict2_Data;
extern UNS8 DI11;		/* Mapped at index 0x2000, subindex 0x00*/
extern UNS8 DI21;		/* Mapped at index 0x2001, subindex 0x00*/
extern UNS32 DO11;		/* Mapped at index 0x2100, subindex 0x00*/
extern UNS8 DO21;		/* Mapped at index 0x2101, subindex 0x00*/

#endif // OBJDICT2_H
