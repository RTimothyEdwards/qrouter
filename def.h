/*
 * def.h --
 *
 * This file includes the DEF I/O functions
 *
 */

#ifndef _DEFINT_H
#define _DEFINT_H

extern int numSpecial;
extern int DefRead(char *inName, float *);

extern GATE DefFindGate(char *name);
extern NET DefFindNet(char *name);

#endif /* _DEFINT_H */
