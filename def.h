/*
 * def.h --
 *
 * This file includes the DEF I/O functions
 *
 */

#ifndef _DEFINT_H
#define _DEFINT_H

/* Retain information from TRACKS entries in a DEF file */

typedef struct tracks_ {
    double start;
    int ntracks;
    double pitch;
} *TRACKS;

extern int numSpecial;
extern int DefRead(char *inName, float *);

extern TRACKS DefGetTracks(int layer);
extern GATE DefFindGate(char *name);
extern NET DefFindNet(char *name);

#endif /* _DEFINT_H */
