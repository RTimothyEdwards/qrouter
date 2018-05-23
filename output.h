/*
 * output.h --
 *
 * This file includes the DEF file output functions
 *
 */

#ifndef _OUTPUTINT_H
#define _OUTPUTINT_H

extern int  Pathon;

/* Function prototypes */
static void emit_routes(char *filename, double oscale, int iscale);

void   cleanup_net(NET net);
int    write_def(char *filename);
int    write_failed(char *filename);
char  *print_node_name(NODE node);
void   print_nets(char *filename);
void   print_routes(char *filename);
void   print_nlgates(char *filename);
void   print_net(NET net);
void   print_gate(GATE gate);

#endif /* _OUTPUTINT_H */
