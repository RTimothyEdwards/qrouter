/*
 * tkSimple.h --
 */

#ifndef _TKSIMPLEINT_H
#define _TKSIMPLEINT_H

#include <tk.h>

int Tk_SimpleObjCmd(ClientData clientData, Tcl_Interp *interp,
                    int objc, Tcl_Obj * CONST objv[]);

#endif /* _TKSIMPLEINT_H */
