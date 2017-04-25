/*--------------------------------------------------------------*/
/* graphics.h -- graphics routines                             	*/
/*--------------------------------------------------------------*/

#ifndef GRAPHICS_H

#include <tk.h>

/* TODO: Can we make this include independent from qrouter.h ? */
#include "qrouter.h"

void   highlight(int, int);
void   highlight_source(void);
void   highlight_dest(void);
void   highlight_starts(POINT glist);
void   highlight_mask(void);

void   draw_net(NET net, u_char single, int *lastlayer);
void   draw_layout(void);

int    GUI_init(Tcl_Interp *interp);
void   expose(Tk_Window tkwind);
int    redraw(ClientData clientData, Tcl_Interp *interp, int objc,
              Tcl_Obj *CONST objv[]);
int    recalc_spacing(void);
void   resize(Tk_Window tkwind, int locwidth, int locheight);

#define GRAPHICS_H
#endif
