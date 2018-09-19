/*------------------------------------------------------*/
/* Graphics routines for qrouter			*/
/*------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <tk.h>

#include "qrouter.h"
#include "qconfig.h"
#include "node.h"
#include "maze.h"
#include "lef.h"

/*------------------------------*/
/* Type declarations		*/
/*------------------------------*/

static void load_font(XFontStruct **);
static void createGC(Window, GC *, XFontStruct *);

/*----------------------------------*/
/* Global variables for X11 drawing */
/*----------------------------------*/

XFontStruct *font_info;
Pixmap buffer = (Pixmap)0;
Display *dpy = NULL;
Window win;
Colormap cmap;
GC gc;
Dimension width, height;

#define SHORTSPAN 10
#define LONGSPAN  127

int spacing;
int bluepix, greenpix, redpix, cyanpix, orangepix, goldpix;
int blackpix, whitepix, graypix, ltgraypix, yellowpix;
int tealpix, mauvepix, tanpix;
int magentapix, purplepix, greenyellowpix;
int brownvector[SHORTSPAN];
int bluevector[LONGSPAN];

/*--------------------------------------------------------------*/
/* Highlight a position on the graph.  Do this on the actual	*/
/* screen, not the buffer.					*/
/*--------------------------------------------------------------*/

void highlight(int x, int y) {

    int i, xspc, yspc, hspc;
    PROUTE *Pr;

    if (dpy == NULL) return;

    // If Obs2[] at x, y is a source or dest, don't highlight
    // Do this only for layer 0;  it doesn't have to be rigorous. 
    for (i = 0; i < Num_layers; i++) {
	Pr = &OBS2VAL(x, y, i);
	if (Pr->flags & (PR_SOURCE | PR_TARGET)) return;
    }

    hspc = spacing >> 1;
    if (hspc == 0) hspc = 1;

    xspc = (x + 1) * spacing - hspc;
    yspc = height - (y + 1) * spacing - hspc;

    XSetForeground(dpy, gc, yellowpix);
    XFillRectangle(dpy, win, gc, xspc, yspc, spacing, spacing);
    XFlush(dpy);
}

/*--------------------------------------*/
/* Highlight source (in magenta)	*/
/*--------------------------------------*/

void highlight_source() {

    int xspc, yspc, hspc;
    int i, x, y;
    PROUTE *Pr;

    if (dpy == NULL) return;

    if (Obs2[0] == NULL) return;

    // Determine the number of routes per width and height, if
    // it has not yet been computed

    hspc = spacing >> 1;
    if (hspc == 0) hspc = 1;

    // Draw source pins as magenta squares
    XSetForeground(dpy, gc, magentapix);
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX; x++) {
	    xspc = (x + 1) * spacing - hspc;
	    for (y = 0; y < NumChannelsY; y++) {
		Pr = &OBS2VAL(x, y, i);
		if (Pr->flags & PR_SOURCE) {
		    yspc = height - (y + 1) * spacing - hspc;
		    XFillRectangle(dpy, win, gc, xspc, yspc,
				spacing, spacing);
		}
	    }
	}
    }
    XFlush(dpy);
}

/*--------------------------------------*/
/* Highlight destination (in purple)	*/
/*--------------------------------------*/

void highlight_dest() {

    int xspc, yspc, hspc, dspc;
    int i, x, y;
    PROUTE *Pr;

    if (dpy == NULL) return;

    if (Obs2[0] == NULL) return;

    // Determine the number of routes per width and height, if
    // it has not yet been computed

    dspc = spacing + 4;			// Make target more visible
    hspc = dspc >> 1;

    // Draw destination pins as purple squares
    XSetForeground(dpy, gc, purplepix);
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX; x++) {
	    xspc = (x + 1) * spacing - hspc;
	    for (y = 0; y < NumChannelsY; y++) {
		Pr = &OBS2VAL(x, y, i);
		if (Pr->flags & PR_TARGET) {
		    yspc = height - (y + 1) * spacing - hspc;
		    XFillRectangle(dpy, win, gc, xspc, yspc,
				dspc, dspc);
		}
	    }
	}
    }
    XFlush(dpy);
}

/*----------------------------------------------*/
/* Highlight all the search starting points	*/
/*----------------------------------------------*/

void highlight_starts(POINT glist) {

    int xspc, yspc, hspc;
    POINT gpoint;

    if (dpy == NULL) return;

    // Determine the number of routes per width and height, if
    // it has not yet been computed

    hspc = spacing >> 1;

    XSetForeground(dpy, gc, greenyellowpix);
    for (gpoint = glist; gpoint; gpoint = gpoint->next) {
	xspc = (gpoint->x1 + 1) * spacing - hspc;
	yspc = height - (gpoint->y1 + 1) * spacing - hspc;
	XFillRectangle(dpy, win, gc, xspc, yspc, spacing, spacing);
    }
    XFlush(dpy);
}

/*--------------------------------------*/
/* Highlight mask (in tan)		*/
/*--------------------------------------*/

void highlight_mask(void) {

    int xspc, yspc, hspc;
    int x, y;

    if (RMask == NULL) return;

    // Determine the number of routes per width and height, if
    // it has not yet been computed

    if (dpy == NULL) return;

    hspc = spacing >> 1;

    // Draw destination pins as tan squares
    for (x = 0; x < NumChannelsX; x++) {
	xspc = (x + 1) * spacing - hspc;
	for (y = 0; y < NumChannelsY; y++) {
	    XSetForeground(dpy, gc, brownvector[RMASK(x, y)]);
	    yspc = height - (y + 1) * spacing - hspc;
	    XFillRectangle(dpy, win, gc, xspc, yspc, spacing, spacing);
	}
    }
    XFlush(dpy);
}

/*----------------------------------------------*/
/* Draw a map of obstructions and pins		*/
/*----------------------------------------------*/

static void
map_obstruction()
{
    int xspc, yspc, hspc;
    int i, x, y;

    if (dpy == NULL) return;

    hspc = spacing >> 1;

    // Draw obstructions as light gray squares
    XSetForeground(dpy, gc, ltgraypix);
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX; x++) {
	    xspc = (x + 1) * spacing - hspc;
	    for (y = 0; y < NumChannelsY; y++) {
		if (OBSVAL(x, y, i) & NO_NET) {
		    yspc = height - (y + 1) * spacing - hspc;
		    XFillRectangle(dpy, buffer, gc, xspc, yspc,
				spacing, spacing);
		}
	    }
	}
    }

    // Draw pins as gray squares
    XSetForeground(dpy, gc, graypix);
    for (i = 0; i < Pinlayers; i++) {
	for (x = 0; x < NumChannelsX; x++) {
	    xspc = (x + 1) * spacing - hspc;
	    for (y = 0; y < NumChannelsY; y++) {
		if (NODEIPTR(x, y, i) != NULL) {
		    yspc = height - (y + 1) * spacing - hspc;
		    XFillRectangle(dpy, buffer, gc, xspc, yspc,
				spacing, spacing);
		}
	    }
	}
    }
}

/*----------------------------------------------*/
/* Draw a map of actual route congestion	*/
/*----------------------------------------------*/

static void
map_congestion()
{
    int xspc, yspc, hspc;
    int i, x, y, n, norm;
    u_char *Congestion;
    u_char value, maxval;

    if (dpy == NULL) return;

    hspc = spacing >> 1;

    Congestion = (u_char *)calloc(NumChannelsX * NumChannelsY,
			sizeof(u_char));

    // Analyze Obs[] array for congestion
    for (i = 0; i < Num_layers; i++) {
	for (x = 0; x < NumChannelsX; x++) {
	    for (y = 0; y < NumChannelsY; y++) {
		value = (u_char)0;
		n = OBSVAL(x, y, i);
		if (n & ROUTED_NET) value++;
		if (n & BLOCKED_MASK) value++;
		if (n & NO_NET) value++;
		if (n & PINOBSTRUCTMASK) value++;
		CONGEST(x, y) += value;
	    }
	}
    }

    maxval = 0;
    for (x = 0; x < NumChannelsX; x++) {
	for (y = 0; y < NumChannelsY; y++) {
	    value = CONGEST(x, y);
	    if (value > maxval) maxval = value;
	}
    }
    norm = (LONGSPAN - 1) / maxval;

    // Draw destination pins as blue squares
    for (x = 0; x < NumChannelsX; x++) {
	xspc = (x + 1) * spacing - hspc;
	for (y = 0; y < NumChannelsY; y++) {
	    XSetForeground(dpy, gc, bluevector[norm * CONGEST(x, y)]);
	    yspc = height - (y + 1) * spacing - hspc;
	    XFillRectangle(dpy, buffer, gc, xspc, yspc, spacing, spacing);
	}
    }

    // Cleanup
    free(Congestion);
}

/*----------------------------------------------------------------------*/
/* Draw a map of route congestion estimated from net bounding boxes	*/
/*----------------------------------------------------------------------*/

static void
map_estimate()
{
    NET net;
    int xspc, yspc, hspc;
    int i, x, y, nwidth, nheight, area, length, value;
    float density, *Congestion, norm, maxval;

    if (dpy == NULL) return;

    hspc = spacing >> 1;

    Congestion = (float *)calloc(NumChannelsX * NumChannelsY,
			sizeof(float));

    // Use net bounding boxes to estimate congestion

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	nwidth = (net->xmax - net->xmin + 1);
	nheight = (net->ymax - net->ymin + 1);
	area = nwidth * nheight;
	if (nwidth > nheight) {
	    length = nwidth + (nheight >> 1) * net->numnodes;
	}
	else {
	    length = nheight + (nwidth >> 1) * net->numnodes;
	}
	density = (float)length / (float)area;

	for (x = net->xmin; x < net->xmax; x++)
	    for (y = net->ymin; y < net->ymax; y++)
		CONGEST(x, y) += density;
    }

    maxval = 0.0;
    for (x = 0; x < NumChannelsX; x++) {
	for (y = 0; y < NumChannelsY; y++) {
	    density = CONGEST(x, y);
	    if (density > maxval) maxval = density;
	}
    }
    norm = (float)(LONGSPAN - 1) / maxval;

    // Draw destination pins as blue squares
    for (x = 0; x < NumChannelsX; x++) {
	xspc = (x + 1) * spacing - hspc;
	for (y = 0; y < NumChannelsY; y++) {
	    value = (int)(norm * CONGEST(x, y));
	    XSetForeground(dpy, gc, bluevector[value]);
	    yspc = height - (y + 1) * spacing - hspc;
	    XFillRectangle(dpy, buffer, gc, xspc, yspc, spacing, spacing);
	}
    }

    // Cleanup
    free(Congestion);
}

/*--------------------------------------*/
/* Draw one net on the display		*/
/*--------------------------------------*/

void draw_net(NET net, u_char single, int *lastlayer) {

    int layer;
    SEG seg;
    ROUTE rt;

    if (dpy == NULL) return;

    // Draw all nets, much like "emit_routes" does when writing
    // routes to the DEF file.

    rt = net->routes;
    if (single && rt)
	for (rt = net->routes; rt->next; rt = rt->next);

    for (; rt; rt = rt->next) {
	for (seg = rt->segments; seg; seg = seg->next) {
	    layer = seg->layer;
	    if (layer != *lastlayer) {
		*lastlayer = layer;
		switch(layer) {
		    case 0:
			XSetForeground(dpy, gc, bluepix);
			break;
		    case 1:
			XSetForeground(dpy, gc, redpix);
			break;
		    case 2:
			XSetForeground(dpy, gc, cyanpix);
			break;
		    case 3:
			XSetForeground(dpy, gc, goldpix);
			break;
		    case 4:
			XSetForeground(dpy, gc, orangepix);
			break;
		    case 5:
			XSetForeground(dpy, gc, tealpix);
			break;
		    case 6:
			XSetForeground(dpy, gc, mauvepix);
			break;
		    case 7:
			XSetForeground(dpy, gc, tanpix);
			break;
		    default:
			XSetForeground(dpy, gc, greenpix);
			break;
		}
	    }
	    XDrawLine(dpy, buffer, gc, spacing * (seg->x1 + 1),
				height - spacing * (seg->y1 + 1),
				spacing * (seg->x2 + 1),
				height - spacing * (seg->y2 + 1));
	    if (single)
		XDrawLine(dpy, win, gc, spacing * (seg->x1 + 1),
				height - spacing * (seg->y1 + 1),
				spacing * (seg->x2 + 1),
				height - spacing * (seg->y2 + 1));
	}
    }   
    if (single) {
	// The following line to be removed
	XCopyArea(dpy, buffer, win, gc, 0, 0, width, height, 0, 0);
	XFlush(dpy);
    }
}

/*--------------------------------------*/
/* Draw one unrouted net on the display	*/
/*--------------------------------------*/

static void
draw_net_nodes(NET net) {

    NODE node;
    SEG bboxlist = NULL; /* bbox list of all the nodes in the net */
    SEG lastbbox, bboxit;
    DPOINT tap;
    int first, w, h, n;

    if (dpy == NULL) return;

    /* Compute bbox for each node and draw it */
    for (node = net->netnodes, n = 0; node != NULL; node = node->next, n++) {
        if (bboxlist == NULL) {
            lastbbox = bboxlist = (SEG)malloc(sizeof(struct seg_));
        }
        else {
            lastbbox->next = (SEG)malloc(sizeof(struct seg_));
            lastbbox = lastbbox->next;
        }
        lastbbox->next = NULL;
        for (tap = node->taps, first = TRUE;
             tap != NULL;
             tap = tap->next, first = FALSE) {
            if (first) {
                lastbbox->x1 = lastbbox->x2 = tap->gridx;
                lastbbox->y1 = lastbbox->y2 = tap->gridy;
            }
            else {
                lastbbox->x1 = MIN(lastbbox->x1, tap->gridx);
                lastbbox->x2 = MAX(lastbbox->x2, tap->gridx);
                lastbbox->y1 = MIN(lastbbox->y1, tap->gridy);
                lastbbox->y2 = MAX(lastbbox->y2, tap->gridy);
            }
        }

        /* Convert to X coordinates */
        lastbbox->x1 = spacing * (lastbbox->x1 + 1);
        lastbbox->y1 = height - spacing * (lastbbox->y1 + 1);
        lastbbox->x2 = spacing * (lastbbox->x2 + 1);
        lastbbox->y2 = height - spacing * (lastbbox->y2 + 1);

        /* Draw the bbox */
        w = lastbbox->x2 - lastbbox->x1;
        h = lastbbox->y1 - lastbbox->y2;
        XDrawRectangle(dpy, buffer, gc,
                       lastbbox->x1, lastbbox->y1, w, h
        );
    }

    /* if net->numnodes == 1 don't draw a wire */
    if (n == 2) {
        XDrawLine(
            dpy, buffer, gc,
            (bboxlist->x1 + bboxlist->x2)/2, (bboxlist->y1 + bboxlist->y2)/2,
            (lastbbox->x1 + lastbbox->x2)/2, (lastbbox->y1 + lastbbox->y2)/2
        );
    }
    else if (n > 2) {
        /* Compute center point */
        POINT midpoint = (POINT)malloc(sizeof(struct point_));
        midpoint->x1 = midpoint->y1 = 0;

        for (bboxit = bboxlist; bboxit != NULL; bboxit = bboxit->next) {
            midpoint->x1 += (bboxit->x1 + bboxit->x2)/2;
            midpoint->y1 += (bboxit->y1 + bboxit->y2)/2;
        }
        midpoint->x1 /= n;
        midpoint->y1 /= n;

        for (bboxit = bboxlist; bboxit != NULL; bboxit = bboxit->next) {
            XDrawLine(
                dpy, buffer, gc,
                midpoint->x1, midpoint->y1,
                (bboxit->x1 + bboxit->x2)/2, (bboxit->y1 + bboxit->y2)/2
            );
        }

        free(midpoint);
    }

    for (bboxit = bboxlist; bboxit != NULL; bboxit = lastbbox) {
        lastbbox = bboxit->next;
        free(bboxit);
    }
}


/*--------------------------------------*/
/* Graphical display of the layout	*/
/*--------------------------------------*/

void draw_layout() {

    int lastlayer;
    int i;
    NET net;

    if (dpy == NULL) return;
    else if (buffer == (Pixmap)NULL) return;

    XSetForeground(dpy, gc, whitepix);
    XFillRectangle(dpy, buffer, gc, 0, 0, width, height);

    // Check if a netlist even exists
    if (Obs[0] == NULL) {
	XCopyArea(dpy, buffer, win, gc, 0, 0, width, height, 0, 0);
	return;
    }

    switch (mapType & MAP_MASK) {
	case MAP_OBSTRUCT:
	    map_obstruction();
	    break;
	case MAP_CONGEST:
	    map_congestion();
	    break;
	case MAP_ESTIMATE:
	    map_estimate();
	    break;
    }

    // Draw all nets, much like "emit_routes" does when writing
    // routes to the DEF file.

    if ((mapType & DRAW_ROUTES) != 0) {
	lastlayer = -1;
	for (i = 0; i < Numnets; i++) {
	    net = Nlnets[i];
	    draw_net(net, FALSE, &lastlayer);
	}
    }

    // Draw unrouted nets

    if ((mapType & DRAW_UNROUTED) != 0) {
        XSetForeground(dpy, gc, blackpix);
        for (i = 0; i < Numnets; i++) {
            net = Nlnets[i];
            if (strcmp(net->netname, gndnet) != 0
                && strcmp(net->netname, vddnet) != 0
                &&net->routes == NULL) {
                draw_net_nodes(net);
            }
        }
    }

    /* Copy double-buffer onto display window */
    XCopyArea(dpy, buffer, win, gc, 0, 0, width, height, 0, 0);
}

/*--------------------------------------*/
/* GUI initialization			*/
/*--------------------------------------*/

int GUI_init(Tcl_Interp *interp)
{
   Tk_Window tkwind, tktop;
   static char *qrouterdrawdefault = ".qrouter";
   char *qrouterdrawwin, *waitcmd;
   XColor cvcolor, cvexact;
   int i;
   float frac;

   tktop = Tk_MainWindow(interp);
   if (tktop == NULL) {
      tcl_printf(stderr, "No Top-level Tk window available. . .\n");
      return TCL_ERROR;
   }

   qrouterdrawwin = (char *)Tcl_GetVar(interp, "drawwindow", TCL_GLOBAL_ONLY);
   if (qrouterdrawwin == NULL)
      qrouterdrawwin = qrouterdrawdefault;

   tkwind = Tk_NameToWindow(interp, qrouterdrawwin, tktop);
   
   if (tkwind == NULL) {
      tcl_printf(stderr, "The Tk window hierarchy must be rooted at "
		".qrouter or $drawwindow must point to the drawing window\n");		
      return TCL_ERROR;
   }
   
   Tk_MapWindow(tkwind);
   dpy = Tk_Display(tkwind);
   win = Tk_WindowId(tkwind);
   cmap = DefaultColormap (dpy, Tk_ScreenNumber(tkwind));

   load_font(&font_info);

   /* create GC for text and drawing */
   createGC(win, &gc, font_info);

   /* Initialize colors */

   XAllocNamedColor(dpy, cmap, "blue", &cvcolor, &cvexact);
   bluepix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "cyan", &cvcolor, &cvexact);
   cyanpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "green", &cvcolor, &cvexact);
   greenpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "red", &cvcolor, &cvexact);
   redpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "orange", &cvcolor, &cvexact);
   orangepix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "gold", &cvcolor, &cvexact);
   goldpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "gray70", &cvcolor, &cvexact);
   ltgraypix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "gray50", &cvcolor, &cvexact);
   graypix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "yellow", &cvcolor, &cvexact);
   yellowpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "purple", &cvcolor, &cvexact);
   purplepix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "magenta", &cvcolor, &cvexact);
   magentapix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "GreenYellow", &cvcolor, &cvexact);
   greenyellowpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "teal", &cvcolor, &cvexact);
   tealpix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "MediumVioletRed", &cvcolor, &cvexact);
   mauvepix = cvcolor.pixel;
   XAllocNamedColor(dpy, cmap, "tan1", &cvcolor, &cvexact);
   tanpix = cvcolor.pixel;

   blackpix = BlackPixel(dpy,DefaultScreen(dpy));
   whitepix = WhitePixel(dpy,DefaultScreen(dpy));

   cvcolor.flags = DoRed | DoGreen | DoBlue;
   for (i = 0; i < SHORTSPAN; i++) {
      frac = (float)i / (float)(SHORTSPAN - 1);
      /* gamma correction */
      frac = pow(frac, 0.5);

      cvcolor.green = (int)(53970 * frac);
      cvcolor.blue = (int)(46260 * frac);
      cvcolor.red = (int)(35980 * frac);
      XAllocColor(dpy, cmap, &cvcolor);
      brownvector[i] = cvcolor.pixel;
   }

   cvcolor.green = 0;
   cvcolor.red = 0;

   for (i = 0; i < LONGSPAN; i++) {
      frac = (float)i / (float)(LONGSPAN - 1);
      /* gamma correction */
      frac = pow(frac, 0.5);

      cvcolor.blue = (int)(65535 * frac);
      XAllocColor(dpy, cmap, &cvcolor);
      bluevector[i] = cvcolor.pixel;
   }

   /* Wait on window to be created */
   waitcmd = (char *)malloc(strlen(qrouterdrawwin) + 20);
   sprintf(waitcmd, "tkwait visibility %s", qrouterdrawwin);
   Tcl_Eval(interp, waitcmd);
   free(waitcmd);

   return TCL_OK;	/* proceed to interpreter */
}

/*----------------------------------------------------------------*/

static void
load_font(XFontStruct **font_info)
{
   char *fontname = "9x15";

   /* Load font and get font information structure. */

   if ((*font_info = XLoadQueryFont (dpy,fontname)) == NULL) {
      (void) Fprintf (stderr, "Cannot open 9x15 font\n");
      // exit(1);
   }
}

/*----------------------------------------------------------------*/

static void
createGC(Window win, GC *gc, XFontStruct *font_info)
{
   unsigned long valuemask = 0; /* ignore XGCvalues and use defaults */
   XGCValues values;
   unsigned int line_width = 1;
   int line_style = LineSolid;
   int cap_style = CapRound;
   int join_style = JoinRound;

   /* Create default Graphics Context */

   *gc = XCreateGC(dpy, win, valuemask, &values);

   /* specify font */

   if (font_info != NULL)
      XSetFont(dpy, *gc, font_info->fid);

   /* specify black foreground since default window background is
    * white and default foreground is undefined. */

   XSetForeground(dpy, *gc, blackpix);

   /* set line, fill attributes */

   XSetLineAttributes(dpy, *gc, line_width, line_style,
            cap_style, join_style);
   XSetFillStyle(dpy, *gc, FillSolid);
   XSetArcMode(dpy, *gc, ArcPieSlice);
}

/*----------------------------------------------------------------*/

void expose(Tk_Window tkwind)
{
   if (Tk_WindowId(tkwind) == 0) return;
   if (dpy == NULL) return;
   draw_layout();
}

/*----------------------------------------------------------------*/

int redraw(ClientData clientData, Tcl_Interp *interp, int objc,
	Tcl_Obj *objv[])
{
   draw_layout();
   return TCL_OK;
}

/*------------------------------------------------------*/
/* Call to recalculate the spacing if NumChannelsX	*/
/* or NumChannelsY changes.				*/
/*							*/
/* Return 1 if the spacing changed, 0 otherwise.	*/
/*------------------------------------------------------*/

int recalc_spacing()
{
   int xspc, yspc;
   int oldspacing = spacing;

   xspc = width / (NumChannelsX + 1);
   yspc = height / (NumChannelsY + 1);
   spacing = (xspc < yspc) ? xspc : yspc;
   if (spacing == 0) spacing = 1;

   return (spacing == oldspacing) ? 0 : 1;
}

/*----------------------------------------------------------------*/

void resize(Tk_Window tkwind, int locwidth, int locheight)
{

   if ((locwidth == 0) || (locheight == 0)) return;

   if (buffer != (Pixmap)0)
      XFreePixmap (Tk_Display(tkwind), buffer);

   if (Tk_WindowId(tkwind) == 0)
      Tk_MapWindow(tkwind);
   
   buffer = XCreatePixmap (Tk_Display(tkwind), Tk_WindowId(tkwind),
	locwidth, locheight, DefaultDepthOfScreen(Tk_Screen(tkwind)));

   width = locwidth;
   height = locheight;

   recalc_spacing();

   if (dpy) draw_layout();
}

/*----------------------------------------------------------------*/
