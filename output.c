/*--------------------------------------------------------------*/
/*  output.c -- qrouter general purpose autorouter              */
/*  output routines for writing DEF file			*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June 2011, based on code by Steve	*/
/* Beccue, 2003							*/
/*--------------------------------------------------------------*/

#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef TCL_QROUTER
#include <tk.h>
#endif

#include "qrouter.h"
#include "qconfig.h"
#include "point.h"
#include "node.h"
#include "maze.h"
#include "mask.h"
#include "output.h"
#include "lef.h"
#include "def.h"
#include "graphics.h"

int  Pathon = -1;

struct _savepath {
    u_char active;
    int x;
    int y;
    int orient;
} path_delayed;

/*--------------------------------------------------------------*/
/* Output a list of failed nets.				*/
/*--------------------------------------------------------------*/

int write_failed(char *filename)
{
    FILE *ffail;
    NET net;
    NETLIST nl;
    int failcount;

    failcount = countlist(FailedNets);
    if (failcount == 0) {
	Fprintf(stdout, "There are no failing net routes.\n");
	return 0;
    }

    ffail = fopen(filename, "w");
    if (ffail == NULL) {
	Fprintf(stderr, "Could not open file %s for writing.\n", filename);
	return 1;
    }
    fprintf(ffail, "%d nets failed to route:\n", failcount);

    for (nl = FailedNets; nl; nl = nl->next) {
        net = nl->net;
        fprintf(ffail, " %s\n", net->netname);
    }

    fclose(ffail);
    return 0;
}

/*--------------------------------------------------------------*/
/* Write the output annotated DEF file.				*/
/*--------------------------------------------------------------*/

int write_def(char *filename)
{
   NET net;
   NETLIST nl;

   emit_routes((filename == NULL) ? DEFfilename : filename,
		Scales.oscale, Scales.iscale);

   Fprintf(stdout, "----------------------------------------------\n");
   Fprintf(stdout, "Final: ");
   if (FailedNets == (NETLIST)NULL)
      Fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL) {
         Fprintf(stdout, "Failed net routes: %d\n", countlist(FailedNets));
	 Fprintf(stdout, "List of failed nets follows:\n");

	 // Output a list of the failed nets

	 for (nl = FailedNets; nl; nl = nl->next) {
	    net = nl->net;
	    Fprintf(stdout, " %s\n", net->netname);
	 }
	 Fprintf(stdout, "\n");
      }
   }
   Fprintf(stdout, "----------------------------------------------\n");

   return 0;

} /* write_def() */

/*--------------------------------------------------------------*/
/* pathstart - begin a DEF format route path           		*/
/*								*/
/* 	If "special" is true, then this path is in a		*/
/*	SPECIALNETS section, in which each route specifies	*/
/*	a width.						*/
/*--------------------------------------------------------------*/

static void
pathstart(FILE *cmd, int layer, int x, int y, u_char special, double oscale,
          double invscale, u_char horizontal, NODEINFO node)
{
   if (Pathon == 1) {
      Fprintf( stderr, "pathstart():  Major error.  Started a new "
		"path while one is in progress!\n"
		"Doing it anyway.\n" );
   }

   if (layer >= 0) {
      if (Pathon == -1)
	 fprintf(cmd, "+ ROUTED ");
      else
	 fprintf(cmd, "\n  NEW ");
      if (special) {
	 double wvia;
	 int vtype = 0;		/* Need to get via type from node record! */

	 if (node != NULL) {
	     if ((node->flags & NI_NO_VIAX) && (!(node->flags & NI_VIA_X)))
		vtype = 2;
	     else if (node->flags & NI_VIA_Y)
		vtype = 2;
	 }
	 else {
	     /* Assume via orientation matches default route direction.	*/
	     /* NOTE:  Need to mark the actual orientation somehow. . . */
	     int ob = LefGetRouteOrientation((layer > 0) ? (layer - 1) : layer);
	     if (ob == 1) vtype = 2;
	 }

	 wvia = LefGetXYViaWidth(layer, layer, horizontal, vtype);
	 if (layer > 0) { 
	    double wvia2;
	    wvia2 = LefGetXYViaWidth(layer - 1, layer, horizontal, vtype);
	    if (wvia2 > wvia) wvia = wvia2;
         }

         fprintf(cmd, "%s %ld ( %ld %ld ) ", CIFLayer[layer],
			(long)(0.5 + invscale * oscale * wvia),
			(long)(0.5 + invscale * x), (long)(0.5 + invscale * y));
      }
      else
         fprintf(cmd, "%s ( %ld %ld ) ", CIFLayer[layer],
			(long)(0.5 + invscale * x), (long)(0.5 + invscale * y));
   }
   Pathon = 1;

} /* pathstart() */

/*--------------------------------------------------------------*/
/* pathto  - continue a path to the next point        		*/
/*								*/
/*   ARGS: coordinate pair					*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

static void
pathto(FILE *cmd, int x, int y, int horizontal, int lastx, int lasty,
       double invscale, u_char nextvia)
{
    if (Pathon <= 0) {
	Fprintf(stderr, "pathto():  Major error.  Added to a "
		"non-existent path!\n"
		"Doing it anyway.\n");
    }

    /* If the route is not manhattan, then it's because an offset
     * was added to the last point, and we need to add a small
     * jog to the route.
     */

    if ((x != lastx) && (y != lasty)) {
	if (horizontal)
	   pathto(cmd, lastx, y, FALSE, lastx, lasty, invscale, 0);
	else
	   pathto(cmd, x, lasty, TRUE, lastx, lasty, invscale, 0);
    }

    if (nextvia) {
	/* Punt on output until via is output, because via may have an	*/
	/* offset position that needs to be applied to the route.	*/
	path_delayed.active = 1;
	path_delayed.x = x;
	path_delayed.y = y;
	path_delayed.orient = horizontal;
	return;
    }

    fprintf(cmd, "( ");
    if (horizontal)
	fprintf(cmd, "%ld ", (long)(0.5 + invscale * x));
    else
	fprintf(cmd, "* ");

    if (horizontal)
	fprintf(cmd, "* ");
    else
	fprintf(cmd, "%ld ", (long)(0.5 + invscale * y));

    fprintf(cmd, ") ");

} /* pathto() */

/*--------------------------------------------------------------*/
/* pathvia  - add a via to a path               		*/
/*								*/
/*   ARGS: coord						*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

static void
pathvia(FILE *cmd, int layer, int x, int y, int lastx, int lasty,
	char *vianame, double invscale)
{
    if (path_delayed.active == 1) {
	/* Output the last path */
	pathto(cmd, path_delayed.x, path_delayed.y, path_delayed.orient,
		path_delayed.x, path_delayed.y, invscale, 0);
	path_delayed.active = 0;
    }

    if (Pathon <= 0) {
       if (Pathon == -1)
	  fprintf(cmd, "+ ROUTED ");
       else 
	  fprintf(cmd, "\n  NEW ");
       fprintf(cmd, "%s ( %ld %ld ) ", CIFLayer[layer],
		(long)(0.5 + invscale * x), (long)(0.5 + invscale * y));
    }
    else {
       // Normally the path will be manhattan and only one of
       // these will be true.  But if the via gets an offset to
       // avoid a DRC spacing violation with an adjacent via,
       // then we may need to apply both paths to make a dog-leg
       // route to the via.

       if (x != lastx)
	  pathto(cmd, x, lasty, TRUE, lastx, lasty, invscale, 0);
       if (y != lasty)
	  pathto(cmd, x, y, FALSE, x, lasty, invscale, 0);
    }
    fprintf(cmd, "%s ", vianame);
    Pathon = 0;

} /* pathvia() */

/*--------------------------------------------------------------*/
/* Nodes aren't saved in a way that makes it easy to recall	*/
/* the name of the cell and pin to which they belong.  But	*/
/* that information doesn't need to be looked up except as a	*/
/* diagnostic output.  This routine does that lookup.		*/
/*--------------------------------------------------------------*/

char *print_node_name(NODE node)
{
    GATE g;
    int i;
    char *nodestr = NULL;

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    if (g->noderec[i] == node) {
		if (nodestr != NULL)
		   free(nodestr);

		if (!strcmp(g->node[i], "pin")) {
		    nodestr = (char *)malloc(strlen(g->gatename) + 5);
		    sprintf(nodestr, "PIN/%s", g->gatename);
		}
		else {
		    nodestr = (char *)malloc(strlen(g->gatename)
				+ strlen(g->node[i]) + 2);
		    sprintf(nodestr, "%s/%s", g->gatename, g->node[i]);
		}
		return nodestr;
	    }
	}
    }
    if (nodestr != NULL) free(nodestr);
    nodestr = (char *)malloc(22);
    sprintf(nodestr, "(error: no such node)");
    return nodestr;
}

/*--------------------------------------------------------------*/
/* print_nets - print the nets list - created from Nlgates list */
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Sat July 26		*/
/*--------------------------------------------------------------*/

void print_nets(char *filename)
{
   FILE *o;
   GATE g;
   int i;
   DSEG drect;

   if (!strcmp(filename, "stdout")) {
	o = stdout;
   } else {
	o = fopen(filename, "w");
   }
   if (!o) {
	Fprintf(stderr, "route:print_nets.  Couldn't open output file\n");
	return;
   }

   for (g = Nlgates; g; g = g->next) {
      fprintf(o, "%s: %s: nodes->", g->gatename, g->gatetype->gatename);
      for (i = 0; i < g->nodes; i++) {
	 // This prints the first tap position only.
	 drect = g->taps[i];
	 fprintf( o, "%s(%g,%g) ", g->node[i], drect->x1, drect->y1);
      }
   }
   fprintf( o, "\n");
} /* print_nets() */

/*--------------------------------------------------------------*/
/* print_routes - print the routes list				*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Sat July 26		*/
/*--------------------------------------------------------------*/

void print_routes( char *filename )
{
    FILE *o;
    GATE g;
    int i;

    if( !strcmp( filename, "stdout" ) ) {
	o = stdout;
    } else {
	o = fopen( filename, "w" );
    }
    if( !o ) {
	Fprintf( stderr, "route:print_routes.  Couldn't open output file\n" );
	return;
    }

    for (g = Nlgates; g; g = g->next) {
	fprintf( o, "%s: %s: nodes->", g->gatename, g->gatetype->gatename );
	for( i = 0 ; i < g->nodes; i++ ) {
	    fprintf( o, "%s ", g->node[i] );
	}
	fprintf(o, "\n");
    }
} /* print_routes() */

/*--------------------------------------------------------------*/
/* print_nlgates - print the nlgate list			*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Wed July 23		*/
/*--------------------------------------------------------------*/

void print_nlgates( char *filename )
{
    FILE *o;
    GATE g;
    int i;
    DSEG drect;

    if( !strcmp( filename, "stdout" ) ) {
	o = stdout;
    } else {
	o = fopen( filename, "w" );
    }
    if( !o ) {
	Fprintf( stderr, "route:print_nlgates.  Couldn't open output file\n" );
	return;
    }

    for (g = Nlgates; g; g = g->next) {
	fprintf( o, "%s: %s: nodes->", g->gatename, g->gatetype->gatename );
	for( i = 0 ; i < g->nodes; i++ ) {
	    // This prints the first tap position only.
	    drect = g->taps[i];
	    fprintf( o, "%s(%g,%g)", g->node[i], drect->x1, drect->y1);
	}
        fprintf(o, "\n");
    }
} /* print_nlgates() */


/*--------------------------------------------------------------*/
/* print_net - print info about the net to stdout               */
/*								*/
/*   ARGS: net to print info about				*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

void print_net(NET net) {
    NODE node;
    DPOINT tap;
    int i, first;

    Fprintf(stdout, "Net %d: %s", net->netnum, net->netname);
    for (node = net->netnodes; node != NULL; node = node->next) {
        Fprintf(stdout, "\n  Node %d: \n    Taps: ", node->nodenum);
        for (tap = node->taps, i = 0, first = TRUE;
             tap != NULL;
             tap = tap->next, i = (i + 1) % 4, first = FALSE) {
            Fprintf(stdout, "%sL%d:(%.2lf,%.2lf)",
                    (i == 0 ? (first ? "" : "\n        ") : " "),
                    tap->layer, tap->x, tap->y
            );
        }
        Fprintf(stdout, "\n    Tap extends: ");
        for (tap = node->extend, i = 0, first = TRUE;
             tap != NULL;
             tap = tap->next, i = (i + 1) % 4, first = FALSE) {
            Fprintf(stdout, "%sL%d:(%.2lf,%.2lf)",
                    (i == 0 ? (first ? "" : "\n        ") : " "),
                    tap->layer, tap->x, tap->y
            );
        }
    }
    Fprintf(stdout, "\n  bbox: (%d,%d)-(%d,%d)\n",
            net->xmin, net->ymin, net->xmax, net->ymax
    );
}


/*--------------------------------------------------------------*/
/* print_gate - print info about the net to stdout              */
/*								*/
/*   ARGS: gate to print info about				*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

void print_gate(GATE gate) {
    int i, j, first;
    DSEG seg;
    NODE node;
    DPOINT tap;

    Fprintf(stdout, "Gate %s\n", gate->gatename);
    Fprintf(stdout, "  Loc: (%.2lf, %.2lf), WxH: %.2lfx%.2lf\n",
            gate->placedX, gate->placedY, gate->width, gate->height
    );
    Fprintf(stdout, "  Pins");
    for (i = 0; i < gate->nodes; i++) {
        Fprintf(stdout, "\n    Pin %s, net %d\n",
                gate->node[i], gate->netnum[i]
        );
        Fprintf(stdout, "      Segs: ");
        for (seg = gate->taps[i], j = 0, first = TRUE;
             seg != NULL;
             seg = seg->next, j = (j + 1) % 3, first = FALSE) {
            Fprintf(stdout, "%sL%d:(%.2lf,%.2lf)-(%.2lf,%.2lf)",
                    (j == 0 ? (first ? "" : "\n        ") : " "),
                    seg->layer, seg->x1, seg->y1, seg->x2, seg->y2
            );
        }
        if ((node = gate->noderec[i]) != NULL) {
            Fprintf(stdout, "\n      Taps: ");
            for (tap = node->taps, j = 0, first = TRUE;
                 tap != NULL;
                 tap = tap->next, j = (j + 1) % 4, first = FALSE) {
                Fprintf(stdout, "%sL%d:(%.2lf,%.2lf)",
                        (j == 0 ? (first ? "" : "\n        ") : " "),
                        tap->layer, tap->x, tap->y
                );
            }
            Fprintf(stdout, "\n      Tap extends: ");
            for (tap = node->extend, j = 0, first = TRUE;
                 tap != NULL;
                 tap = tap->next, j = (j + 1) % 4, first = FALSE) {
                Fprintf(stdout, "%sL%d:(%.2lf,%.2lf)",
                        (j == 0 ? (first ? "" : "\n        ") : " "),
                        tap->layer, tap->x, tap->y
                );
            }
        }
    }
    Fprintf(stdout, "\n  Obstructions: ");
    for (seg = gate->obs, j = 0, first = TRUE;
         seg != NULL;
         seg = seg->next, j = (j + 1) % 3, first = FALSE) {
        Fprintf(stdout, "%sL%d:(%.2lf,%.2lf)-(%.2lf,%.2lf)",
                (j == 0 ? (first ? "" : "\n    ") : " "),
                seg->layer, seg->x1, seg->y1, seg->x2, seg->y2
        );
    }
    Fprintf(stdout, "\n");
}

/*--------------------------------------------------------------*/
/* print_nodes - show the nodes list				*/
/*								*/
/*    ARGS: filename to print to				*/
/*    RETURNS: nothing						*/
/*    SIDE EFFECTS: none					*/
/*    AUTHOR and DATE: steve beccue      Tue Aug 04  2003	*/
/*--------------------------------------------------------------*/

void print_nodes(char *filename)
{
  FILE *o;
  int i;
  NET net;
  NODE node;
  DPOINT dp;

    if (!strcmp(filename, "stdout")) {
	o = stdout;
    } else {
	o = fopen(filename, "w");
    }
    if (!o) {
	Fprintf( stderr, "node.c:print_nodes.  Couldn't open output file\n" );
	return;
    }

    for (i = 0; i < Numnets; i++) {
       net = Nlnets[i];
       for (node = net->netnodes; node; node = node->next) {
	  dp = (DPOINT)node->taps;
	  fprintf(o, "%d\t%s\t(%g,%g)(%d,%d) :%d:num=%d netnum=%d\n",
		node->nodenum, 
		node->netname,
		// legacy:  print only the first point
		dp->x, dp->y, dp->gridx, dp->gridy,
		node->netnum, node->numnodes, node->netnum );
		 
	  /* need to print the routes to this node (deprecated)
	  for (j = 0 ; j < g->nodes; j++) {
	      fprintf(o, "%s(%g,%g) ", g->node[j], *(g->x[j]), *(g->y[j]));
	  }
	  */
       }
    }
    fclose(o);

} /* print_nodes() */

/*--------------------------------------------------------------*/
/* print_nlnets - show the nets					*/
/*								*/
/*   ARGS: filename to print to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: none						*/
/*   AUTHOR and DATE: steve beccue      Tue Aug 04  2003	*/
/*--------------------------------------------------------------*/

void print_nlnets( char *filename )
{
  FILE *o;
  int i;
  NODE nd;
  NET net;

    if (!strcmp(filename, "stdout")) {
	o = stdout;
    } else {
	o = fopen(filename, "w");
    }
    if (!o) {
	Fprintf(stderr, "node.c:print_nlnets.  Couldn't open output file\n");
	return;
    }

    for (i = 0; i < Numnets; i++) {
        net = Nlnets[i];
	fprintf(o, "%d\t#=%d\t%s   \t\n", net->netnum, 
		 net->numnodes, net->netname);

	for (nd = net->netnodes; nd; nd = nd->next) {
	   fprintf(o, "%d ", nd->nodenum);
	}
    }

    fprintf(o, "%d nets\n", Numnets);
    fflush(o);

} /* print_nlnets() */

/*--------------------------------------------------------------*/
/* link_up_seg ---						*/
/*								*/
/* As part of cleanup_net (below), when removing unneeded vias,	*/
/* it may be necessary to check if a segment is being used to	*/
/* connect to another route of a net, in which case removing	*/
/* the segment would break the net.  If that is the case, then	*/
/* keep the segment by linking it to the end of the route that	*/
/* was connected to it.						*/
/*								*/
/* Return 1 (true) if the segment was linked to another route,	*/
/* so the caller knows whether or not to free the segment.	*/
/*								*/
/* "rt" is the route that the segment was connected to.  For	*/
/* the sake of efficiency, it does not need to be checked.	*/
/*--------------------------------------------------------------*/

u_char link_up_seg(NET net, SEG seg, int viabase, ROUTE srt)
{
    ROUTE rt;
    SEG segf, segl;
    int x, y;

    for (rt = net->routes; rt; rt = rt->next) {
	if (rt == srt) continue;
	segf = rt->segments;
	if ((segf->x1 == seg->x1) && (segf->y1 == seg->y1) &&
		((segf->layer == viabase) || (segf->layer == viabase + 1))) {
	    /* Reverse seg and prepend it to the route */
	    seg->next = rt->segments;
	    rt->segments = seg;
	    x = seg->x1;
	    y = seg->y1;
	    seg->x1 = seg->x2;
	    seg->y1 = seg->y2;
	    seg->x2 = x;
	    seg->y2 = y;
	    return (u_char)1;
	}

	/* Move to the last segment of the route */
	for (segl = segf; segl && segl->next; segl = segl->next);

	if (segl && (segl->x2 == seg->x1) && (segl->y2 == seg->y1) &&
		((segl->layer == viabase) || (segl->layer == viabase + 1))) {
	    /* Append seg to the route */
	    segl->next = seg;
	    return (u_char)1;
	}
    }
    return (u_char)0;
}

/*--------------------------------------------------------------*/
/* cleanup_net --						*/
/*								*/
/* Special handling for layers where needblock[] is non-zero,	*/
/* and shows that two vias cannot be placed on adjacent routes. */
/* emit_routed_net() will add specialnets to merge two adjacent	*/
/* vias on the same route.  However, this cannot be used for	*/
/* adjacent vias that are each in a different route record.  It	*/
/* is easier just to find any such instances and remove them by	*/
/* eliminating one of the vias and adding a segment to connect	*/
/* the route to the neighboring via.				*/
/*								*/
/* Note that the ensuing change in connectivity can violate	*/
/* the route endpoints and thereby mess up the delay output	*/
/* routine and/or the antenna violation finding routine unless	*/
/* route_set_connections() is re-run on the modified routes.	*/
/*--------------------------------------------------------------*/

void cleanup_net(NET net)
{
   SEG segf, segl, seg, segp;
   ROUTE rt, rt2;
   NODEINFO lnode;
   int lf, ll, lf2, ll2, viabase;
   u_char fcheck, lcheck, needfix;
   u_char xcheckf, ycheckf, xcheckl, ycheckl; 

   lf = ll = lf2 = ll2 = -1;
   needfix = FALSE;

   for (rt = net->routes; rt; rt = rt->next) {
      fcheck = lcheck = FALSE;

      // This problem will only show up on route endpoints.
      // segf is the first segment of the route.
      // segl is the last segment of the route.
      // lf is the layer at the route start (layer first)
      // lf2 is the layer of the second segment.
      // ll is the layer at the route end (layer last)
      // ll2 is the layer of the next-to-last segment

      segf = rt->segments;
      if (segf == NULL) continue;
      if ((segf->next != NULL) && (segf->segtype == ST_VIA)) {
	 if (segf->next->layer > segf->layer) {
	    lf = segf->layer;
	    lf2 = segf->layer + 1;
	 }
	 else {
	    lf = segf->layer + 1;
	    lf2 = segf->layer;
	 }
         // Set flag fcheck indicating that segf needs checking
	 fcheck = TRUE;

	 // We're going to remove the contact so it can't be a tap
	 if ((lf < Pinlayers) && ((lnode = NODEIPTR(segf->x1, segf->y1, lf)) != NULL)
			&& (lnode->nodesav != NULL))
	    fcheck = FALSE;
      }

      /* This could be done if vias are always too close when	*/
      /* placed on adjacent tracks.  However, that ignores the	*/
      /* problem of vias with offsets, and it ignores the fact	*/
      /* that adjacent vias on the same net are always a	*/
      /* redundancy. 						*/

      xcheckf = needblock[lf] & VIABLOCKX;
      ycheckf = needblock[lf] & VIABLOCKY;
      if (!xcheckf && !ycheckf) fcheck = FALSE;

      // Move to the next-to-last segment
      for (segl = segf->next; segl && segl->next && segl->next->next;
		segl = segl->next);

      if (segl && (segl->next != NULL) && (segl->next->segtype == ST_VIA)) {
	 if (segl->next->layer < segl->layer) {
	    ll = segl->next->layer;
	    ll2 = segl->next->layer + 1;
	 }
	 else {
	    ll = segl->next->layer + 1;
	    ll2 = segl->next->layer;
	 }
	 // Move segl to the last segment
	 segl = segl->next;
	 // Set flag lcheck indicating that segl needs checking.
	 lcheck = TRUE;

	 // We're going to remove the contact so it can't be a tap
	 if ((ll < Pinlayers) && ((lnode = NODEIPTR(segl->x1, segl->y1, ll)) != NULL)
			&& (lnode->nodesav != NULL))
	    lcheck = FALSE;
      }

      xcheckl = needblock[ll] & VIABLOCKX;
      ycheckl = needblock[ll] & VIABLOCKY;
      if (!xcheckl && !ycheckl) lcheck = FALSE;

      // For each route rt2 that is not rt, look at every via
      // and see if it is adjacent to segf or segl.

      for (rt2 = net->routes; rt2; rt2 = rt2->next) {

         if ((fcheck == FALSE) && (lcheck == FALSE)) break;
         if (rt2 == rt) continue;

         for (seg = rt2->segments; seg; seg = seg->next) {
	    if (seg->segtype & ST_VIA) {
	       if (fcheck) {
		  if ((seg->layer == lf) || ((seg->layer + 1) == lf)) {
		     if (xcheckf && (seg->y1 == segf->y1) &&
				(ABSDIFF(seg->x1, segf->x1) == 1)) {
			needfix = TRUE;
			if (seg->layer != segf->layer) {

			   // Adjacent vias are different types.
			   // Deal with it by creating a route between
			   // the vias on their shared layer.  This
			   // will later be made into a special net to
			   // avoid notch DRC errors.

			   SEG newseg;
			   newseg = (SEG)malloc(sizeof(struct seg_));
			   rt->segments = newseg;
			   newseg->next = segf;
			   newseg->layer = lf;
			   newseg->segtype = ST_WIRE;
			   newseg->x1 = segf->x1;
			   newseg->y1 = segf->y1;
			   newseg->x2 = seg->x1; 
			   newseg->y2 = seg->y1;
			}
			else {
			   // Change via to wire route, connect it to seg,
			   // and make sure it has the same layer type as
			   // the following route.
			   segf->segtype = ST_WIRE;
			   segf->x1 = seg->x1;
			   segf->layer = lf2;
		        }
		     }
		     else if (ycheckf && (seg->x1 == segf->x1) &&
				(ABSDIFF(seg->y1, segf->y1) == 1)) {
			needfix = TRUE;
			if (seg->layer != segf->layer) {
			   // Adjacent vias are different types.
			   // Deal with it by creating a route between
			   // the vias on their shared layer.  This
			   // will later be made into a special net to
			   // avoid notch DRC errors.

			   SEG newseg;
			   newseg = (SEG)malloc(sizeof(struct seg_));
			   rt->segments = newseg;
			   newseg->next = segf;
			   newseg->layer = lf;
			   newseg->segtype = ST_WIRE;
			   newseg->x1 = segf->x1;
			   newseg->y1 = segf->y1;
			   newseg->x2 = seg->x1; 
			   newseg->y2 = seg->y1;
		        }
		        else {
			   // Change via to wire route, connect it to seg,
			   // and make sure it has the same layer type as
			   // the following route.
			   segf->segtype = ST_WIRE;
			   segf->y1 = seg->y1;
			   segf->layer = lf2;
			}
		     }
		  }
	       }

               if (lcheck) {
		  if ((seg->layer == ll) || ((seg->layer + 1) == ll)) {
		     if (xcheckl && (seg->y1 == segl->y1) &&
				(ABSDIFF(seg->x1, segl->x1) == 1)) {
			needfix = TRUE;
			if (seg->layer != segl->layer) {

			   // Adjacent vias are different types.
			   // Deal with it by creating a route between
			   // the vias on their shared layer.  This
			   // will later be made into a special net to
			   // avoid notch DRC errors.

			   SEG newseg;
			   newseg = (SEG)malloc(sizeof(struct seg_));
			   segl->next = newseg;
			   newseg->next = NULL;
			   newseg->layer = ll;
			   newseg->segtype = ST_WIRE;
			   newseg->x1 = segl->x1;
			   newseg->y1 = segl->y1;
			   newseg->x2 = seg->x1; 
			   newseg->y2 = seg->y1;
			}
			else {
			   // Change via to wire route, connect it to seg,
			   // and make sure it has the same layer type as
			   // the previous route.
			   segl->segtype = ST_WIRE;
			   segl->x2 = seg->x2;
			   segl->layer = ll2;
			}
		     }
		     else if (ycheckl && (seg->x1 == segl->x1) &&
				(ABSDIFF(seg->y1, segl->y1) == 1)) {
			needfix = TRUE;
			if (seg->layer != segl->layer) {

			   // Adjacent vias are different types.
			   // Deal with it by creating a route between
			   // the vias on their shared layer.  This
			   // will later be made into a special net to
			   // avoid notch DRC errors.

			   SEG newseg;
			   newseg = (SEG)malloc(sizeof(struct seg_));
			   segl->next = newseg;
			   newseg->next = NULL;
			   newseg->layer = ll;
			   newseg->segtype = ST_WIRE;
			   newseg->x1 = segl->x1;
			   newseg->y1 = segl->y1;
			   newseg->x2 = seg->x1; 
			   newseg->y2 = seg->y1;
			}
			else {
			   // Change via to wire route, connect it to seg,
			   // and make sure it has the same layer type as
			   // the previous route.
			   segl->segtype = ST_WIRE;
			   segl->y2 = seg->y2;
			   segl->layer = ll2;
			}
		     }
		  }
	       }
	    }
	 }
      }

      /* One case not covered by the checks above:  If the second or	*/
      /* penultimate segment is a via and the final segment is one	*/
      /* track in length and connects to a via, then the same		*/
      /* replacement can be made.  The other routes do not need to be	*/
      /* checked, as it is sufficient to check that the grid is 	*/
      /* occupied at that point on two metal layers with the same net.	*/

      /* NOTE:  Another route could be connecting to the via on the	*/
      /* penultimate segment, and removing it would cause an open net.	*/
      /* Check for this case and resolve if needed.			*/

      if ((fcheck == FALSE) && (lcheck == FALSE)) {
	 int wlen, oval0, oval1, oval2;

	 segf = rt->segments;
	 if ((segf == NULL) || (segf->next == NULL)) continue;
	 seg = segf->next;
	 if ((segf->segtype == ST_WIRE) && (seg->segtype == ST_VIA)) {
	    if ((segf->x1 - segf->x2) == 0) {
		wlen = segf->y1 - segf->y2;
	        if ((wlen == 1) || (wlen == -1)) {
		    oval1 = OBSVAL(segf->x1, segf->y1, seg->layer) & ROUTED_NET_MASK;
		    oval2 = OBSVAL(segf->x1, segf->y1, seg->layer + 1) & ROUTED_NET_MASK;
		    if (oval1 == oval2) {
			/* Check false case in which (layer + 1) is a min area stub */
			segp = seg->next;
			if (segp && (segp->x2 == segf->x1) && (segp->y2 == segf->y1))
			    continue;
			/* Remove via and change wire layer */
			needfix = TRUE;
			segf->next = seg->next;
			viabase = segf->layer;
			segf->layer = (viabase == seg->layer) ? seg->layer + 1 :
				seg->layer;
			if (!link_up_seg(net, seg, viabase, rt)) free(seg);
		    }
		}
	    }
	    else if ((segf->y1 - segf->y2) == 0) {
		wlen = segf->x1 - segf->x2;
	        if ((wlen == 1) || (wlen == -1)) {
		    oval1 = OBSVAL(segf->x1, segf->y1, seg->layer) & ROUTED_NET_MASK;
		    oval2 = OBSVAL(segf->x1, segf->y1, seg->layer + 1) & ROUTED_NET_MASK;
		    if (oval1 == oval2) {
			/* Check false case in which (layer + 1) is a min area stub */
			segp = seg->next;
			if (segp && (segp->x2 == segf->x1) && (segp->y2 == segf->y1))
			    continue;
			/* Remove via and change wire layer */
			needfix = TRUE;
			segf->next = seg->next;
			viabase = segf->layer;
			segf->layer = (viabase == seg->layer) ? seg->layer + 1 :
				seg->layer;
			if (!link_up_seg(net, seg, viabase, rt)) free(seg);
		    }
		}
	    }
	 }
	 segp = NULL;
	 for (seg = rt->segments; seg && seg->next && seg->next->next; seg = seg->next)
	     segp = seg;
	 if ((seg == NULL) || (seg->next == NULL)) continue;
	 segl = seg->next;
	 if ((segl->segtype == ST_WIRE) && (seg->segtype == ST_VIA)) {
	    if ((segl->x1 - segl->x2) == 0) {
		wlen = segl->y1 - segl->y2;
	        if ((wlen == 1) || (wlen == -1)) {
		    oval1 = OBSVAL(segl->x2, segl->y2, seg->layer) & ROUTED_NET_MASK;
		    oval2 = OBSVAL(segl->x2, segl->y2, seg->layer + 1) & ROUTED_NET_MASK;
		    if (oval1 == oval2) {
			/* Check false case in which (layer + 1) is a min area stub */
			if (segp && (segp->x1 == segl->x2) && (segp->y1 == segl->y2))
			    continue;
			/* Remove via and change wire layer */
			needfix = TRUE;
			seg->next = NULL;
			seg->segtype = ST_WIRE;
			viabase = seg->layer;
			seg->layer = (viabase == segl->layer) ? viabase + 1 : viabase;
			seg->x1 = segl->x1;
			seg->y1 = segl->y1;
			seg->x2 = segl->x2;
			seg->y2 = segl->y2;
			if (!link_up_seg(net, segl, viabase, rt)) free(segl);
		    }
		}
	    }
	    else if ((segl->y1 - segl->y2) == 0) {
		wlen = segl->x1 - segl->x2;
	        if ((wlen == 1) || (wlen == -1)) {
		    oval1 = OBSVAL(segl->x2, segl->y2, seg->layer) & ROUTED_NET_MASK;
		    oval2 = OBSVAL(segl->x2, segl->y2, seg->layer + 1) & ROUTED_NET_MASK;
		    if (oval1 == oval2) {
			/* Check false case in which (layer + 1) is a min area stub */
			if (segp && (segp->x1 == segl->x2) && (segp->y1 == segl->y2))
			    continue;
			/* Remove via and change wire layer */
			needfix = TRUE;
			viabase = seg->layer;
			seg->next = NULL;
			seg->segtype = ST_WIRE;
			seg->layer = (viabase == segl->layer) ? viabase + 1 : viabase;
			seg->x1 = segl->x1;
			seg->y1 = segl->y1;
			seg->x2 = segl->x2;
			seg->y2 = segl->y2;
			if (!link_up_seg(net, segl, viabase, rt)) free(segl);
		    }
		}
	    }
	 }
      }
   }
   if (needfix == TRUE)
      for (rt = net->routes; rt; rt = rt->next)
	 route_set_connections(net, rt);
}

/*--------------------------------------------------------------*/
/* emit_routed_net --						*/
/*								*/
/* Core part of emit_routes().  Dumps the DEF format for a	*/
/* complete net route to file Cmd.  If "special" is TRUE, then	*/
/* it looks only for stub routes between a grid point and an	*/
/* off-grid terminal, and dumps only the stub route geometry as	*/
/* a SPECIALNET, which takes a width parameter.  This allows	*/
/* the stub routes to be given the same width as a via, when	*/
/* the via is larger than a route width, to avoid DRC notch	*/
/* errors between the via and the terminal.  The SPECIALNETS	*/
/* are redundant;  all routing information is in the NETS	*/
/* section.  The SPECIALNETS only specify a wider route for the	*/
/* stub connection.						*/
/*--------------------------------------------------------------*/

static void
emit_routed_net(FILE *Cmd, NET net, u_char special, double oscale, int iscale)
{
   SEG seg, saveseg, lastseg, prevseg;
   NODEINFO lnode, lnode1, lnode2;
   ROUTE rt;
   u_int dir1, dir2, tdir;
   int layer;
   int x = 0, y = 0, x2, y2;
   double dc;
   int lastx = -1, lasty = -1, lastlay;
   int horizontal;
   float offset1, offset2, stub, offset;
   u_char cancel, segtype, nextvia;
   double invscale = (double)(1.0 / (double)iscale); 

   /* If the STUB flag is set, then we need to write out the net name	*/
   /* in the SPECIALNETS section.					*/

   if ((special == (u_char)1) && (net->flags & NET_STUB)) {
      fprintf(Cmd, ";\n- %s\n", net->netname);
   }

   Pathon = -1;
   lastlay = -1;

   /* Insert routed net here */
   for (rt = net->routes; rt; rt = rt->next) {

      path_delayed.active = 0;

      if (rt->segments && !(rt->flags & RT_OUTPUT)) {
	 horizontal = FALSE;
	 cancel = FALSE;

	 // Check first position for terminal offsets
	 seg = (SEG)rt->segments;
	 lastseg = saveseg = seg;
	 layer = seg->layer;
	 if (seg) {
	    nextvia = (seg->next) ? ((seg->next->segtype == ST_VIA) ? 1 : 0) : 0;

	    // It is rare but possible to have a stub route off of an
	    // endpoint via, so check this case, and use the layer type
	    // of the via top if needed.

	    if ((seg->segtype & ST_VIA) && seg->next && (seg->next->layer <=
			seg->layer))
	       layer++;

	    lnode = (layer < Pinlayers) ? NODEIPTR(seg->x1, seg->y1, layer) : NULL;
	    stub = (lnode) ? lnode->stub : 0.0;
	    if (OBSVAL(seg->x1, seg->y1, layer) & STUBROUTE) {
	       if ((special == (u_char)0) && (Verbose > 2))
		  Fprintf(stdout, "Stub route distance %g to terminal"
				" at %d %d (%d)\n", stub,
				seg->x1, seg->y1, layer);

	       dc = Xlowerbound + (double)seg->x1 * PitchX;
	       x = (int)((REPS(dc)) * oscale);
	       if (lnode->flags & NI_STUB_EW)
		  dc += stub;
	       x2 = (int)((REPS(dc)) * oscale);
	       dc = Ylowerbound + (double)seg->y1 * PitchY;
	       y = (int)((REPS(dc)) * oscale);
	       if (lnode->flags & NI_STUB_NS)
		  dc += stub;
	       y2 = (int)((REPS(dc)) * oscale);
	       if (lnode->flags & NI_STUB_EW) {
		  horizontal = TRUE;

		  // If the gridpoint ahead of the stub has a route
		  // on the same net, and the stub is long enough
		  // to come within a DRC spacing distance of the
		  // other route, then lengthen it to close up the
		  // distance and resolve the error.  (NOTE:  This
		  // unnecessarily stretches routes to cover taps
		  // that have not been routed to.  At least on the
		  // test standard cell set, these rules remove a
		  // handful of DRC errors and don't create any new
		  // ones.  If necessary, a flag can be added to
		  // distinguish routes from taps.

		  if ((x < x2) && (seg->x1 < (NumChannelsX - 1))) {
		     tdir = OBSVAL(seg->x1 + 1, seg->y1, layer);
		     if ((tdir & ROUTED_NET_MASK) ==
					(net->netnum | ROUTED_NET)) {
			if (stub + LefGetRouteKeepout(layer) >= PitchX) {
		      	   dc = Xlowerbound + (double)(seg->x1 + 1) * PitchX;
		      	   x2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }
		  else if ((x > x2) && (seg->x1 > 0)) {
		     tdir = OBSVAL(seg->x1 - 1, seg->y1, layer);
		     if ((tdir & ROUTED_NET_MASK) ==
					(net->netnum | ROUTED_NET)) {
			if (-stub + LefGetRouteKeepout(layer) >= PitchX) {
		      	   dc = Xlowerbound + (double)(seg->x1 - 1) * PitchX;
		      	   x2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }

		  dc = oscale * 0.5 * LefGetRouteWidth(layer);
		  if (special == (u_char)0) {
		     // Regular nets include 1/2 route width at
		     // the ends, so subtract from the stub terminus
		     if (x < x2) {
			x2 -= dc;
			if (x >= x2) cancel = TRUE;
		     }
		     else {
			x2 += dc;
			if (x <= x2) cancel = TRUE;
		     }
		  }
		  else {
		     // Special nets don't include 1/2 route width
		     // at the ends, so add to the route at the grid
		     if (x < x2)
			x -= dc;
		     else
			x += dc;

		     // Routes that extend for more than one track
		     // without a bend do not need a wide stub
		     if (seg->x1 != seg->x2) cancel = TRUE;
	  	  }
	       }
	       else {
		  horizontal = FALSE;

		  // If the gridpoint ahead of the stub has a route
		  // on the same net, and the stub is long enough
		  // to come within a DRC spacing distance of the
		  // other route, then lengthen it to close up the
		  // distance and resolve the error.

		  if ((y < y2) && (seg->y1 < (NumChannelsY - 1))) {
		     tdir = OBSVAL(seg->x1, seg->y1 + 1, layer);
		     if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			if (stub + LefGetRouteKeepout(layer) >= PitchY) {
		      	   dc = Ylowerbound + (double)(seg->y1 + 1) * PitchY;
		      	   y2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }
		  else if ((y > y2) && (seg->y1 > 0)) {
		     tdir = OBSVAL(seg->x1, seg->y1 - 1, layer);
		     if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			if (-stub + LefGetRouteKeepout(layer) >= PitchY) {
		      	   dc = Ylowerbound + (double)(seg->y1 - 1) * PitchY;
		      	   y2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }

		  dc = oscale * 0.5 * LefGetRouteWidth(layer);
		  if (special == (u_char)0) {
		     // Regular nets include 1/2 route width at
		     // the ends, so subtract from the stub terminus
		     if (y < y2) {
			y2 -= dc;
			if (y >= y2) cancel = TRUE;
		     }
		     else {
			y2 += dc;
			if (y <= y2) cancel = TRUE;
		     }
		  }
		  else {
		     // Special nets don't include 1/2 route width
		     // at the ends, so add to the route at the grid
		     if (y < y2)
			y -= dc;
		     else
			y += dc;

		     // Routes that extend for more than one track
		     // without a bend do not need a wide stub
		     if (seg->y1 != seg->y2) cancel = TRUE;
		  }
	       }

	       if (cancel == FALSE) {
		  net->flags |= NET_STUB;
		  rt->flags |= RT_STUB;
		  pathstart(Cmd, layer, x2, y2, special, oscale, invscale, horizontal,
			lnode);
		  pathto(Cmd, x, y, horizontal, x2, y2, invscale, nextvia);
	       }
	       lastx = x;
	       lasty = y;
	       lastlay = layer;
	    }
	 }

	 prevseg = NULL;
	 lastseg = NULL;
	 for (seg = rt->segments; seg; seg = seg->next) {
	    nextvia = (seg->next) ? ((seg->next->segtype == ST_VIA) ? 1 : 0) : 0;
	    layer = seg->layer;

	    // Check for offset terminals at either point

	    offset1 = 0.0;
	    offset2 = 0.0;
	    dir1 = 0;
	    dir2 = 0;
	    lnode1 = lnode2 = NULL;

	    if (seg->segtype & ST_OFFSET_START) {
	       dir1 = OBSVAL(seg->x1, seg->y1, seg->layer) & OFFSET_TAP;
	       if ((dir1 == 0) && lastseg) {
		  dir1 = OBSVAL(lastseg->x2, lastseg->y2, lastseg->layer)
				& OFFSET_TAP;
		  lnode1 = NODEIPTR(lastseg->x2, lastseg->y2, lastseg->layer);
		  offset1 = lnode1->offset;
	       }
	       else {
		  lnode1 = NODEIPTR(seg->x1, seg->y1, seg->layer);
		  offset1 = lnode1->offset;
	       }

	       // Offset was calculated for vias;  plain metal routes
	       // typically will need less offset distance, so subtract off
	       // the difference.

	       if (!(seg->segtype & ST_VIA)) {
		  if (offset1 < 0) {
		     offset1 += 0.5 * (LefGetViaWidth(seg->layer, seg->layer, 
				horizontal) - LefGetRouteWidth(seg->layer));
		     if (offset1 > 0) offset1 = 0;
		  }
		  else if (offset1 > 0) {
		     offset1 -= 0.5 * (LefGetViaWidth(seg->layer, seg->layer,
				horizontal) - LefGetRouteWidth(seg->layer));
		     if (offset1 < 0) offset1 = 0;
		  }
	       }

	       if (special == (u_char)0) {
		  if ((seg->segtype & ST_VIA) && (Verbose > 2))
		     Fprintf(stdout, "Offset terminal distance %g to grid"
					" at %d %d (%d)\n", offset1,
					seg->x1, seg->y1, layer);
	       }
	    }
	    if (seg->segtype & ST_OFFSET_END) {
	       dir2 = OBSVAL(seg->x2, seg->y2, seg->layer) & OFFSET_TAP;
	       if ((dir2 == 0) && seg->next) {
		  dir2 = OBSVAL(seg->next->x1, seg->next->y1, seg->next->layer) &
					OFFSET_TAP;
		  lnode2 = NODEIPTR(seg->next->x1, seg->next->y1, seg->next->layer);
		  if (lnode2 != NULL)
		      offset2 = lnode2->offset;
	       }
	       else {
		  lnode2 = NODEIPTR(seg->x2, seg->y2, seg->layer);
		  if (lnode2 != NULL)
		      offset2 = lnode2->offset;
	       }

	       // Offset was calculated for vias;  plain metal routes
	       // typically will need less offset distance, so subtract off
	       // the difference.

	       if (!(seg->segtype & ST_VIA)) {
		  if (offset2 < 0) {
		     offset2 += 0.5 * (LefGetViaWidth(seg->layer, seg->layer,
				horizontal) - LefGetRouteWidth(seg->layer));
		     if (offset2 > 0) offset2 = 0;
		  }
		  else if (offset2 > 0) {
		     offset2 -= 0.5 * (LefGetViaWidth(seg->layer, seg->layer, 
				horizontal) - LefGetRouteWidth(seg->layer));
		     if (offset2 < 0) offset2 = 0;
		  }
	       }

	       if (special == (u_char)0) {
		  if ((seg->segtype & ST_VIA)
					&& !(seg->segtype & ST_OFFSET_START))
		     if (Verbose > 2)
		        Fprintf(stdout, "Offset terminal distance %g to grid"
					" at %d %d (%d)\n", offset2,
					seg->x2, seg->y2, layer);
	       }
	    }

	    // To do: pick up route layer name from lefInfo.
	    // At the moment, technology names don't even match,
	    // and are redundant between CIFLayer[] from the
	    // config file and lefInfo.

	    dc = Xlowerbound + (double)seg->x1 * PitchX;
	    if ((dir1 & OFFSET_TAP) && (lnode1->flags & NI_OFFSET_EW)) dc += offset1;
	    x = (int)((REPS(dc)) * oscale);
	    dc = Ylowerbound + (double)seg->y1 * PitchY;
	    if ((dir1 & OFFSET_TAP) && (lnode1->flags & NI_OFFSET_NS)) dc += offset1;
	    y = (int)((REPS(dc)) * oscale);
	    dc = Xlowerbound + (double)seg->x2 * PitchX;
	    if ((dir2 & OFFSET_TAP) && (lnode2->flags & NI_OFFSET_EW)) dc += offset2;
	    x2 = (int)((REPS(dc)) * oscale);
	    dc = Ylowerbound + (double)seg->y2 * PitchY;
	    if ((dir2 & OFFSET_TAP) && (lnode2->flags & NI_OFFSET_NS)) dc += offset2;
	    y2 = (int)((REPS(dc)) * oscale);
	    segtype = seg->segtype & ~(ST_OFFSET_START | ST_OFFSET_END);
	    switch (segtype) {
	       case ST_WIRE:

		  // Normally layers change only at a via.  However, if
		  // a via has been removed and replaced by a 1-track
		  // segment to a neighboring via to avoid DRC errors
		  // (see cleanup_net()), then a layer change may happen
		  // between two ST_WIRE segments, and a new path should
		  // be started.

		  if ((Pathon != -1) && (lastlay != -1) && (lastlay != seg->layer))
		     Pathon = 0;

		  if (Pathon != 1) {	// 1st point of route seg
		     if (x == x2) {
			horizontal = FALSE;
		     }
		     else if (y == y2) {
			horizontal = TRUE;
		     }
		     else if (Verbose > 3) {
			// NOTE:  This is a development diagnostic.  The
			// occasional non-Manhanhattan route is due to a
			// tap offset and is corrected automatically by
			// making an L-bend in the wire.

		     	Flush(stdout);
			Fprintf(stderr, "Warning:  non-Manhattan wire in route"
				" at (%d %d) to (%d %d)\n", x, y, x2, y2);
		     }
		     if (special == (u_char)0) {
			if (lastseg && (lastseg->segtype & ST_OFFSET_START) &&
				((lastx != x) || (lasty != y))) {
			   /* Add bend and connect to offset via */
			   int vertical = (horizontal) ? FALSE : TRUE;
			   pathstart(Cmd, seg->layer, lastx, lasty, special,
				oscale, invscale, vertical, lnode2);
			   pathto(Cmd, x, y, vertical, lastx, lasty, invscale, nextvia);
			}
			else if (lastseg && (lastseg->segtype & ST_VIA) &&
				(lastx != x) && (lasty == y) &&
				(LefGetRouteOrientation(seg->layer) == 1)) {
			   /* Via offset in direction of route (horizontal) */
			   pathstart(Cmd, seg->layer, lastx, y, special, oscale,
				invscale, horizontal, lnode2);
			}
			else if (lastseg && (lastseg->segtype & ST_VIA) &&
				(lastx == x) && (lasty != y) &&
				(LefGetRouteOrientation(seg->layer) == 0)) {
			   /* Via offset in direction of route (vertical) */
			   pathstart(Cmd, seg->layer, x, lasty, special, oscale,
				invscale, horizontal, lnode2);
			}
			else {
			   pathstart(Cmd, seg->layer, x, y, special, oscale,
				invscale, horizontal, lnode2);
			}
			lastx = x;
			lasty = y;
			lastlay = seg->layer;
		     }
		  }
		  rt->flags |= RT_OUTPUT;
		  if (horizontal && x == x2) {
		     horizontal = FALSE;
		  }
		  if ((!horizontal) && y == y2) {
		     horizontal = TRUE;
		  }
		  if (!(x == x2) && !(y == y2)) {
		     horizontal = FALSE;
		  }
		  if (special == (u_char)0) {
		     pathto(Cmd, x2, y2, horizontal, lastx, lasty, invscale, nextvia);
		     lastx = x2;
		     lasty = y2;
		  }

		  // Check for path segments that are used for minimum metal
		  // area requirements so that they do not become false
		  // positives for inter-via special nets.

		  if (lastseg && seg->next && (lastseg->x1 == seg->next->x2) &&
				(lastseg->y1 == seg->next->y2)) 
		      seg->segtype |= ST_MINMETAL;

		  // If a segment is 1 track long, there is a via on either
		  // end, and the needblock flag is set for the layer, then
		  // draw a stub route along the length of the track.

		  if (horizontal && needblock[seg->layer] & VIABLOCKX) {
		     if ((ABSDIFF(seg->x2, seg->x1) == 1) &&
				!(seg->segtype & ST_MINMETAL)) {
			if ((lastseg && lastseg->segtype == ST_VIA) ||
			    (seg->next && seg->next->segtype == ST_VIA)) {
			   if (special == (u_char)0) {
			      net->flags |= NET_STUB;
			      rt->flags |= RT_STUB;
			   }
			   else {
			      if (Pathon != -1) Pathon = 0;
			      pathstart(Cmd, layer, x, y, special, oscale,
						invscale, horizontal, lnode2);
			      pathto(Cmd, x2, y2, horizontal, x, y, invscale, 0);
			      lastlay = layer;
			   }
			}
		     }
		  }
		  else if (!horizontal && needblock[seg->layer] & VIABLOCKY) {
		     if ((ABSDIFF(seg->y2, seg->y1) == 1) &&
				!(seg->segtype & ST_MINMETAL)) {
			if ((lastseg && lastseg->segtype == ST_VIA) ||
			    (seg->next && seg->next->segtype == ST_VIA)) {
			   if (special == (u_char)0) {
			      net->flags |= NET_STUB;
			      rt->flags |= RT_STUB;
			   }
			   else {
			      if (Pathon != -1) Pathon = 0;
			      pathstart(Cmd, layer, x, y, special, oscale,
						invscale, horizontal, lnode2);
			      pathto(Cmd, x2, y2, horizontal, x, y, invscale, 0);
			      lastlay = layer;
			   }
			}
		     }
		  }
		  break;

	       case ST_VIA:
		  rt->flags |= RT_OUTPUT;
		  if (special == (u_char)0) {
		     double viaoffx, viaoffy;
		     double w0, w1, dc, altwx, altwy;
		     float offsetx, offsety;
		     int vx = 0;
		     int vy = 0;
		     int flags;
		     u_int tdirpp, tdirp, tdirn;
		     u_char viaNL, viaNM, viaNU;
		     u_char viaSL, viaSM, viaSU;
		     u_char viaEL, viaEM, viaEU;
		     u_char viaWL, viaWM, viaWU;
		     u_char rteNL, rteNU;
		     u_char rteSL, rteSU;
		     u_char rteEL, rteEU;
		     u_char rteWL, rteWU;
		     char *s;
		     char checkersign;
		     int ob, ot;

		     if (lastseg == NULL) {
			// Make sure last position is valid
			lastx = x;
			lasty = y;
		     }

		     // If vias need to be rotated then they do so on a
		     // checkerboard pattern.
		     checkersign = (char)((seg->x1 + seg->y1) & 0x01);

		     // Get the default orientation of the via
		     ob = LefGetRouteOrientation(layer);
		     ot = LefGetRouteOrientation(layer + 1);
		     if (ob == 0 && ot == 1)
			s = ViaYX[layer];
		     else if (ob == 1 && ot == 0)
			s = ViaXY[layer];
		     else if (ob == 0 && ot == 0)
			s = ViaYY[layer];
		     else
			s = ViaXX[layer];

		     // If via is on a pin and rotation is restricted, then
		     // set the rotation accordingly.

		     flags = 0;
		     if (layer < Pinlayers) {
			if ((lnode = NODEIPTR(seg->x1, seg->y1, layer)) != NULL) {
			    if (lnode->flags & NI_NO_VIAX) {
				flags = NI_NO_VIAX;
				if (s == ViaXY[layer])
				    s = ViaYY[layer];
				else if (s == ViaXX[layer])
				    s = ViaYX[layer];
			    }
			    if (lnode->flags & NI_NO_VIAY) {
				flags = NI_NO_VIAY;
				if (s == ViaYX[layer])
				    s = ViaXX[layer];
				else if (s == ViaYY[layer])
				    s = ViaXY[layer];
			    }
			    /* Mark the node with which via direction was used */
			    if ((s == ViaYY[layer]) || (s == ViaYX[layer]))
				lnode->flags |= NI_VIA_Y;
			    else
				lnode->flags |= NI_VIA_X;
			}
		     }

		     // Check for vias between adjacent but different nets
		     // that need rotation and/or position offsets to avoid
		     // a DRC spacing error

		     viaEL = viaEM = viaEU = 0;
		     viaWL = viaWM = viaWU = 0;
		     viaNL = viaNM = viaNU = 0;
		     viaSL = viaSM = viaSU = 0;
		     rteEL = rteEU = rteWL = rteWU = 0;
		     rteNL = rteNU = rteSL = rteSU = 0;

		     // Check for via/route to west
		     if (seg->x1 > 0) {
			tdir = OBSVAL(seg->x1 - 1, seg->y1, layer)
					& ROUTED_NET_MASK;

			if (((tdir & NO_NET) == 0) && (tdir != 0) &&
				(tdir != (net->netnum | ROUTED_NET))) {
			   rteWL = 1;
			   if (layer > 0) {
			      tdirn = OBSVAL(seg->x1 - 1, seg->y1, layer - 1)
					& ROUTED_NET_MASK;
			      if (tdir == tdirn) viaWL = 1;
			   }
			}

			if (layer < Num_layers - 1) {
			   tdirp = OBSVAL(seg->x1 - 1, seg->y1, layer + 1)
					& ROUTED_NET_MASK;
			   if (((tdirp & NO_NET) == 0) && (tdirp != 0) &&
				     	(tdirp != (net->netnum | ROUTED_NET))) {
			      rteWU = 1;
			      if (layer < Num_layers - 2) {
			         tdirpp = OBSVAL(seg->x1 - 1, seg->y1, layer + 2)
						& ROUTED_NET_MASK;
			         if (tdirp == tdirpp) viaWU = 1;
			      }
			   }
			   if (rteWL && (tdir == tdirp)) viaWM = 1;
			}
		     }

		     // Check for via/route to east
		     if (seg->x1 < NumChannelsX - 1) {
			tdir = OBSVAL(seg->x1 + 1, seg->y1, layer)
					& ROUTED_NET_MASK;

			if (((tdir & NO_NET) == 0) && (tdir != 0) &&
				(tdir != (net->netnum | ROUTED_NET))) {
			   rteEL = 1;
			   if (layer > 0) {
			      tdirn = OBSVAL(seg->x1 + 1, seg->y1, layer - 1)
					& ROUTED_NET_MASK;
			      if (tdir == tdirn) viaEL = 1;
			   }
			}

			if (layer < Num_layers - 1) {
			   tdirp = OBSVAL(seg->x1 + 1, seg->y1, layer + 1)
					& ROUTED_NET_MASK;
			   if (((tdirp & NO_NET) == 0) && (tdirp != 0) &&
				     	(tdirp != (net->netnum | ROUTED_NET))) {
			      rteEU = 1;
			      if (layer < Num_layers - 2) {
			         tdirpp = OBSVAL(seg->x1 + 1, seg->y1, layer + 2)
						& ROUTED_NET_MASK;
			         if (tdirp == tdirpp) viaEU = 1;
			      }
			   }
			   if (rteEL && (tdir == tdirp)) viaEM = 1;
			}
		     }

		     // Check for via/route to south
		     if (seg->y1 > 0) {
			tdir = OBSVAL(seg->x1, seg->y1 - 1, layer)
					& ROUTED_NET_MASK;

			if (((tdir & NO_NET) == 0) && (tdir != 0) &&
				(tdir != (net->netnum | ROUTED_NET))) {
			   rteSL = 1;
			   if (layer > 0) {
			      tdirn = OBSVAL(seg->x1, seg->y1 - 1, layer - 1)
					& ROUTED_NET_MASK;
			      if (tdir == tdirn) viaSL = 1;
			   }
			}

			if (layer < Num_layers - 1) {
			   tdirp = OBSVAL(seg->x1, seg->y1 - 1, layer + 1)
					& ROUTED_NET_MASK;
			   if (((tdirp & NO_NET) == 0) && (tdirp != 0) &&
				     	(tdirp != (net->netnum | ROUTED_NET))) {
			      rteSU = 1;
			      if (layer < Num_layers - 2) {
			         tdirpp = OBSVAL(seg->x1, seg->y1 - 1, layer + 2)
						& ROUTED_NET_MASK;
			         if (tdirp == tdirpp) viaSU = 1;
			      }
			   }
			   if (rteSL && (tdir == tdirp)) viaSM = 1;
			}
		     }

		     // Check for via/route to north
		     if (seg->y1 < NumChannelsY - 1) {
			tdir = OBSVAL(seg->x1, seg->y1 + 1, layer)
					& ROUTED_NET_MASK;

			if (((tdir & NO_NET) == 0) && (tdir != 0) &&
				(tdir != (net->netnum | ROUTED_NET))) {
			   rteNL = 1;
			   if (layer > 0) {
			      tdirn = OBSVAL(seg->x1, seg->y1 + 1, layer - 1)
					& ROUTED_NET_MASK;
			      if (tdir == tdirn) viaNL = 1;
			   }
			}

			if (layer < Num_layers - 1) {
			   tdirp = OBSVAL(seg->x1, seg->y1 + 1, layer + 1)
					& ROUTED_NET_MASK;
			   if (((tdirp & NO_NET) == 0) && (tdirp != 0) &&
				     	(tdirp != (net->netnum | ROUTED_NET))) {
			      rteNU = 1;
			      if (layer < Num_layers - 2) {
			         tdirpp = OBSVAL(seg->x1, seg->y1 + 1, layer + 2)
						& ROUTED_NET_MASK;
			         if (tdirp == tdirpp) viaNU = 1;
			      }
			   }
			   if (rteNL && (tdir == tdirp)) viaNM = 1;
			}
		     }

		     // Check for any tap offset on the neighboring via,
		     // which needs to be accounted for in the calculations.
		     offsetx = offsety = 0.0;
		     if (viaEL)
			lnode = (layer < Pinlayers && layer > 0) ?
					NODEIPTR(seg->x1 + 1, seg->y1, layer - 1) : NULL;
		     else if (viaWL)
			lnode = (layer < Pinlayers && layer > 0) ?
					NODEIPTR(seg->x1 - 1, seg->y1, layer - 1) : NULL;
		     else if (viaEM)
			lnode = (layer < Pinlayers) ?
					NODEIPTR(seg->x1 + 1, seg->y1, layer) : NULL;
		     else if (viaWM)
			lnode = (layer < Pinlayers) ?
					NODEIPTR(seg->x1 - 1, seg->y1, layer) : NULL;
		     else if (viaEU)
			lnode = (layer < Pinlayers - 1) ?
					NODEIPTR(seg->x1 + 1, seg->y1, layer + 1) : NULL;
		     else if (viaWU)
			lnode = (layer < Pinlayers - 1) ?
					NODEIPTR(seg->x1 - 1, seg->y1, layer + 1) : NULL;

		     if (lnode && (lnode->flags & NI_OFFSET_EW)) {
			offsetx = lnode->offset;
			// offsetx defined as positive in the direction away from
			// the via under consideration.
			if (viaWL || viaWM || viaWU) offsetx = -offsetx;
		     }

		     if (viaNL)
			lnode = (layer < Pinlayers && layer > 0) ?
					NODEIPTR(seg->x1, seg->y1 + 1, layer - 1) : NULL;
		     else if (viaSL)
			lnode = (layer < Pinlayers && layer > 0) ?
					NODEIPTR(seg->x1, seg->y1 - 1, layer - 1) : NULL;
		     else if (viaNM)
			lnode = (layer < Pinlayers) ?
					NODEIPTR(seg->x1, seg->y1 + 1, layer) : NULL;
		     else if (viaSM)
			lnode = (layer < Pinlayers) ?
					NODEIPTR(seg->x1, seg->y1 - 1, layer) : NULL;
		     else if (viaNU)
			lnode = (layer < Pinlayers - 1) ?
					NODEIPTR(seg->x1, seg->y1 + 1, layer + 1) : NULL;
		     else if (viaSU)
			lnode = (layer < Pinlayers - 1) ?
					NODEIPTR(seg->x1, seg->y1 - 1, layer + 1) : NULL;

		     if (lnode && (lnode->flags & NI_OFFSET_NS)) {
			offsety = lnode->offset;
			// offsety defined as positive in the direction away from
			// the via under consideration.
			if (viaSL || viaSM || viaSU) offsety = -offsety;
		     }	

		     // Actions needed only if a via is on the long side and
		     // has a spacing violation.

		     viaoffx = viaoffy = 0.0;
		     altwx = altwy = 0.0;
		     if (ob == 1) {  /* bottom (layer) route is horizontal */
			if (viaEM || viaWM) {
			    /* Assume default rotations and calculate spacing */
			    /* Only take action if there is a spacing violation */

			    w1 = LefGetXYViaWidth(layer, layer, 0, 0);
			    w0 = w1;
		            dc = LefGetRouteSpacing(layer) + w1;
			    if (dc > PitchX + EPS + offsetx) {

				/* via on checkerboard may be rotated */
				if (checkersign && ((flags & NI_NO_VIAY) == 0)) {
				    /* Check spacing violation to routes N and S */
				    /* If via being checked is rotated */
				    if (rteNL || rteSL)
					dc = LefGetRouteSpacing(layer) +
						(LefGetXYViaWidth(layer, layer, 1, 2) +
						LefGetRouteWidth(layer)) / 2;
				    if ((!(rteNL || rteSL))
						|| (dc <= PitchY + EPS)) {
					/* Okay to rotate the via bottom */
					w0 = LefGetXYViaWidth(layer, layer, 0, 2);
					s = (ot == 1) ? ViaYX[layer] : ViaYY[layer];
				    }
				    else if (rteNL || rteSL)
					altwx = w1 -
						LefGetXYViaWidth(layer, layer, 0, 2);

				    /* Measure spacing violation to via */
		        	    dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
				    if (dc > PitchX + EPS + offsetx) {
					/* Calculate offset */
					if (viaEM)
					    viaoffx = PitchX + 2 * offsetx - dc;
					else
					    viaoffx = dc - PitchX - 2 * offsetx;
				    }
				}
				else {
				    /* Measure spacing violation to rotated via */
				    w0 = LefGetXYViaWidth(layer, layer, 0, 2);
		        	    dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
				    if (dc > PitchX + EPS + offsetx) {
					/* Calculate offset */
					if (viaEM)
					    viaoffx = PitchX + 2 * offsetx - dc;
					else
					    viaoffx = dc - PitchX - 2 * offsetx;
				    }
				}
			    }
			}
			else if (viaEL || viaWL) {
			    /* Assume default rotations and calculate spacing */
			    /* Only take action if there is a spacing violation */

			    w0 = LefGetXYViaWidth(layer - 1, layer, 0, 0);
			    w1 = LefGetXYViaWidth(layer, layer, 0, 0);
		            dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
			    if (dc > PitchX + EPS + offsetx) {

				/* via on checkerboard may be rotated */
				if (checkersign && ((flags & NI_NO_VIAY) == 0)) {
				    /* Check spacing violation to routes N and S */
				    /* If via being checked is rotated */
				    if (rteNL || rteSL)
					dc = LefGetRouteSpacing(layer) +
						(LefGetXYViaWidth(layer, layer, 1, 2) +
						LefGetRouteWidth(layer)) / 2;
				    if ((!(rteNL || rteSL))
						|| (dc <= PitchY + EPS)) {
					/* Okay to rotate the via bottom */
					w1 = LefGetXYViaWidth(layer, layer, 0, 2);
					s = (ot == 1) ? ViaYX[layer] : ViaYY[layer];
				    }
				    else if (rteNL || rteSL)
					altwx = w1 -
						LefGetXYViaWidth(layer, layer, 0, 2);

				    /* Measure spacing violation to via */
		        	    dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
				    if (dc > PitchX + EPS + offsetx) {
					/* Calculate offset */
					if (viaEL)
					    viaoffx = PitchX + 2 * offsetx - dc;
					else
					    viaoffx = dc - PitchX - 2 * offsetx;
				    }
				}
				else {
				    /* Measure spacing violation to rotated via */
				    w0 = LefGetXYViaWidth(layer - 1, layer, 0, 1);
		        	    dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
				    if (dc > PitchX + EPS + offsetx) {
					/* Calculate offset */
					if (viaEL)
					    viaoffx = PitchX + 2 * offsetx - dc;
					else
					    viaoffx = dc - PitchX - 2 * offsetx;
				    }
				}
			    }
			}
		     } else {	     /* bottom route vertical */
			if (viaNM || viaSM) {
			    /* Assume default rotations and calculate spacing */
			    /* Only take action if there is a spacing violation */

			    w1 = LefGetXYViaWidth(layer, layer, 1, 3);
			    w0 = w1;
		            dc = LefGetRouteSpacing(layer) + w1;
			    if (dc > PitchY + EPS + offsety) {

				/* via on checkerboard may be rotated */
				if (checkersign && ((flags & NI_NO_VIAX) == 0)) {
				    /* Check spacing violation to routes E and W */
				    /* If via being checked is rotated */
				    if (rteEL || rteWL)
					dc = LefGetRouteSpacing(layer) +
						(LefGetXYViaWidth(layer, layer, 0, 1) +
						LefGetRouteWidth(layer)) / 2;
				    if ((!(rteEL || rteWL))
						|| (dc <= PitchX + EPS)) {
					/* Okay to rotate the via bottom */
					w0 = LefGetXYViaWidth(layer, layer, 1, 1);
					s = (ot == 1) ? ViaXX[layer] : ViaXY[layer];
				    }
				    else if (rteEL || rteWL)
					altwy = w1 -
						LefGetXYViaWidth(layer, layer, 1, 1);

				    /* Measure spacing violation to via */
		        	    dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
				    if (dc > PitchY + EPS + offsety) {
					/* Calculate offset */
					if (viaNM)
					    viaoffy = PitchY + 2 * offsety - dc;
					else
					    viaoffy = dc - PitchY - 2 * offsety;
				    }
				}
				else {
				    /* Measure spacing violation to rotated via */
				    w0 = LefGetXYViaWidth(layer, layer, 1, 1);
		        	    dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
				    if (dc > PitchY + EPS + offsety) {
					/* Calculate offset */
					if (viaNM)
					    viaoffy = PitchY + 2 * offsety - dc;
					else
					    viaoffy = dc - PitchY - 2 * offsety;
				    }
				}
			    }
			}
			else if (viaNL || viaSL) {
			    /* Assume default rotations and calculate spacing */
			    /* Only take action if there is a spacing violation */

			    w0 = LefGetXYViaWidth(layer - 1, layer, 1, 3);
			    w1 = LefGetXYViaWidth(layer, layer, 1, 3);
		            dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
			    if (dc > PitchY + EPS + offsety) {

				/* via on checkerboard may be rotated */
				if (checkersign && ((flags & NI_NO_VIAX) == 0)) {
				    /* Check spacing violation to routes E and W */
				    /* If via being checked is rotated */
				    if (rteEL || rteWL)
					dc = LefGetRouteSpacing(layer) +
						(LefGetXYViaWidth(layer, layer, 0, 1) +
						LefGetRouteWidth(layer)) / 2;
				    if ((!(rteEL || rteWL))
						|| (dc <= PitchX + EPS)) {
					/* Okay to rotate the via bottom */
					w1 = LefGetXYViaWidth(layer, layer, 1, 1);
					s = (ot == 1) ? ViaXX[layer] : ViaXY[layer];
				    }
				    else if (rteEL || rteWL)
					altwy = w1 -
						LefGetXYViaWidth(layer, layer, 1, 1);

				    /* Measure spacing violation to via */
		        	    dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
				    if (dc > PitchY + EPS + offsety) {
					/* Calculate offset */
					if (viaNL)
					    viaoffy = PitchY + 2 * offsety - dc;
					else
					    viaoffy = dc - PitchY - 2 * offsety;
				    }
				}
				else {
				    /* Measure spacing violation to rotated via */
				    w0 = LefGetXYViaWidth(layer - 1, layer, 1, 2);
		        	    dc = LefGetRouteSpacing(layer) + (w1 + w0) / 2;
				    if (dc > PitchY + EPS + offsety) {
					/* Calculate offset */
					if (viaNL)
					    viaoffy = PitchY + 2 * offsety - dc;
					else
					    viaoffy = dc - PitchY - 2 * offsety;
				    }
				}
			    }
			}
		     }

		     if (ot == 1) {  /* top route horizontal */
			if (viaEU || viaWU) {
			    /* Assume default rotations and calculate spacing */
			    /* Only take action if there is a spacing violation */

			    w0 = LefGetXYViaWidth(layer + 1, layer + 1, 0, 0);
			    w1 = LefGetXYViaWidth(layer, layer + 1, 0, 0);
		            dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
			    if (dc > PitchX + EPS + offsetx) {

				/* via on checkerboard may be rotated */
				if (checkersign) {
				    /* Check spacing violation to routes N and S */
				    /* If via being checked is rotated */
				    if (rteNU || rteSU)
					dc = LefGetRouteSpacing(layer + 1) +
						(LefGetXYViaWidth(layer, layer + 1, 1, 1)
						+ LefGetRouteWidth(layer + 1)) / 2;
				    if ((!(rteNU || rteSU))
						|| (dc <= PitchY + EPS)) {
					/* Okay to rotate the via top */
					w1 = LefGetXYViaWidth(layer, layer + 1, 0, 1);
					s = (s == ViaYX[layer]) ? ViaYY[layer] :
							ViaXY[layer];
				    }
				    else if (rteNU || rteSU)
					altwx = w1 -
						LefGetXYViaWidth(layer, layer + 1, 0, 1);

				    /* Measure spacing violation to via */
		        	    dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
				    if (dc > PitchX + EPS + offsetx) {
					/* Calculate offset */
					if (viaEU)
					    viaoffx = PitchX + 2 * offsetx - dc;
					else
					    viaoffx = dc - PitchX - 2 * offsetx;
				    }
				}
				else {
				    /* Measure spacing violation to rotated via */
				    w0 = LefGetXYViaWidth(layer + 1, layer + 1, 0, 2);
		        	    dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
				    if (dc > PitchX + EPS + offsetx) {
					/* Calculate offset */
					if (viaEU)
					    viaoffx = PitchX + 2 * offsetx - dc;
					else
					    viaoffx = dc - PitchX - 2 * offsetx;
				    }
				}
			    }
			}
			else if (viaEM || viaWM) {
			    /* Assume default rotations and calculate spacing */
			    /* Only take action if there is a spacing violation */

			    w1 = LefGetXYViaWidth(layer, layer + 1, 0, 0);
			    w0 = w1;
		            dc = LefGetRouteSpacing(layer + 1) + w1;
			    if (dc > PitchX + EPS + offsetx) {

				/* via on checkerboard may be rotated */
				if (checkersign) {
				    /* Check spacing violation to routes N and S */
				    /* If via being checked is rotated */
				    if (rteNU || rteSU)
					dc = LefGetRouteSpacing(layer) +
						(LefGetXYViaWidth(layer, layer + 1, 1, 1)
						+ LefGetRouteWidth(layer)) / 2;
				    if ((!(rteNU || rteSU)) || (dc <= PitchY + EPS)) {
					/* Okay to rotate the via top */
					w0 = LefGetXYViaWidth(layer, layer + 1, 0, 1);
					s = (s == ViaYX[layer]) ? ViaYY[layer] :
							ViaXY[layer];
				    }
				    else if (rteNU || rteSU)
					altwx = w1 -
						LefGetXYViaWidth(layer, layer + 1, 0, 1);

				    /* Measure spacing violation to via */
		        	    dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
				    if (dc > PitchX + EPS + offsetx) {
					/* Calculate offset */
					if (viaEM)
					    viaoffx = PitchX + 2 * offsetx - dc;
					else
					    viaoffx = dc - PitchX - 2 * offsetx;
				    }
				}
				else {
				    /* Measure spacing violation to rotated via */
				    w0 = LefGetXYViaWidth(layer, layer + 1, 0, 1);
		        	    dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
				    if (dc > PitchX + EPS + offsetx) {
					/* Calculate offset */
					if (viaEM)
					    viaoffx = PitchX + 2 * offsetx - dc;
					else
					    viaoffx = dc - PitchX - 2 * offsetx;
				    }
				}
			    }
			}
		     } else {	     /* top route vertical */
			if (viaNU || viaSU) {
			    /* Assume default rotations and calculate spacing */
			    /* Only take action if there is a spacing violation */

			    w0 = LefGetXYViaWidth(layer + 1, layer + 1, 1, 3);
			    w1 = LefGetXYViaWidth(layer, layer + 1, 1, 3);
		            dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
			    if (dc > PitchY + EPS + offsety) {

				/* via on checkerboard may be rotated */
				if (checkersign) {
				    /* Check spacing violation to routes E and W */
				    /* If via being checked is rotated */
				    if (rteEU || rteWU)
					dc = LefGetRouteSpacing(layer + 1) +
						(LefGetXYViaWidth(layer, layer + 1, 0, 2)
						+ LefGetRouteWidth(layer + 1)) / 2;
				    if ((!(rteEU || rteWU))
						|| (dc <= PitchX + EPS)) {
					/* Okay to rotate the via top */
					w1 = LefGetXYViaWidth(layer, layer + 1, 1, 2);
					s = (s == ViaYY[layer]) ? ViaYX[layer] :
							ViaXX[layer];
				    }
				    else if (rteEU || rteWU)
					altwy = w1 -
						LefGetXYViaWidth(layer, layer + 1, 1, 2);

				    /* Measure spacing violation to via */
		        	    dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
				    if (dc > PitchY + EPS + offsety) {
					/* Calculate offset */
					if (viaNU)
					    viaoffy = PitchY + 2 * offsety - dc;
					else
					    viaoffy = dc - PitchY - 2 * offsety;
				    }
				}
				else {
				    /* Measure spacing violation to rotated via */
				    w0 = LefGetXYViaWidth(layer + 1, layer + 1, 1, 1);
		        	    dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
				    if (dc > PitchY + EPS + offsety) {
					/* Calculate offset */
					if (viaNU)
					    viaoffy = PitchY + 2 * offsety - dc;
					else
					    viaoffy = dc - PitchY - 2 * offsety;
				    }
				}
			    }
			}
			else if (viaNM || viaSM) {
			    /* Assume default rotations and calculate spacing */
			    /* Only take action if there is a spacing violation */

			    w1 = LefGetXYViaWidth(layer, layer + 1, 1, 3);
			    w0 = w1;
		            dc = LefGetRouteSpacing(layer + 1) + w1;
			    if (dc > PitchY + EPS + offsety) {

				/* via on checkerboard may be rotated */
				if (checkersign) {
				    /* Check spacing violation to routes E and W */
				    /* If via being checked is rotated */
				    if (rteEU || rteWU)
					dc = LefGetRouteSpacing(layer + 1) +
						(LefGetXYViaWidth(layer, layer + 1, 0, 2)
						+ LefGetRouteWidth(layer + 1)) / 2;
				    if ((!(rteEU || rteWU))
						|| (dc <= PitchX + EPS)) {
					/* Okay to rotate the via top */
					w1 = LefGetXYViaWidth(layer, layer + 1, 1, 2);
					s = (s == ViaYY[layer]) ? ViaYX[layer] :
							ViaXX[layer];
				    }
				    else if (rteEU || rteWU)
					altwy = w1 -
						LefGetXYViaWidth(layer, layer + 1, 1, 2);

				    /* Measure spacing violation to via */
		        	    dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
				    if (dc > PitchY + EPS + offsety) {
					/* Calculate offset */
					if (viaNM)
					    viaoffy = PitchY + 2 * offsety - dc;
					else
					    viaoffy = dc - PitchY - 2 * offsety;
				    }
				}
				else {
				    /* Measure spacing violation to rotated via */
				    w0 = LefGetXYViaWidth(layer, layer + 1, 1, 2);
		        	    dc = LefGetRouteSpacing(layer + 1) + (w1 + w0) / 2;
				    if (dc > PitchY + EPS + offsety) {
					/* Calculate offset */
					if (viaNM)
					    viaoffy = PitchY + 2 * offsety - dc;
					else
					    viaoffy = dc - PitchY - 2 * offsety;
				    }
				}
			    }
			}
		     }

		     /* If a via is constrained by routes on the side	*/
		     /* and cannot be rotated, then the calculations on	*/
		     /* the via in the next track will not take that	*/
		     /* into consideration, and not leave enough space.	*/
		     /* So this via must offset	more to make up the	*/
		     /* difference.					*/

		     if (altwx > 0.0) {
			if (viaoffx < 0) altwx = -altwx;
			viaoffx += altwx;
		     }
		     if (altwy > 0.0) {
			if (viaoffy < 0) altwy = -altwy;
			viaoffy += altwy;
		     }

		     /* When offset, each via moves by half the distance. */
		     viaoffx /= 2;
		     viaoffy /= 2;

		     vx = (int)((REPS(viaoffx)) * oscale);
		     vy = (int)((REPS(viaoffy)) * oscale);

		     /* If via is offset in the direction of the last	*/
		     /* route segment, then move the last route segment	*/
		     /* position to the via center.			*/
		     if ((path_delayed.active == 1) && (vy != 0) &&
				(path_delayed.orient == 0))
			path_delayed.y = y + vy;

		     else if ((path_delayed.active == 1) && (vx != 0) &&
				(path_delayed.orient == 1))
			path_delayed.x = x + vx;

		     pathvia(Cmd, layer, x + vx, y + vy, lastx, lasty, s, invscale);

		     lastx = x + vx;
		     lasty = y + vy;
		     lastlay = -1;
		  }
		  break;
	       default:
		  break;
	    }

	    // Break here on last segment so that seg and lastseg are valid
	    // in the following section of code.

	    if (seg->next == NULL) break;
	    prevseg = lastseg;
	    lastseg = seg;
	 }

	 // For stub routes, reset the path between terminals, since
	 // the stubs are not connected.
	 if (special == (u_char)1 && Pathon != -1) Pathon = 0;

	 // Check last position for terminal offsets
	 if (seg && ((seg != saveseg) || (seg->segtype & ST_WIRE))) {
	     cancel = FALSE;
	     layer = seg->layer;
	     lnode = (layer < Pinlayers) ? NODEIPTR(seg->x2, seg->y2, layer) : NULL;

	     // Look for stub routes and offset taps
	     dir2 = OBSVAL(seg->x2, seg->y2, layer) & (STUBROUTE | OFFSET_TAP);

	     if ((dir2 & OFFSET_TAP) && (seg->segtype & ST_VIA) && prevseg) {

	        // Additional handling for offset taps.  When a tap position
	        // is a via and is offset in the direction of the last
	        // route segment, then a DRC violation can be created if
	        // (1) the via is wider than the route width, and (2) the
	        // adjacent track position is another via or a bend in the
	        // route, and (3) the tap offset is large enough to create
	        // a spacing violation between the via and the adjacent via
	        // or perpendicular route.  If these three conditions are
	        // satisfied, then generate a stub route the width of the
	        // via and one track pitch in length back toward the last
	        // track position.

 	        // Problems only arise when the via width is larger than
	        // the width of the metal route leaving the via.
 
	        offset = lnode->offset;
	        if (LefGetViaWidth(seg->layer, lastseg->layer, 1 - horizontal) >
			LefGetRouteWidth(lastseg->layer)) {

		   // Problems only arise when the last segment is exactly
		   // one track long.

		   if ((ABSDIFF(lastseg->x2, lastseg->x1) == 1) ||
			(ABSDIFF(lastseg->y2, lastseg->y1) == 1)) {

		      if (prevseg->segtype & ST_VIA) {

		 	 dc = Xlowerbound + (double)seg->x1 * PitchX;
			 x = (int)((REPS(dc)) * oscale);
			 dc = Ylowerbound + (double)seg->y1 * PitchY;
			 y = (int)((REPS(dc)) * oscale);

			 dc = Xlowerbound + (double)prevseg->x1 * PitchX;
			 x2 = (int)((REPS(dc)) * oscale);
			 dc = Ylowerbound + (double)prevseg->y1 * PitchY;
			 y2 = (int)((REPS(dc)) * oscale);

			 // Setup is (via, 1 track route, via with offset)

			 if (prevseg->x1 != seg->x1) {
			    if ((PitchX -
				0.5 * LefGetViaWidth(seg->layer, lastseg->layer, 1) -
				0.5 * LefGetViaWidth(prevseg->layer, lastseg->layer, 1) -
				(prevseg->x1 - seg->x1) * offset)
				< LefGetRouteSpacing(lastseg->layer)) {
			       if (special == (u_char)0) {
				  rt->flags |= RT_STUB;
				  net->flags |= NET_STUB;
			       }
			       else {
				  pathstart(Cmd, lastseg->layer, x, y,
					(u_char)1, oscale, invscale, 1, lnode);
				  pathto(Cmd, x2, y2, 1, x, y, invscale, 0);
		      		  lastx = x2;
				  lasty = y2;
			       }
			    }
			 }
			 else if (prevseg->y1 != seg->y1) {
			    if ((PitchY -
				0.5 * LefGetViaWidth(seg->layer, lastseg->layer, 0) -
				0.5 * LefGetViaWidth(prevseg->layer, lastseg->layer, 0)
				- (prevseg->y1 - seg->y1) * offset)
				< LefGetRouteSpacing(lastseg->layer)) {
			       if (special == (u_char)0) {
				 rt->flags |= RT_STUB;
				 net->flags |= NET_STUB;
			       }
			       else {
				  pathstart(Cmd, lastseg->layer, x, y,
					(u_char)1, oscale, invscale, 0, lnode);
				  pathto(Cmd, x2, y2, 0, x, y, invscale, 0);
		      		  lastx = x2;
				  lasty = y2;
			       }
			    }
			 }
		      }
		      else {	// Metal route bends at next track
			 if (prevseg->x1 != seg->x1) {
			    if ((PitchX -
				0.5 * LefGetViaWidth(seg->layer, lastseg->layer, 1) -
				0.5 * LefGetRouteWidth(prevseg->layer) -
				(prevseg->x1 - seg->x1) * offset)
				< LefGetRouteSpacing(lastseg->layer)) {
			       if (special == (u_char)0) {
				 rt->flags |= RT_STUB;
				 net->flags |= NET_STUB;
			       }
			       else {
				  pathstart(Cmd, lastseg->layer, x, y,
					(u_char)1, oscale, invscale, 1, lnode);
				  pathto(Cmd, x2, y2, 1, x, y, invscale, 0);
		      		  lastx = x2;
				  lasty = y2;
			       }
			    }
			 }
			 else if (prevseg->y1 != seg->y1) {
			    if ((PitchY -
				0.5 * LefGetViaWidth(seg->layer, lastseg->layer, 0) -
				0.5 * LefGetRouteWidth(prevseg->layer) -
				(prevseg->y1 - seg->y1) * offset)
				< LefGetRouteSpacing(lastseg->layer)) {
			       if (special == (u_char)0) {
				  rt->flags |= RT_STUB;
				  net->flags |= NET_STUB;
			       }
			       else {
				  pathstart(Cmd, lastseg->layer, x, y,
					(u_char)1, oscale, invscale, 0, lnode);
				  pathto(Cmd, x2, y2, 0, x, y, invscale, 0);
		      		  lastx = x2;
				  lasty = y2;
			       }
			    }
			 }
		      }
		   }
	        }
	     }

	     // For stub routes, reset the path between terminals, since
	     // the stubs are not connected.
	     if (special == (u_char)1 && Pathon != -1) Pathon = 0;

	     // Handling of stub routes
	     if (dir2 & STUBROUTE) {
	        stub = lnode->stub;
		if ((special == (u_char)0) && (Verbose > 2))
		   Fprintf(stdout, "Stub route distance %g to terminal"
				" at %d %d (%d)\n",
				stub, seg->x2, seg->y2, layer);

		dc = Xlowerbound + (double)seg->x2 * PitchX;
		if (lnode->flags & NI_OFFSET_EW)
		   dc += offset;
		x = (int)((REPS(dc)) * oscale);
		if (lnode->flags & NI_STUB_EW)
		   dc += stub;
		x2 = (int)((REPS(dc)) * oscale);
		dc = Ylowerbound + (double)seg->y2 * PitchY;
		if (lnode->flags & NI_OFFSET_NS)
		   dc += offset;
		y = (int)((REPS(dc)) * oscale);
		if (lnode->flags & NI_STUB_NS)
		   dc += stub;
		y2 = (int)((REPS(dc)) * oscale);
		if (lnode->flags & NI_STUB_EW) {
		   horizontal = TRUE;

		   // If the gridpoint ahead of the stub has a route
		   // on the same net, and the stub is long enough
		   // to come within a DRC spacing distance of the
		   // other route, then lengthen it to close up the
		   // distance and resolve the error.

		   if ((x < x2) && (seg->x2 < (NumChannelsX - 1))) {
		      tdir = OBSVAL(seg->x2 + 1, seg->y2, layer);
		      if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (stub + LefGetRouteKeepout(layer) >= PitchX) {
		      	    dc = Xlowerbound + (double)(seg->x2 + 1) * PitchX;
		      	    x2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }
		   else if ((x > x2) && (seg->x2 > 0)) {
		      tdir = OBSVAL(seg->x2 - 1, seg->y2, layer);
		      if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (-stub + LefGetRouteKeepout(layer) >= PitchX) {
		      	    dc = Xlowerbound + (double)(seg->x2 - 1) * PitchX;
		      	    x2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }

		   dc = oscale * 0.5 * LefGetRouteWidth(layer);
		   if (special == (u_char)0) {
		      // Regular nets include 1/2 route width at
		      // the ends, so subtract from the stub terminus
		      if (x < x2) {
			 x2 -= dc;
			 if (x >= x2) cancel = TRUE;
		      }
		      else {
			 x2 += dc;
			 if (x <= x2) cancel = TRUE;
		      }
		   }
		   else {
		      // Special nets don't include 1/2 route width
		      // at the ends, so add to the route at the grid
		      if (x < x2)
			 x -= dc;
		      else
			 x += dc;

		      // Routes that extend for more than one track
		      // without a bend do not need a wide stub
		      if (seg->x1 != seg->x2) cancel = TRUE;
		   }
		}
		else {  /* lnode->flags & NI_STUB_EW implied */
		   horizontal = FALSE;

		   // If the gridpoint ahead of the stub has a route
		   // on the same net, and the stub is long enough
		   // to come within a DRC spacing distance of the
		   // other route, then lengthen it to close up the
		   // distance and resolve the error.

		   if ((y < y2) && (seg->y2 < (NumChannelsY - 1))) {
		      tdir = OBSVAL(seg->x2, seg->y2 + 1, layer);
		      if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (stub + LefGetRouteKeepout(layer) >= PitchY) {
		      	    dc = Ylowerbound + (double)(seg->y2 + 1) * PitchY;
		      	    y2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }
		   else if ((y > y2) && (seg->y2 > 0)) {
		      tdir = OBSVAL(seg->x2, seg->y2 - 1, layer);
		      if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (-stub + LefGetRouteKeepout(layer) >= PitchY) {
		      	    dc = Ylowerbound + (double)(seg->y2 - 1) * PitchY;
		      	    y2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }

		   dc = oscale * 0.5 * LefGetRouteWidth(layer);
		   if (special == (u_char)0) {
		      // Regular nets include 1/2 route width at
		      // the ends, so subtract from the stub terminus
		      if (y < y2) {
			 y2 -= dc;
			 if (y >= y2) cancel = TRUE;
		      }
		      else {
			 y2 += dc;
			 if (y <= y2) cancel = TRUE;
		      }
		   }
		   else {
		      // Special nets don't include 1/2 route width
		      // at the ends, so add to the route at the grid
		      if (y < y2)
			 y -= dc;
		      else
			 y += dc;

		      // Routes that extend for more than one track
		      // without a bend do not need a wide stub
		      if (seg->y1 != seg->y2) cancel = TRUE;
		   }
		}
		if (cancel == FALSE) {
	           net->flags |= NET_STUB;
	           rt->flags |= RT_STUB;
		   if (Pathon != 1) {
		      pathstart(Cmd, layer, x, y, special, oscale, invscale,
				horizontal, lnode);
		      lastx = x;
		      lasty = y;
		   }
		   pathto(Cmd, x2, y2, horizontal, lastx, lasty, invscale, 0);
		   lastx = x2;
		   lasty = y2;
		}
	    }
	 }
	 if (Pathon != -1) Pathon = 0;

      } // if (rt->segments && !(rt->flags & RT_OUTPUT))
   }
}

/*--------------------------------------------------------------*/
/* emit_routes - DEF file output from the list of routes	*/
/*								*/
/*  Reads the <project>.def file and rewrites file		*/
/*  <project>_route.def, where each net definition has the	*/
/*  physical route appended.					*/
/*								*/
/*   ARGS: filename to list to					*/
/*   RETURNS: nothing						*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Mon Aug 11 2003		*/
/*--------------------------------------------------------------*/

static void emit_routes(char *filename, double oscale, int iscale)
{
    FILE *Cmd;
    int i, j, numnets, numvias, stubroutes;
    char line[MAX_LINE_LEN + 1], *lptr = NULL;
    char netname[MAX_NAME_LEN];
    NET net = NULL;
    ROUTE rt;
    FILE *fdef;
    u_char errcond = FALSE;
    u_char need_cleanup = FALSE;
    u_char purge_routed = FALSE;
    u_char skip_net = FALSE;

    fdef = fopen(DEFfilename, "r");
    if ((fdef == NULL) && (DEFfilename != NULL)) {
	if (strchr(DEFfilename, '.') == NULL) {
	    char *extfilename = malloc(strlen(DEFfilename) + 5);
	    sprintf(extfilename, "%s.def", DEFfilename);
	    fdef = fopen(extfilename, "r");
	    free(extfilename);
	}
    }
    if (fdef == NULL) {
	Fprintf(stderr, "emit_routes(): Cannot open DEF file for reading.\n");
	return;
    } 

    if (!strcmp(filename, "stdout")) {
	Cmd = stdout;
    }
    else {
	char *dotptr;

	if (filename == DEFfilename) {
	    char *newDEFfile = (char *)malloc(strlen(filename) + 11);
	    strcpy(newDEFfile, filename);
	    dotptr = strrchr(newDEFfile, '.');
	    if (dotptr)
		strcpy(dotptr, "_route.def");
	    else
		strcat(newDEFfile, "_route.def");
	    
	    Cmd = fopen(newDEFfile, "w");
	    free(newDEFfile);
	}
	else {
	    dotptr = strrchr(filename, '.');
	    if (dotptr)
	       Cmd = fopen(filename, "w");
	    else {
	       char *newDEFfile = (char *)malloc(strlen(filename) + 11);
	       strcpy(newDEFfile, filename);
	       strcat(newDEFfile, ".def");
	       Cmd = fopen(newDEFfile, "w");
	       free(newDEFfile);
	    }
	}
    }
    if (!Cmd) {
	Fprintf(stderr, "emit_routes():  Couldn't open output (routed) DEF file.\n");
	return;
    }

    // Copy DEF file up to NETS line
    numnets = 0;
    numvias = 0;
    while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
       lptr = line;
       while (isspace(*lptr)) lptr++;
       if (!strncmp(lptr, "NETS", 4)) {
	   sscanf(lptr + 4, "%d", &numnets);
	   break;
       }
       if (!strncmp(lptr, "VIAS", 4)) {
	   sscanf(lptr + 4, "%d", &numvias);
	   LefWriteGeneratedVias(Cmd, (double)(oscale / (double)iscale), numvias);
	   continue;	/* VIAS line already written;  do not output. */
       }
       if (!strncmp(lptr, "PINS", 4) && (numvias == 0)) {
	   /* Check if there are any generated vias, and write them	*/
	   /* prior to the PINS section.				*/
	   LefWriteGeneratedVias(Cmd, (double)(oscale / (double)iscale), 0);
       }
       fputs(line, Cmd);
    }
    fputs(line, Cmd);	// Write the NETS line

    if ((numnets + numSpecial) != Numnets) {
      	Flush(stdout);
	Fprintf(stderr, "emit_routes():  DEF file has %d nets and %d specialnets.\n",
			numnets, numSpecial);
	Fprintf(stderr, "but qrouter wants to write %d nets and specialnets.\n",
			Numnets);
    }

    for (i = 0; i < numnets; i++) {
       if (errcond == TRUE) break;
       while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
	  if ((lptr = strchr(line, ';')) != NULL) {
	     *lptr = '\n';
	     *(lptr + 1) = '\0';
	     break;
	  }
	  else {
             lptr = line;
             while (isspace(*lptr)) lptr++;
	     if (*lptr == '-') {
		lptr++;
                while (isspace(*lptr)) lptr++;
	        sscanf(lptr, "%s", netname);
		fputs(line, Cmd);
	     }
	     else if (*lptr == '+') {
		lptr++;
                while (isspace(*lptr)) lptr++;
		if (!strncmp(lptr, "ROUTED", 6)) {
		   // This net is being handled by qrouter, so remove
		   // the original routing information
		   while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
		      if ((lptr = strchr(line, ';')) != NULL) {
			 // *lptr = '\n';
			 // *(lptr + 1) = '\0';
			 *line = '\0';
			 break;
		      }
		   }
		   break;
		}
		else
		   fputs(line, Cmd);
	     }
	     else if (!strncmp(lptr, "END", 3)) {	// This should not happen
		fputs(line, Cmd);
		errcond = TRUE;
		break;
	     }
	     else
		fputs(line, Cmd);
	  }
       }

       /* Find this net */

       net = DefFindNet(netname);
       if (!net || (net->flags & NET_IGNORED)) {
	  if (!net)
	     Fprintf(stderr, "emit_routes():  Net %s cannot be found.\n",
			netname);

	  /* Dump rest of net and continue */
	  *(lptr) = ';';
	  *(lptr + 1) = '\n';
	  *(lptr + 2) = '\0';
	  fputs(line, Cmd);
	  continue;
       }
       else {
	  /* Add last net terminal, without the semicolon */
	  fputs(line, Cmd);

	  emit_routed_net(Cmd, net, (u_char)0, oscale, iscale);
	  fprintf(Cmd, ";\n");
       }
    }

    // Finish copying the rest of the NETS section
    if (errcond == FALSE) {
       while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
	  lptr = line;
	  while (isspace(*lptr)) lptr++;
	  fputs(line, Cmd);
	  if (!strncmp(lptr, "END", 3)) {
	     break;
	  }
       }
    }

    // Determine how many stub routes we will write to SPECIALNETS
    // Also reset the OUTPUT flag for each route needing a stubroute
    // to be written.

    stubroutes = 0;
    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	if (net->flags & NET_IGNORED) continue;
	if (net->flags & NET_STUB) {
	    stubroutes++;
	    for (rt = net->routes; rt; rt = rt->next)
		if (rt->flags & RT_STUB)
		    rt->flags &= ~RT_OUTPUT;
	}
    }

    // If there were stub routes, repeat them in SPECIALNETS at the
    // proper width.
    if (stubroutes > 0) {

        fprintf(Cmd, "\nSPECIALNETS %d ", stubroutes + numSpecial);
	for (i = 0; i < Numnets; i++) {
	     net = Nlnets[i];
	     if (net->flags & NET_IGNORED) continue;
	     emit_routed_net(Cmd, net, (u_char)1, oscale, iscale);
	}
	if (numSpecial == 0)
	    fprintf(Cmd, ";\nEND SPECIALNETS\n");
	else
	    fprintf(Cmd, ";\n");
    }    

    // Finish copying the rest of the file.  Ignore ROUTED specialnets if
    // the nets are known nets and not power or ground nets.  FIXED or
    // COVER nets are output verbatim.

    while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
       lptr = line;
       while (isspace(*lptr)) lptr++;
       if (!strncmp(lptr, "SPECIALNETS", 11)) {
	   if (stubroutes > 0) {
	       purge_routed = TRUE;
	       continue;	/* SPECIALNETS line already written;  do not output. */
	   }
       }
       if (!purge_routed)
	  fputs(line, Cmd);
       else {
          lptr = line;
          while (isspace(*lptr)) lptr++;
	  if (*lptr == '-') {
	     lptr++;
             while (isspace(*lptr)) lptr++;
	     sscanf(lptr, "%s", netname);

	     // Find this net
	     net = DefFindNet(netname);
	     if (!net || (net->flags & NET_IGNORED))
		skip_net = FALSE;
	     else if (net->netnum == VDD_NET || net->netnum == GND_NET)
		skip_net = FALSE;
	     else
		skip_net = TRUE;
	  }
	  if (!skip_net) fputs(line, Cmd);
	  else if ((lptr = strchr(line, ';')) != NULL) {
	      skip_net = FALSE;
	  }
       }
    }
    fclose(fdef);
    fclose(Cmd);

} /* emit_routes() */

/* end of output.c */
