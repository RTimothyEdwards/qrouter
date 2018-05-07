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

	 // Make sure FailedNets is cleaned up as we output the failed nets

 	 while (FailedNets) {
	    net = FailedNets->net;
	    Fprintf(stdout, " %s\n", net->netname);
	    nl = FailedNets->next;
	    free(FailedNets);
	    FailedNets = nl;
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
          double invscale, u_char horizontal)
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

	 wvia = LefGetViaWidth(layer, layer, horizontal);
	 if (layer > 0) { 
	    double wvia2;
	    wvia2 = LefGetViaWidth(layer - 1, layer, horizontal);
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
       double invscale)
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
	   pathto(cmd, lastx, y, FALSE, lastx, lasty, invscale);
	else
	   pathto(cmd, x, lasty, TRUE, lastx, lasty, invscale);
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
        int gridx, int gridy, double invscale)
{
    char *s;
    char checkersign = (gridx + gridy + layer) & 0x01;
    NODEINFO lnode;

    if ((ViaPattern == VIA_PATTERN_NONE) || (ViaY[layer] == NULL))
	s = ViaX[layer];
    else if (ViaPattern == VIA_PATTERN_NORMAL)
	s = (checkersign == 0) ?  ViaX[layer] : ViaY[layer];
    else
	s = (checkersign == 0) ?  ViaY[layer] : ViaX[layer];

    /* If the position is a node, then the via type may be switched if	*/
    /* there is a prohibition declared in the flags.			*/

    if (layer < Pinlayers) {
	if (((lnode = NODEIPTR(gridx, gridy, layer)) != NULL)
			&& (lnode->nodesav != NULL)) {
	    if ((lnode->flags & NI_NO_VIAX) && (s == ViaX[layer]))
		s = ViaY[layer];
	    if ((lnode->flags & NI_NO_VIAY) && (s == ViaY[layer]))
		s = ViaX[layer];
	}
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
	  pathto(cmd, x, lasty, TRUE, lastx, lasty, invscale);
       if (y != lasty)
	  pathto(cmd, x, y, FALSE, x, lasty, invscale);
    }
    fprintf(cmd, "%s ", s);
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
    static char *nodestr = NULL;

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    if (g->noderec[i] == node) {
		if (nodestr != NULL)
		   free(nodestr);

		nodestr = (char *)malloc(strlen(g->gatename)
			+ strlen(g->node[i]) + 2);
		if (!strcmp(g->node[i], "pin"))
		    sprintf(nodestr, "PIN/%s", g->gatename);
		else
		    sprintf(nodestr, "%s/%s", g->gatename, g->node[i]);
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
/* routine unless route_set_connections() is re-run on the	*/
/* modified routes.						*/
/*--------------------------------------------------------------*/

static void cleanup_net(NET net)
{
   SEG segf, segl, seg;
   ROUTE rt, rt2;
   NODEINFO lnode;
   int lf, ll, lf2, ll2;
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
   u_char cancel, segtype;
   double invscale = (double)(1.0 / (double)iscale); 

   /* If the STUB flag is set, then we need to write out the net name	*/
   /* in the SPECIALNETS section.					*/

   if ((special == (u_char)1) && (net->flags & NET_STUB)) {
      fprintf(Cmd, ";\n- %s\n", net->netname);
   }

   u_char viaCheckX[MAX_LAYERS];
   u_char viaCheckY[MAX_LAYERS];
   double viaOffsetX[MAX_LAYERS][3];
   double viaOffsetY[MAX_LAYERS][3];

   /* Compute via offsets, if needed for adjacent vias on different nets. */

   /* A well-designed standard cell set should not have DRC errors	*/
   /* between vias spaced on adjacent tracks.  But not every standard	*/
   /* cell set is well-designed. . .					*/

   /* Example of offset measurements:					*/
   /* viaOffsetX[layer][n]:  layer is the base layer of the via, n is	*/
   /* 0 for the via one layer below, 1 for the same via, and 2 for the	*/
   /* via one layer above.  Note that the n = 1 has interactions on two	*/
   /* different metal layers.  The maximum distance is used.		*/

   /* viaCheckX[1] is 0 if all of viaOffsetX[1][0-2] is zero.  This	*/
   /*	 allows a quick determination if a check for neighboring vias	*/
   /*    is required.							*/
   /* viaOffsetX[1][0] is the additional spacing above the grid	width	*/
   /*	 for via2-to-via1 (on metal2 only).				*/
   /* viaOffsetX[1][1] is the additional spacing above the grid	width	*/
   /*	 for via2-to-via2 (maximum for metal2 and metal3)		*/
   /* viaOffsetX[1][2] is the additional spacing above the grid	width	*/
   /*	 for via2-to-via3 (on metal3 only).				*/

   viaOffsetX[0][0] = 0;		// nothing below the 1st via
   viaOffsetY[0][0] = 0;
   viaOffsetX[Num_layers - 1][2] = 0;	// nothing above the last via
   viaOffsetY[Num_layers - 1][2] = 0;

   for (layer = 0; layer < Num_layers - 1; layer++) {
      double s1  = LefGetRouteSpacing(layer);
      double s2  = LefGetRouteSpacing(layer + 1);
      double p1x = PitchX[layer];
      double p2x = PitchX[layer + 1];
      double p1y = PitchY[layer];
      double p2y = PitchY[layer + 1];
      double w1x = LefGetViaWidth(layer, layer, 0);
      double w1y = LefGetViaWidth(layer, layer, 1);
      double w2x = LefGetViaWidth(layer, layer + 1, 0);
      double w2y = LefGetViaWidth(layer, layer + 1, 1);
    
      double w0x, w0y, w3x, w3y;

      viaCheckX[layer] = 0;
      viaCheckY[layer] = 0;

      if (layer > 0) {

	 /* Space from via to (via - 1) */

         w0x = LefGetViaWidth(layer - 1, layer, 0);
         w0y = LefGetViaWidth(layer - 1, layer, 1);

         dc = s1 + (w1x + w0x) / 2 - p1x;
         viaOffsetX[layer][0] = (dc > 0.0) ? dc : 0.0;

         dc = s1 + (w1y + w0y) / 2 - p1y;
         viaOffsetY[layer][0] = (dc > 0.0) ? dc : 0.0;
      }

      /* Space from via to via (check both lower and upper metal layers) */

      dc = s1 + w1x - p1x;
      viaOffsetX[layer][1] = (dc > 0.0) ? dc : 0.0;

      dc = s2 + w2x - p2x;
      if (dc < 0.0) dc = 0.0;
      if (dc > viaOffsetX[layer][1]) viaOffsetX[layer][1] = dc;

      dc = s1 + w1y - p1y;
      viaOffsetY[layer][1] = (dc > 0.0) ? dc : 0.0;

      dc = s2 + w2y - p2y;
      if (dc < 0.0) dc = 0.0;
      if (dc > viaOffsetY[layer][1]) viaOffsetY[layer][1] = dc;

      if (layer < Num_layers - 1) {

	 /* Space from via to (via + 1) */

         w3x = LefGetViaWidth(layer + 1, layer, 0);
         w3y = LefGetViaWidth(layer + 1, layer, 1);

         dc = s2 + (w2x + w3x) / 2 - p2x;
         viaOffsetX[layer][2] = (dc > 0.0) ? dc : 0.0;

         dc = s2 + (w2y + w3y) / 2 - p2y;
         viaOffsetY[layer][2] = (dc > 0.0) ? dc : 0.0;
      }

      if (viaOffsetX[layer][0] > 0 || viaOffsetX[layer][1] > 0 ||
		viaOffsetX[layer][2] > 0)
	 viaCheckX[layer] = 1;
      if (viaOffsetY[layer][0] > 0 || viaOffsetY[layer][1] > 0 ||
		viaOffsetY[layer][2] > 0)
	 viaCheckY[layer] = 1;
   }

   Pathon = -1;
   lastlay = -1;

   /* Insert routed net here */
   for (rt = net->routes; rt; rt = rt->next) {
      if (rt->segments && !(rt->flags & RT_OUTPUT)) {
	 horizontal = FALSE;
	 cancel = FALSE;

	 // Check first position for terminal offsets
	 seg = (SEG)rt->segments;
	 lastseg = saveseg = seg;
	 layer = seg->layer;
	 if (seg) {

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

	       dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
	       x = (int)((REPS(dc)) * oscale);
	       if (lnode->flags & NI_STUB_EW)
		  dc += stub;
	       x2 = (int)((REPS(dc)) * oscale);
	       dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
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

		  if ((x < x2) && (seg->x1 < (NumChannelsX[layer] - 1))) {
		     tdir = OBSVAL(seg->x1 + 1, seg->y1, layer);
		     if ((tdir & ROUTED_NET_MASK) ==
					(net->netnum | ROUTED_NET)) {
			if (stub + LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	   dc = Xlowerbound + (double)(seg->x1 + 1)
					* PitchX[layer];
		      	   x2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }
		  else if ((x > x2) && (seg->x1 > 0)) {
		     tdir = OBSVAL(seg->x1 - 1, seg->y1, layer);
		     if ((tdir & ROUTED_NET_MASK) ==
					(net->netnum | ROUTED_NET)) {
			if (-stub + LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	   dc = Xlowerbound + (double)(seg->x1 - 1)
					* PitchX[layer];
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

		  if ((y < y2) && (seg->y1 < (NumChannelsY[layer] - 1))) {
		     tdir = OBSVAL(seg->x1, seg->y1 + 1, layer);
		     if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			if (stub + LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	   dc = Ylowerbound + (double)(seg->y1 + 1)
					* PitchY[layer];
		      	   y2 = (int)((REPS(dc)) * oscale);
			}
		     }
		  }
		  else if ((y > y2) && (seg->y1 > 0)) {
		     tdir = OBSVAL(seg->x1, seg->y1 - 1, layer);
		     if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			if (-stub + LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	   dc = Ylowerbound + (double)(seg->y1 - 1)
					* PitchY[layer];
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
		  pathstart(Cmd, layer, x2, y2, special, oscale, invscale, horizontal);
		  pathto(Cmd, x, y, horizontal, x2, y2, invscale);
	       }
	       lastx = x;
	       lasty = y;
	       lastlay = layer;
	    }
	 }

	 prevseg = NULL;
	 lastseg = NULL;
	 for (seg = rt->segments; seg; seg = seg->next) {
	    layer = seg->layer;

	    // Check for offset terminals at either point

	    offset1 = 0.0;
	    offset2 = 0.0;
	    dir1 = 0;
	    dir2 = 0;

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

	    dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
	    if ((dir1 & OFFSET_TAP) && (lnode1->flags & NI_OFFSET_EW)) dc += offset1;
	    x = (int)((REPS(dc)) * oscale);
	    dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
	    if ((dir1 & OFFSET_TAP) && (lnode1->flags & NI_OFFSET_NS)) dc += offset1;
	    y = (int)((REPS(dc)) * oscale);
	    dc = Xlowerbound + (double)seg->x2 * PitchX[layer];
	    if ((dir2 & OFFSET_TAP) && (lnode2->flags & NI_OFFSET_EW)) dc += offset2;
	    x2 = (int)((REPS(dc)) * oscale);
	    dc = Ylowerbound + (double)seg->y2 * PitchY[layer];
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
				oscale, invscale, vertical);
			   pathto(Cmd, x, y, vertical, lastx, lasty, invscale);
			}
			else {
			   pathstart(Cmd, seg->layer, x, y, special, oscale,
				invscale, horizontal);
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
		     pathto(Cmd, x2, y2, horizontal, lastx, lasty, invscale);
		     lastx = x2;
		     lasty = y2;
		  }

		  // If a segment is 1 track long, there is a via on either
		  // end, and the needblock flag is set for the layer, then
		  // draw a stub route along the length of the track.

		  if (horizontal && needblock[seg->layer] & VIABLOCKX) {
		     if (ABSDIFF(seg->x2, seg->x1) == 1) {
			if ((lastseg && lastseg->segtype == ST_VIA) ||
			    (seg->next && seg->next->segtype == ST_VIA)) {
			   if (special == (u_char)0) {
			      net->flags |= NET_STUB;
			      rt->flags |= RT_STUB;
			   }
			   else {
			      if (Pathon != -1) Pathon = 0;
			      pathstart(Cmd, layer, x, y, special, oscale,
						invscale, horizontal);
			      pathto(Cmd, x2, y2, horizontal, x, y, invscale);
			      lastlay = layer;
			   }
			}
		     }
		  }
		  else if (!horizontal && needblock[seg->layer] & VIABLOCKY) {
		     if (ABSDIFF(seg->y2, seg->y1) == 1)  {
			if ((lastseg && lastseg->segtype == ST_VIA) ||
			    (seg->next && seg->next->segtype == ST_VIA)) {
			   if (special == (u_char)0) {
			      net->flags |= NET_STUB;
			      rt->flags |= RT_STUB;
			   }
			   else {
			      if (Pathon != -1) Pathon = 0;
			      pathstart(Cmd, layer, x, y, special, oscale,
						invscale, horizontal);
			      pathto(Cmd, x2, y2, horizontal, x, y, invscale);
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
		     int vx = 0;
		     int vy = 0;
		     u_int tdirpp, tdirp, tdirn;
		     u_char viaNL, viaNM, viaNU;
		     u_char viaSL, viaSM, viaSU;
		     u_char viaEL, viaEM, viaEU;
		     u_char viaWL, viaWM, viaWU;

		     if (lastseg == NULL) {
			// Make sure last position is valid
			lastx = x;
			lasty = y;
		     }

		     // Check for vias between adjacent but different nets
		     // that need position offsets to avoid a DRC spacing error

		     // viaCheckX[layer] indicates whether a check for
		     // vias is needed.  If so, record what vias are to east
		     // and west.

		     if (viaCheckX[layer] > 0) {

			viaEL = viaEM = viaEU = 0;
			viaWL = viaWM = viaWU = 0;

			// Check for via to west
			if (seg->x1 > 0) {
			   tdir = OBSVAL(seg->x1 - 1, seg->y1, layer)
					& ROUTED_NET_MASK;

			   if (((tdir & NO_NET) == 0) && (tdir != 0) &&
				(tdir != (net->netnum | ROUTED_NET))) {

			      if (layer < Num_layers - 1) {
			         tdirp = OBSVAL(seg->x1 - 1, seg->y1, layer + 1)
					& ROUTED_NET_MASK;
			         if (((tdirp & NO_NET) == 0) && (tdirp != 0) &&
				     	(tdirp != (net->netnum | ROUTED_NET))) {

			            if (layer < Num_layers - 2) {
			               tdirpp = OBSVAL(seg->x1 - 1, seg->y1, layer + 2)
						& ROUTED_NET_MASK;
			               if (tdirp == tdirpp) viaWU = 1;
				    }
				 }
			         if (tdir == tdirp) viaWM = 1;
			      }
			
			      if (layer > 0) {
			         tdirn = OBSVAL(seg->x1 - 1, seg->y1, layer - 1)
					& ROUTED_NET_MASK;
			         if (tdir == tdirn) viaWL = 1;
			      }
			   }
			}

			// Check for via to east
			if (seg->x1 < NumChannelsX[layer] - 1) {
			   tdir = OBSVAL(seg->x1 + 1, seg->y1, layer)
					& ROUTED_NET_MASK;

			   if (((tdir & NO_NET) == 0) && (tdir != 0) &&
				(tdir != (net->netnum | ROUTED_NET))) {

			      if (layer < Num_layers - 1) {
			         tdirp = OBSVAL(seg->x1 + 1, seg->y1, layer + 1)
					& ROUTED_NET_MASK;
			         if (((tdirp & NO_NET) == 0) && (tdirp != 0) &&
				     	(tdirp != (net->netnum | ROUTED_NET))) {

			            if (layer < Num_layers - 2) {
			               tdirpp = OBSVAL(seg->x1 + 1, seg->y1, layer + 2)
						& ROUTED_NET_MASK;
			               if (tdirp == tdirpp) viaEU = 1;
				    }
				 }
			         if (tdir == tdirp) viaEM = 1;
			      }
			
			      if (layer > 0) {
			         tdirn = OBSVAL(seg->x1 + 1, seg->y1, layer - 1)
					& ROUTED_NET_MASK;
			         if (tdir == tdirn) viaEL = 1;
			      }
			   }
			}

			// Compute X offset
			viaoffx = 0.0;

			if (viaWL) viaoffx = viaOffsetX[layer][0];
			else if (viaEL) viaoffx = -viaOffsetX[layer][0];

			if (viaWM && viaOffsetX[layer][1] > viaoffx)
			   viaoffx = viaOffsetX[layer][1];
			else if (viaEM && -viaOffsetX[layer][1] < viaoffx)
			   viaoffx = -viaOffsetX[layer][1];

			if (viaWU && viaOffsetX[layer][2] > viaoffx)
			   viaoffx = viaOffsetX[layer][2];
			else if (viaEU && -viaOffsetX[layer][2] < viaoffx)
			   viaoffx = -viaOffsetX[layer][2];

		        vx = (int)((REPS(viaoffx)) * oscale);
		     }

		     // viaCheckY[layer] indicates whether a check for
		     // vias is needed.  If so, record what vias are to north
		     // and south.

		     if (viaCheckY[layer] > 0) {

			viaNL = viaNM = viaNU = 0;
			viaSL = viaSM = viaSU = 0;

			// Check for via to south
			if (seg->y1 > 0) {
			   tdir = OBSVAL(seg->x1, seg->y1 - 1, layer)
					& ROUTED_NET_MASK;

			   if (((tdir & NO_NET) == 0) && (tdir != 0) &&
				(tdir != (net->netnum | ROUTED_NET))) {

			      if (layer < Num_layers - 1) {
			         tdirp = OBSVAL(seg->x1, seg->y1 - 1, layer + 1)
					& ROUTED_NET_MASK;
			         if (((tdirp & NO_NET) == 0) && (tdirp != 0) &&
				     	(tdirp != (net->netnum | ROUTED_NET))) {

			            if (layer < Num_layers - 2) {
			               tdirpp = OBSVAL(seg->x1, seg->y1 - 1, layer + 2)
						& ROUTED_NET_MASK;
			               if (tdirp == tdirpp) viaSU = 1;
				    }
				 }
			         if (tdir == tdirp) viaSM = 1;
			      }
			
			      if (layer > 0) {
			         tdirn = OBSVAL(seg->x1, seg->y1 - 1, layer - 1)
					& ROUTED_NET_MASK;
			         if (tdir == tdirn) viaSL = 1;
			      }
			   }
			}

			// Check for via to north
			if (seg->y1 < NumChannelsY[layer] - 1) {
			   tdir = OBSVAL(seg->x1, seg->y1 + 1, layer)
					& ROUTED_NET_MASK;

			   if (((tdir & NO_NET) == 0) && (tdir != 0) &&
				(tdir != (net->netnum | ROUTED_NET))) {

			      if (layer < Num_layers - 1) {
			         tdirp = OBSVAL(seg->x1, seg->y1 + 1, layer + 1)
					& ROUTED_NET_MASK;
			         if (((tdirp & NO_NET) == 0) && (tdirp != 0) &&
				     	(tdirp != (net->netnum | ROUTED_NET))) {

			            if (layer < Num_layers - 2) {
			               tdirpp = OBSVAL(seg->x1, seg->y1 + 1, layer + 2)
						& ROUTED_NET_MASK;
			               if (tdirp == tdirpp) viaNU = 1;
				    }
				 }
			         if (tdir == tdirp) viaNM = 1;
			      }
			
			      if (layer > 0) {
			         tdirn = OBSVAL(seg->x1, seg->y1 + 1, layer - 1)
					& ROUTED_NET_MASK;
			         if (tdir == tdirn) viaNL = 1;
			      }
			   }
			}

			// Compute Y offset
			viaoffy = 0;

			if (viaSL) viaoffy = viaOffsetY[layer][0];
			else if (viaNL) viaoffy = -viaOffsetY[layer][0];

			if (viaSM && viaOffsetY[layer][1] > viaoffy)
			   viaoffy = viaOffsetY[layer][1];
			else if (viaNM && -viaOffsetY[layer][1] < viaoffy)
			   viaoffy = -viaOffsetY[layer][1];

			if (viaSU && viaOffsetY[layer][2] > viaoffy)
			   viaoffy = viaOffsetY[layer][2];
			else if (viaNU && -viaOffsetY[layer][2] < viaoffy)
			   viaoffy = -viaOffsetY[layer][2];

		        vy = (int)((REPS(viaoffy)) * oscale);
		     }

		     // via-to-via interactions are symmetric, so move each
		     // via half the distance (?)

		     pathvia(Cmd, layer, x + vx, y + vy, lastx, lasty,
					seg->x1, seg->y1, invscale);

		     lastx = x;
		     lasty = y;
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

		 	 dc = Xlowerbound + (double)seg->x1 * PitchX[layer];
			 x = (int)((REPS(dc)) * oscale);
			 dc = Ylowerbound + (double)seg->y1 * PitchY[layer];
			 y = (int)((REPS(dc)) * oscale);

			 dc = Xlowerbound + (double)prevseg->x1 * PitchX[layer];
			 x2 = (int)((REPS(dc)) * oscale);
			 dc = Ylowerbound + (double)prevseg->y1 * PitchY[layer];
			 y2 = (int)((REPS(dc)) * oscale);

			 // Setup is (via, 1 track route, via with offset)

			 if (prevseg->x1 != seg->x1) {
			    if ((PitchX[lastseg->layer] -
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
					(u_char)1, oscale, invscale, 1);
				  pathto(Cmd, x2, y2, 1, x, y, invscale);
		      		  lastx = x2;
				  lasty = y2;
			       }
			    }
			 }
			 else if (prevseg->y1 != seg->y1) {
			    if ((PitchY[lastseg->layer] -
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
					(u_char)1, oscale, invscale, 0);
				  pathto(Cmd, x2, y2, 0, x, y, invscale);
		      		  lastx = x2;
				  lasty = y2;
			       }
			    }
			 }
		      }
		      else {	// Metal route bends at next track
			 if (prevseg->x1 != seg->x1) {
			    if ((PitchX[lastseg->layer] -
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
					(u_char)1, oscale, invscale, 1);
				  pathto(Cmd, x2, y2, 1, x, y, invscale);
		      		  lastx = x2;
				  lasty = y2;
			       }
			    }
			 }
			 else if (prevseg->y1 != seg->y1) {
			    if ((PitchY[lastseg->layer] -
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
					(u_char)1, oscale, invscale, 0);
				  pathto(Cmd, x2, y2, 0, x, y, invscale);
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

		dc = Xlowerbound + (double)seg->x2 * PitchX[layer];
		if (lnode->flags & NI_OFFSET_EW)
		   dc += offset;
		x = (int)((REPS(dc)) * oscale);
		if (lnode->flags & NI_STUB_EW)
		   dc += stub;
		x2 = (int)((REPS(dc)) * oscale);
		dc = Ylowerbound + (double)seg->y2 * PitchY[layer];
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

		   if ((x < x2) && (seg->x2 < (NumChannelsX[layer] - 1))) {
		      tdir = OBSVAL(seg->x2 + 1, seg->y2, layer);
		      if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (stub + LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	    dc = Xlowerbound + (double)(seg->x2 + 1)
					* PitchX[layer];
		      	    x2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }
		   else if ((x > x2) && (seg->x2 > 0)) {
		      tdir = OBSVAL(seg->x2 - 1, seg->y2, layer);
		      if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (-stub + LefGetRouteKeepout(layer) >= PitchX[layer]) {
		      	    dc = Xlowerbound + (double)(seg->x2 - 1)
					* PitchX[layer];
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

		   if ((y < y2) && (seg->y2 < (NumChannelsY[layer] - 1))) {
		      tdir = OBSVAL(seg->x2, seg->y2 + 1, layer);
		      if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (stub + LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	    dc = Ylowerbound + (double)(seg->y2 + 1)
					* PitchY[layer];
		      	    y2 = (int)((REPS(dc)) * oscale);
			 }
		      }
		   }
		   else if ((y > y2) && (seg->y2 > 0)) {
		      tdir = OBSVAL(seg->x2, seg->y2 - 1, layer);
		      if ((tdir & ROUTED_NET_MASK) ==
						(net->netnum | ROUTED_NET)) {
			 if (-stub + LefGetRouteKeepout(layer) >= PitchY[layer]) {
		      	    dc = Ylowerbound + (double)(seg->y2 - 1)
					* PitchY[layer];
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
				horizontal);
		      lastx = x;
		      lasty = y;
		   }
		   pathto(Cmd, x2, y2, horizontal, lastx, lasty, invscale);
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
    int i, j, numnets, stubroutes;
    char line[MAX_LINE_LEN + 1], *lptr = NULL;
    char netname[MAX_NAME_LEN];
    NET net = NULL;
    ROUTE rt;
    FILE *fdef;
    u_char errcond = FALSE;
    u_char need_cleanup = FALSE;

    fdef = fopen(DEFfilename, "r");
    if (fdef == NULL) {
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
    while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
       lptr = line;
       while (isspace(*lptr)) lptr++;
       if (!strncmp(lptr, "NETS", 4)) {
	  sscanf(lptr + 4, "%d", &numnets);
	  break;
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

    // Quick check to see if cleanup_nets can be avoided
    for (i = 0; i < Num_layers; i++)
       if (needblock[i] & (VIABLOCKX | VIABLOCKY))
	  break;

    if (i != Num_layers) need_cleanup = TRUE;

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
			 *lptr = '\n';
			 *(lptr + 1) = '\0';
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

       for (j = 0; j < Numnets; j++) {
          net = Nlnets[j];
	  if (!strcmp(net->netname, netname))
	     break;
       }
       if (!net) {
	  Fprintf(stderr, "emit_routes():  Net %s cannot be found.\n",
		netname);

	  /* Dump rest of net and continue---no routing information */
	  *(lptr) = ';';
	  fputs(line, Cmd);
	  continue;
       }
       else {
	  /* Add last net terminal, without the semicolon */
	  fputs(line, Cmd);

	  if (need_cleanup) cleanup_net(net);
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

        fprintf(Cmd, "\nSPECIALNETS %d ", stubroutes);
	for (i = 0; i < Numnets; i++) {
	     net = Nlnets[i];
	     emit_routed_net(Cmd, net, (u_char)1, oscale, iscale);
	}
	fprintf(Cmd, ";\nEND SPECIALNETS\n");
    }    

    // Finish copying the rest of the file
    while (fgets(line, MAX_LINE_LEN, fdef) != NULL) {
       fputs(line, Cmd);
    }
    fclose(fdef);
    fclose(Cmd);

} /* emit_routes() */

/* end of output.c */
