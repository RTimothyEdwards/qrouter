/*--------------------------------------------------------------*/
/* node.c -- Generation	of detailed network and obstruction	*/
/* information on the routing grid based on the geometry of the	*/
/* layout of the standard cell macros.				*/
/*								*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June, 2011, based on work by Steve	*/
/* Beccue.							*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "qrouter.h"
#include "node.h"
#include "qconfig.h"
#include "lef.h"
#include "output.h"

/*--------------------------------------------------------------*/
/* SetNodeinfo --						*/
/*	Allocate a NODEINFO record and put it in the Nodeinfo	*/
/*	array at position (gridx, gridy, d->layer).  Return the	*/
/* 	pointer to the location.				*/
/*--------------------------------------------------------------*/

NODEINFO
SetNodeinfo(int gridx, int gridy, int layer, NODE node)
{
    DPOINT dp;
    NODEINFO *lnodeptr;

    lnodeptr = &NODEIPTR(gridx, gridy, layer);
    if (*lnodeptr == NULL) {
	*lnodeptr = (NODEINFO)calloc(1, sizeof(struct nodeinfo_));

	/* Make sure this position is in the list of node's taps.  Add	*/
	/* it if it is not there.					*/

	for (dp = (DPOINT)node->taps; dp; dp = dp->next)
	    if (dp->gridx == gridx && dp->gridy == gridy && dp->layer == layer)
		break;
	if (dp == NULL)
	    for (dp = (DPOINT)node->extend; dp; dp = dp->next)
		if (dp->gridx == gridx && dp->gridy == gridy && dp->layer == layer)
		    break;
	if (dp == NULL) {
	    dp = (DPOINT)malloc(sizeof(struct dpoint_));
	    dp->gridx = gridx;
	    dp->gridy = gridy;
	    dp->layer = layer;
	    dp->x = (gridx * PitchX) + Xlowerbound;
	    dp->y = (gridy * PitchY) + Ylowerbound;
	    dp->next = node->extend;
	    node->extend = dp;
	}
    }
    return *lnodeptr;
}

/*--------------------------------------------------------------*/
/* FreeNodeinfo --						*/
/*	Free a NODEINFO record at array Nodeinfo position	*/
/*	(gridx, gridy, d->layer).  Set the position pointer to	*/
/*	NULL.  							*/
/*--------------------------------------------------------------*/

void
FreeNodeinfo(int gridx, int gridy, int layer)
{
    NODEINFO *lnodeptr;
    lnodeptr = &NODEIPTR(gridx, gridy, layer);

    if (*lnodeptr != NULL) {
        free(*lnodeptr);
	*lnodeptr = NULL;
    }
}

/*--------------------------------------------------------------*/
/* count_reachable_taps()					*/
/*								*/
/*  For each grid point in the layout, find if it corresponds	*/
/*  to a node and is unobstructed.  If so, increment the node's	*/
/*  count of reachable taps.  Then work through the list of	*/
/*  nodes and determine if any are completely unreachable.  If	*/
/*  so, then unobstruct any position that is inside tap		*/
/*  geometry that can contain a via.				*/
/*								*/
/*  NOTE:  This routine should check for tap rectangles that	*/
/*  may combine to form an area large enough to place a via;	*/
/*  also, it should check for tap points that are routable	*/
/*  by a wire and not a tap.  However, those conditions are	*/
/*  rare and are left unhandled for now.			*/
/*--------------------------------------------------------------*/

void
count_reachable_taps()
{
    NODE node;
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    int l, i, j, orient;
    int gridx, gridy;
    double deltax, deltay;
    double dx, dy;

    for (l = 0; l < Num_layers; l++) {
	for (j = 0; j < NumChannelsX * NumChannelsY; j++) {
	    if (Nodeinfo[l][j]) {
		node = Nodeinfo[l][j]->nodeloc;
		if (node != NULL) {

		    // Redundant check;  if Obs has NO_NET set, then
		    // Nodeinfo->nodeloc for that position should already
		    // be NULL

		    if (!(Obs[l][j] & NO_NET))
			node->numtaps++;
		}
	    }
	}
    }

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    node = g->noderec[i];
	    if (node == NULL) continue;
	    if (node->numnodes == 0) continue;	 // e.g., vdd or gnd bus
	    if (node->numtaps == 0) {

		/* Will try more than one via if available */
		for (orient = 0; orient < 4; orient += 2) {
		    for (ds = g->taps[i]; ds; ds = ds->next) {
			deltax = 0.5 * LefGetXYViaWidth(ds->layer, ds->layer, 0, orient);
			deltay = 0.5 * LefGetXYViaWidth(ds->layer, ds->layer, 1, orient);

			gridx = (int)((ds->x1 - Xlowerbound) / PitchX) - 1;
			if (gridx < 0) gridx = 0;
			while (1) {
			    dx = (gridx * PitchX) + Xlowerbound;
			    if (dx > ds->x2 || gridx >= NumChannelsX) break;

			    if (((dx - ds->x1 + EPS) > deltax) &&
					((ds->x2 - dx + EPS) > deltax)) {
				gridy = (int)((ds->y1 - Ylowerbound)
					/ PitchY) - 1;
				if (gridy < 0) gridy = 0;
				while (1) {
				    dy = (gridy * PitchY) + Ylowerbound;
				    if (dy > ds->y2 || gridy >= NumChannelsY)
					break;

				    if (((dy - ds->y1 + EPS) > deltay) &&
						((ds->y2 - dy + EPS) > deltay)) {

					if ((ds->layer == Num_layers - 1) ||
						!(OBSVAL(gridx, gridy, ds->layer + 1)
						& NO_NET)) {

					    // Grid position is clear for placing a via

					    if ((orient == 0) && (Verbose > 1))
						Fprintf(stdout, "Tap position (%g, %g)"
						    " appears to be technically routable"
						    " so it is being forced routable.\n",
						    dx, dy);
					    else if (Verbose > 1)
						Fprintf(stdout, "Tap position (%g, %g)"
						    " appears to be technically routable"
						    " with alternate via, so it is being"
						    " forced routable.\n", dx, dy);

					    OBSVAL(gridx, gridy, ds->layer) =
						(OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK)
						| (u_int)node->netnum;
					    lnode = SetNodeinfo(gridx, gridy, ds->layer,
							node);
					    lnode->nodeloc = node;
					    lnode->nodesav = node;

					    /* If we got to orient = 2, mark NI_NO_VIAX */
					    if (orient == 2) lnode->flags |= NI_NO_VIAX;
					    /* This is a bit harsh, but should work */
					    else lnode->flags |= NI_NO_VIAY;
					    node->numtaps++;
					}
				    }
				    gridy++;
				}
			    }
			    gridx++;
			}
		    }
		    /* If there's a solution, don't go looking at other vias */
		    if (node->numtaps > 0) break;
		}
	    }
	    if (node->numtaps == 0) {
		/* Node wasn't cleanly within tap geometry when centered */
		/* on a grid point.  But if the via can be offset and is */
		/* cleanly within the tap geometry, then allow it.	 */

		double dist, mindist;
		int dir, mask, tapx, tapy, tapl;

		/* Will try more than one via if available */
		for (orient = 0; orient < 4; orient += 2) {

		    /* Initialize mindist to a large value */
		    mask = 0;
		    mindist = PitchX + PitchY;
		    dir = 0;	/* Indicates no solution found */

		    for (ds = g->taps[i]; ds; ds = ds->next) {
			deltax = 0.5 * LefGetXYViaWidth(ds->layer, ds->layer, 0, orient);
			deltay = 0.5 * LefGetXYViaWidth(ds->layer, ds->layer, 1, orient);

			gridx = (int)((ds->x1 - Xlowerbound) / PitchX) - 1;
			if (gridx < 0) gridx = 0;
			while (1) {
			    dx = (gridx * PitchX) + Xlowerbound;
			    if (dx > ds->x2 || gridx >= NumChannelsX) break;

			    if (((dx - ds->x1 + EPS) > -deltax) &&
					((ds->x2 - dx + EPS) > -deltax)) {
				gridy = (int)((ds->y1 - Ylowerbound) / PitchY) - 1;
				if (gridy < 0) gridy = 0;

				while (1) {
				    dy = (gridy * PitchY) + Ylowerbound;
				    if (dy > ds->y2 || gridy >= NumChannelsY)
					break;

				    // Check that the grid position is inside the
				    // tap rectangle.

				    // NOTE: If the point above the grid is blocked,
				    // then a via cannot be placed here, so skip it.
				    // This currently looks only for completely
				    // obstructed positions.  To do:  For directionally
				    // obstructed positions, see if the obstruction
				    // is in the opposite direction of the via's
				    // offset and at least the same distance.
				    // Otherwise, it won't clear.

				    if (((ds->layer == Num_layers - 1) ||
						!(OBSVAL(gridx, gridy, ds->layer + 1)
						& (NO_NET || OBSTRUCT_MASK))) &&
						((dy - ds->y1 + EPS) > -deltay) &&
						((ds->y2 - dy + EPS) > -deltay)) {

					// Grid point is inside tap geometry.
					// Since it did not pass the simple insideness
					// test previously, it can be assumed that
					// one of the edges is closer to the grid point
					// than 1/2 via width.  Find that edge and use
					// it to determine the offset.

					// Check right edge
					if ((ds->x2 - dx + EPS) < deltax) {
					    dist = deltax - ds->x2 + dx;
					    // Confirm other edges
					    if ((dx - dist - deltax + EPS > ds->x1) &&
							(dy - deltay + EPS > ds->y1) &&
							(dy + deltay - EPS < ds->y2)) {
						if (dist < fabs(mindist)) {
						    mindist = dist;
						    mask = STUBROUTE;
						    dir = NI_STUB_EW;
						    tapx = gridx;
						    tapy = gridy;
						    tapl = ds->layer;
						}
					    }
					}
					// Check left edge
					if ((dx - ds->x1 + EPS) < deltax) {
					    dist = deltax - dx + ds->x1;
					    // Confirm other edges
					    if ((dx + dist + deltax - EPS < ds->x2) &&
							(dy - deltay + EPS > ds->y1) &&
							(dy + deltay - EPS < ds->y2)) {
						if (dist < fabs(mindist)) {
						    mindist = -dist;
						    mask = STUBROUTE;
						    dir = NI_STUB_EW;
						    tapx = gridx;
						    tapy = gridy;
						    tapl = ds->layer;
						}
					    }
					}
					// Check top edge
					if ((ds->y2 - dy + EPS) < deltay) {
					    dist = deltay - ds->y2 + dy;
					    // Confirm other edges
					    if ((dx - deltax + EPS > ds->x1) &&
						    (dx + deltax - EPS < ds->x2) &&
						    (dy - dist - deltay + EPS > ds->y1)) {
						if (dist < fabs(mindist)) {
						    mindist = -dist;
						    mask = STUBROUTE;
						    dir = NI_STUB_NS;
						    tapx = gridx;
						    tapy = gridy;
						    tapl = ds->layer;
						}
					    }
					}
					// Check bottom edge
					if ((dy - ds->y1 + EPS) < deltay) {
					    dist = deltay - dy + ds->y1;
					    // Confirm other edges
					    if ((dx - deltax + EPS > ds->x1) &&
						    (dx + deltax - EPS < ds->x2) &&
						    (dy + dist + deltay - EPS < ds->y2)) {
						if (dist < fabs(mindist)) {
					 	    mindist = dist;
						    mask = STUBROUTE;
						    dir = NI_STUB_NS;
						    tapx = gridx;
						    tapy = gridy;
						    tapl = ds->layer;
						}
					    }
					}
				    }
				    gridy++;
				}
			    }
			    gridx++;
			}
		    }

		    /* Was a solution found? */
		    if (mask != 0) {
			// Grid position is clear for placing a via

			if (Verbose > 1)
			    Fprintf(stdout, "Tap position (%d, %d) appears to be"
					" technically routable with an offset, so"
					" it is being forced routable.\n",
					tapx, tapy);

			OBSVAL(tapx, tapy, tapl) =
				(OBSVAL(tapx, tapy, tapl) & BLOCKED_MASK)
				| mask | (u_int)node->netnum;
			lnode = SetNodeinfo(tapx, tapy, tapl, node);
			lnode->nodeloc = node;
			lnode->nodesav = node;
			lnode->stub = dist;
			lnode->flags |= dir;

			/* If we got to orient = 2 then mark NI_NO_VIAX */
			if (orient == 2) lnode->flags |= NI_NO_VIAX;

			node->numtaps++;
		    }

		    /* If there's a solution, don't go looking at other vias */
		    if (node->numtaps > 0) break;
		}
	    }
	}
    }

    /* Last pass to output error messages for any taps that were not	*/
    /* handled by the code above.					*/

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    node = g->noderec[i];
	    if (node == NULL) continue;
	    if (node->numnodes == 0) continue;	 // e.g., vdd or gnd bus
	    if (node->numtaps == 0) {
		Fprintf(stderr, "Error: Node %s of net \"%s\" has no taps!\n",
			print_node_name(node), node->netname);
		Fprintf(stderr, "Qrouter will not be able to completely"
			" route this net.\n");
	    }
	}
    }
}

/*--------------------------------------------------------------*/
/* check_variable_pitch()					*/
/*								*/
/*  This routine is used by the routine below it to generate	*/
/*  obstructions that force routes to be placed in 1-of-N	*/
/*  tracks.  However, it is also used to determine the same	*/
/*  information for the .info file, so that the effective	*/
/*  pitch is output, not the pitch copied from the LEF file.	*/
/*  Output is the vertical and horizontal pitch multipliers,	*/
/*  passed back through pointers.				*/
/*--------------------------------------------------------------*/

void check_variable_pitch(int l, int *hptr, int *vptr)
{
   int o, hnum, vnum;
   double vpitch, hpitch, wvia, wviax, wviay;

   o = LefGetRouteOrientation(l);

   // Note that when "horizontal" (o = 1) is passed to LefGetXYViaWidth,
   // it returns the via width top-to-bottom (orient meaning is
   // reversed for LefGetXYViaWidth), which is what we want. . .

   // Try both via orientations and choose the best, assuming that it is
   // a lot easier to rotate and shift vias around than it is to lose
   // half the routing grid.

   if (l == 0) {
	 wviax = LefGetXYViaWidth(l, l, o, 0);
	 wviay = LefGetXYViaWidth(l, l, o, 3);
   }
   else {
	 wviax = LefGetXYViaWidth(l - 1, l, o, 0);
	 wviay = LefGetXYViaWidth(l - 1, l, o, 3);
   }
   wvia = (wviax < wviay) ? wviax : wviay;

   if (o == 1) {	// Horizontal route
      vpitch = LefGetRoutePitch(l);
      // Changed:  routes must be able to accomodate the placement
      // of a via in the track next to it.

      // hpitch = LefGetRouteWidth(l) + LefGetRouteSpacing(l);
      hpitch = 0.5 * (LefGetRouteWidth(l) + wvia) + LefGetRouteSpacing(l);
   }
   else {		// Vertical route
      hpitch = LefGetRoutePitch(l);
      // vpitch = LefGetRouteWidth(l) + LefGetRouteSpacing(l);
      vpitch = 0.5 * (LefGetRouteWidth(l) + wvia) + LefGetRouteSpacing(l);
   }

   vnum = 1;
   while (vpitch > PitchY + EPS) {
      vpitch /= 2.0;
      vnum++;
   }
   hnum = 1;
   while (hpitch > PitchX + EPS) {
      hpitch /= 2.0;
      hnum++;
   }

   *vptr = vnum;
   *hptr = hnum;
}

/*--------------------------------------------------------------*/
/* create_obstructions_from_variable_pitch()			*/
/*								*/
/*  Although it would be nice to have an algorithm that would	*/
/*  work with any arbitrary pitch, qrouter will work around	*/
/*  having larger pitches on upper metal layers by selecting	*/
/*  1 out of every N tracks for routing, and placing 		*/
/*  obstructions in the interstices.  This makes the possibly	*/
/*  unwarranted assumption that the contact down to the layer	*/
/*  below does not cause spacing violations to neighboring	*/
/*  tracks.  If that assumption fails, this routine will have	*/
/*  to be revisited.						*/
/*--------------------------------------------------------------*/

void create_obstructions_from_variable_pitch(void)
{
   int l, vnum, hnum, x, y;
   NODEINFO lnode;

   for (l = 0; l < Num_layers; l++) {

      check_variable_pitch(l, &hnum, &vnum);

      // This could be better handled by restricting
      // access from specific directions rather than
      // marking a position as NO_NET.  Since the
      // routine below will mark no positions restricted
      // if either hnum is 1 or vnum is 1, regardless of
      // the other value, then we force both values to
      // be at least 2.

      if (vnum > 1 && hnum == 1) hnum++;
      if (hnum > 1 && vnum == 1) vnum++;

      if (vnum > 1 || hnum > 1) {
	 for (x = 0; x < NumChannelsX; x++) {
	    if (x % hnum == 0) continue;
	    for (y = 0; y < NumChannelsY; y++) {
	       if (y % vnum == 0) continue;

	       // If the grid position itself is a node, don't restrict
	       // routing based on variable pitch.
	       if (((lnode = NODEIPTR(x, y, l)) != NULL) && (lnode->nodeloc != NULL))
		  continue;

	       // If there is a node in an adjacent grid then allow
	       // routing from that direction.

	       if ((x > 0) && ((lnode = NODEIPTR(x - 1, y, l)) != NULL) &&
			(lnode->nodeloc != NULL))
		  OBSVAL(x, y, l) = BLOCKED_MASK & ~BLOCKED_W;
	       else if ((y > 0) && ((lnode = NODEIPTR(x , y - 1, l)) != NULL) &&
			(lnode->nodeloc != NULL))
		  OBSVAL(x, y, l) = BLOCKED_MASK & ~BLOCKED_S;
	       else if ((x < NumChannelsX - 1)
			&& ((lnode = NODEIPTR(x + 1, y, l)) != NULL) &&
			(lnode->nodeloc != NULL))
		  OBSVAL(x, y, l) = BLOCKED_MASK & ~BLOCKED_E;
	       else if ((y < NumChannelsY - 1)
			&& ((lnode = NODEIPTR(x, y + 1, l)) != NULL) &&
			(lnode->nodeloc != NULL))
		  OBSVAL(x, y, l) = BLOCKED_MASK & ~BLOCKED_N;
	       else
		  OBSVAL(x, y, l) = NO_NET;
	    }
	 }
      }
   }
}

/*--------------------------------------------------------------*/
/* disable_gridpos() ---					*/
/*	Render the position at (x, y, lay) unroutable by	*/
/*	setting its Obs[] entry to NO_NET and removing it from	*/
/*	the Nodeinfo->nodeloc and Nodeinfo->nodesav records.	*/
/*--------------------------------------------------------------*/

static void
disable_gridpos(int x, int y, int lay)
{
    int apos = OGRID(x, y);

    Obs[lay][apos] = (u_int)(NO_NET | OBSTRUCT_MASK);
    if (Nodeinfo[lay][apos]) {
	free(Nodeinfo[lay][apos]);
	Nodeinfo[lay][apos] = NULL;
    }
}

/*--------------------------------------------------------------*/
/* count_pinlayers()---						*/
/*	Check which layers have non-NULL Nodeinfo entries.  	*/
/*	Then set "Pinlayers" and free all the unused layers.	*/
/* 	This saves a lot of memory, especially when the number	*/
/*	of routing layers becomes large.			*/ 
/*--------------------------------------------------------------*/

void
count_pinlayers(void)
{
   int j, l;

   Pinlayers = 0;
   for (l = 0; l < Num_layers; l++) {
      for (j = 0; j < NumChannelsX * NumChannelsY; j++) {
	 if (Nodeinfo[l][j]) {
	    Pinlayers = l + 1;
	    break;
	 }
      }
   }

   for (l = Pinlayers; l < Num_layers; l++) {
      free(Nodeinfo[l]);
      Nodeinfo[l] = NULL;
   }
}

/*--------------------------------------------------------------*/
/* check_obstruct()---						*/
/*	Called from create_obstructions_from_gates(), this	*/
/* 	routine takes a grid point at (gridx, gridy) (physical	*/
/* 	position (dx, dy)) and an obstruction defined by the	*/
/*	rectangle "ds", and sets flags and fills the Obsinfo	*/
/*	array to reflect how the obstruction affects routing to	*/
/*	the grid position.					*/
/*--------------------------------------------------------------*/

static void
check_obstruct(int gridx, int gridy, DSEG ds, double dx, double dy, double delta)
{
    u_int *obsptr;
    float dist;

    obsptr = &(OBSVAL(gridx, gridy, ds->layer));
    dist = OBSINFO(gridx, gridy, ds->layer);

    // Grid point is inside obstruction + halo.
    *obsptr |= NO_NET;

    // Completely inside obstruction?
    if (dy > ds->y1 && dy < ds->y2 && dx > ds->x1 && dx < ds->x2)
       *obsptr |= OBSTRUCT_MASK;

    else {
       // Make more detailed checks in each direction

       if (dy <= ds->y1) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_N)) == 0) {
	     if ((dist == 0) || ((ds->y1 - dy) < dist))
		OBSINFO(gridx, gridy, ds->layer) = ds->y1 - dy;
	     *obsptr |= OBSTRUCT_N;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       else if (dy >= ds->y2) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_S)) == 0) {
	     if ((dist == 0) || ((dy - ds->y2) < dist))
		OBSINFO(gridx, gridy, ds->layer) = dy - ds->y2;
	     *obsptr |= OBSTRUCT_S;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }

       if (dx <= ds->x1) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_E)) == 0) {
	     if ((dist == 0) || ((ds->x1 - dx) < dist))
		OBSINFO(gridx, gridy, ds->layer) = ds->x1 - dx;
             *obsptr |= OBSTRUCT_E;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
       else if (dx >= ds->x2) {
	  if ((*obsptr & (OBSTRUCT_MASK & ~OBSTRUCT_W)) == 0) {
	     if ((dist == 0) || ((dx - ds->x2) < dist))
		OBSINFO(gridx, gridy, ds->layer) = dx - ds->x2;
	     *obsptr |= OBSTRUCT_W;
	  }
	  else *obsptr |= OBSTRUCT_MASK;
       }
    }
}

/*--------------------------------------------------------------*/
/* Find the amount of clearance needed between an obstruction	*/
/* and a route track position.  This takes into consideration	*/
/* whether the obstruction is wide or narrow metal, if the	*/
/* spacing rules are graded according to metal width, and if a	*/
/* via placed at the position is or is not symmetric in X and Y	*/
/* "orient" is the via orientation, of which there are four;	*/
/* however, as this only concerns the via bottom layer, the	*/
/* two orientations are represented by orient = 0 and 2.	*/
/*--------------------------------------------------------------*/

static double get_via_clear(int lay, int horiz, int orient, DSEG rect) {
   double vdelta, v2delta, mdelta, mwidth;

   vdelta = LefGetXYViaWidth(lay, lay, 1 - horiz, orient);
   if (lay > 0) {
	v2delta = LefGetXYViaWidth(lay - 1, lay, 1 - horiz, orient);
	if (v2delta > vdelta) vdelta = v2delta;
   }
   vdelta = vdelta / 2.0;

   // Spacing rule is determined by the minimum metal width,
   // either in X or Y, regardless of the position of the
   // metal being checked.

   mwidth = MIN(rect->x2 - rect->x1, rect->y2 - rect->y1);
   mdelta = LefGetRouteWideSpacing(lay, mwidth);

   return vdelta + mdelta;
}

/*--------------------------------------------------------------*/
/* Find the distance from an obstruction to a grid point, 	*/
/* considering only routes which are placed at the position,	*/
/* not vias.							*/
/*--------------------------------------------------------------*/

static double get_route_clear(int lay, DSEG rect) {
   double rdelta, mdelta, mwidth;

   rdelta = LefGetRouteWidth(lay);
   rdelta = rdelta / 2.0;

   // Spacing rule is determined by the minimum metal width,
   // either in X or Y, regardless of the position of the
   // metal being checked.

   mwidth = MIN(rect->x2 - rect->x1, rect->y2 - rect->y1);
   mdelta = LefGetRouteWideSpacing(lay, mwidth);

   return rdelta + mdelta;
}

/*--------------------------------------------------------------*/
/* Truncate gates to the set of tracks.  Warn about any gates	*/
/* with nodes that are clipped entirely outside the routing	*/
/* area.							*/
/*--------------------------------------------------------------*/

void clip_gate_taps(void)
{
    NET net; 
    NODE node;
    DPOINT dp, dpl;
    int i, lay;

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	for (node = net->netnodes; node; node = node->next) {
	    dpl = NULL;
	    for (dp = (DPOINT)node->taps; dp; ) {

		lay = dp->layer;

		if (dp->gridx < 0 || dp->gridy < 0 ||
			dp->gridx >= NumChannelsX ||
			dp->gridy >= NumChannelsY) {
		    Fprintf(stderr, "Tap of port of node %d of net %s"
			" is outside of route area\n",
			node->nodenum, node->netname);

		    if (dpl == NULL)
			node->taps = dp->next;
		    else
			dpl->next = dp->next;

		    free(dp);
		    dp = (dpl == NULL) ? node->taps : dpl->next;
		}
		else {
		    dpl = dp;
		    dp = dp->next;
		}
	    }
	}
    }
}

/*--------------------------------------------------------------*/
/* create_obstructions_from_gates()				*/
/*								*/
/*  Fills in the Obs[][] grid from obstructions that were	*/
/*  defined for each macro in the technology LEF file and	*/
/*  translated into a list of grid coordinates in each		*/
/*  instance.							*/
/*								*/
/*  Also, fills in the Obs[][] grid with obstructions that	*/
/*  are defined by nodes of the gate that are unconnected in	*/
/*  this netlist.						*/
/*--------------------------------------------------------------*/

void create_obstructions_from_gates(void)
{
    GATE g;
    DSEG ds;
    int i, gridx, gridy, orient;
    double deltax, deltay, delta[MAX_LAYERS];
    double dx, dy, deltaxy;

    // Give a single net number to all obstructions, over the range of the
    // number of known nets, so these positions cannot be routed through.
    // If a grid position is not wholly inside an obstruction, then we
    // maintain the direction of the nearest obstruction in Obs and the
    // distance to it in Obsinfo.  This indicates that a route can avoid
    // the obstruction by moving away from it by the amount in Obsinfo
    // plus spacing clearance.  If another obstruction is found that
    // prevents such a move, then all direction flags will be set, indicating
    // that the position is not routable under any condition. 

    for (g = Nlgates; g; g = g->next) {
       orient = 0;
       for (ds = g->obs;; ds = ds->next) {
	  // Run through ds list twice, checking against horizontally and
	  // vertically oriented vias.

	  if (ds == NULL) {
	     if (orient == 2)
		break;
	     else {
		orient = 2;
	 	ds = g->obs;
		if (ds == NULL) break;
	     }
	  }

	  deltax = get_via_clear(ds->layer, 1, orient, ds);
	  gridx = (int)((ds->x1 - Xlowerbound - deltax) / PitchX) - 1;
	  while (1) {
	     dx = (gridx * PitchX) + Xlowerbound;
	     if ((dx + EPS) > (ds->x2 + deltax)
			|| gridx >= NumChannelsX) break;
	     else if ((dx - EPS) > (ds->x1 - deltax) && gridx >= 0) {
		deltay = get_via_clear(ds->layer, 0, orient, ds);
	        gridy = (int)((ds->y1 - Ylowerbound - deltay) / PitchY) - 1;
	        while (1) {
		   dy = (gridy * PitchY) + Ylowerbound;
	           if ((dy + EPS) > (ds->y2 + deltay)
				|| gridy >= NumChannelsY) break;
		   if ((dy - EPS) > (ds->y1 - deltay) && gridy >= 0) {
		      double s, edist, xp, yp;

		      // Check Euclidean distance measure
		      s = LefGetRouteSpacing(ds->layer);

		      if (dx < (ds->x1 + s - deltax)) {
		         xp = dx + deltax - s;
			 edist = (ds->x1 - xp) * (ds->x1 - xp);
		      }
		      else if (dx > (ds->x2 - s + deltax)) {
		         xp = dx - deltax + s;
			 edist = (xp - ds->x2) * (xp - ds->x2);
		      }
		      else edist = 0;
		      if ((edist > 0) && (dy < (ds->y1 + s - deltay))) {
		         yp = dy + deltay - s;
			 edist += (ds->y1 - yp) * (ds->y1 - yp);
		      }
		      else if ((edist > 0) && (dy > (ds->y2 - s + deltay))) {
		         yp = dy - deltay + s;
			 edist += (yp - ds->y2) * (yp - ds->y2);
		      }
		      else edist = 0;

		      if ((edist + EPS) < (s * s))
			 check_obstruct(gridx, gridy, ds, dx, dy, s);
		      else
			 edist = 0;	// diagnostic break
		   }
		   gridy++;
		}
	     }
	     gridx++;
	  }
       }

       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] == 0) {	/* Unconnected node */
	     // Diagnostic, and power bus handling
	     if (g->node[i]) {
		// Should we flag a warning if we see something that looks
		// like a power or ground net here?
		if (Verbose > 1)
		   Fprintf(stdout, "Gate instance %s unconnected node %s\n",
			g->gatename, g->node[i]);
	     }
	     else {
		if (Verbose > 1)
	           Fprintf(stdout, "Gate instance %s unconnected node (%d)\n",
			g->gatename, i);
	     }
             for (ds = g->taps[i]; ds; ds = ds->next) {

		deltax = get_via_clear(ds->layer, 1, orient, ds);
		gridx = (int)((ds->x1 - Xlowerbound - deltax) / PitchX) - 1;
		while (1) {
		   dx = (gridx * PitchX) + Xlowerbound;
		   if (dx > (ds->x2 + deltax)
				|| gridx >= NumChannelsX) break;
		   else if (dx >= (ds->x1 - deltax) && gridx >= 0) {
		      deltay = get_via_clear(ds->layer, 0, orient, ds);
		      gridy = (int)((ds->y1 - Ylowerbound - deltay) / PitchY) - 1;
		      while (1) {
		         dy = (gridy * PitchY) + Ylowerbound;
		         if ((dy + EPS) > (ds->y2 + deltay)
					|| gridy >= NumChannelsY) break;
		         if ((dy - EPS) >= (ds->y1 - deltay) && gridy >= 0) {

		            double s, edist = 0.0, xp, yp;

		            // Check Euclidean distance measure
		            s = LefGetRouteSpacing(ds->layer);

		            if (dx < (ds->x1 + s - deltax)) {
		               xp = dx + deltax - s;
			       edist += (ds->x1 - xp) * (ds->x1 - xp);
		            }
		            else if (dx > (ds->x2 - s + deltax)) {
		               xp = dx - deltax + s;
			       edist += (xp - ds->x2) * (xp - ds->x2);
		            }
			    else edist = 0;
		            if ((edist > 0) && (dy < (ds->y1 + s - deltay))) {
		               yp = dy + deltay - s;
			       edist += (ds->y1 - yp) * (ds->y1 - yp);
		            }
		            else if ((edist > 0) && (dy > (ds->y2 - s + deltay))) {
		               yp = dy - deltay + s;
			       edist += (yp - ds->y2) * (yp - ds->y2);
		            }
			    else edist = 0;

		            if ((edist + EPS) < (s * s))
			       check_obstruct(gridx, gridy, ds, dx, dy, s);
			 }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

    // Create additional obstructions from the UserObs list
    // These obstructions are not considered to be metal layers,
    // so we don't compute a distance measure.  However, we need
    // to compute a boundary of 1/2 route width to avoid having
    // the route overlapping the obstruction area.

    for (i = 0; i < Num_layers; i++) {
	delta[i] = LefGetRouteWidth(i) / 2.0;
    }

    for (ds = UserObs; ds; ds = ds->next) {
	if (ds->layer >= Num_layers) continue;
	gridx = (int)((ds->x1 - Xlowerbound - delta[ds->layer]) / PitchX) - 1;
	while (1) {
	    dx = (gridx * PitchX) + Xlowerbound;
	    if (dx > (ds->x2 + delta[ds->layer])
			|| gridx >= NumChannelsX) break;
	    else if (dx >= (ds->x1 - delta[ds->layer]) && gridx >= 0) {
		gridy = (int)((ds->y1 - Ylowerbound - delta[ds->layer]) / PitchY) - 1;
		while (1) {
		    dy = (gridy * PitchY) + Ylowerbound;
		    if (dy > (ds->y2 + delta[ds->layer])
				|| gridy >= NumChannelsY) break;
		    if (dy >= (ds->y1 - delta[ds->layer]) && gridy >= 0)
		       check_obstruct(gridx, gridy, ds, dx, dy, delta[i]);

		    gridy++;
		}
	    }
	    gridx++;
	}
    }
}

/*--------------------------------------------------------------*/
/* expand_tap_geometry()					*/
/*								*/
/*  For each rectangle defining part of a gate terminal,	*/
/*  search the surrounding terminal geometry.  If the rectangle	*/
/*  can expand in any direction, then allow it to grow to the	*/
/*  maximum size.  This generates overlapping geometry for each	*/
/*  terminal, but avoids bad results for determining how to	*/
/*  route to a terminal point if the terminal is broken up into	*/
/*  numerous nonoverlapping rectangles.				*/
/*								*/
/*  Note that this is not foolproof.  It also needs a number of	*/
/*  enhancements.  For example, to expand east, other geometry	*/
/*  should be looked at in order of increasing left edge X	*/
/*  value, and so forth.					*/
/*--------------------------------------------------------------*/

void expand_tap_geometry(void)
{
    DSEG ds, ds2;
    GATE g;
    int i;
    u_char expanded;

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    if (g->netnum[i] == 0) continue;
	    if (g->taps == NULL) continue;

	    for (ds = g->taps[i]; ds; ds = ds->next) {
		expanded = TRUE;
		while (expanded == TRUE) {
		    expanded = FALSE;

		    for (ds2 = g->taps[i]; ds2; ds2 = ds2->next) {
		        if (ds == ds2) continue;
			if (ds->layer != ds2->layer) continue;
		    
		        if ((ds2->y1 <= ds->y1) && (ds2->y2 >= ds->y2)) {
			    // Expand east
			    if ((ds2->x1 > ds->x1) && (ds2->x1 <= ds->x2))
			        if (ds->x2 < ds2->x2) {
				    ds->x2 = ds2->x2;
				    expanded = TRUE;
			        }
		    
			    // Expand west
			    if ((ds2->x2 < ds->x2) && (ds2->x2 >= ds->x1))
			        if (ds->x1 > ds2->x1) {
				    ds->x1 = ds2->x1;
				    expanded = TRUE;
			        }
		        }
		    
		        if ((ds2->x1 <= ds->x1) && (ds2->x2 >= ds->x2)) {
			    // Expand north
			    if ((ds2->y1 > ds->y1) && (ds2->y1 <= ds->y2))
			        if (ds->y2 < ds2->y2) {
				    ds->y2 = ds2->y2;
				    expanded = TRUE;
			        }
		    
			    // Expand south
			    if ((ds2->y2 < ds->y2) && (ds2->y2 >= ds->y1))
			        if (ds->y1 > ds2->y1) {
				    ds->y1 = ds2->y1;
				    expanded = TRUE;
			        }
		        }
		    }
		}
	    }
	}
    }
}

/*--------------------------------------------------------------*/
/* create_obstructions_inside_nodes()				*/
/*								*/
/*  Fills in the Obs[][] grid from the position of each node	*/
/*  (net terminal), which may have multiple unconnected		*/
/*  positions.							*/
/*								*/
/*  Also fills in the Nodeinfo.nodeloc[] grid with the node	*/
/*  number, which causes the router to put a premium on		*/
/*  routing other nets over or under this position, to		*/
/*  discourage boxing in a pin position and making it 		*/
/*  unroutable.							*/
/*								*/
/*  This routine is split into two passes.  This pass adds	*/
/*  information for points inside node regions.			*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, June 2011, based on code by Steve	*/
/*	Beccue.							*/
/*--------------------------------------------------------------*/

void create_obstructions_inside_nodes(void)
{
    NODE node;
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    u_int dir, mask, k;
    int i, gridx, gridy;
    double dx, dy, xdist, vwx, vwy;
    u_char o0okay, o2okay, duplicate;
    float dist;

    // For each node terminal (gate pin), mark each grid position with the
    // net number.  This overrides any obstruction that may be placed at that
    // point.

    // For each pin position, we also find the "keepout" area around the
    // pin where we may not place an unrelated route.  For this, we use a
    // flag bit, so that the position can be ignored when routing the net
    // associated with the pin.  Normal obstructions take precedence.

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] != 0) {

	     // Get the node record associated with this pin.
	     node = g->noderec[i];
	     if (node == NULL) continue;

	     // First mark all areas inside node geometry boundary.

             for (ds = g->taps[i]; ds; ds = ds->next) {
		gridx = (int)((ds->x1 - Xlowerbound) / PitchX) - 1;
		if (gridx < 0) gridx = 0;
		while (1) {
		   dx = (gridx * PitchX) + Xlowerbound;
		   if (dx > ds->x2 || gridx >= NumChannelsX) break;
		   else if (dx >= ds->x1 && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound) / PitchY) - 1;
		      if (gridy < 0) gridy = 0;
		      while (1) {
		         dy = (gridy * PitchY) + Ylowerbound;
		         if (dy > ds->y2 || gridy >= NumChannelsY) break;

			 // Area inside defined pin geometry

			 if (dy > ds->y1 && gridy >= 0) {
			     int orignet = OBSVAL(gridx, gridy, ds->layer);

			     duplicate = FALSE;
			     lnode = NULL;
			     if ((orignet & ROUTED_NET_MASK & ~ROUTED_NET)
					== (u_int)node->netnum) {

				// Duplicate tap point, or pre-existing route.
				// Re-process carefully.  Check for alternate
				// restrictions.  This geometry may be better
				// than the last one(s) processed.

				if (((lnode = NODEIPTR(gridx, gridy, ds->layer)) != NULL)
					&& (lnode->nodeloc != NULL))
				    duplicate = TRUE;
			     }

			     else if (!(orignet & NO_NET) &&
					((orignet & ROUTED_NET_MASK) != (u_int)0)) {

				// Net was assigned to other net, but is inside
				// this pin's geometry.  Declare point to be
				// unroutable, as it is too close to both pins.
				// NOTE:  This is a conservative rule and could
				// potentially make a pin unroutable.
				// Another note:  By setting Obs[] to
				// OBSTRUCT_MASK as well as NO_NET, we ensure
				// that it falls through on all subsequent
				// processing.

				disable_gridpos(gridx, gridy, ds->layer);
				gridy++;
				continue;
			     }

			     if (!(orignet & NO_NET)) {

				// A grid point that is within 1/2 route width
				// of a tap rectangle corner can violate metal
				// width rules, and so should declare a stub.
				
				mask = 0;
				dir = 0;
				dist = 0.0;
			        xdist = 0.5 * LefGetRouteWidth(ds->layer);

				if (dx >= ds->x2 - xdist) {
				   if (dy > ds->y2 - xdist + EPS) {
				      // Check northeast corner

				      if ((ds->x2 - dx) > (ds->y2 - dy)) {
					 // West-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x2 - dx - 2.0 * xdist;
				      }
				      else {
					 // South-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y2 - dy - 2.0 * xdist;
				      }

				   }
				   else if (dy < ds->y1 + xdist - EPS) {
				      // Check southeast corner

				      if ((ds->x2 - dx) > (dy - ds->y1)) {
					 // West-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x2 - dx - 2.0 * xdist;
				      }
				      else {
					 // North-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y1 - dy + 2.0 * xdist;
				      }
				   }
				}
				else if (dx <= ds->x1 + xdist) {
				   if (dy > ds->y2 - xdist + EPS) {
				      // Check northwest corner

				      if ((dx - ds->x1) > (ds->y2 - dy)) {
					 // East-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x1 - dx + 2.0 * xdist;
				      }
				      else {
					 // South-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y2 - dy - 2.0 * xdist;
				      }

				   }
				   else if (dy < ds->y1 + xdist - EPS) {
				      // Check southwest corner

				      if ((dx - ds->x2) > (dy - ds->y1)) {
					 // East-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x1 - dx + 2.0 * xdist;
				      }
				      else {
					 // North-pointing stub
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y1 - dy + 2.0 * xdist;
				      }
				   }
				}

				if (!duplicate) {
			           OBSVAL(gridx, gridy, ds->layer)
			        	= (OBSVAL(gridx, gridy, ds->layer)
					   & BLOCKED_MASK) | (u_int)node->netnum | mask;
				   if (!lnode)
				      lnode = SetNodeinfo(gridx, gridy, ds->layer,
						node);
				   lnode->nodeloc = node;
				   lnode->nodesav = node;
				   lnode->stub = dist;
				   lnode->flags |= dir;
				}

				/* If a horizontal or vertical via fits completely */
				/* inside the pin but the other orientation	   */
				/* doesn't, then mark as prohibiting the other	   */
				/* orientation.					   */

				vwx = LefGetXYViaWidth(ds->layer, ds->layer, 0, 0) / 2.0;
				vwy = LefGetXYViaWidth(ds->layer, ds->layer, 1, 0) / 2.0;
				if ((dx - vwx > ds->x1 - EPS) &&
					(dx + vwx < ds->x2 + EPS) &&
				    	(dy - vwy > ds->y1 - EPS) &&
					(dy + vwy < ds->y2 + EPS)) {
				    o0okay = TRUE;
				} else {
				    o0okay = FALSE;
				}
				vwx = LefGetXYViaWidth(ds->layer, ds->layer, 0, 2) / 2.0;
				vwy = LefGetXYViaWidth(ds->layer, ds->layer, 1, 2) / 2.0;
				if ((dx - vwx > ds->x1 - EPS) &&
					(dx + vwx < ds->x2 + EPS) &&
				    	(dy - vwy > ds->y1 - EPS) &&
					(dy + vwy < ds->y2 + EPS)) {
				    o2okay = TRUE;
				} else {
				    o2okay = FALSE;
				}
				if ((o0okay == TRUE) && (o2okay == FALSE))
				    lnode->flags |= NI_NO_VIAY;
				else if ((o0okay == FALSE) && (o2okay == TRUE))
				    lnode->flags |= NI_NO_VIAX;
			     }
			     else if ((orignet & NO_NET) && ((orignet & OBSTRUCT_MASK)
					!= OBSTRUCT_MASK)) {
				/* Handled on next pass */
			     }

			     // Check that we have not created a PINOBSTRUCT
			     // route directly over this point.
			     if ((!duplicate) && (ds->layer < Num_layers - 1)) {
			        k = OBSVAL(gridx, gridy, ds->layer + 1);
			        if (k & PINOBSTRUCTMASK) {
			           if ((k & ROUTED_NET_MASK) != (u_int)node->netnum) {
				       OBSVAL(gridx, gridy, ds->layer + 1) = NO_NET;
				       FreeNodeinfo(gridx, gridy, ds->layer + 1);
				   }
				}
			     }
			 }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

} /* void create_obstructions_inside_nodes( void ) */

/*--------------------------------------------------------------*/
/* create_obstructions_outside_nodes()				*/
/*								*/
/*  Fills in the Obs[][] grid from the position of each node	*/
/*  (net terminal), which may have multiple unconnected		*/
/*  positions.							*/
/*								*/
/*  Also fills in the Nodeinfo.nodeloc[] grid with the node	*/
/*  number, which causes the router to put a premium on		*/
/*  routing other nets over or under this position, to		*/
/*  discourage boxing in a pin position and making it 		*/
/*  unroutable.							*/
/*								*/
/*  This routine is split into two passes.  This pass adds	*/
/*  information for points outside node regions but close	*/
/*  enough to interact with the node.				*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, June 2011, based on code by Steve	*/
/*	Beccue.							*/
/*--------------------------------------------------------------*/


void create_obstructions_outside_nodes(void)
{
    NODE node, n2;
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    u_int dir, mask, k;
    int i, gridx, gridy, orient;
    double dx, dy, xdist, deltax, deltay;
    float dist;

    // For each node terminal (gate pin), mark each grid position with the
    // net number.  This overrides any obstruction that may be placed at that
    // point.

    // For each pin position, we also find the "keepout" area around the
    // pin where we may not place an unrelated route.  For this, we use a
    // flag bit, so that the position can be ignored when routing the net
    // associated with the pin.  Normal obstructions take precedence.

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] != 0) {

	     // Get the node record associated with this pin.
	     node = g->noderec[i];
	     if (node == NULL) continue;

	     // Repeat this whole exercise for areas in the halo outside
	     // the node geometry.  We have to do this after enumerating
	     // all inside areas because the tap rectangles often overlap,
	     // and one rectangle's halo may be inside another tap.

	     orient = 0;
             for (ds = g->taps[i];; ds = ds->next) {
		if (ds == NULL) {
		    if (orient == 2)
			break;
		    else {
			orient = 2;
			ds = g->taps[i];
			if (ds == NULL) break;
		    }
		}

		// Note:  Should be handling get_route_clear as a less
		// restrictive case, as was done above.
 
		deltax = get_via_clear(ds->layer, 1, orient, ds);
		gridx = (int)((ds->x1 - Xlowerbound - deltax) / PitchX) - 1;
		if (gridx < 0) gridx = 0;

		while (1) {
		   dx = (gridx * PitchX) + Xlowerbound;

		   // Check if obstruction position is too close for
		   // a via in either orientation.  The position can
		   // be marked as prohibiting one orientation or the
		   // other.

		   if (((dx + EPS) > (ds->x2 + deltax)) ||
				(gridx >= NumChannelsX))
		      break;

		   else if ((dx - EPS) > (ds->x1 - deltax) && gridx >= 0) {
		      deltay = get_via_clear(ds->layer, 0, orient, ds);
		      gridy = (int)((ds->y1 - Ylowerbound - deltay) / PitchY) - 1;
		      if (gridy < 0) gridy = 0;

		      while (1) {
		         dy = (gridy * PitchY) + Ylowerbound;

		         if (((dy + EPS) > (ds->y2 + deltay)) ||
				(gridy >= NumChannelsY))
			    break;

			 // 2nd pass on area inside defined pin geometry,
			 // allowing terminal connections to be made using
			 // an offset tap, where possible.

			 if ((dy >= ds->y1 && gridy >= 0) && (dx >= ds->x1)
					&& (dy <= ds->y2) && (dx <= ds->x2)) {
			     int orignet = OBSVAL(gridx, gridy, ds->layer);

			     if ((orignet & ROUTED_NET_MASK) == (u_int)node->netnum) {

				// Duplicate tap point.   Don't re-process it.
				gridy++;
				continue;
			     }

			     if (!(orignet & NO_NET) &&
					((orignet & ROUTED_NET_MASK) != (u_int)0)) {
				/* Do nothing;  previously handled */
			     }

			     else if ((orignet & NO_NET) && ((orignet & OBSTRUCT_MASK)
					!= OBSTRUCT_MASK)) {
				double sdistxx, sdistxy, sdistyx, sdistyy, offd;

				sdistxx = LefGetXYViaWidth(ds->layer, ds->layer,
					0, 0) / 2.0 +
					LefGetRouteSpacing(ds->layer);
				sdistxy = LefGetXYViaWidth(ds->layer, ds->layer,
					0, 2) / 2.0 +
					LefGetRouteSpacing(ds->layer);
				sdistyx = LefGetXYViaWidth(ds->layer, ds->layer,
					1, 0) / 2.0 +
					LefGetRouteSpacing(ds->layer);
				sdistyy = LefGetXYViaWidth(ds->layer, ds->layer,
					1, 2) / 2.0 +
					LefGetRouteSpacing(ds->layer);

				// Define a maximum offset we can have in X or
				// Y above which the placement of a via will
				// cause a DRC violation with a wire in the
				// adjacent route track in the direction of the
				// offset.

				int maxerr = 0;

				// If a cell is positioned off-grid, then a grid
				// point may be inside a pin and still be unroutable.
				// The Obsinfo[] array tells where an obstruction is,
				// if there was only one obstruction in one direction
				// blocking the grid point.  If so, then we set the
				// Nodeinfo.stub[] distance to move the tap away from
				// the obstruction to resolve the DRC error.

				// Make sure we have marked this as a node.
				lnode = SetNodeinfo(gridx, gridy, ds->layer, node);
				lnode->nodeloc = node;
				lnode->nodesav = node;
			        OBSVAL(gridx, gridy, ds->layer)
			        	= (OBSVAL(gridx, gridy, ds->layer)
					   & BLOCKED_MASK) | (u_int)node->netnum;

			        offd = OBSINFO(gridx, gridy, ds->layer);
				if (orignet & OBSTRUCT_N) {
				   if (sdistyy - offd > EPS) {
				      lnode->flags |= NI_NO_VIAY;
				      if (sdistyx - offd > EPS) {
					 /* Cannot route cleanly, so use offset */
				         if (offd - sdistyx > PitchX / 2.0) 
					    /* Offset distance is too large */
					    maxerr = 1;
					 else {

			                    OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				            lnode->offset = offd - sdistyx;
				            lnode->flags |= NI_OFFSET_NS;

				            /* If position above has obstruction, then */
				            /* add up/down block to prevent vias.      */

				            if ((ds->layer < Num_layers - 1) &&
							(gridy > 0) &&
							(OBSVAL(gridx, gridy - 1,
							ds->layer + 1) & OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				   }
				}
				else if (orignet & OBSTRUCT_S) {
				   if (sdistyy - offd > EPS) {
				      lnode->flags |= NI_NO_VIAY;
				      if (sdistyx - offd > EPS) {
				         if (offd - sdistyx > PitchX / 2.0)
					    /* Offset distance is too large */
					    maxerr = 1;
					 else {
			                    OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				            lnode->offset = sdistyx - offd;
				            lnode->flags |= NI_OFFSET_NS;

				            /* If position above has obstruction, then */
				            /* add up/down block to prevent vias.      */

				             if ((ds->layer < Num_layers - 1) &&
							(gridy < NumChannelsY - 1) &&
						   	(OBSVAL(gridx, gridy + 1,
							ds->layer + 1) & OBSTRUCT_MASK))
					        block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				   }
				}
				else if (orignet & OBSTRUCT_E) {
				   if (sdistxx - offd > EPS) {
				      lnode->flags |= NI_NO_VIAX;
				      if (sdistxy - offd > EPS) {
				         if (offd - sdistxy > PitchY / 2.0)
					    /* Offset distance is too large */
					    maxerr = 1;
					 else {
			                    OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				            lnode->offset = offd - sdistxy;
				            lnode->flags |= NI_OFFSET_EW;

				            /* If position above has obstruction, then */
				            /* add up/down block to prevent vias.      */

				             if ((ds->layer < Num_layers - 1) &&
							(gridx > 0) &&
							(OBSVAL(gridx - 1, gridy,
							ds->layer + 1) & OBSTRUCT_MASK))
					        block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				   }
				}
				else if (orignet & OBSTRUCT_W) {
				   if (sdistxx - offd > EPS) {
				      lnode->flags |= NI_NO_VIAX;
				      if (sdistxy - offd > EPS) {
				         if (offd - sdistxy > PitchY / 2.0)
					    /* Offset distance is too large */
					    maxerr = 1;
					 else {
			                    OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				            lnode->offset = sdistxy - offd;
				            lnode->flags |= NI_OFFSET_EW;

				            /* If position above has obstruction, then */
				            /* add up/down block to prevent vias.      */

				            if ((ds->layer < Num_layers - 1) &&
							(gridx < NumChannelsX - 1) &&
							(OBSVAL(gridx + 1, gridy,
							ds->layer + 1) & OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				   }
				}

			        if (maxerr == 1)
				   disable_gridpos(gridx, gridy, ds->layer);

				// Diagnostic
				else if (Verbose > 3)
				   Fprintf(stderr, "Port overlaps obstruction"
					" at grid %d %d, position %g %g\n",
					gridx, gridy, dx, dy);
			     }
			 }

		         if ((dy - EPS) > (ds->y1 - deltay) && gridy >= 0) {

			    double s, edist, xp, yp;
			    unsigned char epass = 0;

			    // Area inside halo around defined pin geometry.
			    // Exclude areas already processed (areas inside
			    // some pin geometry have been marked with netnum)

			    // Also check that we are not about to define a
			    // route position for a pin on a layer above 0 that
			    // blocks a pin underneath it.

			    // Flag positions that pass a Euclidean distance check.
			    // epass = 1 indicates that position clears a
			    // Euclidean distance measurement.

			    s = LefGetRouteSpacing(ds->layer);

			    if (dx < (ds->x1 + s - deltax)) {
				xp = dx + deltax - s;
				edist = (ds->x1 - xp) * (ds->x1 - xp);
			    }
			    else if (dx > (ds->x2 - s + deltax)) {
				xp = dx - deltax + s;
				edist = (xp - ds->x2) * (xp - ds->x2);
			    }
			    else edist = 0;
			    if ((edist > 0) && (dy < (ds->y1 + s - deltay))) {
				yp = dy + deltay - s;
				edist += (ds->y1 - yp) * (ds->y1 - yp);
			    }
			    else if ((edist > 0) && (dy > (ds->y2 - s + deltay))) {
				yp = dy - deltay + s;
				edist += (yp - ds->y2) * (yp - ds->y2);
			    }
			    else edist = 0;
			    if ((edist + EPS) > (s * s)) epass = 1;

			    xdist = 0.5 * LefGetRouteWidth(ds->layer);

			    n2 = NULL;
			    if (ds->layer > 0) {
			       lnode = NODEIPTR(gridx, gridy, ds->layer - 1);
			       n2 = (lnode) ? lnode->nodeloc : NULL;
			    }
			    if (n2 == NULL) {
			       lnode = NODEIPTR(gridx, gridy, ds->layer);
			       n2 = (lnode) ? lnode->nodeloc : NULL;
			    }
			    else {
			       // Watch out for the case where a tap crosses
			       // over a different tap.  Don't treat the tap
			       // on top as if it is not there!

			       NODE n3;
			       lnode = NODEIPTR(gridx, gridy, ds->layer);
			       n3 = (lnode) ? lnode->nodeloc : NULL;
			       if (n3 != NULL && n3 != node) n2 = n3;
			    }

			    // Ignore my own node.
			    if (n2 == node) n2 = NULL;

			    k = OBSVAL(gridx, gridy, ds->layer);

			    // In case of a port that is inaccessible from a grid
			    // point, or not completely overlapping it, the
			    // stub information will show how to adjust the
			    // route position to cleanly attach to the port.

			    mask = STUBROUTE;
			    dir = NI_STUB_NS | NI_STUB_EW;
			    dist = 0.0;

			    if (((k & ROUTED_NET_MASK) != (u_int)node->netnum)
					&& (n2 == NULL)) {

				if ((k & OBSTRUCT_MASK) != 0) {
				   float sdist = OBSINFO(gridx, gridy, ds->layer);

				   // If the point is marked as close to an
				   // obstruction, we can declare this an
				   // offset tap if we are not on a corner.
				   // Because we cannot define both an offset
				   // and a stub simultaneously, if the distance
				   // to clear the obstruction does not make the
				   // route reach the tap, then we mark the grid
				   // position as unroutable.

				   if (dy >= (ds->y1 - xdist) &&
						dy <= (ds->y2 + xdist)) {
				      if ((dx >= ds->x2) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_E)) {
				         dist = sdist - LefGetRouteKeepout(ds->layer);
					 if ((dx - ds->x2 + dist) < xdist) {
				 	    mask = OFFSET_TAP;
				 	    dir = NI_OFFSET_EW;

				            if ((ds->layer < Num_layers - 1) &&
							(gridx > 0) &&
							(OBSVAL(gridx - 1, gridy,
							ds->layer + 1)
							& OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				      else if ((dx <= ds->x1) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_W)) {
				         dist = LefGetRouteKeepout(ds->layer) - sdist;
					 if ((ds->x1 - dx - dist) < xdist) {
				 	    mask = OFFSET_TAP;
				            dir = NI_OFFSET_EW;

				            if ((ds->layer < Num_layers - 1) &&
							gridx <
							(NumChannelsX - 1)
							&& (OBSVAL(gridx + 1, gridy,
							ds->layer + 1)
							& OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
			 	   }	
				   if (dx >= (ds->x1 - xdist) &&
						dx <= (ds->x2 + xdist)) {
				      if ((dy >= ds->y2) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_N)) {
				         dist = sdist - LefGetRouteKeepout(ds->layer);
					 if ((dy - ds->y2 + dist) < xdist) {
				 	    mask = OFFSET_TAP;
				            dir = NI_OFFSET_NS;

				            if ((ds->layer < Num_layers - 1) &&
							gridy < 
							(NumChannelsY - 1)
							&& (OBSVAL(gridx, gridy - 1,
							ds->layer + 1)
							& OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				      else if ((dy <= ds->y1) &&
						((k & OBSTRUCT_MASK) == OBSTRUCT_S)) {
				         dist = LefGetRouteKeepout(ds->layer) - sdist;
					 if ((ds->y1 - dy - dist) < xdist) {
				 	    mask = OFFSET_TAP;
				            dir = NI_OFFSET_NS;

				            if ((ds->layer < Num_layers - 1) &&
							(gridy > 0) &&
							(OBSVAL(gridx, gridy + 1,
							ds->layer + 1)
							& OBSTRUCT_MASK))
					       block_route(gridx, gridy, ds->layer, UP);
					 }
				      }
				   }
				   // Otherwise, dir is left as NI_STUB_MASK
				}
				else {

				   // Cleanly unobstructed area.  Define stub
				   // route from point to tap, with a route width
				   // overlap if necessary to avoid a DRC width
				   // violation.

				   if ((dx >= ds->x2) &&
					((dx - ds->x2) > (dy - ds->y2)) &&
					((dx - ds->x2) > (ds->y1 - dy))) {
				      // West-pointing stub
				      if ((dy - ds->y2) <= xdist &&
					  (ds->y1 - dy) <= xdist) {
					 // Within reach of tap rectangle
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x2 - dx;
					 if (dy < (ds->y2 - xdist) &&
						dy > (ds->y1 + xdist)) {
					    if (dx < ds->x2 + xdist) dist = 0.0;
					 }
					 else {
					    dist -= 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dx <= ds->x1) &&
					((ds->x1 - dx) > (dy - ds->y2)) &&
					((ds->x1 - dx) > (ds->y1 - dy))) {
				      // East-pointing stub
				      if ((dy - ds->y2) <= xdist &&
					  (ds->y1 - dy) <= xdist) {
					 // Within reach of tap rectangle
					 mask = STUBROUTE;
					 dir = NI_STUB_EW;
					 dist = ds->x1 - dx;
					 if (dy < (ds->y2 - xdist) &&
						dy > (ds->y1 + xdist)) {
					    if (dx > ds->x1 - xdist) dist = 0.0;
					 }
					 else {
					    dist += 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dy >= ds->y2) &&
					((dy - ds->y2) > (dx - ds->x2)) &&
					((dy - ds->y2) > (ds->x1 - dx))) {
				      // South-pointing stub
				      if ((dx - ds->x2) <= xdist &&
					  (ds->x1 - dx) <= xdist) {
					 // Within reach of tap rectangle
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y2 - dy;
					 if (dx < (ds->x2 - xdist) &&
						dx > (ds->x1 + xdist)) {
					    if (dy < ds->y2 + xdist) dist = 0.0;
					 }
					 else {
					    dist -= 2.0 * xdist;
					 }
				      }
				   }
				   else if ((dy <= ds->y1) &&
					((ds->y1 - dy) > (dx - ds->x2)) &&
					((ds->y1 - dy) > (ds->x1 - dx))) {
				      // North-pointing stub
				      if ((dx - ds->x2) <= xdist &&
					  (ds->x1 - dx) <= xdist) {
					 // Within reach of tap rectangle
					 mask = STUBROUTE;
					 dir = NI_STUB_NS;
					 dist = ds->y1 - dy;
					 if (dx < (ds->x2 - xdist) &&
						dx > (ds->x1 + xdist)) {
					    if (dy > ds->y1 - xdist) dist = 0.0;
					 }
					 else {
					    dist += 2.0 * xdist;
					 }
				      }
				   }

				   if ((mask == STUBROUTE) && (dir == NI_STUB_MASK)) {

				      // Outside of pin at a corner.  First, if one
				      // direction is too far away to connect to a
				      // pin, then we must route the other direction.

				      if (dx < ds->x1 - xdist || dx > ds->x2 + xdist) {
				         if (dy >= ds->y1 - xdist &&
							dy <= ds->y2 + xdist) {
					    mask = STUBROUTE;
				            dir = NI_STUB_EW;
				            dist = (float)(((ds->x1 + ds->x2) / 2.0)
							- dx);
					 }
				      }
				      else if (dy < ds->y1 - xdist ||
							dy > ds->y2 + xdist) {
					 mask = STUBROUTE;
				         dir = NI_STUB_NS;
				         dist = (float)(((ds->y1 + ds->y2) / 2.0) - dy);
				      }

				      // Otherwise we are too far away at a diagonal
				      // to reach the pin by moving in any single
				      // direction.  To be pedantic, we could define
				      // some jogged stub, but for now, we just call
				      // the point unroutable (leave dir = NI_STUB_MASK)

				      // To do:  Apply offset + stub
				   }
				}

				// Additional checks on stub routes

				// Stub distances of <= 1/2 route width are
				// unnecessary, so don't create them.

				if (mask == STUBROUTE && (dir == NI_STUB_NS
					|| dir == NI_STUB_EW) &&
					(fabs(dist) < (xdist + EPS))) {
				    mask = 0;
				    dir = 0;
				    dist = 0.0;
				}
				else if (mask == STUBROUTE && (dir == NI_STUB_NS
					|| dir == NI_STUB_EW)) {
				   struct dseg_ de;
				   DSEG ds2;
				   u_char errbox = TRUE;

				   // Additional check:  Sometimes the above
				   // checks put stub routes where they are
				   // not needed because the stub is completely
				   // covered by other tap geometry.  Take the
				   // stub area and remove parts covered by
				   // other tap rectangles.  If the entire
				   // stub is gone, then don't put a stub here.

				   if (dir == NI_STUB_NS) {
				       de.x1 = dx - xdist;
				       de.x2 = dx + xdist;
				       if (dist > 0) {
					  de.y1 = dy + xdist;
					  de.y2 = dy + dist;
				       }
				       else {
					  de.y1 = dy + dist;
					  de.y2 = dy - xdist;
				       }
				   }
				   if (dir == NI_STUB_EW) {
				       de.y1 = dy - xdist;
				       de.y2 = dy + xdist;
				       if (dist > 0) {
					  de.x1 = dx + xdist;
					  de.x2 = dx + dist;
				       }
				       else {
					  de.x1 = dx + dist;
					  de.x2 = dx - xdist;
				       }
				   }

				   // For any tap that overlaps the
				   // stub extension box, remove that
				   // part of the box.

			           for (ds2 = g->taps[i]; ds2; ds2 = ds2->next) {
				      if (ds2 == ds) continue;
				      if (ds2->layer != ds->layer) continue;

				      if (ds2->x1 <= de.x1 && ds2->x2 >= de.x2 &&
						ds2->y1 <= de.y1 && ds2->y2 >= de.y2) {
				         errbox = FALSE;	// Completely covered
				         break;
				      }

				      // Look for partial coverage.  Note that any
				      // change can cause a change in the original
				      // two conditionals, so we have to keep
				      // evaluating those conditionals.

				      // ds2 covers left side of de
				      if (ds2->x1 < de.x2 && ds2->x2 > de.x1 + EPS)
				         if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					    if (ds2->x1 < de.x1 + EPS &&
							ds2->x2 < de.x2 - EPS) {
					       de.x1 = ds2->x2;
					       if (de.x1 > de.x2 - EPS) errbox = FALSE;
					    }

				      // ds2 covers right side of de
				      if (ds2->x1 < de.x2 - EPS && ds2->x2 > de.x1)
				         if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					    if (ds2->x2 > de.x2 - EPS &&
							ds2->x1 > de.x1 + EPS) {
					       de.x2 = ds2->x1;
					       if (de.x2 < de.x1 + EPS) errbox = FALSE;
					    }

				      // ds2 covers bottom side of de
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1 + EPS)
				         if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
					    if (ds2->y1 < de.y1 + EPS &&
							ds2->y2 < de.y2 - EPS) {
					       de.y1 = ds2->y2;
					       if (de.y1 > de.y2 - EPS) errbox = FALSE;
					    }

				      // ds2 covers top side of de
				      if (ds2->y1 < de.y2 - EPS && ds2->y2 > de.y1)
				         if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
					    if (ds2->y2 > de.y2 - EPS &&
							ds2->y1 > de.y1 + EPS) {
					       de.y2 = ds2->y1;
					       if (de.y2 < de.y1 + EPS) errbox = FALSE;
					    }
			           }

				   // If nothing is left of the stub box,
				   // then remove the stub.

				   if (errbox == FALSE) {
				      mask = 0;
				      dir = 0;
				      dist = 0;
				   }
                                }

				lnode = SetNodeinfo(gridx, gridy, ds->layer, node);
				lnode->nodeloc = node;
				lnode->nodesav = node;

				if ((k < Numnets) && (dir != NI_STUB_MASK)) {
				   OBSVAL(gridx, gridy, ds->layer)
				   	= (OBSVAL(gridx, gridy, ds->layer)
					  & BLOCKED_MASK) | (u_int)g->netnum[i] | mask; 
				   lnode->flags |= dir;
				}
				else if ((OBSVAL(gridx, gridy, ds->layer)
					& NO_NET) != 0) {
				   // Keep showing an obstruction, but add the
				   // direction info and log the stub distance.
				   OBSVAL(gridx, gridy, ds->layer) |= mask;
				   lnode->flags |= dir;
				}
				else {
				   OBSVAL(gridx, gridy, ds->layer)
					|= (mask | (g->netnum[i] & ROUTED_NET_MASK));
				   lnode->flags |= dir;
				}
				if ((mask & STUBROUTE) != 0) {
				   lnode->stub = dist;
				}
				else if (((mask & OFFSET_TAP) != 0) || (dist != 0.0)) {
				   lnode->offset = dist;
				}
				
				// Remove entries with NI_STUB_MASK---these
				// are blocked-in taps that are not routable
				// without causing DRC violations (formerly
				// called STUBROUTE_X).

				if (dir == NI_STUB_MASK) {
				   disable_gridpos(gridx, gridy, ds->layer);
				}
			    }
			    else if (epass == 0) {

			       // Position fails euclidean distance check

			       int othernet = (k & ROUTED_NET_MASK);

			       if (othernet != 0 && othernet != (u_int)node->netnum) {

			          // This location is too close to two different
				  // node terminals and should not be used

				  // If there is a stub, then we can't specify
				  // an offset tap, so just disable it.  If
				  // there is already an offset, then just
				  // disable it.  Otherwise, check if othernet
				  // could be routed using a tap offset.

				  // To avoid having to check all nearby
				  // geometry, place a restriction that the
				  // next grid point in the direction of the
				  // offset must be free (not a tap point of
				  // any net, including this one).  That is
				  // still "more restrictive than necessary",
				  // but since the alternative is an efficient
				  // area search for interacting geometry, this
				  // restriction will stand until an example
				  // comes along that requires the detailed
				  // search.

				  // Such an example has come along, leading to
				  // an additional relaxation allowing an offset
				  // if the neighboring channel does not have a
				  // node record.  This will probably need
				  // revisiting.
				
				  if ((k & PINOBSTRUCTMASK) != 0)
				     disable_gridpos(gridx, gridy, ds->layer);
				  else if ((lnode = NODEIPTR(gridx, gridy, ds->layer))
					!= NULL && (lnode->nodesav != NULL)) {

				     u_char no_offsets = TRUE;
				     int offset_net;

				     // By how much would a tap need to be moved
				     // to clear the obstructing geometry?

				     // Check tap to right

				     if ((dx > ds->x2) && (gridx <
						NumChannelsX - 1)) {
					offset_net = OBSVAL(gridx + 1, gridy, ds->layer);
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetXYViaWidth(ds->layer,
							ds->layer, 0, orient);
					   dist = ds->x2 - dx + xdist +
							LefGetRouteSpacing(ds->layer);
					   // Only accept an alternative solution if
					   // it has a smaller offset than previously
					   // found.
					   if ((no_offsets == FALSE) ||
						    (fabs(lnode->offset) > fabs(dist))) {
					      mask = OFFSET_TAP;
					      dir = NI_OFFSET_EW;
					      OBSVAL(gridx, gridy, ds->layer) |= mask;
					      lnode->offset = dist;
					      lnode->flags &= ~(NI_OFFSET_NS);
					      lnode->flags |= dir;
					      no_offsets = FALSE;

				              if ((ds->layer < Num_layers - 1) &&
							(gridx > 0) &&
							(OBSVAL(gridx + 1, gridy,
							ds->layer + 1) & OBSTRUCT_MASK))
					         block_route(gridx, gridy, ds->layer, UP);
				              else if ((ds->layer < Num_layers - 1) &&
							(gridx > 0) &&
							(dist > PitchX / 2))
					         block_route(gridx, gridy, ds->layer, UP);
					   }
					}
				     }

				     // Check tap to left

				     if ((dx < ds->x1) && (gridx > 0)) {
					offset_net = OBSVAL(gridx - 1, gridy, ds->layer);
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetXYViaWidth(ds->layer,
							ds->layer, 0, orient);
					   dist = ds->x1 - dx - xdist -
							LefGetRouteSpacing(ds->layer);
					   if ((no_offsets == FALSE) ||
						    (fabs(lnode->offset) > fabs(dist))) {
					      mask = OFFSET_TAP;
					      dir = NI_OFFSET_EW;
					      OBSVAL(gridx, gridy, ds->layer) |= mask;
					      lnode->offset = dist;
					      lnode->flags &= ~(NI_OFFSET_NS);
					      lnode->flags |= dir;
					      no_offsets = FALSE;

				              if ((ds->layer < Num_layers - 1) && gridx <
							(NumChannelsX - 1) &&
							(OBSVAL(gridx - 1, gridy,
							ds->layer + 1) & OBSTRUCT_MASK))
					         block_route(gridx, gridy, ds->layer, UP);
				              else if ((ds->layer < Num_layers - 1) &&
							gridx <
							(NumChannelsX - 1) &&
							(dist < -PitchX / 2))
					         block_route(gridx, gridy, ds->layer, UP);
					   }
					}
				     }

				     // Check tap up

				     if ((dy > ds->y2) && (gridy <
						NumChannelsY - 1)) {
					offset_net = OBSVAL(gridx, gridy + 1, ds->layer);
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetXYViaWidth(ds->layer,
							ds->layer, 1, orient);
					   dist = ds->y2 - dy + xdist +
							LefGetRouteSpacing(ds->layer);
					   if ((no_offsets == FALSE) ||
						    (fabs(lnode->offset) > fabs(dist))) {
					      mask = OFFSET_TAP;
					      dir = NI_OFFSET_NS;
					      OBSVAL(gridx, gridy, ds->layer) |= mask;
					      lnode->offset = dist;
					      lnode->flags &= ~(NI_OFFSET_EW);
					      lnode->flags |= dir;
					      no_offsets = FALSE;

				              if ((ds->layer < Num_layers - 1) &&
							(gridy > 0) && (OBSVAL(gridx,
							gridy + 1, ds->layer + 1)
							& OBSTRUCT_MASK))
					         block_route(gridx, gridy, ds->layer, UP);
				              else if ((ds->layer < Num_layers - 1) &&
							(gridy > 0) &&
							(dist > PitchY / 2))
					         block_route(gridx, gridy, ds->layer, UP);
					   }
					}
				     }

				     // Check tap down

				     if ((dy < ds->y1) && (gridy > 0)) {
					offset_net = OBSVAL(gridx, gridy - 1, ds->layer);
					if (offset_net == 0 || offset_net == othernet) {
					   xdist = 0.5 * LefGetXYViaWidth(ds->layer,
							ds->layer, 1, orient);
					   dist = ds->y1 - dy - xdist -
							LefGetRouteSpacing(ds->layer);
					   if ((no_offsets == FALSE) ||
						    (fabs(lnode->offset) > fabs(dist))) {
					      mask = OFFSET_TAP;
					      dir = NI_OFFSET_NS;
					      OBSVAL(gridx, gridy, ds->layer) |= mask;
					      lnode->offset = dist;
					      lnode->flags &= ~(NI_OFFSET_EW);
					      lnode->flags |= dir;
					      no_offsets = FALSE;

				              if ((ds->layer < Num_layers - 1) &&
							gridx <
							(NumChannelsX - 1) &&
							(OBSVAL(gridx, gridy - 1,
							ds->layer + 1) & OBSTRUCT_MASK))
					         block_route(gridx, gridy, ds->layer, UP);
				              else if ((ds->layer < Num_layers - 1) &&
							gridx <
							(NumChannelsX - 1) &&
							(dist < -PitchY / 2))
					         block_route(gridx, gridy, ds->layer, UP);
					   }
					}
				     }

				     // No offsets were possible.  If orient is 0
				     // then mark as NI_NO_VIAX and try again with
				     // orient 2.  If orient is 2 and no offsets
				     // are possible, then disable the position.

				     if (no_offsets == TRUE) {
				 	if (orient == 2) {
					    // Maybe no need to revert the flag?
					    lnode->flags &= ~NI_NO_VIAX;
				            disable_gridpos(gridx, gridy, ds->layer);
					}
					else
					    lnode->flags |= NI_NO_VIAX;
				     }
				  }
				  else
				     disable_gridpos(gridx, gridy, ds->layer);
			       }

			       /* If we are on a layer > 0, then this geometry	*/
			       /* may block or partially block a pin on layer	*/
			       /* zero.  Mark this point as belonging to the	*/
			       /* net with a stub route to it.			*/
			       /* NOTE:  This is possibly too restrictive.	*/
			       /* May want to force a tap offset for vias on	*/
			       /* layer zero. . .				*/

			       if ((ds->layer > 0) && (n2 != NULL) && (n2->netnum
					!= node->netnum) && ((othernet == 0) ||
					(othernet == (u_int)node->netnum))) {

				  lnode = NODEIPTR(gridx, gridy, ds->layer);
				  xdist = 0.5 * LefGetXYViaWidth(ds->layer, ds->layer,
						0, orient);
				  if ((dy + xdist + LefGetRouteSpacing(ds->layer) >
					ds->y1) && (dy + xdist < ds->y1)) {
				     if ((dx - xdist < ds->x2) &&
						(dx + xdist > ds->x1) &&
						(lnode == NULL || lnode->stub
						== 0.0)) {
					OBSVAL(gridx, gridy, ds->layer)
				   		= (OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK) |
						node->netnum | STUBROUTE;
				        lnode = SetNodeinfo(gridx, gridy, ds->layer,
						node);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					lnode->stub = ds->y1 - dy;
					lnode->flags |= NI_STUB_NS;
				     }
				  }
				  if ((dy - xdist - LefGetRouteSpacing(ds->layer) <
					ds->y2) && (dy - xdist > ds->y2)) {
				     if ((dx - xdist < ds->x2) &&
						(dx + xdist > ds->x1) &&
						(lnode == NULL || lnode->stub
						== 0.0)) {
					OBSVAL(gridx, gridy, ds->layer)
				   		= (OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK) |
						node->netnum | STUBROUTE;
				        lnode = SetNodeinfo(gridx, gridy, ds->layer,
						node);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					lnode->stub = ds->y2 - dy;
					lnode->flags |= NI_STUB_NS;
				     }
				  }

				  xdist = 0.5 * LefGetXYViaWidth(ds->layer, ds->layer,
						1, orient);
				  if ((dx + xdist + LefGetRouteSpacing(ds->layer) >
					ds->x1) && (dx + xdist < ds->x1)) {
				     if ((dy - xdist < ds->y2) &&
						(dy + xdist > ds->y1) &&
						(lnode == NULL || lnode->stub
						 == 0.0)) {
					OBSVAL(gridx, gridy, ds->layer)
				   		= (OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK) |
						node->netnum | STUBROUTE;
				        lnode = SetNodeinfo(gridx, gridy, ds->layer,
						node);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					lnode->stub = ds->x1 - dx;
					lnode->flags |= NI_STUB_EW;
				     }
				  }
				  if ((dx - xdist - LefGetRouteSpacing(ds->layer) <
					ds->x2) && (dx - xdist > ds->x2)) {
				     if ((dy - xdist < ds->y2) &&
						(dy + xdist > ds->y1) &&
						(lnode == NULL || lnode->stub
						== 0.0)) {
					OBSVAL(gridx, gridy, ds->layer)
				   		= (OBSVAL(gridx, gridy, ds->layer)
						& BLOCKED_MASK) |
						node->netnum | STUBROUTE;
				        lnode = SetNodeinfo(gridx, gridy, ds->layer,
						node);
					lnode->nodeloc = node;
					lnode->nodesav = node;
					lnode->stub = ds->x2 - dx;
					lnode->flags |= NI_STUB_EW;
				     }
				  }
			       }
			    }
		         }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

} /* void create_obstructions_outside_nodes( void ) */

/*--------------------------------------------------------------*/
/* tap_to_tap_interactions()					*/
/*								*/
/*  Similar to create_obstructions_from_nodes(), but looks at	*/
/*  each node's tap geometry, looks at every grid point in a	*/
/*  wider area surrounding the tap.  If any other node has an	*/
/*  offset that would place it too close to this node's	tap	*/
/*  geometry, then we mark the other node as unroutable at that	*/
/*  grid point.							*/
/*--------------------------------------------------------------*/

void tap_to_tap_interactions(void)
{
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    struct dseg_ de;
    int mingridx, mingridy, maxgridx, maxgridy;
    int i, gridx, gridy, net, orignet;
    double dx, dy;
    float dist;

    double deltaxx[MAX_LAYERS];
    double deltaxy[MAX_LAYERS];
    double deltayx[MAX_LAYERS];
    double deltayy[MAX_LAYERS];

    for (i = 0; i < Num_layers; i++) {
	deltaxx[i] = 0.5 * LefGetXYViaWidth(i, i, 0, 0) + LefGetRouteSpacing(i);
	deltayx[i] = 0.5 * LefGetXYViaWidth(i, i, 1, 0) + LefGetRouteSpacing(i);
	deltaxy[i] = 0.5 * LefGetXYViaWidth(i, i, 0, 2) + LefGetRouteSpacing(i);
	deltayy[i] = 0.5 * LefGetXYViaWidth(i, i, 1, 2) + LefGetRouteSpacing(i);
    }

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  net = g->netnum[i];
	  if (net > 0) {
             for (ds = g->taps[i]; ds; ds = ds->next) {

		mingridx = (int)((ds->x1 - Xlowerbound) / PitchX) - 1;
		if (mingridx < 0) mingridx = 0;
		maxgridx = (int)((ds->x2 - Xlowerbound) / PitchX) + 2;
		if (maxgridx >= NumChannelsX)
		   maxgridx = NumChannelsX - 1;
		mingridy = (int)((ds->y1 - Ylowerbound) / PitchY) - 1;
		if (mingridy < 0) mingridy = 0;
		maxgridy = (int)((ds->y2 - Ylowerbound) / PitchY) + 2;
		if (maxgridy >= NumChannelsY)
		   maxgridy = NumChannelsY - 1;

		for (gridx = mingridx; gridx <= maxgridx; gridx++) {
		   for (gridy = mingridy; gridy <= maxgridy; gridy++) {

		      /* Is there an offset tap at this position, and	*/
		      /* does it belong to a net that is != net?	*/

		      orignet = OBSVAL(gridx, gridy, ds->layer);
		      if (orignet & OFFSET_TAP) {
			 orignet &= ROUTED_NET_MASK;
			 if (orignet != net) {

		            dx = (gridx * PitchX) + Xlowerbound;
		            dy = (gridy * PitchY) + Ylowerbound;

			    lnode = NODEIPTR(gridx, gridy, ds->layer);
			    dist = (lnode) ? lnode->offset : 0.0;

			    /* "de" is the bounding box of a via placed	  */
			    /* at (gridx, gridy) and offset as specified. */
			    /* Expanded by metal spacing requirement.	  */

			    de.x1 = dx - deltaxx[ds->layer];
			    de.x2 = dx + deltaxx[ds->layer];
			    de.y1 = dy - deltayx[ds->layer];
			    de.y2 = dy + deltayx[ds->layer];

			    if (lnode->flags & NI_OFFSET_NS) {
			       de.y1 += dist;
			       de.y2 += dist;
			    }
			    else if (lnode->flags & NI_OFFSET_EW) {
			       de.x1 += dist;
			       de.x2 += dist;
			    }

			    // Shrink by EPS to avoid roundoff errors
			    de.x1 += EPS;
			    de.x2 -= EPS;
			    de.y1 += EPS;
			    de.y2 -= EPS;

			    /* Does the via bounding box interact with	*/
			    /* the tap geometry?			*/

			    if ((de.x1 < ds->x2) && (ds->x1 < de.x2) &&
					(de.y1 < ds->y2) && (ds->y1 < de.y2))
			       disable_gridpos(gridx, gridy, ds->layer);
			 }
		      }

		      /* Does the distance to the tap prohibit a specific */
		      /* via orientation?				  */
		      if ((orignet & (~(BLOCKED_N | BLOCKED_S | BLOCKED_E | BLOCKED_W)))
				== 0) {
			 lnode = NODEIPTR(gridx, gridy, ds->layer);
			 /* Positions belonging to nodes should have	*/
			 /* already been handled.			*/
			 if (lnode == NULL) {
		            dx = (gridx * PitchX) + Xlowerbound;
		            dy = (gridy * PitchY) + Ylowerbound;

			    /* For a horizontally-oriented via on ds->layer */
			    de.x1 = dx - deltaxx[ds->layer];
			    de.x2 = dx + deltaxx[ds->layer];
			    de.y1 = dy - deltayx[ds->layer];
			    de.y2 = dy + deltayx[ds->layer];

			    if (ds->x2 > de.x1 && ds->x1 < de.x2) {
			        /* Check north and south */
			        if ((ds->y1 < de.y2 && ds->y2 > de.y2) ||
			 		(ds->y2 > de.y1 && ds->y1 < de.y1)) {
				    /* prohibit horizontal via */
				    lnode = SetNodeinfo(gridx, gridy, ds->layer,
						g->noderec[i]);
				    lnode->flags |= NI_NO_VIAX;
				}
			    }

			    if (ds->y2 > de.y1 && ds->y1 < de.y2) {
			        /* Check east and west*/
			        if ((ds->x1 < de.x2 && ds->x2 > de.x2) ||
					(ds->x2 > de.x1 && ds->x1 < de.x1)) {
				    /* prohibit horizontal via */
				    lnode = SetNodeinfo(gridx, gridy, ds->layer,
						g->noderec[i]);
				    lnode->flags |= NI_NO_VIAX;
				}
			    }

			    /* For a vertically-oriented via on ds->layer */
			    de.x1 = dx - deltaxy[ds->layer];
			    de.x2 = dx + deltaxy[ds->layer];
			    de.y1 = dy - deltayy[ds->layer];
			    de.y2 = dy + deltayy[ds->layer];

			    if (ds->x2 > de.x1 && ds->x1 < de.x2) {
			        /* Check north and south */
			        if ((ds->y1 < de.y2 && ds->y2 > de.y2) ||
			 		(ds->y2 > de.y1 && ds->y1 < de.y1)) {
				    /* prohibit horizontal via */
				    lnode = SetNodeinfo(gridx, gridy, ds->layer,
						g->noderec[i]);
				    lnode->flags |= NI_NO_VIAY;
				}
			    }

			    if (ds->y2 > de.y1 && ds->y1 < de.y2) {
			        /* Check east and west*/
			        if ((ds->x1 < de.x2 && ds->x2 > de.x2) ||
					(ds->x2 > de.x1 && ds->x1 < de.x1)) {
				    /* prohibit horizontal via */
				    lnode = SetNodeinfo(gridx, gridy, ds->layer,
						g->noderec[i]);
				    lnode->flags |= NI_NO_VIAY;
				}
			    }
			 }
		      }
		   }
		}
	     }
	  }
       }
    }
}

/*--------------------------------------------------------------*/
/* make_routable()						*/
/*								*/
/*  In the case that a node can't be routed because it has no	*/
/*  available tap points, but there is tap geometry recorded	*/
/*  for the node, then take the first available grid location	*/
/*  near the tap.  This, of course, bypasses all of qrouter's	*/
/*  DRC checks.  But it is only meant to be a stop-gap measure	*/
/*  to get qrouter to complete all routes, and may work in	*/
/*  cases where, say, the tap passes euclidean rules but not	*/
/*  manhattan rules.						*/
/*--------------------------------------------------------------*/

void
make_routable(NODE node)
{
    NODEINFO lnode;
    GATE g;
    DSEG ds;
    int i, gridx, gridy;
    double dx, dy;

    /* The database is not organized to find tap points	*/
    /* from nodes, so we have to search for the node.	*/
    /* Fortunately this routine isn't normally called.	*/

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->noderec[i] == node) {
             for (ds = g->taps[i]; ds; ds = ds->next) {
		gridx = (int)((ds->x1 - Xlowerbound) / PitchX) - 1;
		if (gridx < 0) gridx = 0;
		while (1) {
		   dx = (gridx * PitchX) + Xlowerbound;
		   if (dx > ds->x2 || gridx >= NumChannelsX) break;
		   else if (dx >= ds->x1 && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound) / PitchY) - 1;
		      if (gridy < 0) gridy = 0;
		      while (1) {
		         dy = (gridy * PitchY) + Ylowerbound;
		         if (dy > ds->y2 || gridy >= NumChannelsY) break;

			 // Area inside defined pin geometry

			 if (dy > ds->y1 && gridy >= 0) {
			    int orignet = OBSVAL(gridx, gridy, ds->layer);

			    if (orignet & NO_NET) {
				OBSVAL(gridx, gridy, ds->layer) = g->netnum[i];
				lnode = SetNodeinfo(gridx, gridy, ds->layer,
						g->noderec[i]);
				lnode->nodeloc = node;
				lnode->nodesav = node;
				return;
			    }
			 }
			 gridy++;
		      }
		   }
		   gridx++;
	        }
	     }
	  }
       }
    }
}

/*--------------------------------------------------------------*/
/* adjust_stub_lengths()					*/
/*								*/
/*  Makes an additional pass through the tap and obstruction	*/
/*  databases, checking geometry against the potential stub	*/
/*  routes for DRC spacing violations.  Adjust stub routes as	*/
/*  necessary to resolve the DRC error(s).			*/
/*								*/
/*  ARGS: none.							*/
/*  RETURNS: nothing						*/
/*  SIDE EFFECTS: none						*/
/*  AUTHOR:  Tim Edwards, April 2013				*/
/*--------------------------------------------------------------*/

void adjust_stub_lengths(void)
{
    NODE node;
    NODEINFO lnode;
    GATE g;
    DSEG ds, ds2;
    struct dseg_ dt, de;
    int i, gridx, gridy, orignet, o;
    double dx, dy, wx, wy, s;
    float dist;
    u_char errbox;
    int orient = 0;

    // For each node terminal (gate pin), look at the surrounding grid points.
    // If any define a stub route or an offset, check if the stub geometry
    // or offset geometry would create a DRC spacing violation.  If so, adjust
    // the stub route to resolve the error.  If the error cannot be resolved,
    // mark the position as unroutable.  If it is the ONLY grid point accessible
    // to the pin, keep it as-is and flag a warning.

    // Unlike blockage-finding routines, which look in an area of a size equal
    // to the DRC interaction distance around a tap rectangle, this routine looks
    // out one grid pitch in each direction, to catch information about stubs that
    // may terminate within a DRC interaction distance of the tap rectangle.

    for (g = Nlgates; g; g = g->next) {
       for (i = 0; i < g->nodes; i++) {
	  if (g->netnum[i] != 0) {

	     // Get the node record associated with this pin.
	     node = g->noderec[i];
	     if (node == NULL) continue;

	     // Work through each rectangle in the tap geometry twice.  The
	     // second sweep checks for errors created by geometry that
	     // interacts with stub routes created by the first sweep, and
	     // also for geometry that interacts with vias in 90 degree
	     // orientation.

	     orient = 0;
             for (ds = g->taps[i];; ds = ds->next) {
		if (ds == NULL) {
		    if (orient == 0) {
			orient = 2;
			ds = g->taps[i];
			if (ds == NULL) break;
		    }
		    else
			break;
		}
		wx = 0.5 * LefGetXYViaWidth(ds->layer, ds->layer, 0, orient);
		wy = 0.5 * LefGetXYViaWidth(ds->layer, ds->layer, 1, orient);
		s = LefGetRouteSpacing(ds->layer);
		gridx = (int)((ds->x1 - Xlowerbound - PitchX) / PitchX) - 1;
		while (1) {
		   dx = (gridx * PitchX) + Xlowerbound;
		   if (dx > (ds->x2 + PitchX) ||
				gridx >= NumChannelsX) break;
		   else if (dx >= (ds->x1 - PitchX) && gridx >= 0) {
		      gridy = (int)((ds->y1 - Ylowerbound - PitchY) / PitchY) - 1;
		      while (1) {
		         dy = (gridy * PitchY) + Ylowerbound;
		         if (dy > (ds->y2 + PitchY) ||
				gridy >= NumChannelsY) break;
		         if (dy >= (ds->y1 - PitchY) && gridy >= 0) {

			     orignet = OBSVAL(gridx, gridy, ds->layer);

			     // Ignore this location if it is assigned to another
			     // net, or is assigned to NO_NET.

			     if ((orignet & ROUTED_NET_MASK) != node->netnum) {
				gridy++;
				continue;
			     }
			     lnode = NODEIPTR(gridx, gridy, ds->layer);

			     // Even if it's on the same net, we need to check
			     // if the stub is to this node, otherwise it is not
			     // an issue.
			     if ((!lnode) || (lnode->nodesav != node)) {
				gridy++;
				continue;
			     }

			     // NI_STUB_MASK are unroutable;  leave them alone
			     if (orignet & STUBROUTE) {
				if ((lnode->flags & NI_OFFSET_MASK) == NI_OFFSET_MASK) {
				   gridy++;
				   continue;
				}
			     }

			     // define a route box around the grid point

			     errbox = FALSE;
			     dt.x1 = dx - wx;
			     dt.x2 = dx + wx;
			     dt.y1 = dy - wy;
			     dt.y2 = dy + wy;

			     // adjust the route box according to the stub
			     // or offset geometry, provided that the stub
			     // is longer than the route box.

			     if (orignet & OFFSET_TAP) {
			        dist = lnode->offset;
				if (lnode->flags & NI_OFFSET_EW) {
				   dt.x1 += dist;
				   dt.x2 += dist;
				}
				else if (lnode->flags & NI_OFFSET_NS) {
				   dt.y1 += dist;
				   dt.y2 += dist;
				}
			     }
			     else if (orignet & STUBROUTE) {
			        dist = (double)lnode->stub;
				if (lnode->flags & NI_STUB_EW) {
				   if (dist > EPS) {
				      if (dx + dist > dt.x2)
				 	 dt.x2 = dx + dist;
				   }
				   else {
				      if (dx + dist < dt.x1)
					 dt.x1 = dx + dist;
				   }
				}
				else if (lnode->flags & NI_STUB_NS) {
				   if (dist > EPS) {
				      if (dy + dist > dt.y2)
					 dt.y2 = dy + dist;
				   }
				   else {
				      if (dy + dist < dt.y1)
					 dt.y1 = dy + dist;
				   }
				}
			     }

			     de = dt;

			     // check for DRC spacing interactions between
			     // the tap box and the route box

			     if ((dt.y1 - ds->y2) > EPS && (dt.y1 - ds->y2) + EPS < s) {
				if (ds->x2 > (dt.x1 - s) && ds->x1 < (dt.x2 + s)) {
				   de.y2 = dt.y1;
				   de.y1 = ds->y2;
				   if (ds->x2 + s < dt.x2) de.x2 = ds->x2 + s;
				   if (ds->x1 - s > dt.x1) de.x1 = ds->x1 - s;
				   errbox = TRUE;
				}
			     }
			     else if ((ds->y1 - dt.y2) > EPS && (ds->y1 - dt.y2) + EPS < s) {
				if (ds->x2 > (dt.x1 - s) && ds->x1 < (dt.x2 + s)) {
				   de.y1 = dt.y2;
				   de.y2 = ds->y1;
				   if (ds->x2 + s < dt.x2) de.x2 = ds->x2 + s;
				   if (ds->x1 - s > dt.x1) de.x1 = ds->x1 - s;
				   errbox = TRUE;
				}
			     }

			     if ((dt.x1 - ds->x2) > EPS && (dt.x1 - ds->x2) + EPS < s) {
				if (ds->y2 > (dt.y1 - s) && ds->y1 < (dt.y2 + s)) {
				   de.x2 = dt.x1;
				   de.x1 = ds->x2;
				   if (ds->y2 + s < dt.y2) de.y2 = ds->y2 + s;
				   if (ds->y1 - s > dt.y1) de.y1 = ds->y1 - s;
				   errbox = TRUE;
				}
			     }
			     else if ((ds->x1 - dt.x2) > EPS && (ds->x1 - dt.x2) + EPS < s) {
				if (ds->y2 > (dt.y1 - s) && ds->y1 < (dt.y2 + s)) {
				   de.x1 = dt.x2;
				   de.x2 = ds->x1;
				   if (ds->y2 + s < dt.y2) de.y2 = ds->y2 + s;
				   if (ds->y1 - s > dt.y1) de.y1 = ds->y1 - s;
				   errbox = TRUE;
				}
			     }

			     if (errbox == TRUE) {
	
			        // Chop areas off the error box that are covered by
			        // other taps of the same port.

			        for (ds2 = g->taps[i]; ds2; ds2 = ds2->next) {
				   if (ds2 == ds) continue;
				   if (ds2->layer != ds->layer) continue;

				   if (ds2->x1 <= de.x1 && ds2->x2 >= de.x2 &&
					ds2->y1 <= de.y1 && ds2->y2 >= de.y2) {
				      errbox = FALSE;	// Completely covered
				      break;
				   }

				   // Look for partial coverage.  Note that any
				   // change can cause a change in the original
				   // two conditionals, so we have to keep
				   // evaluating those conditionals.

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					 // if (ds2->x1 < de.x1 - EPS &&
					 if (ds2->x1 < de.x1 + EPS &&
							ds2->x2 < de.x2 - EPS) {
					    de.x1 = ds2->x2;
					    if (ds2->x2 >= ds->x2) errbox = FALSE;
					 }

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					 // if (ds2->x2 > de.x2 + EPS &&
					 if (ds2->x2 > de.x2 - EPS &&
							ds2->x1 > de.x1 + EPS) {
					    de.x2 = ds2->x1;
					    if (ds2->x1 <= ds->x1) errbox = FALSE;
					 }

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					 // if (ds2->y1 < de.y1 - EPS &&
					 if (ds2->y1 < de.y1 + EPS &&
							ds2->y2 < de.y2 - EPS) {
					    de.y1 = ds2->y2;
					    if (ds2->y2 >= ds->y2) errbox = FALSE;
					 }

				   if (ds2->x1 < de.x2 && ds2->x2 > de.x1)
				      if (ds2->y1 < de.y2 && ds2->y2 > de.y1)
					 // if (ds2->y2 > de.y2 + EPS &&
					 if (ds2->y2 > de.y2 - EPS &&
							ds2->y1 > de.y1 + EPS) {
					    de.y2 = ds2->y1;
					    if (ds2->y1 <= ds->y1) errbox = FALSE;
					 }
				}
			     }

			     // Any area left over is a potential DRC error.

			     if ((de.x2 <= de.x1) || (de.y2 <= de.y1))
				errbox = FALSE;
		
			     if (errbox == TRUE) {

				// Create stub route to cover error box, or
				// if possible, stretch existing stub route
				// to cover error box.

				// Allow EW stubs to be changed to NS stubs and
				// vice versa if the original stub length was less
				// than a route width.  This means the grid position
				// makes contact without the stub.  Moving the stub
				// to another side should not create an error.

				// NOTE:  error box must touch ds geometry, and by
				// more than just a point.

				// 8/31/2016:
				// If DRC violations are created on two adjacent
				// sides, then create both a stub route and a tap
				// offset.  Put the stub route in the preferred
				// metal direction of the layer, and set the tap
				// offset to prevent the DRC error in the other
				// direction.
				// 10/3/2016:  The tap offset can be set either
				// by moving toward the obstructing edge to
				// remove the gap, or moving away from it to
				// avoid the DRC spacing error.  Choose the one
				// that offsets by the smaller distance.

				if ((de.x2 > dt.x2) && (de.y1 < ds->y2) &&
						(de.y2 > ds->y1)) {
				   if ((orignet & STUBROUTE) == 0) {
			              OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				      lnode->stub = de.x2 - dx;
				      lnode->flags |= NI_STUB_EW;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_EW)) {
				      // Beware, if dist > 0 then this reverses
				      // the stub.  For U-shaped ports may need
				      // to have separate E and W stubs.
				      lnode->stub = de.x2 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_NS)) {

				      // If preferred route direction is
				      // horizontal, then change the stub

			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      if (LefGetRouteOrientation(ds->layer) == 1) {
					 lnode->flags = NI_OFFSET_NS | NI_STUB_EW;
					 if (lnode->stub > 0) {
					    lnode->offset = lnode->stub - wy;
					    if (lnode->offset < 0) lnode->offset = 0;
					 }
					 else {
					    lnode->offset = lnode->stub + wy;
					    if (lnode->offset > 0) lnode->offset = 0;
					 }
				         lnode->stub = de.x2 - dx;
				         errbox = FALSE;
				      }
				      else {
					 // Add the offset
				         lnode->offset = de.x2 - dx - wx;
					 if (lnode->offset > s - lnode->offset)
					     lnode->offset -= s;
					 lnode->flags |= NI_OFFSET_EW;
				         errbox = FALSE;
				      }
				   }
				}
				else if ((de.x1 < dt.x1) && (de.y1 < ds->y2) &&
						(de.y2 > ds->y1)) {
				   if ((orignet & STUBROUTE) == 0) {
			              OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				      lnode->stub = de.x1 - dx;
				      lnode->flags |= NI_STUB_EW;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_EW)) {
				      // Beware, if dist > 0 then this reverses
				      // the stub.  For U-shaped ports may need
				      // to have separate E and W stubs.
				      lnode->stub = de.x1 - dx;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_NS)) {

				      // If preferred route direction is
				      // horizontal, then change the stub

			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      if (LefGetRouteOrientation(ds->layer) == 1) {
					 lnode->flags = NI_OFFSET_NS | NI_STUB_EW;
					 if (lnode->stub > 0) {
					    lnode->offset = lnode->stub - wy;
					    if (lnode->offset < 0) lnode->offset = 0;
					 }
					 else {
					    lnode->offset = lnode->stub + wy;
					    if (lnode->offset > 0) lnode->offset = 0;
					 }
				         lnode->stub = de.x1 - dx;
				         errbox = FALSE;
				      }
				      else {
					 // Add the offset
				         lnode->offset = de.x1 - dx + wx;
					 if (-lnode->offset > s + lnode->offset)
					     lnode->offset += s;
					 lnode->flags |= NI_OFFSET_EW;
				         errbox = FALSE;
				      }
				   }
				}
				else if ((de.y2 > dt.y2) && (de.x1 < ds->x2) &&
					(de.x2 > ds->x1)) {
				   if ((orignet & STUBROUTE) == 0) {
			              OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				      lnode->stub = de.y2 - dy;
				      lnode->flags |= NI_STUB_NS;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_NS)) {
				      // Beware, if dist > 0 then this reverses
				      // the stub.  For C-shaped ports may need
				      // to have separate N and S stubs.
				      lnode->stub = de.y2 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_EW)) {

				      // If preferred route direction is
				      // vertical, then change the stub

			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      if (LefGetRouteOrientation(ds->layer) == 0) {
					 lnode->flags = NI_OFFSET_EW | NI_STUB_NS;
					 if (lnode->stub > 0) {
					    lnode->offset = lnode->stub - wx;
					    if (lnode->offset < 0) lnode->offset = 0;
					 }
					 else {
					    lnode->offset = lnode->stub + wx;
					    if (lnode->offset > 0) lnode->offset = 0;
					 }
				         lnode->stub = de.y2 - dy;
				         errbox = FALSE;
				      }
				      else {
					 // Add the offset
				         lnode->offset = de.y2 - dy - wy;
					 if (lnode->offset > s - lnode->offset)
					     lnode->offset -= s;
					 lnode->flags |= NI_OFFSET_NS;
				         errbox = FALSE;
				      }
				   }
				}
				else if ((de.y1 < dt.y1) && (de.x1 < ds->x2) &&
					(de.x2 > ds->x1)) {
				   if ((orignet & STUBROUTE) == 0) {
			              OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				      lnode->stub = de.y1 - dy;
				      lnode->flags |= NI_STUB_NS;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_NS)) {
				      // Beware, if dist > 0 then this reverses
				      // the stub.  For C-shaped ports may need
				      // to have separate N and S stubs.
				      lnode->stub = de.y1 - dy;
				      errbox = FALSE;
				   }
				   else if ((orignet & STUBROUTE)
						&& (lnode->flags & NI_STUB_EW)) {

				      // If preferred route direction is
				      // vertical, then change the stub

			              OBSVAL(gridx, gridy, ds->layer) |= OFFSET_TAP;
				      if (LefGetRouteOrientation(ds->layer) == 0) {
					 lnode->flags = NI_OFFSET_EW | NI_STUB_NS;
					 if (lnode->stub > 0) {
					    lnode->offset = lnode->stub - wx;
					    if (lnode->offset < 0) lnode->offset = 0;
					 }
					 else {
					    lnode->offset = lnode->stub + wx;
					    if (lnode->offset > 0) lnode->offset = 0;
					 }
				         lnode->stub = de.y1 - dy + wy;
				         errbox = FALSE;
				      }
				      else {
					 // Add the offset
				         lnode->offset = de.y1 - dy + wy;
					 if (-lnode->offset > s + lnode->offset)
					     lnode->offset += s;
					 lnode->flags |= NI_OFFSET_NS;
				         errbox = FALSE;
				      }
				   }
				}

				// Where the error box did not touch the stub
				// route, there is assumed to be no error.

				if (errbox == TRUE)
				   if ((de.x2 > dt.x2) || (de.x1 < dt.x1) ||
					(de.y2 > dt.y2) || (de.y1 < dt.y1))
				      errbox = FALSE;

				if (errbox == TRUE) {
				   // Unroutable position, so mark it unroutable
			           OBSVAL(gridx, gridy, ds->layer) |= STUBROUTE;
				   lnode->flags |= NI_STUB_MASK;
				}
			     }
		         }
		         gridy++;
		      }
		   }
		   gridx++;
		}
	     }
	  }
       }
    }

} /* void adjust_stub_lengths() */

/*--------------------------------------------------------------*/
/* block_route()						*/
/*								*/
/*  Mark a specific length along the route tracks as unroutable	*/
/*  by finding the grid point in the direction indicated, and	*/
/*  setting the appropriate block bit in the Obs[] array for	*/
/*  that position.  The original grid point is marked as	*/
/*  unroutable in the opposite direction, for symmetry.		*/
/*--------------------------------------------------------------*/

void
block_route(int x, int y, int lay, u_char dir)
{
   int bx, by, bl, ob;

   bx = x;
   by = y;
   bl = lay;

   switch (dir) {
      case NORTH:
	 if (y == NumChannelsY - 1) return;
	 by = y + 1;
	 break;
      case SOUTH:
	 if (y == 0) return;
	 by = y - 1;
	 break;
      case EAST:
	 if (x == NumChannelsX - 1) return;
	 bx = x + 1;
	 break;
      case WEST:
	 if (x == 0) return;
	 bx = x - 1;
	 break;
      case UP:
	 if (lay == Num_layers - 1) return;
	 bl = lay + 1;
	 break;
      case DOWN:
	 if (lay == 0) return;
	 bl = lay - 1;
	 break;
   }
   
   ob = OBSVAL(bx, by, bl);

   if ((ob & NO_NET) != 0) return;

   switch (dir) {
      case NORTH:
	 OBSVAL(bx, by, bl) |= BLOCKED_S;
	 OBSVAL(x, y, lay) |= BLOCKED_N;
	 break;
      case SOUTH:
	 OBSVAL(bx, by, bl) |= BLOCKED_N;
	 OBSVAL(x, y, lay) |= BLOCKED_S;
	 break;
      case EAST:
	 OBSVAL(bx, by, bl) |= BLOCKED_W;
	 OBSVAL(x, y, lay) |= BLOCKED_E;
	 break;
      case WEST:
	 OBSVAL(bx, by, bl) |= BLOCKED_E;
	 OBSVAL(x, y, lay) |= BLOCKED_W;
	 break;
      case UP:
	 OBSVAL(bx, by, bl) |= BLOCKED_D;
	 OBSVAL(x, y, lay) |= BLOCKED_U;
	 break;
      case DOWN:
	 OBSVAL(bx, by, bl) |= BLOCKED_U;
	 OBSVAL(x, y, lay) |= BLOCKED_D;
	 break;
   }
}

/*--------------------------------------------------------------*/
/* find_route_blocks() ---					*/
/*								*/
/*	Search tap geometry for edges that cause DRC spacing	*/
/*	errors with route edges.  This specifically checks	*/
/*	edges of the route tracks, not the intersection points.	*/
/*	If a tap would cause an error with a route segment,	*/
/*	the grid points on either end of the segment are	*/
/*	flagged to prevent generating a route along that	*/
/*	specific segment.					*/
/*--------------------------------------------------------------*/

void
find_route_blocks()
{
   GATE g;
   NODEINFO lnode;
   DSEG ds;
   struct dseg_ dt, lds;
   int i, gridx, gridy;
   double dx, dy, w, v, s, u;
   double dist;
   int orient = 0;	/* Need to check orient = 2! */

   for (g = Nlgates; g; g = g->next) {
      for (i = 0; i < g->nodes; i++) {
	 if (g->netnum[i] != 0) {

	    // Work through each rectangle in the tap geometry

            for (ds = g->taps[i]; ds; ds = ds->next) {
	       lds = *ds;	/* Make local copy of tap rect */

	       /* Trim to array bounds and reject if out-of-bounds */
	       gridx = (int)((lds.x1 - Xlowerbound) / PitchX);
	       if (gridx >= NumChannelsX) continue;
	       if (gridx < 0) lds.x1 = Xlowerbound;

	       gridx = (int)((lds.x2 - Xlowerbound) / PitchX);
	       if (gridx < 0) continue;
	       if (gridx >= NumChannelsX)
		   lds.x2 = Xlowerbound + (NumChannelsX * PitchX);
	       
	       gridy = (int)((lds.y1 - Ylowerbound) / PitchY);
	       if (gridy >= NumChannelsY) continue;
	       if (gridy < 0) lds.y1 = Ylowerbound;

	       gridy = (int)((lds.y2 - Ylowerbound) / PitchY);
	       if (gridy < 0) continue;
	       if (gridy >= NumChannelsY)
		   lds.y2 = Ylowerbound + (NumChannelsY * PitchY);
	       
	       w = 0.5 * LefGetRouteWidth(lds.layer);
	       v = 0.5 * LefGetXYViaWidth(lds.layer, lds.layer, 0, orient);
	       s = LefGetRouteSpacing(lds.layer);

	       // Look west

	       gridx = (int)((lds.x1 - Xlowerbound) / PitchX);
	       dx = (gridx * PitchX) + Xlowerbound;
	       dist = lds.x1 - dx - w;
	       if (dist > 0 && dist < s && gridx >= 0) {
		  dt.x1 = dt.x2 = dx;
		  dt.y1 = lds.y1;
		  dt.y2 = lds.y2;

		  // Check for other taps covering this edge
		  // (to do)

		  // Find all grid points affected
	          gridy = (int)((lds.y1 - Ylowerbound - PitchY) / PitchY);
	          dy = (gridy * PitchY) + Ylowerbound;
		  while (dy < lds.y1 - s) {
		     dy += PitchY;
		     gridy++;
		  }
		  while (dy < lds.y2 + s) {
		     lnode = NODEIPTR(gridx, gridy, lds.layer);
		     u = ((OBSVAL(gridx, gridy, lds.layer) & STUBROUTE)
				&& (lnode->flags & NI_STUB_EW)) ? v : w;
		     if (dy + EPS < lds.y2 - u)
			block_route(gridx, gridy, lds.layer, NORTH);
		     if (dy - EPS > lds.y1 + u)
			block_route(gridx, gridy, lds.layer, SOUTH);
		     dy += PitchY;
		     gridy++;
		  }
	       }

	       // Look east

	       gridx = (int)(1.0 + (lds.x2 - Xlowerbound) / PitchX);
	       dx = (gridx * PitchX) + Xlowerbound;
	       dist = dx - lds.x2 - w;
	       if (dist > 0 && dist < s && gridx < NumChannelsX) {
		  dt.x1 = dt.x2 = dx;
		  dt.y1 = lds.y1;
		  dt.y2 = lds.y2;

		  // Check for other taps covering this edge
		  // (to do)

		  // Find all grid points affected
	          gridy = (int)((lds.y1 - Ylowerbound - PitchY) / PitchY);
	          dy = (gridy * PitchY) + Ylowerbound;
		  while (dy < lds.y1 - s) {
		     dy += PitchY;
		     gridy++;
		  }
		  while (dy < lds.y2 + s) {
		     lnode = NODEIPTR(gridx, gridy, lds.layer);
		     u = ((OBSVAL(gridx, gridy, lds.layer) & STUBROUTE)
				&& (lnode->flags & NI_STUB_EW)) ? v : w;
		     if (dy + EPS < lds.y2 - u)
			block_route(gridx, gridy, lds.layer, NORTH);
		     if (dy - EPS > lds.y1 + u)
			block_route(gridx, gridy, lds.layer, SOUTH);
		     dy += PitchY;
		     gridy++;
		  }
	       }

	       // Look south

	       gridy = (int)((lds.y1 - Ylowerbound) / PitchY);
	       dy = (gridy * PitchY) + Ylowerbound;
	       dist = lds.y1 - dy - w;
	       if (dist > 0 && dist < s && gridy >= 0) {
		  dt.x1 = lds.x1;
		  dt.x2 = lds.x2;
		  dt.y1 = dt.y2 = dy;

		  // Check for other taps covering this edge
		  // (to do)

		  // Find all grid points affected
	          gridx = (int)((lds.x1 - Xlowerbound - PitchX) / PitchX);
	          dx = (gridx * PitchX) + Xlowerbound;
		  while (dx < lds.x1 - s) {
		     dx += PitchX;
		     gridx++;
		  }
		  while (dx < lds.x2 + s) {
		     lnode = NODEIPTR(gridx, gridy, lds.layer);
		     u = ((OBSVAL(gridx, gridy, lds.layer) & STUBROUTE)
				&& (lnode->flags & NI_STUB_NS)) ? v : w;
		     if (dx + EPS < lds.x2 - u)
			block_route(gridx, gridy, lds.layer, EAST);
		     if (dx - EPS > lds.x1 + u)
			block_route(gridx, gridy, lds.layer, WEST);
		     dx += PitchX;
		     gridx++;
		  }
	       }

	       // Look north

	       gridy = (int)(1.0 + (lds.y2 - Ylowerbound) / PitchY);
	       dy = (gridy * PitchY) + Ylowerbound;
	       dist = dy - lds.y2 - w;
	       if (dist > 0 && dist < s && gridy < NumChannelsY) {
		  dt.x1 = lds.x1;
		  dt.x2 = lds.x2;
		  dt.y1 = dt.y2 = dy;

		  // Check for other taps covering this edge
		  // (to do)

		  // Find all grid points affected
	          gridx = (int)((lds.x1 - Xlowerbound - PitchX) / PitchX);
	          dx = (gridx * PitchX) + Xlowerbound;
		  while (dx < lds.x1 - s) {
		     dx += PitchX;
		     gridx++;
		  }
		  while (dx < lds.x2 + s) {
		     lnode = NODEIPTR(gridx, gridy, lds.layer);
		     u = ((OBSVAL(gridx, gridy, lds.layer) & STUBROUTE)
				&& (lnode->flags & NI_STUB_NS)) ? v : w;
		     if (dx + EPS < lds.x2 - u)
			block_route(gridx, gridy, lds.layer, EAST);
		     if (dx - EPS > lds.x1 + u)
			block_route(gridx, gridy, lds.layer, WEST);
		     dx += PitchX;
		     gridx++;
		  }
	       }
	    }
	 }
      }
   }
}

/* node.c */
