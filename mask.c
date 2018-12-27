/*--------------------------------------------------------------*/
/*  mask.c -- qrouter general purpose autorouter                */
/*  Route mask generation					*/
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

u_char   *RMask;    	        // mask out best area to route

/*--------------------------------------------------------------*/
/* Comparison routine used for qsort.  Sort nets by number of	*/
/* nodes.							*/
/*--------------------------------------------------------------*/

int compNets(NET *a, NET *b)
{
   NET p = *a;
   NET q = *b;

   // NULL nets get shoved up front
   if (p == NULL) return ((q == NULL) ? 0 : -1);
   if (q == NULL) return 1;

   // Sort critical nets at the front by assigned order

   if ((p->flags & NET_CRITICAL) || (q->flags & NET_CRITICAL)) {
      if (!(p->flags & NET_CRITICAL)) return 1;
      else if (!(q->flags & NET_CRITICAL)) return -1;
      else return (p->netorder < q->netorder) ? -1 : 1;
   }

   // Otherwise sort by number of nodes

   if (p->numnodes < q->numnodes)
      return 1;
   if (p->numnodes > q->numnodes)
      return -1;
   return 0;
}

/*--------------------------------------------------------------*/
/* Alternative net comparison used for qsort.  Sort nets by	*/
/* minimum dimension of the bounding box, and if equal, by the	*/
/* number of nodes in the net.  Bounding box dimensions are	*/
/* ordered smallest to largest, and number of nodes are ordered	*/
/* largest to smallest.						*/
/*--------------------------------------------------------------*/

int altCompNets(NET *a, NET *b)
{
   NET p = *a;
   NET q = *b;

   int pwidth, qwidth, pheight, qheight, pdim, qdim;

   // Any NULL nets get shoved up front
   if (p == NULL) return ((q == NULL) ? 0 : -1);
   if (q == NULL) return 1;

   // Sort critical nets at the front by assigned order

   if ((p->flags & NET_CRITICAL) || (q->flags & NET_CRITICAL)) {
      if (!(p->flags & NET_CRITICAL)) return 1;
      else if (!(q->flags & NET_CRITICAL)) return -1;
      else return (p->netorder < q->netorder) ? -1 : 1;
   }


   // Otherwise sort as described above.

   pwidth = p->xmax - p->xmin;
   pheight = p->ymax - p->ymin;
   pdim = (pwidth > pheight) ? pheight : pwidth;

   qwidth = q->xmax - q->xmin;
   qheight = q->ymax - q->ymin;
   qdim = (qwidth > qheight) ? qheight : qwidth;

   if (pdim < qdim)
      return (-1);
   else if (pdim > qdim)
      return (1);
   else {
      if (p->numnodes < q->numnodes)
         return (1);
      if (p->numnodes > q->numnodes)
         return (-1);
      return (0);
   }
}

/*--------------------------------------------------------------*/
/* create_netorder --- assign indexes to net->netorder    	*/
/* Re-sort Nlnets according to net order.  Since Nlnets is a	*/
/* global variable, nothing is returned from this routine.	*/
/*								*/
/* method = 0							*/
/* 	Nets are ordered simply from those with the most nodes	*/
/*	to those with the fewest.  However, any nets marked	*/
/* 	critical in the configuration or critical net files	*/
/*	will be given precedence.				*/
/*								*/
/* method = 1							*/
/*	Nets are ordered by minimum bounding box dimension.	*/
/*	This is based on the principle that small or narrow	*/
/*	nets have little room to be moved around without	*/
/*	greatly increasing the net length.  If these are put	*/
/*	down first, then remaining nets can route around them.	*/
/*--------------------------------------------------------------*/

void create_netorder(u_char method)
{
  int i, j;
  NET  net;
  STRING cn;

  i = 1;
  for (cn = CriticalNet; cn; cn = cn->next) {
     if (Verbose > 1)
	Fprintf(stdout, "critical net %s\n", cn->name);
     net = DefFindNet(cn->name);
     if (net) {
        net->netorder = i++;
	net->flags |= NET_CRITICAL;
     }
  }

  switch (method) {
      case 0:
	 qsort((char *)Nlnets, Numnets, (int)sizeof(NET),
			(__compar_fn_t)compNets);
	 break;
      case 1:
	 qsort((char *)Nlnets, Numnets, (int)sizeof(NET),
			(__compar_fn_t)altCompNets);
	 break;
  }

  for (i = 0; i < Numnets; i++) {
     net = Nlnets[i];
     net->netorder = i++;
  }

} /* create_netorder() */

/*--------------------------------------------------------------*/
/* Measure and record the bounding box of a net.		*/
/* This is preparatory to generating a mask for the net.	*/
/* Find the bounding box of each node, and record that		*/
/* information, at the same time computing the whole net's	*/
/* bounding box as the area bounding all of the nodes.		*/
/* Determine if the bounding box is more horizontal or		*/
/* vertical, and specify a direction for the net's trunk line.	*/
/* Initialize the trunk line as the midpoint between all of the	*/
/* nodes, extending the width (or height) of the bounding box.	*/
/* Initialize the node branch position as the line extending	*/
/* from the middle of the node's bounding box to the trunk	*/
/* line.  These positions (trunk and branches) will be sorted	*/
/* and readjusted by "create_nodeorder()".			*/
/*--------------------------------------------------------------*/

void find_bounding_box(NET net)
{
   NODE n1, n2;
   DPOINT d1tap, d2tap, dtap, mintap;
   int mindist, dist, dx, dy;

   if (net->numnodes == 2) {

      n1 = (NODE)net->netnodes;
      n2 = (NODE)net->netnodes->next;

      // Simple 2-pass---pick up first tap on n1, find closest tap on n2,
      // then find closest tap on n1.

      d1tap = (n1->taps == NULL) ? n1->extend : n1->taps;
      if (d1tap == NULL) return;
      d2tap = (n2->taps == NULL) ? n2->extend : n2->taps;
      if (d2tap == NULL) return;
      dx = d2tap->gridx - d1tap->gridx;
      dy = d2tap->gridy - d1tap->gridy;
      mindist = dx * dx + dy * dy;
      mintap = d2tap;
      for (d2tap = d2tap->next; d2tap != NULL; d2tap = d2tap->next) {
         dx = d2tap->gridx - d1tap->gridx;
         dy = d2tap->gridy - d1tap->gridy;
         dist = dx * dx + dy * dy;
         if (dist < mindist) {
            mindist = dist;
            mintap = d2tap;
         }
      }
      d2tap = mintap;
      d1tap = (n1->taps == NULL) ? n1->extend : n1->taps;
      dx = d2tap->gridx - d1tap->gridx;
      dy = d2tap->gridy - d1tap->gridy;
      mindist = dx * dx + dy * dy;
      mintap = d1tap;
      for (d1tap = d1tap->next; d1tap != NULL; d1tap = d1tap->next) {
         dx = d2tap->gridx - d1tap->gridx;
         dy = d2tap->gridy - d1tap->gridy;
         dist = dx * dx + dy * dy;
         if (dist < mindist) {
            mindist = dist;
            mintap = d1tap;
         }
      }
      d1tap = mintap;

      net->xmin = (d1tap->gridx < d2tap->gridx) ? d1tap->gridx : d2tap->gridx;
      net->xmax = (d1tap->gridx < d2tap->gridx) ? d2tap->gridx : d1tap->gridx;
      net->ymin = (d1tap->gridy < d2tap->gridy) ? d1tap->gridy : d2tap->gridy;
      net->ymax = (d1tap->gridy < d2tap->gridy) ? d2tap->gridy : d1tap->gridy;
   }
   else {	// Net with more than 2 nodes

      // Use the first tap point for each node to get a rough bounding box and
      // centroid of all taps
      net->xmax = net->ymax = -(MAXRT);
      net->xmin = net->ymin = MAXRT;
      for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
         dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	 if (dtap) {
            if (dtap->gridx > net->xmax) net->xmax = dtap->gridx;
            if (dtap->gridx < net->xmin) net->xmin = dtap->gridx;
            if (dtap->gridy > net->ymax) net->ymax = dtap->gridy;
            if (dtap->gridy < net->ymin) net->ymin = dtap->gridy;
	 }
      }
   }
}

/*--------------------------------------------------------------*/
/* defineRouteTree() ---					*/
/*								*/
/* Define a trunk-and-branches potential best route for a net.	*/
/*								*/
/* The net is analyzed for aspect ratio, and is determined if	*/
/* it will have a horizontal or vertical trunk.  Then, each	*/
/* node will define a branch line extending from the node	*/
/* position to the trunk.  Trunk position is recorded in the	*/
/* net record, and branch positions are recorded in the	node	*/
/* records.							*/
/*								*/
/* To do:							*/
/* Trunk and branch lines will be analyzed for immediate	*/
/* collisions and sorted to help ensure a free track exists for	*/
/* each net's trunk line.					*/
/*--------------------------------------------------------------*/

void defineRouteTree(NET net)
{
    NODE n1;
    DPOINT dtap;
    int xcent, ycent, xmin, ymin, xmax, ymax;

    // This is called after create_bounding_box(), so bounds have
    // been calculated.

    xmin = net->xmin;
    xmax = net->xmax;
    ymin = net->ymin;
    ymax = net->ymax;

    if (net->numnodes == 2) {

	// For 2-node nets, record the initial position as
	// one horizontal trunk + one branch for one "L" of
	// the bounding box, and one vertical trunk + one
	// branch for the other "L" of the bounding box.

	net->trunkx = xmin;
	net->trunky = ymin;
    }
    else if (net->numnodes > 0) {

	// Use the first tap point for each node to get a rough
	// centroid of all taps

	xcent = ycent = 0;
	for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
	    dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	    if (dtap == NULL) continue;
	    xcent += dtap->gridx;
	    ycent += dtap->gridy;
	}
	xcent /= net->numnodes;
	ycent /= net->numnodes;

	// Record the trunk line in the net record

	net->trunkx = xcent;
	net->trunky = ycent;
    }

    if (xmax - xmin > ymax - ymin) {
	// Horizontal trunk preferred
	net->flags &= ~NET_VERTICAL_TRUNK;
    }
    else {
	// Vertical trunk preferred
	net->flags |= NET_VERTICAL_TRUNK;
    }

    // Set the branch line positions to the node tap points

    for (n1 = net->netnodes; n1; n1 = n1->next) {
	dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
	if (!dtap) continue;
	n1->branchx = dtap->gridx;
	n1->branchy = dtap->gridy;
    }
}

/*--------------------------------------------------------------*/
/* initMask() ---						*/
/*--------------------------------------------------------------*/

void initMask(void)
{
   RMask = (u_char *)calloc(NumChannelsX * NumChannelsY,
			sizeof(u_char));
   if (!RMask) {
      fprintf(stderr, "Out of memory 3.\n");
      exit(3);
   }
}

/*--------------------------------------------------------------*/
/* Fill mask around the area of a vertical line			*/
/*--------------------------------------------------------------*/

void
create_vbranch_mask(int x, int y1, int y2, u_char slack, u_char halo)
{
   int gx1, gx2, gy1, gy2;
   int i, j, v;
   u_char m;

   gx1 = x - slack;
   gx2 = x + slack;
   if (y1 > y2) {
      gy1 = y2 - slack;
      gy2 = y1 + slack;
   }
   else {
      gy1 = y1 - slack;
      gy2 = y2 + slack;
   }
   if (gx1 < 0) gx1 = 0;
   if (gx2 >= NumChannelsX) gx2 = NumChannelsX - 1;
   if (gy1 < 0) gy1 = 0;
   if (gy2 >= NumChannelsY) gy2 = NumChannelsY - 1;

   for (i = gx1; i <= gx2; i++)
      for (j = gy1; j <= gy2; j++)
	 RMASK(i, j) = (u_char)0;

   for (v = 1; v < halo; v++) {
      if (gx1 > 0) gx1--;
      if (gx2 < NumChannelsX - 1) gx2++;
      if (y1 > y2) {
         if (gy1 < NumChannelsY - 1) gy1++;
         if (gy2 < NumChannelsY - 1) gy2++;
      }
      else {
	 if (gy1 > 0) gy1--;
	 if (gy2 > 0) gy2--;
      }
      for (i = gx1; i <= gx2; i++)
         for (j = gy1; j <= gy2; j++) {
	    m = RMASK(i, j);
	    if (m > v) RMASK(i, j) = (u_char)v;
	 }
   }
}

/*--------------------------------------------------------------*/
/* Fill mask around the area of a horizontal line		*/
/*--------------------------------------------------------------*/

void
create_hbranch_mask(int y, int x1, int x2, u_char slack, u_char halo)
{
   int gx1, gx2, gy1, gy2;
   int i, j, v;
   u_char m;

   gy1 = y - slack;
   gy2 = y + slack;
   if (x1 > x2) {
      gx1 = x2 - slack;
      gx2 = x1 + slack;
   }
   else {
      gx1 = x1 - slack;
      gx2 = x2 + slack;
   }
   if (gx1 < 0) gx1 = 0;
   if (gx2 >= NumChannelsX) gx2 = NumChannelsX - 1;
   if (gy1 < 0) gy1 = 0;
   if (gy2 >= NumChannelsY) gy2 = NumChannelsY - 1;

   for (i = gx1; i <= gx2; i++)
      for (j = gy1; j <= gy2; j++)
	 RMASK(i, j) = (u_char)0;

   for (v = 1; v < halo; v++) {
      if (gy1 > 0) gy1--;
      if (gy2 < NumChannelsY - 1) gy2++;
      if (x1 > x2) {
         if (gx1 < NumChannelsX - 1) gx1++;
         if (gx2 < NumChannelsX - 1) gx2++;
      }
      else {
	 if (gx1 > 0) gx1--;
	 if (gx2 > 0) gx2--;
      }
      for (i = gx1; i <= gx2; i++)
         for (j = gy1; j <= gy2; j++) {
	    m = RMASK(i, j);
	    if (m > v) RMASK(i, j) = (u_char)v;
	 }
   }
}

/*--------------------------------------------------------------*/
/* setBboxCurrent() ---						*/
/*								*/
/* Alter the net's bounding box information to include the	*/
/* existing bounding box around all net route segments.  This	*/
/* allows stage 3 routing to minimize the search area.		*/
/*								*/
/*--------------------------------------------------------------*/

void setBboxCurrent(NET net)
{
    ROUTE rt;
    SEG seg;

    // If net is routed, increase the bounding box to
    // include the current route solution.

    for (rt = net->routes; rt; rt = rt->next)
	for (seg = rt->segments; seg; seg = seg->next)
	{
	    if (seg->x1 < net->xmin) net->xmin = seg->x1;
	    else if (seg->x1 > net->xmax) net->xmax = seg->x1;

	    if (seg->x2 < net->xmin) net->xmin = seg->x2;
	    else if (seg->x2 > net->xmax) net->xmax = seg->x2;

	    if (seg->y1 < net->ymin) net->ymin = seg->y1;
	    else if (seg->y1 > net->ymax) net->ymax = seg->y1;

	    if (seg->y2 < net->ymin) net->ymin = seg->y2;
	    else if (seg->y2 > net->ymax) net->ymax = seg->y2;
	}
}

/*--------------------------------------------------------------*/
/* createBboxMask() ---						*/
/*								*/
/* Create mask limiting the area to search for routing		*/
/*								*/
/* The bounding box mask generates an area including the	*/
/* bounding box as defined in the net record, includes all pin	*/
/* positions in the mask, and increases the mask area by one	*/
/* route track for each pass, up to "halo".			*/
/*--------------------------------------------------------------*/

void createBboxMask(NET net, u_char halo)
{
    int xmin, ymin, xmax, ymax;
    int i, j, gx1, gy1, gx2, gy2;

    fillMask((u_char)halo);

    xmin = net->xmin;
    xmax = net->xmax;
    ymin = net->ymin;
    ymax = net->ymax;

    for (gx1 = xmin; gx1 <= xmax; gx1++)
	for (gy1 = ymin; gy1 <= ymax; gy1++)
	    RMASK(gx1, gy1) = (u_char)0;

    for (i = 1; i <= halo; i++) {
	gx1 = xmin - i;
	if (gx1 >= 0 && gx1 < NumChannelsX)
           for (j = ymin - i; j <= ymax + i; j++)
	      if (j >= 0 && j < NumChannelsY)
		 RMASK(gx1, j) = (u_char)i;

	gx2 = xmax + i;
	if (gx2 >= 0 && gx2 < NumChannelsX)
           for (j = ymin - i; j <= ymax + i; j++)
	      if (j >= 0 && j < NumChannelsY)
		 RMASK(gx2, j) = (u_char)i;

	gy1 = ymin - i;
	if (gy1 >= 0 && gy1 < NumChannelsY)
           for (j = xmin - i; j <= xmax + i; j++)
	      if (j >= 0 && j < NumChannelsX)
		 RMASK(j, gy1) = (u_char)i;

	gy2 = ymax + i;
	if (gy2 >= 0 && gy2 < NumChannelsY)
           for (j = xmin - i; j <= xmax + i; j++)
	      if (j >= 0 && j < NumChannelsX)
		 RMASK(j, gy2) = (u_char)i;
     }
}

/*--------------------------------------------------------------*/
/* analyzeCongestion() ---					*/
/*								*/
/* Given a trunk route at ycent, between ymin and ymax, score	*/
/* the neighboring positions as a function of congestion and	*/
/* offset from the ideal location.  Return the position of the	*/
/* best location for the trunk route.				*/
/*--------------------------------------------------------------*/

int analyzeCongestion(int ycent, int ymin, int ymax, int xmin, int xmax)
{
    int x, y, i, minidx = -1, sidx, n;
    int *score, minscore;

    score = (int *)malloc((ymax - ymin + 1) * sizeof(int));

    for (y = ymin; y <= ymax; y++) {
	sidx = y - ymin;
	score[sidx] = ABSDIFF(ycent, y) * Num_layers;
	for (x = xmin; x <= xmax; x++) {
	    for (i = 0; i < Num_layers; i++) {
		n = OBSVAL(x, y, i);
		if (n & ROUTED_NET) score[sidx]++;
		if (n & NO_NET) score[sidx]++;
		if (n & PINOBSTRUCTMASK) score[sidx]++;
	    }
	}
    }
    minscore = MAXRT;
    for (i = 0; i < (ymax - ymin + 1); i++) {
	if (score[i] < minscore) {
	    minscore = score[i];
	    minidx = i + ymin;
	}
    }

    free(score);
    return minidx;
}

/*--------------------------------------------------------------*/
/* createMask() ---						*/
/*								*/
/* Create mask limiting the area to search for routing		*/
/*								*/
/* For 2-node routes, find the two L-shaped routes between the	*/
/* two closest points of the nodes.				*/
/* For multi-node (>2) routes, find the best trunk line that	*/
/* passes close to all nodes, and generate stems to the closest	*/
/* point on each node.						*/
/*								*/
/* Optimizations:  (1) multi-node routes that are in a small	*/
/* enough area, just mask the bounding box.  (2) Where nodes	*/
/* at the end of two branches are closer to each other than to	*/
/* the trunk, mask an additional cross-connection between the	*/
/* two branches.						*/
/*								*/
/* Values are "halo" where there is no mask, 0 on the		*/
/* closest "slack" routes to the ideal (typically 1), and	*/
/* values increasing out to a distance of "halo" tracks away	*/
/* from the ideal.  This allows a greater search area as the	*/
/* number of passes of the search algorithm increases.		*/
/*								*/
/* To do:  Choose the position of trunk line based on		*/
/* congestion analysis.						*/
/*--------------------------------------------------------------*/

void createMask(NET net, u_char slack, u_char halo)
{
  NODE n1, n2;
  DPOINT dtap;
  int i, j, orient;
  int dx, dy, gx1, gx2, gy1, gy2;
  int xcent, ycent, xmin, ymin, xmax, ymax;
  int oxmin, oymin, oxmax, oymax;

  fillMask((u_char)halo);

  oxmin = net->xmin;
  oxmax = net->xmax;
  oymin = net->ymin;
  oymax = net->ymax;

  xcent = net->trunkx;
  ycent = net->trunky;

  orient = 0;

  // Construct the trunk line mask

  if (!(net->flags & NET_VERTICAL_TRUNK) || (net->numnodes == 2)) {
     // Horizontal trunk
     orient |= 1;

     ycent = analyzeCongestion(net->trunky, oymin, oymax, oxmin, oxmax);
     ymin = ymax = ycent;
     xmin = oxmin;
     xmax = oxmax;

     // Avoid faulting if the min/max values were never set
     if (xmin > xmax) {
	xmin = 0;
	xmax = NumChannelsX - 1;
     }

     for (i = xmin - slack; i <= xmax + slack; i++) {
	if (i < 0 || i >= NumChannelsX) continue;
	for (j = ycent - slack; j <= ycent + slack; j++) {
	   if (j < 0 || j >= NumChannelsY) continue;
	   RMASK(i, j) = (u_char)0;
	}
     }

     for (i = 1; i < halo; i++) {
	gy1 = ycent - slack - i;
	gy2 = ycent + slack + i;
        for (j = xmin - slack - i; j <= xmax + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsX) continue;
	   if (gy1 >= 0)
	      RMASK(j, gy1) = (u_char)i;
	   if (gy2 < NumChannelsY)
	      RMASK(j, gy2) = (u_char)i;
	}
	gx1 = xmin - slack - i;
	gx2 = xmax + slack + i;
        for (j = ycent - slack - i; j <= ycent + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsY) continue;
	   if (gx1 >= 0)
	      RMASK(gx1, j) = (u_char)i;
	   if (gx2 < NumChannelsX)
	      RMASK(gx2, j) = (u_char)i;
	}
     }
  }
  if ((net->flags & NET_VERTICAL_TRUNK) || (net->numnodes == 2)) {
     // Vertical trunk
     orient |= 2;
     xmin = xmax = xcent;
     ymin = oymin;
     ymax = oymax;

     // Avoid faulting if the min/max values were never set
     if (ymin > ymax) {
	ymin = 0;
	ymax = NumChannelsY - 1;
     }

     for (i = xcent - slack; i <= xcent + slack; i++) {
	if (i < 0 || i >= NumChannelsX) continue;
	for (j = ymin - slack; j <= ymax + slack; j++) {
	   if (j < 0 || j >= NumChannelsY) continue;
	   RMASK(i, j) = (u_char)0;
	}
     }

     for (i = 1; i < halo; i++) {
	gx1 = xcent - slack - i;
	gx2 = xcent + slack + i;
        for (j = ymin - slack - i; j <= ymax + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsY) continue;
	   if (gx1 >= 0)
	      RMASK(gx1, j) = (u_char)i;
	   if (gx2 < NumChannelsX)
	      RMASK(gx2, j) = (u_char)i;
	}
	gy1 = ymin - slack - i;
	gy2 = ymax + slack + i;
        for (j = xcent - slack - i; j <= xcent + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsX) continue;
	   if (gy1 >= 0)
	      RMASK(j, gy1) = (u_char)i;
	   if (gy2 < NumChannelsY)
	      RMASK(j, gy2) = (u_char)i;
	}
     }
  }
     
  // Construct the branch line masks

  for (n1 = net->netnodes; n1; n1 = n1->next) {
     dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
     if (!dtap) continue;

     if (orient & 1) 	// Horizontal trunk, vertical branches
	create_vbranch_mask(n1->branchx, n1->branchy, ycent, slack, halo);
     if (orient & 2) 	// Vertical trunk, horizontal branches
	create_hbranch_mask(n1->branchy, n1->branchx, xcent, slack, halo);
  }

  // Look for branches that are closer to each other than to the
  // trunk line.  If any are found, make a cross-connection between
  // the branch end that is closer to the trunk and the branch that
  // is its nearest neighbor.

  if (orient & 1) {	// Horizontal trunk, vertical branches
     for (n1 = net->netnodes; n1; n1 = n1->next) {
	for (n2 = net->netnodes->next; n2; n2 = n2->next) {

	   // Check if both ends are on the same side of the trunk
	   if ((n2->branchy > ycent && n1->branchy > ycent) ||
		  	(n2->branchy < ycent && n1->branchy < ycent)) {

	      // Check if branches are closer to each other than
	      // the shortest branch is away from the trunk
	      dx = ABSDIFF(n2->branchx, n1->branchx);
	      gy1 = ABSDIFF(n1->branchy, ycent);
	      gy2 = ABSDIFF(n2->branchy, ycent);
	      if ((dx < gy1) && (dx < gy2)) {
		 if (gy1 < gy2)
		    create_hbranch_mask(n1->branchy, n2->branchx,
				n1->branchx, slack, halo);
		 else
		    create_hbranch_mask(n2->branchy, n2->branchx,
				n1->branchx, slack, halo);
	      }
 	   }
        }
     }
  }
  if (orient & 2) {		// Vertical trunk, horizontal branches
     for (n1 = net->netnodes; n1; n1 = n1->next) {
	for (n2 = net->netnodes->next; n2; n2 = n2->next) {

	   // Check if both ends are on the same side of the trunk
	   if ((n2->branchx > xcent && n1->branchx > xcent) ||
		  	(n2->branchx < xcent && n1->branchx < xcent)) {

	      // Check if branches are closer to each other than
	      // the shortest branch is away from the trunk
	      dy = ABSDIFF(n2->branchy, n1->branchy);
	      gx1 = ABSDIFF(n1->branchx, xcent);
	      gx2 = ABSDIFF(n2->branchx, xcent);
	      if ((dy < gx1) && (dy < gx2)) {
		 if (gx1 < gx2)
		    create_vbranch_mask(n1->branchx, n2->branchy,
				n1->branchy, slack, halo);
		 else
		    create_vbranch_mask(n2->branchx, n2->branchy,
				n1->branchy, slack, halo);
	      }
 	   }
        }
     }
  }

  // Allow routes at all tap and extension points
  for (n1 = net->netnodes; n1 != NULL; n1 = n1->next) {
     for (dtap = n1->taps; dtap != NULL; dtap = dtap->next)
	RMASK(dtap->gridx, dtap->gridy) = (u_char)0;
     for (dtap = n1->extend; dtap != NULL; dtap = dtap->next)
	RMASK(dtap->gridx, dtap->gridy) = (u_char)0;
  }

  if (Verbose > 2) {
     if (net->numnodes == 2)
        Fprintf(stdout, "Two-port mask has bounding box (%d %d) to (%d %d)\n",
			oxmin, oymin, oxmax, oymax);
     else
        Fprintf(stdout, "multi-port mask has trunk line (%d %d) to (%d %d)\n",
			xmin, ymin, xmax, ymax);
  }
}

/*--------------------------------------------------------------*/
/* fillMask() fills the Mask[] array with all 1s as a last	*/
/* resort, ensuring that no valid routes are missed due to a	*/
/* bad guess about the optimal route positions.			*/
/*--------------------------------------------------------------*/

void fillMask(u_char value) {
   memset((void *)RMask, (int)value,
		(size_t)(NumChannelsX * NumChannelsY
		* sizeof(u_char)));
}

/* end of mask.c */
