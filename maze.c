/*--------------------------------------------------------------*/
/* maze.c -- general purpose maze router routines.		*/
/*								*/
/* This file contains the main cost evaluation routine, 	*/
/* the route segment generator, and a routine to search		*/
/* the database for all parts of a routed network.		*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June 2011, based on code by Steve	*/
/* Beccue							*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define  MAZE

#include "qrouter.h"
#include "qconfig.h"
#include "point.h"
#include "node.h"
#include "maze.h"
#include "lef.h"

extern int TotalRoutes;

/*--------------------------------------------------------------*/
/* find_unrouted_node() --					*/
/*								*/
/* On a power bus, the nodes are routed individually, using the	*/
/* entire power bus as the destination.  So to find out if a	*/
/* node is already routed or not, the only way is to check the	*/
/* routes recorded for the net and determine if any net		*/
/* endpoint is on a node.					*/
/*								*/
/* Return the first node found that is not connected to any	*/
/* route endpoint.  If all nodes are routed, then return NULL	*/
/*--------------------------------------------------------------*/

NODE find_unrouted_node(NET net)
{
    int i, numroutes;
    u_char *routednode;
    NODE node;
    ROUTE rt;
    SEG seg1, seg2;
    DPOINT tap;

    // Quick check:  If the number of routes == number of nodes,
    // then return NULL and we're done.

    numroutes = 0;
    for (rt = net->routes; rt; rt = rt->next) numroutes++;
    if (numroutes == net->numnodes) return NULL;

    routednode = (u_char *)malloc(net->numnodes * sizeof(u_char));
    for (i = 0; i < net->numnodes; i++) routednode[i] = 0;

    // Otherwise, we don't know which nodes have been routed,
    // so check each one individually.

    for (rt = net->routes; rt; rt = rt->next) {
	seg1 = rt->segments;
	if (seg1 == NULL) continue;
	seg2 = seg1;
	while (seg2->next) seg2 = seg2->next;

	for (node = net->netnodes; node; node = node->next) {
	    if (routednode[node->nodenum] == 1) continue;

	    for (tap = node->taps; tap; tap = tap->next) {
		if (seg1->x1 == tap->gridx && seg1->y1 == tap->gridy
				&& seg1->layer == tap->layer) {
		    routednode[node->nodenum] = 1;
		    break;
		}
		else if (seg1->x2 == tap->gridx && seg1->y2 == tap->gridy
				&& seg1->layer == tap->layer) {
		    routednode[node->nodenum] = 1;
		    break;
		}
		else if (seg2->x1 == tap->gridx && seg2->y1 == tap->gridy
				&& seg2->layer == tap->layer) {
		    routednode[node->nodenum] = 1;
		    break;
		}
		else if (seg2->x2 == tap->gridx && seg2->y2 == tap->gridy
				&& seg2->layer == tap->layer) {
		    routednode[node->nodenum] = 1;
		    break;
		}
	    }
	    if (tap == NULL) {
		for (tap = node->extend; tap; tap = tap->next) {

		    seg1 = rt->segments;
		    seg2 = seg1;
		    while (seg2->next) seg2 = seg2->next;

		    if (seg1->x1 == tap->gridx && seg1->y1 == tap->gridy
				&& seg1->layer == tap->layer) {
			routednode[node->nodenum] = 1;
			break;
		    }
		    else if (seg1->x2 == tap->gridx && seg1->y2 == tap->gridy
				&& seg1->layer == tap->layer) {
			routednode[node->nodenum] = 1;
			break;
		    }
		    else if (seg2->x1 == tap->gridx && seg2->y1 == tap->gridy
				&& seg2->layer == tap->layer) {
			routednode[node->nodenum] = 1;
			break;
		    }
		    else if (seg2->x2 == tap->gridx && seg2->y2 == tap->gridy
				&& seg2->layer == tap->layer) {
			routednode[node->nodenum] = 1;
			break;
		    }
		}
	    }
	}
    }

    for (node = net->netnodes; node; node = node->next) {
	if (routednode[node->nodenum] == 0) {
	    free(routednode);
	    return node;
	}
    }

    free(routednode);
    return NULL;	/* Statement should never be reached */
}

/*--------------------------------------------------------------*/
/* set_powerbus_to_net()					*/
/* If we have a power or ground net, go through the entire Obs	*/
/* array and mark all points matching the net as TARGET in Obs2	*/
/*								*/
/* We do this after the call to PR_SOURCE, before the calls	*/
/* to set PR_TARGET.						*/
/*								*/
/* If any grid position was marked as TARGET, return 1, else	*/
/* return 0 (meaning the net has been routed already).		*/
/*--------------------------------------------------------------*/

int set_powerbus_to_net(int netnum)
{
    int x, y, lay, rval;
    PROUTE *Pr;

    rval = 0;
    if ((netnum == VDD_NET) || (netnum == GND_NET) || (netnum == ANTENNA_NET)) {
       for (lay = 0; lay < Num_layers; lay++)
          for (x = 0; x < NumChannelsX; x++)
	     for (y = 0; y < NumChannelsY; y++)
		if ((OBSVAL(x, y, lay) & NETNUM_MASK) == netnum) {
		   Pr = &OBS2VAL(x, y, lay);
		   // Skip locations that have been purposefully disabled
		   if (!(Pr->flags & PR_COST) && (Pr->prdata.net == MAXNETNUM))
		      continue;
		   else if (!(Pr->flags & PR_SOURCE)) {
		      Pr->flags |= (PR_TARGET | PR_COST);
		      Pr->prdata.cost = MAXRT;
		      rval = 1;
		   }
		}
    }
    return rval;
}

/*--------------------------------------------------------------*/
/* clear_non_source_targets --					*/
/*								*/
/* Look at all target nodes of a net.  For any that are not	*/
/* marked as SOURCE, but have terminal points marked as		*/
/* PROCESSED, remove the PROCESSED flag and put the position	*/
/* back on the stack for visiting on the next round.		*/
/*--------------------------------------------------------------*/

void clear_non_source_targets(NET net, POINT *pushlist)
{
   NODE node;
   DPOINT ntap;
   PROUTE *Pr;
   POINT gpoint;
   int lay, x, y;

   for (node = net->netnodes; node; node = node->next) {
      for (ntap = node->taps; ntap; ntap = ntap->next) {
	 lay = ntap->layer;
	 x = ntap->gridx;
	 y = ntap->gridy;
	 Pr = &OBS2VAL(x, y, lay);
	 if (Pr->flags & PR_TARGET) {
	    if (Pr->flags & PR_PROCESSED) {
	       Pr->flags &= ~PR_PROCESSED;
	       if (~(Pr->flags & PR_ON_STACK)) {
		  Pr->flags |= PR_ON_STACK;
		  gpoint = allocPOINT();
		  gpoint->x1 = x;
		  gpoint->y1 = y;
		  gpoint->layer = lay;
		  gpoint->next = *pushlist;
		  *pushlist = gpoint;
	       }
	    }
	 }
      }
      if (ntap == NULL) {
	 // Try extended tap areas
         for (ntap = node->extend; ntap; ntap = ntap->next) {
	    lay = ntap->layer;
	    x = ntap->gridx;
	    y = ntap->gridy;
	    Pr = &OBS2VAL(x, y, lay);
	    if (Pr->flags & PR_TARGET) {
		if (Pr->flags & PR_PROCESSED) {
		   Pr->flags &= ~PR_PROCESSED;
		   if (~(Pr->flags & PR_ON_STACK)) {
		      Pr->flags |= PR_ON_STACK;
		      gpoint = allocPOINT();
		      gpoint->x1 = x;
		      gpoint->y1 = y;
		      gpoint->layer = lay;
		      gpoint->next = pushlist[1];
		      pushlist[1] = gpoint;
		   }
		}
	    }
         }
      }
   }
}

/*--------------------------------------------------------------*/
/* clear_target_node --						*/
/*								*/
/* Remove PR_TARGET flags from all points belonging to a node	*/
/*--------------------------------------------------------------*/

void clear_target_node(NODE node)
{
    int x, y, lay;
    NODEINFO lnode;
    DPOINT ntap;
    PROUTE *Pr;

    /* Process tap points of the node */

    for (ntap = node->taps; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;
       if ((lay < Pinlayers) && (((lnode = NODEIPTR(x, y, lay)) == NULL)
		|| (lnode->nodesav == NULL)))
	  continue;
       Pr = &OBS2VAL(x, y, lay);
       Pr->flags = 0;
       Pr->prdata.net = node->netnum;
    }

    for (ntap = node->extend; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;

       if (lay < Pinlayers) {
	  lnode = NODEIPTR(x, y, lay);
	  if (lnode == NULL) continue;
	  if (lnode->nodesav != node) continue;
       }
	
       Pr = &OBS2VAL(x, y, lay);
       Pr->flags = 0;
       Pr->prdata.net = node->netnum;
    }
}

/*--------------------------------------------------------------*/
/* count_targets() ---						*/
/*								*/
/* Count the number of nodes of a net that are still marked as	*/
/* TARGET.							*/
/*--------------------------------------------------------------*/

int
count_targets(NET net)
{
   NODE node;
   PROUTE *Pr;
   DPOINT ntap;
   int lay, x, y;
   int count = 0;

   for (node = net->netnodes; node; node = node->next) {
      for (ntap = node->taps; ntap; ntap = ntap->next) {
	 lay = ntap->layer;
	 x = ntap->gridx;
	 y = ntap->gridy;
	 Pr = &OBS2VAL(x, y, lay);
	 if (Pr->flags & PR_TARGET) {
	    count++;
	    break;
	 }
      }
      if (ntap == NULL) {
	 // Try extended tap areas
         for (ntap = node->extend; ntap; ntap = ntap->next) {
	    lay = ntap->layer;
	    x = ntap->gridx;
	    y = ntap->gridy;
	    Pr = &OBS2VAL(x, y, lay);
	    if (Pr->flags & PR_TARGET) {
	       count++;
	       break;
	    }
         }
      }
   }
   return count;
}

/*--------------------------------------------------------------*/
/* set_node_to_net() ---					*/
/*								*/
/* Change the Obs2[][] flag values to "newflags" for all tap	*/
/* positions of route terminal "node".				*/
/*								*/
/* Return value is 1 if at least one terminal of the node	*/
/* is already marked as PR_SOURCE, indicating that the node	*/
/* has already been routed.  Otherwise, the return value is	*/
/* zero if no error occured, and -1 if any point was found to	*/
/* be unoccupied by any net, which should not happen.		*/
/*								*/
/* If "bbox" is non-null, record the grid extents of the node	*/
/* in the x1, x2, y1, y2 values					*/
/*								*/
/* If "stage" is 1 (rip-up and reroute), then don't let an	*/
/* existing route prevent us from adding terminals.  However,	*/
/* the area will be first checked for any part of the terminal	*/
/* that is routable, only resorting to overwriting colliding	*/
/* nets if there are no other available taps.  Defcon stage 3	*/
/* indicates desperation due to a complete lack of routable	*/
/* taps.  This happens if, for example, a port is offset from	*/
/* the routing grid and tightly boxed in by obstructions.  In	*/
/* such case, we allow routing on an obstruction, but flag the	*/
/* point.  In the output stage, the stub route information will	*/
/* be used to reposition the contact on the port and away from	*/
/* the obstruction.						*/
/*								*/
/* If we completely fail to find a tap point under any		*/
/* condition, then return -2.  This is a fatal error;  there	*/
/* will be no way to route the net.				*/
/*--------------------------------------------------------------*/

int set_node_to_net(NODE node, int newflags, POINT *pushlist,
	SEG bbox, u_char stage)
{
    int x, y, lay, rank, base, obsnet = 0;
    int result = 0;
    u_char found_one = FALSE;
    NODEINFO lnode;
    POINT gpoint;
    DPOINT ntap;
    PROUTE *Pr;

    /* If called from set_routes_to_net, the node has no taps, and the	*/
    /* net is a power bus, just return.					*/

    if ((node->taps == NULL) && (node->extend == NULL)) {
       if ((node->netnum == VDD_NET) || (node->netnum == GND_NET) ||
		(node->netnum == ANTENNA_NET))
	   return result;
    }

    /* Process tap points of the node */

    for (ntap = node->taps; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;

       Pr = &OBS2VAL(x, y, lay);
       if ((Pr->flags & (newflags | PR_COST)) == PR_COST) {
	  Fprintf(stderr, "Error:  Tap position %d, %d layer %d not "
			"marked as source!\n", x, y, lay);
	  return -1;	// This should not happen.
       }

       if (Pr->flags & PR_SOURCE) {
	  if (!found_one)
	     return 1;			// Node is already connected!
	  else
	     continue;			// May be duplicate tap position
       }
       else if ((Pr->flags & PR_TARGET) && (newflags & PR_TARGET)) {
	  if (!found_one)
	     return 1;
	  else
	     continue;
       }
       else if (((Pr->prdata.net == node->netnum) || (stage == (u_char)2))
			&& !(Pr->flags & newflags)) {

	  // If we got here, we're on the rip-up stage, and there
	  // is an existing route completely blocking the terminal.
	  // So we will route over it and flag it as a collision.

	  if (Pr->prdata.net != node->netnum) {
	     if ((Pr->prdata.net == (NO_NET | OBSTRUCT_MASK)) ||
			(Pr->prdata.net == NO_NET))
		continue;
	     else
	        Pr->flags |= PR_CONFLICT;
	  }

	  // Do the source and dest nodes need to be marked routable?
	  Pr->flags |= (newflags == PR_SOURCE) ? newflags : (newflags | PR_COST);

	  Pr->prdata.cost = (newflags == PR_SOURCE) ? 0 : MAXRT;

	  // Rank the position according to how difficult it is to route to:
	  //
	  // 0: no restrictions
	  // 1: requires a stub route
	  // 2: inside the halo
	  // 3: requires an offset
	  // 4: requires an offset and a stub route

	  rank = 0;
          if (lay < Pinlayers) {
	     if (lnode = NODEIPTR(x, y, lay)) {
		if (lnode->flags & NI_OFFSET_MASK) rank = 2;
		if (lnode->flags & NI_STUB_MASK) rank++;
	     }
          }

	  // push this point on the stack to process

	  if (pushlist != NULL) {
	     if (~(Pr->flags & PR_ON_STACK)) {
		Pr->flags |= PR_ON_STACK;
	        gpoint = allocPOINT();
	        gpoint->x1 = x;
	        gpoint->y1 = y;
	        gpoint->layer = lay;
	        gpoint->next = pushlist[rank];
		pushlist[rank] = gpoint;
	     }
	  }
	  found_one = TRUE;

	  // record extents
	  if (bbox) {
	     if (x < bbox->x1) bbox->x1 = x;
	     if (x > bbox->x2) bbox->x2 = x;
	     if (y < bbox->y1) bbox->y1 = y;
	     if (y > bbox->y2) bbox->y2 = y;
	  }
       }
       else if ((Pr->prdata.net < MAXNETNUM) && (Pr->prdata.net > 0)) obsnet++;
    }

    // Do the same for point in the halo around the tap, but only if
    // they have been attached to the net during a past routing run.

    for (ntap = node->extend; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;

       // Don't process extended areas if they coincide with other nodes,
       // or those that are out-of-bounds

       base = 0;
       if (lay < Pinlayers) {
	  lnode = NODEIPTR(x, y, lay);
	  if (lnode == NULL) continue;
	  if (lnode->nodesav != node) continue;

	  // Otherwise, rank according to ease of reaching the tap.
          if (lnode->flags & NI_OFFSET_MASK) base = 2;
          if (lnode->flags & NI_STUB_MASK) base++;
       }

       Pr = &OBS2VAL(x, y, lay);
       if (Pr->flags & PR_SOURCE) {
	  if (!found_one)
	     return 1;			// Node is already connected!
	  else
	     continue;			// May be duplicate tap record
       }
       else if ((Pr->flags & PR_TARGET) && (newflags & PR_TARGET)) {
	  if (!found_one)
	     return 1;
	  else
	     continue;
       }
       else if ( !(Pr->flags & newflags) &&
		((Pr->prdata.net == node->netnum) ||
		(stage == (u_char)2 && Pr->prdata.net < MAXNETNUM) ||
		(stage == (u_char)3))) {

	  if (Pr->prdata.net != node->netnum) Pr->flags |= PR_CONFLICT;
	  Pr->flags |= (newflags == PR_SOURCE) ? newflags : (newflags | PR_COST);
	  Pr->prdata.cost = (newflags == PR_SOURCE) ? 0 : MAXRT;

	  // push this point on the stack to process

	  if (pushlist != NULL) {
	     if (~(Pr->flags & PR_ON_STACK)) {
		Pr->flags |= PR_ON_STACK;
	        gpoint = allocPOINT();
	        gpoint->x1 = x;
	        gpoint->y1 = y;
	        gpoint->layer = lay;
		rank = (found_one == TRUE) ? 2 + base : base;

	        gpoint->next = pushlist[rank];
		pushlist[rank] = gpoint;
	     }
	  }
	  found_one = TRUE;

	  // record extents
	  if (bbox) {
	     if (x < bbox->x1) bbox->x1 = x;
	     if (x > bbox->x2) bbox->x2 = x;
	     if (y < bbox->y1) bbox->y1 = y;
	     if (y > bbox->y2) bbox->y2 = y;
	  }
       }
       else if ((Pr->prdata.net < MAXNETNUM) && (Pr->prdata.net > 0)) obsnet++;
    }

    // In the case that no valid tap points were found,	if we're on the
    // rip-up and reroute section, try again, ignoring existing routes that
    // are in the way of the tap point.  If that fails, then we will
    // route over obstructions and shift the contact when committing the
    // route solution.  And if that fails, we're basically hosed.
    //
    // Make sure to check for the case that the tap point is simply not
    // reachable from any grid point, in the first stage, so we don't
    // wait until the rip-up and reroute stage to route them.

    if ((result == 0) && (!found_one)) {
       if (stage == (u_char)1)
          return set_node_to_net(node, newflags, pushlist, bbox, (u_char)2);
       else if (stage == (u_char)2)
          return set_node_to_net(node, newflags, pushlist, bbox, (u_char)3);
       else if ((stage == (u_char)0) && (obsnet == 0))
          return set_node_to_net(node, newflags, pushlist, bbox, (u_char)3);
       else
	  return -2;
    }

    return result;
}

/*--------------------------------------------------------------*/
/* Set all taps of node "node" to MAXNETNUM, so that it will	*/
/* not be routed to. 						*/
/*--------------------------------------------------------------*/

int disable_node_nets(NODE node)
{
    int x, y, lay;
    int result = 0;
    DPOINT ntap;
    PROUTE *Pr;

    /* Process tap points of the node */

    for (ntap = node->taps; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;
       Pr = &OBS2VAL(x, y, lay);
       if (Pr->flags & PR_SOURCE || Pr->flags & PR_TARGET || Pr->flags & PR_COST) {
	  result = 1;
       }
       else if (Pr->prdata.net == node->netnum) {
	  Pr->prdata.net = MAXNETNUM;
       }
    }

    // Do the same for point in the halo around the tap, but only if
    // they have been attached to the net during a past routing run.

    for (ntap = node->extend; ntap; ntap = ntap->next) {
       lay = ntap->layer;
       x = ntap->gridx;
       y = ntap->gridy;
       Pr = &OBS2VAL(x, y, lay);
       if (Pr->flags & PR_SOURCE || Pr->flags & PR_TARGET || Pr->flags & PR_COST) {
	  result = 1;
       }
       else if (Pr->prdata.net == node->netnum) {
	  Pr->prdata.net = MAXNETNUM;
       }
    }
    return result;
}

/*--------------------------------------------------------------*/
/* That which is already routed (routes should be attached to	*/
/* source nodes) is routable by definition. . .			*/
/*--------------------------------------------------------------*/

int set_route_to_net(NET net, ROUTE rt, int newflags, POINT *pushlist,
		SEG bbox, u_char stage)
{
    int x, y, lay;
    int result = 0;
    NODEINFO lnode;
    POINT gpoint;
    SEG seg;
    NODE n2;
    PROUTE *Pr;

    if (rt && rt->segments) {
	for (seg = rt->segments; seg; seg = seg->next) {
	    lay = seg->layer;
	    x = seg->x1;
	    y = seg->y1;
	    while (1) {
		Pr = &OBS2VAL(x, y, lay);
		Pr->flags = (newflags == PR_SOURCE) ? newflags : (newflags | PR_COST);
		// Conflicts should not happen (check for this?)
		// if (Pr->prdata.net != node->netnum) Pr->flags |= PR_CONFLICT;
		Pr->prdata.cost = (newflags == PR_SOURCE) ? 0 : MAXRT;

		// push this point on the stack to process

		if (pushlist != NULL) {
		   if (~(Pr->flags & PR_ON_STACK)) {
		      Pr->flags |= PR_ON_STACK;
	  	      gpoint = allocPOINT();
	  	      gpoint->x1 = x;
	  	      gpoint->y1 = y;
	  	      gpoint->layer = lay;
	  	      gpoint->next = *pushlist;
	 	      *pushlist = gpoint;
		   }
		}

		// record extents
		if (bbox) {
		   if (x < bbox->x1) bbox->x1 = x;
		   if (x > bbox->x2) bbox->x2 = x;
		   if (y < bbox->y1) bbox->y1 = y;
		   if (y > bbox->y2) bbox->y2 = y;
		}

		// If we found another node connected to the route,
		// then process it, too.

		lnode = (lay >= Pinlayers) ? NULL : NODEIPTR(x, y, lay);
		n2 = (lnode) ? lnode->nodesav : NULL;
		if ((n2 != (NODE)NULL) && (n2 != net->netnodes)) {
		   if (newflags == PR_SOURCE) clear_target_node(n2);
		   result = set_node_to_net(n2, newflags, pushlist, bbox, stage);
		   // On error, continue processing
		}

		// Process top part of via
		if (seg->segtype & ST_VIA) {
		   if (lay != seg->layer) break;
		   lay++;
		   continue;
		}

		// Move to next grid position in segment
		if (x == seg->x2 && y == seg->y2) break;
		if (seg->x2 > seg->x1) x++;
		else if (seg->x2 < seg->x1) x--;
		if (seg->y2 > seg->y1) y++;
		else if (seg->y2 < seg->y1) y--;
	    }
	}
    }
    return result;
}

/*--------------------------------------------------------------*/
/* Process a route and all routes that connect to it.  Works	*/
/* like the routine above, but searches the route endpoints for	*/
/* connecting nodes and routes, and then recursively calls	*/
/* itself on the connecting routes, and other routes that	*/
/* connect do the nodes.					*/
/*--------------------------------------------------------------*/

int set_route_to_net_recursive(NET net, ROUTE rt, int newflags,
		POINT *pushlist, SEG bbox, u_char stage)
{
    ROUTE route;
    int result;

    /* If route has been marked, return */
    if (rt->flags & RT_VISITED) return 0;
    rt->flags |= RT_VISITED;

    /* First mark this route */
    result = set_route_to_net(net, rt, newflags, pushlist, bbox, stage);
    if (result < 0) return result;

    /* Recursively mark the routes connected to the nodes of	*/
    /* the endpoints or connected directly to the endpoints.	*/

    if (rt->flags & RT_START_NODE) {
	for (route = net->routes; route; route = route->next) {
	    if (!(route->flags & RT_START_NODE) && (route->start.route == rt)) {
		result = set_route_to_net_recursive(net, route, newflags,
				pushlist, bbox, stage);
		if (result < 0) return result;
	    }
	    if (!(route->flags & RT_END_NODE) && (route->end.route == rt)) {
		result = set_route_to_net_recursive(net, route, newflags,
				pushlist, bbox, stage);
		if (result < 0) return result;
	    }
	}
    }
    else if (rt->start.route) {
	result = set_route_to_net_recursive(net, rt->start.route, newflags,
			pushlist, bbox, stage);
	if (result < 0) return result;
    }
    else
	Fprintf(stderr, "Error:  Route start information not recorded, cannot walk.\n");
	
    if (rt->flags & RT_END_NODE) {
	for (route = net->routes; route; route = route->next) {
	    if (!(route->flags & RT_START_NODE) && (route->start.route == rt)) {
		result = set_route_to_net_recursive(net, route, newflags,
				pushlist, bbox, stage);
		if (result < 0) return result;
	    }
	    if (!(route->flags & RT_END_NODE) && (route->end.route == rt)) {
		result = set_route_to_net_recursive(net, route, newflags,
				pushlist, bbox, stage);
		if (result < 0) return result;
	    }
	}
    }
    else if (rt->end.route) {
	result = set_route_to_net_recursive(net, rt->end.route, newflags,
			pushlist, bbox, stage);
	if (result < 0) return result;
    }
    else
	Fprintf(stderr, "Error:  Route end information not recorded, cannot walk.\n");
    return result;
}

/*--------------------------------------------------------------*/
/* Process all routes of a net that are connected in some way	*/
/* node "node", and set their routed positions to the value of	*/
/* "newflags" (PR_SOURCE or PR_DEST) in Obs2[].			*/
/*--------------------------------------------------------------*/

int set_routes_to_net(NODE node, NET net, int newflags, POINT *pushlist,
		SEG bbox, u_char stage)
{
    ROUTE rt;
    int result = 0;

    /* Clear marks on all routes */
    for (rt = net->routes; rt; rt = rt->next) rt->flags &= ~RT_VISITED;

    /* Find any route that has node as an endpoint */
    for (rt = net->routes; rt; rt = rt->next) {
	if ((rt->flags & RT_START_NODE) && (rt->start.node == node))
	    result = set_route_to_net_recursive(net, rt, newflags,
				pushlist, bbox, stage);
	else if ((rt->flags & RT_END_NODE) && (rt->end.node == node))
	    result = set_route_to_net_recursive(net, rt, newflags,
				pushlist, bbox, stage);
	if (result < 0) return result;
    }
    return result;
}

/*--------------------------------------------------------------*/
/* Used by find_colliding() (see below).  Save net "netnum"	*/
/* to the list of colliding nets if it is not already in the	*/
/* list.  Return 1 if the list got longer, 0 otherwise.		*/
/* Find the route of the net that includes the point of		*/
/* collision, and mark it for rip-up.				*/
/*--------------------------------------------------------------*/

static int
addcollidingnet(NETLIST *nlptr, int netnum, int x, int y, int lay)
{
    ROUTE rt;
    NETLIST cnl;
    NET fnet;
    SEG seg;
    int i;
    int sx, sy;
    u_char found;

    for (cnl = *nlptr; cnl; cnl = cnl->next)
	if (cnl->net->netnum == netnum)
	    return 0;

    for (i = 0; i < Numnets; i++) {
	fnet = Nlnets[i];
	if (fnet->netnum == netnum) {
	    cnl = (NETLIST)malloc(sizeof(struct netlist_));
	    cnl->net = fnet;
	    cnl->next = *nlptr;
	    *nlptr = cnl;

	    /* If there are no routes then we're done. */

	    if (fnet->routes == NULL) return 0;

	    /* If there is only one route then there is no need */
	    /* to search or shuffle.				*/

	    if (fnet->routes->next == NULL) {
		fnet->routes->flags |= RT_RIP;
		return 1;
	    }

	    for (rt = fnet->routes; rt; rt = rt->next) {
		found = 0;
		for (seg = rt->segments; seg; seg = seg->next) {
		    if ((seg->layer == lay) || ((seg->segtype & ST_VIA) &&
				((seg->layer + 1) == lay))) {
			sx = seg->x1;
			sy = seg->y1;
			while (1) {
			    if ((sx == x) && (sy == y)) {
				found = 1;
				break;
			    }
			    if ((sx == seg->x2) && (sy == seg->y2)) break;
			    if (sx < seg->x2) sx++;
			    else if (sx > seg->x2) sx--;
			    if (sy < seg->y2) sy++;
			    else if (sy > seg->y2) sy--;
			}
			if (found) break;
		    }
		}
		if (found) rt->flags |= RT_RIP;
	    }
	    return 1;
	}
    }
    return 0;
}

/*--------------------------------------------------------------*/
/* Find nets that are colliding with the given net "net", and	*/
/* create and return a list of them.				*/
/*--------------------------------------------------------------*/

NETLIST find_colliding(NET net, int *ripnum)
{
   NETLIST nl = (NETLIST)NULL, cnl;
   ROUTE rt;
   SEG seg;
   int lay, x, y, orignet, rnum;

   /* Scan the routed points for recorded collisions.	*/

   rnum = 0;
   for (rt = net->routes; rt; rt = rt->next) {
      if (rt->segments) {
	 for (seg = rt->segments; seg; seg = seg->next) {
	    lay = seg->layer;
	    x = seg->x1;
	    y = seg->y1;

	    // The following skips over vias, which is okay, since
	    // those positions are covered by segments on both layers
	    // or are terminal positions that by definition can't
	    // belong to a different net.

	    while (1) {
	       orignet = OBSVAL(x, y, lay) & ROUTED_NET_MASK;

	       if ((orignet & DRC_BLOCKAGE) == DRC_BLOCKAGE) {

		  /* If original position was a DRC-related blockage,	*/
		  /* find out which net or nets would be in conflict.	*/

		  if (needblock[lay] & (ROUTEBLOCKX | VIABLOCKX)) {
		     if (x < NumChannelsX - 1) {
		        orignet = OBSVAL(x + 1, y, lay) & ROUTED_NET_MASK;
		        if (!(orignet & NO_NET)) {
			   orignet &= NETNUM_MASK;
			   if ((orignet != 0) && (orignet != net->netnum))
		 	       rnum += addcollidingnet(&nl, orignet, x, y, lay);
		        }
		     }
		     if (x > 0) {
		        orignet = OBSVAL(x - 1, y, lay) & ROUTED_NET_MASK;
		        if (!(orignet & NO_NET)) {
			   orignet &= NETNUM_MASK;
			   if ((orignet != 0) && (orignet != net->netnum))
		 	       rnum += addcollidingnet(&nl, orignet, x, y, lay);
		        }
		     }
		  }
		  if (needblock[lay] & (ROUTEBLOCKY | VIABLOCKY)) {
		     if (y < NumChannelsY - 1) {
		        orignet = OBSVAL(x, y + 1, lay) & ROUTED_NET_MASK;
		        if (!(orignet & NO_NET)) {
			   orignet &= NETNUM_MASK;
			   if ((orignet != 0) && (orignet != net->netnum))
		 	       rnum += addcollidingnet(&nl, orignet, x, y, lay);
			}
		     }
		     if (y > 0) {
		        orignet = OBSVAL(x, y - 1, lay) & ROUTED_NET_MASK;
		        if (!(orignet & NO_NET)) {
			   orignet &= NETNUM_MASK;
			   if ((orignet != 0) && (orignet != net->netnum))
		 	       rnum += addcollidingnet(&nl, orignet, x, y, lay);
			}
		     }
		  }
	       }
	       else {
		  orignet &= NETNUM_MASK;
		  if ((orignet != net->netnum) && (orignet != 0))
		     rnum += addcollidingnet(&nl, orignet, x, y, lay);
	       }

	       if ((x == seg->x2) && (y == seg->y2)) break;

	       if (x < seg->x2) x++;
	       else if (x > seg->x2) x--;
	       if (y < seg->y2) y++;
	       else if (y > seg->y2) y--;
	    }
	 }
      }
   }

   /* Diagnostic */

   if ((nl != NULL) && (Verbose > 0)) {
      Fprintf(stdout, "Best route of %s collides with net%s: ",
		net->netname, (rnum > 1) ? "s" : "");
      for (cnl = nl; cnl; cnl = cnl->next) {
         Fprintf(stdout, "%s ", cnl->net->netname);
      }
      Fprintf(stdout, "\n");
   }

   if (ripnum) *ripnum = rnum;
   return nl;
}

/*--------------------------------------------------------------*/
/* ripup_dependent ---						*/
/*								*/
/* If a set of routes is being ripped out of a net (marked by	*/
/* RT_RIP in the flags), check if any routes below them have	*/
/* endpoints landing on a ripped-out net.  If so, flag those	*/
/* nets for being ripped out as well.  Repeat recursively.	*/
/*--------------------------------------------------------------*/

void ripup_dependent(NET net)
{
    ROUTE rt, route;
    u_char rerun = TRUE;

    while (rerun) {
	rerun = FALSE;
	for (rt = net->routes; rt; rt = rt->next) {
	    if (rt->flags & RT_RIP) continue;
	    if (!(rt->flags & RT_START_NODE)) {
		route = rt->start.route;
		// route should not be NULL here. . .
		if (route && (route->flags & RT_RIP)) {
		    rt->flags |= RT_RIP;
		    rerun = TRUE;
		}
	    }
	    if (!(rt->flags & RT_END_NODE)) {
		route = rt->end.route;
		// route should not be NULL here. . .
		if (route && (route->flags & RT_RIP)) {
		    rt->flags |= RT_RIP;
		    rerun = TRUE;
		}
	    }
	}
    }
}

/*--------------------------------------------------------------*/
/* Failure analysis (debug procedure)				*/
/* Occasionally when ripping up a net or net route, the Obs	*/
/* array shows a different net in the position that is being	*/
/* ripped, which should not happen.  This routine does a quick	*/
/* analysis to determine if the position is orphaned.  If so,	*/
/* it just returns and ripup_net will overwrite the position.	*/
/* If it appears to be connected to a valid route, it will find	*/
/* the net and route segment and run rip-up on it.		*/
/*--------------------------------------------------------------*/

void analyze_route_overwrite(int x, int y, int lay, int netnum)
{
    u_char is_valid = FALSE;
    int i, sx, sy, l;
    NET fnet;
    ROUTE rt;
    SEG seg;

    /* Check on all sides to see if position is orphaned */

    if ((x < NumChannelsX - 1) && (OBSVAL(x + 1, y, lay) & NETNUM_MASK) == netnum)
	is_valid = TRUE;
    else if ((x > 0) && (OBSVAL(x - 1, y, lay) & NETNUM_MASK) == netnum)
	is_valid = TRUE;
    else if ((y < NumChannelsY - 1) && (OBSVAL(x, y + 1, lay) & NETNUM_MASK) == netnum)
	is_valid = TRUE;
    else if ((y > 0) && (OBSVAL(x, y - 1, lay) & NETNUM_MASK) == netnum)
	is_valid = TRUE;
    else if ((lay < Num_layers - 1) && (OBSVAL(x, y, lay + 1) & NETNUM_MASK) == netnum)
	is_valid = TRUE;
    else if ((lay > 0) && (OBSVAL(x, y, lay - 1) & NETNUM_MASK) == netnum)
	is_valid = TRUE;

    if (is_valid == FALSE) {
	Fprintf(stderr, "Net position %d %d %d appears to be orphaned.\n",
			x, y, lay);
	return; 	/* No action, just overwrite */
    }

    for (i = 0; i < Numnets; i++) {
	fnet = Nlnets[i];
	if (fnet->netnum == netnum) {
	    for (rt = fnet->routes; rt; rt = rt->next) {
		for (seg = rt->segments; seg; seg = seg->next) {
		    sx = seg->x1;
		    sy = seg->y1;
		    l = seg->layer;
		    while (1) {
			if ((sx == x) && (sy == y) && (l == lay)) {
			    Fprintf(stderr, "Net position %d %d %d appears to "
					"belong to a valid network route.\n",
					x, y, lay);
			    /* Found the route containing this position, */
			    /* so rip up the net now.			 */
			    Fprintf(stderr, "Taking evasive action against net "
					"%d\n", netnum);
			    ripup_net(fnet, TRUE, FALSE, FALSE);
			    return;
			}
			if ((sx == seg->x2) && (sy == seg->y2)) {
			    if ((seg->segtype == ST_WIRE) || (l >= (lay + 1))) break;
			    else l++;
			}
			else {
			    if (seg->x2 > seg->x1) sx++;
			    else if (seg->x2 < seg->x1) sx--;
			    if (seg->y2 > seg->y1) sy++;
			    else if (seg->y2 < seg->y1) sy--;
			}
		    }
		}
	    }
	    break;
	}
    }
}

/*--------------------------------------------------------------*/
/* Remove all route records from a net.				*/
/*--------------------------------------------------------------*/

void remove_routes(ROUTE netroutes, u_char flagged)
{
   ROUTE rt, rsave, rlast;
   SEG seg;

   /* Remove all flagged routing information from this net	*/
   /* if "flagged" is true, otherwise remove all routing	*/
   /* information.						*/

   if (flagged && (netroutes != NULL)) {
      rlast = NULL;
      rsave = netroutes;
      while (rsave) {
	 if (rsave->flags & RT_RIP) {
	    rt = rsave;
	    if (rlast == NULL)
		netroutes = rsave->next;
	    else
		rlast->next = rsave->next;
	    rsave = rsave->next;
	    while (rt->segments) {
	       seg = rt->segments->next;
	       free(rt->segments);
	       rt->segments = seg;
	    }
	    free(rt);
	 }
	 else {
	    rlast = rsave;
	    rsave = rsave->next;
	 }
      }
   }
   else {
      while (netroutes) {
         rt = netroutes;
         netroutes = rt->next;
         while (rt->segments) {
	    seg = rt->segments->next;
	    free(rt->segments);
	    rt->segments = seg;
         }
         free(rt);
      }
   }
}

/*--------------------------------------------------------------*/
/* Clear a DRC blockage.  Normally, just remove the		*/
/* DRC_BLOCKAGE flag from Obs[].  However, more than one net	*/
/* may set a block on the position, so the lower four bits	*/
/* (OBSTRUCT_MASK) holds the reference count.			*/
/*--------------------------------------------------------------*/

void clear_drc_blockage(x, y, lay)
{
    int blockcount;

    blockcount = OBSVAL(x, y, lay) & OBSTRUCT_MASK;
    OBSVAL(x, y, lay) &= ~OBSTRUCT_MASK;
    if (blockcount > 0)
	OBSVAL(x, y, lay) |= (blockcount - 1);
    else
	OBSVAL(x, y, lay) &= ~DRC_BLOCKAGE;
}

/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/

void set_drc_blockage(x, y, lay)
{
    int blockcount, obsval;

    obsval = OBSVAL(x, y, lay);
    if ((obsval & DRC_BLOCKAGE) == DRC_BLOCKAGE) {
	blockcount = OBSVAL(x, y, lay) & OBSTRUCT_MASK;
	OBSVAL(x, y, lay) &= ~OBSTRUCT_MASK;
	OBSVAL(x, y, lay) |= (blockcount + 1);
    }
    else if ((obsval & NETNUM_MASK) == 0) {
	OBSVAL(x, y, lay) &= ~OBSTRUCT_MASK;
	OBSVAL(x, y, lay) |= DRC_BLOCKAGE;
    }
}

/*--------------------------------------------------------------*/
/* ripup_net ---						*/
/*								*/
/* Rip up the entire network located at position x, y, lay.	*/
/*								*/
/* If argument "restore" is TRUE, then at each node, restore	*/
/* the crossover cost by attaching the node back to the		*/
/* Nodeinfo array.						*/
/*								*/
/* If argument "flagged" is TRUE, then only remove routes	*/
/* that have been flagged with RT_RIP.				*/
/*								*/
/* If argument "retain" is TRUE, then do not remove the route	*/
/* records from the net.  This assumes that the calling routine	*/
/* will retain them for possible replacement in case of route	*/
/* failure.							*/
/*--------------------------------------------------------------*/

u_char ripup_net(NET net, u_char restore, u_char flagged, u_char retain)
{
   int thisnet, oldnet, x, y, lay, dir;
   NODEINFO lnode;
   NODE node;
   ROUTE rt;
   SEG seg;
   DPOINT ntap;

   if (flagged) ripup_dependent(net);

   thisnet = net->netnum;

   for (rt = net->routes; rt; rt = rt->next) {
      if (flagged && !(rt->flags & RT_RIP)) continue;
      if (rt->segments) {
	 for (seg = rt->segments; seg; seg = seg->next) {
	    lay = seg->layer;
	    x = seg->x1;
	    y = seg->y1;
	    while (1) {
	       oldnet = OBSVAL(x, y, lay) & NETNUM_MASK;
	       if ((oldnet > 0) && (oldnet < MAXNETNUM)) {
	          if (oldnet != thisnet) {
		     Fprintf(stderr, "Error: position %d %d layer %d has net "
				"%d not %d!\n", x, y, lay, oldnet, thisnet);
		     // Stop-gap:  Need to analyze the root of this problem.
		     // However, a reasonable action is to try to find the
		     // net and route associated with the incorrect net.
		     analyze_route_overwrite(x, y, lay, oldnet);

		     if ((rt == NULL) || (rt->segments == NULL))
			 return FALSE;	// Something went wrong
	          }

	          // Reset the net number to zero along this route for
	          // every point that is not a node tap.  Points that
		  // were routed over obstructions to reach off-grid
		  // taps are returned to obstructions.

	          if ((lay >= Pinlayers) || ((lnode = NODEIPTR(x, y, lay)) == NULL)
				|| (lnode->nodesav == NULL)) {
		     dir = OBSVAL(x, y, lay) & PINOBSTRUCTMASK;
		     if (dir == 0)
		        OBSVAL(x, y, lay) = OBSVAL(x, y, lay) & BLOCKED_MASK;
		     else
		        OBSVAL(x, y, lay) = NO_NET | dir;
		  }
	          else {
		     // Clear routed mask bit
		     OBSVAL(x, y, lay) &= ~ROUTED_NET;
		  }

		  // Routes which had blockages added on the sides due
		  // to spacing constraints have DRC_BLOCKAGE set;
		  // these flags should be removed.

		  if (needblock[lay] & (ROUTEBLOCKX | VIABLOCKX)) {
		     if ((x > 0) && ((OBSVAL(x - 1, y, lay) &
				DRC_BLOCKAGE) == DRC_BLOCKAGE))
			clear_drc_blockage(x - 1, y, lay);
		     else if ((x < NumChannelsX - 1) &&
				((OBSVAL(x + 1, y, lay) &
				DRC_BLOCKAGE) == DRC_BLOCKAGE))
			clear_drc_blockage(x + 1, y, lay);
		  }
		  if (needblock[lay] & (ROUTEBLOCKY | VIABLOCKY)) {
		     if ((y > 0) && ((OBSVAL(x, y - 1, lay) &
				DRC_BLOCKAGE) == DRC_BLOCKAGE))
			clear_drc_blockage(x, y - 1, lay);
		     else if ((y < NumChannelsY - 1) &&
				((OBSVAL(x, y + 1, lay) &
				DRC_BLOCKAGE) == DRC_BLOCKAGE))
			clear_drc_blockage(x, y + 1, lay);
		  }
	       }

	       // Check for and handle via end on last route segment.

	       if ((x == seg->x2) && (y == seg->y2)) {
		  if (seg->segtype & ST_VIA) {
		      if (lay == seg->layer)
			 lay++;
		      else
		         break;
		  }
		  else
		     break;
	       }

	       if (x < seg->x2) x++;
	       else if (x > seg->x2) x--;
	       if (y < seg->y2) y++;
	       else if (y > seg->y2) y--;
	    }
	 }
      }
   }

   // For each net node tap, restore the node pointer on Nodeinfo->nodeloc
   // so that crossover costs are again applied to routes over this node
   // tap.

   if (restore != 0) {
      if (flagged) {
	 for (rt = net->routes; rt; rt = rt->next) {
	    if (!(rt->flags & RT_RIP)) continue;
	    for (seg = rt->segments; seg; seg = seg->next) {
	       lay = seg->layer;
	       if (lay >= Pinlayers) continue;
	       x = seg->x1;
	       y = seg->y1;
	       lnode = NODEIPTR(x, y, lay);
	       if (lnode && lnode->nodesav)
		   lnode->nodeloc = lnode->nodesav;
	    }
	 }
      }
      else {
         for (node = net->netnodes; node; node = node->next) {
	    for (ntap = node->taps; ntap; ntap = ntap->next) {
	       lay = ntap->layer;
	       x = ntap->gridx;
	       y = ntap->gridy;
	       if (lay < Pinlayers) {
		   lnode = NODEIPTR(x, y, lay);
		   if (lnode) lnode->nodeloc = lnode->nodesav;
	       }
	    }
	 }
      }
   }

   if (retain == FALSE) {

      remove_routes(net->routes, flagged);
      net->routes = NULL;

      // If we just ripped out a few of the routes, make sure all the
      // other net routes have not been overwritten.
      if (flagged) writeback_all_routes(net);
   }

   // If this was a specialnet (numnodes set to 0), then routes are
   // considered fixed obstructions and cannot be removed.

   return (net->numnodes == 0) ? FALSE : TRUE;
}

/*--------------------------------------------------------------*/
/* eval_pt - evaluate cost to get from given point to		*/
/*	current point.  Current point is passed in "ept", and	*/
/* 	the direction from the new point to the current point	*/
/*	is indicated by "flags".				*/
/*								*/
/*	ONLY consider the cost of the single step itself.	*/
/*								*/
/*      If "stage" is nonzero, then this is a second stage	*/
/*	routing, where we should consider other nets to be a	*/
/*	high cost to short to, rather than a blockage.  This	*/
/* 	will allow us to finish the route, but with a minimum	*/
/*	number of collisions with other nets.  Then, we rip up	*/
/*	those nets, add them to the "failed" stack, and re-	*/
/*	route this one.						*/
/*								*/
/*  ARGS: none							*/
/*  RETURNS: pointer to a new POINT record to put on the stack	*/
/*	if the node needs to be (re)processed and isn't	already	*/
/*	on the stack, NULL otherwise.				*/
/*  SIDE EFFECTS: none (get this right or else)			*/
/*--------------------------------------------------------------*/

POINT eval_pt(GRIDP *ept, u_char flags, u_char stage)
{
    int thiscost = 0;
    int netnum;
    NODE node;
    NODEINFO nodeptr, lnode;
    NETLIST nl;
    PROUTE *Pr, *Pt;
    GRIDP newpt;
    POINT ptret = NULL;

    newpt = *ept;

    // ConflictCost is passed in flags if "force" option is set
    // and this route crosses a prohibited boundary.  This allows
    // the prohibited move but gives it a high cost.

    if (flags & PR_CONFLICT) {
	thiscost = ConflictCost * 10;
	flags &= ~PR_CONFLICT;
    }

    switch (flags) {
       case PR_PRED_N:
	  newpt.y--;
	  break;
       case PR_PRED_S:
	  newpt.y++;
	  break;
       case PR_PRED_E:
	  newpt.x--;
	  break;
       case PR_PRED_W:
	  newpt.x++;
	  break;
       case PR_PRED_U:
	  newpt.lay--;
	  break;
       case PR_PRED_D:
	  newpt.lay++;
	  break;
    }

    Pr = &OBS2VAL(newpt.x, newpt.y, newpt.lay);
    nodeptr = (newpt.lay < Pinlayers) ?
		NODEIPTR(newpt.x, newpt.y, newpt.lay) : NULL;

    if (!(Pr->flags & (PR_COST | PR_SOURCE))) {
       // 2nd stage allows routes to cross existing routes
       netnum = Pr->prdata.net;
       if (stage && (netnum < MAXNETNUM)) {
	  if ((newpt.lay < Pinlayers) && nodeptr && (nodeptr->nodesav != NULL))
	     return NULL;		// But cannot route over terminals!

	  // Is net k in the "noripup" list?  If so, don't route it */

	  for (nl = CurNet->noripup; nl; nl = nl->next) {
	     if (nl->net->netnum == netnum)
		return NULL;
	  }

	  // In case of a collision, we change the grid point to be routable
	  // but flag it as a point of collision so we can later see what
	  // were the net numbers of the interfering routes by cross-referencing
	  // the Obs[][] array.

	  Pr->flags |= (PR_CONFLICT | PR_COST);
	  Pr->prdata.cost = MAXRT;
	  thiscost += ConflictCost;
       }
       else if (stage && ((netnum & DRC_BLOCKAGE) == DRC_BLOCKAGE)) {
	  if ((newpt.lay < Pinlayers) && nodeptr && (nodeptr->nodesav != NULL))
	     return NULL;		// But cannot route over terminals!

	  // Position does not contain the net number, so we have to
	  // go looking for it.  Fortunately this is a fairly rare
	  // occurrance.  But it is necessary to find all neighboring
	  // nets that might have created the blockage, and refuse to
	  // route here if any of them are on the noripup list.

	  if (needblock[newpt.lay] & (ROUTEBLOCKX | VIABLOCKX)) {
	     if (newpt.x < NumChannelsX - 1) {
	        netnum = OBSVAL(newpt.x + 1, newpt.y, newpt.lay) & ROUTED_NET_MASK;
	        if (!(netnum & NO_NET)) {
		   netnum &= NETNUM_MASK;
		   if ((netnum != 0) && (netnum != CurNet->netnum))
	              // Is net k in the "noripup" list?  If so, don't route it */
	              for (nl = CurNet->noripup; nl; nl = nl->next)
	                 if (nl->net->netnum == netnum)
		            return NULL;
		}
	     }

	     if (newpt.x > 0) {
	        netnum = OBSVAL(newpt.x - 1, newpt.y, newpt.lay) & ROUTED_NET_MASK;
	        if (!(netnum & NO_NET)) {
		   netnum &= NETNUM_MASK;
		   if ((netnum != 0) && (netnum != CurNet->netnum))
	              // Is net k in the "noripup" list?  If so, don't route it */
	              for (nl = CurNet->noripup; nl; nl = nl->next)
	                 if (nl->net->netnum == netnum)
		            return NULL;
		}
	     }
	  } 
	  if (needblock[newpt.lay] & (ROUTEBLOCKY | VIABLOCKY)) {
	     if (newpt.y < NumChannelsY - 1) {
	        netnum = OBSVAL(newpt.x, newpt.y + 1, newpt.lay) & ROUTED_NET_MASK;
	        if (!(netnum & NO_NET)) {
		   netnum &= NETNUM_MASK;
		   if ((netnum != 0) && (netnum != CurNet->netnum))
	              // Is net k in the "noripup" list?  If so, don't route it */
	              for (nl = CurNet->noripup; nl; nl = nl->next)
	                 if (nl->net->netnum == netnum)
		            return NULL;
		}
	     }

	     if (newpt.y > 0) {
	        netnum = OBSVAL(newpt.x, newpt.y - 1, newpt.lay) & ROUTED_NET_MASK;
	        if (!(netnum & NO_NET)) {
		   netnum &= NETNUM_MASK;
		   if ((netnum != 0) && (netnum != CurNet->netnum))
	              // Is net k in the "noripup" list?  If so, don't route it */
	              for (nl = CurNet->noripup; nl; nl = nl->next)
	                 if (nl->net->netnum == netnum)
		            return NULL;
		}
	     }
	  }

	  // In case of a collision, we change the grid point to be routable
	  // but flag it as a point of collision so we can later see what
	  // were the net numbers of the interfering routes by cross-referencing
	  // the Obs[][] array.

	  Pr->flags |= (PR_CONFLICT | PR_COST);
	  Pr->prdata.cost = MAXRT;
	  thiscost += ConflictCost;
       }
       else
          return NULL;		// Position is not routeable
    }

    // Compute the cost to step from the current point to the new point.
    // "BlockCost" is used if the node has only one point to connect to,
    // so that routing over it could block it entirely.

    if ((newpt.lay > 0) && (newpt.lay < Pinlayers)) {
	if (((lnode = NODEIPTR(newpt.x, newpt.y, newpt.lay - 1)) != (NODEINFO)NULL)
		&& ((node = lnode->nodeloc) != NULL)) {
	    Pt = &OBS2VAL(newpt.x, newpt.y, newpt.lay - 1);
	    if (!(Pt->flags & PR_TARGET) && !(Pt->flags & PR_SOURCE)) {
		if (node->taps && (node->taps->next == NULL))
		   thiscost += BlockCost;	// Cost to block out a tap
		else if (node->taps == NULL) {
		   if (node->extend != NULL) {
			if (node->extend->next == NULL)
			   // Node has only one extended access point:  Try
			   // very hard to avoid routing over it 
			   thiscost += 10 * BlockCost;
			else
			   thiscost += BlockCost;
		   }
		   // If both node->taps and node->extend are NULL, then
		   // the node has no access and will never be routed, so
		   // don't bother costing it.
		}
		else
	           thiscost += XverCost;	// Cross-under cost
	    }
	}
    }
    if (((newpt.lay + 1) < Pinlayers) && (newpt.lay < Num_layers - 1)) {
	if (((lnode = NODEIPTR(newpt.x, newpt.y, newpt.lay + 1)) != (NODEINFO)NULL)
		&& ((node = lnode->nodeloc) != NULL)) {
	    Pt = &OBS2VAL(newpt.x, newpt.y, newpt.lay + 1);
	    if (!(Pt->flags & PR_TARGET) && !(Pt->flags & PR_SOURCE)) {
		if (node->taps && (node->taps->next == NULL))
		   thiscost += BlockCost;	// Cost to block out a tap
		else
	           thiscost += XverCost;	// Cross-over cost
	    }
	}
    }
    if (ept->lay != newpt.lay) thiscost += ViaCost;
    if (ept->x != newpt.x) thiscost += (Vert[newpt.lay] * JogCost +
			(1 - Vert[newpt.lay]) * SegCost);
    if (ept->y != newpt.y) thiscost += (Vert[newpt.lay] * SegCost +
			(1 - Vert[newpt.lay]) * JogCost);

    // Add the cost to the cost of the original position
    thiscost += ept->cost;

    // Routes that reach nodes are given a cost based on the "quality"
    // of the node location:  higher cost given to stub routes and
    // offset positions.

    if (nodeptr != NULL) {
       thiscost += (int)(fabsf(nodeptr->stub) * (float)OffsetCost);
    }
   
    // Replace node information if cost is minimum

    if (Pr->flags & PR_CONFLICT)
       thiscost += ConflictCost;	// For 2nd stage routes

    if (thiscost < Pr->prdata.cost) {
       Pr->flags &= ~PR_PRED_DMASK;
       Pr->flags |= flags;
       Pr->prdata.cost = thiscost;
       Pr->flags &= ~PR_PROCESSED;	// Need to reprocess this node

       if (Verbose > 3) {
	  Fprintf(stdout, "New cost %d at (%d %d %d)\n", thiscost,
		newpt.x, newpt.y, newpt.lay);
       }
       if (~(Pr->flags & PR_ON_STACK)) {
	  Pr->flags |= PR_ON_STACK;
	  ptret = allocPOINT();
	  ptret->x1 = newpt.x;
	  ptret->y1 = newpt.y;
	  ptret->layer = newpt.lay;
	  ptret->next = NULL;
	  return ptret;
       }
    }
    return NULL;	// New position did not get a lower cost

} /* eval_pt() */

/*------------------------------------------------------*/
/* writeback_segment() ---				*/
/*							*/
/*	Copy information from a single segment back	*/
/* 	the Obs[] array.				*/
/*							*/
/* NOTE:  "needblock" is used to handle			*/
/* cases where the existence of a route prohibits any	*/
/* routing on adjacent tracks on the same plane due to	*/
/* DRC restrictions (i.e., metal is too wide for single	*/
/* track spacing).  Be sure to check the value of	*/
/* adjacent positions in Obs against the mask		*/
/* NETNUM_MASK, because NETNUM_MASK includes NO_NET.	*/
/* By replacing only empty and routable positions with	*/
/* the unique flag combination	DRC_BLOCKAGE, it	*/
/* is possible to detect and remove the same if that	*/
/* net is ripped up, without touching any position	*/
/* originally marked NO_NET.				*/
/*							*/
/* Another NOTE:  Tap offsets can cause the position	*/
/* in front of the offset to be unroutable.  So if the	*/
/* segment is on a tap offset, mark the position in	*/
/* front as unroutable.  If the segment neighbors an	*/
/* offset tap, then mark the tap unroutable.		*/
/*------------------------------------------------------*/

void writeback_segment(SEG seg, int netnum)
{
   double dist;
   int  i, layer, dir;
   u_int sobs;
   NODEINFO lnode;

   if (seg->segtype & ST_VIA) {
      /* Preserve blocking information */
      dir = OBSVAL(seg->x1, seg->y1, seg->layer + 1) & (BLOCKED_MASK | PINOBSTRUCTMASK);
      OBSVAL(seg->x1, seg->y1, seg->layer + 1) = netnum | dir;
      if (needblock[seg->layer + 1] & VIABLOCKX) {
	 if (seg->x1 < (NumChannelsX - 1))
 	    set_drc_blockage(seg->x1 + 1, seg->y1, seg->layer + 1);
	 if (seg->x1 > 0)
	    set_drc_blockage(seg->x1 - 1, seg->y1, seg->layer + 1);
      }
      if (needblock[seg->layer + 1] & VIABLOCKY) {
	 if (seg->y1 < (NumChannelsY - 1))
	    set_drc_blockage(seg->x1, seg->y1 + 1, seg->layer + 1);
	 if (seg->y1 > 0)
	    set_drc_blockage(seg->x1, seg->y1 - 1, seg->layer + 1);
      }

      // If position itself is an offset route, then make the route position
      // on the forward side of the offset unroutable, on both via layers.
      // (Like the above code, there is no check for whether the offset
      // distance is enough to cause a DRC violation.)

      layer = (seg->layer == 0) ? 0 : seg->layer - 1;
      sobs = OBSVAL(seg->x1, seg->y1, seg->layer);
      if (sobs & OFFSET_TAP) {
	 lnode = NODEIPTR(seg->x1, seg->y1, seg->layer);
	 dist = lnode->offset;
	 if (lnode->flags & NI_OFFSET_EW) {
	    if ((dist > 0) && (seg->x1 < (NumChannelsX - 1))) {
	       set_drc_blockage(seg->x1 + 1, seg->y1, seg->layer);
	       set_drc_blockage(seg->x1 + 1, seg->y1, seg->layer + 1);
	    }
	    if ((dist < 0) && (seg->x1 > 0)) {
	       set_drc_blockage(seg->x1 - 1, seg->y1, seg->layer);
	       set_drc_blockage(seg->x1 - 1, seg->y1, seg->layer + 1);
	    }
	 }
	 else if (lnode->flags & NI_OFFSET_NS) {
	    if ((dist > 0) && (seg->y1 < (NumChannelsY - 1))) {
	       set_drc_blockage(seg->x1, seg->y1 + 1, seg->layer);
	       set_drc_blockage(seg->x1, seg->y1 + 1, seg->layer + 1);
	    }
	    if ((dist < 0) && (seg->y1 > 0)) {
	       set_drc_blockage(seg->x1, seg->y1 - 1, seg->layer);
	       set_drc_blockage(seg->x1, seg->y1 - 1, seg->layer + 1);
	    }
	 }
      }
   }

   for (i = seg->x1; ; i += (seg->x2 > seg->x1) ? 1 : -1) {
      dir = OBSVAL(i, seg->y1, seg->layer) & (BLOCKED_MASK | PINOBSTRUCTMASK);
      OBSVAL(i, seg->y1, seg->layer) = netnum | dir;
      if (needblock[seg->layer] & ROUTEBLOCKY) {
         if (seg->y1 < (NumChannelsY - 1))
	    set_drc_blockage(i, seg->y1 + 1, seg->layer);
	 if (seg->y1 > 0)
	    set_drc_blockage(i, seg->y1 - 1, seg->layer);
      }

      // Check position on each side for an offset tap on a different net, and
      // mark the position unroutable.
      //
      // NOTE:  This is a bit conservative, as it will block positions next to
      // an offset tap without checking if the offset distance is enough to
      // cause a DRC error.  Could be refined. . .

      layer = (seg->layer == 0) ? 0 : seg->layer - 1;
      if (seg->y1 < (NumChannelsY - 1)) {
	 sobs = OBSVAL(i, seg->y1 + 1, layer);
	 if ((sobs & OFFSET_TAP) && !(sobs & ROUTED_NET)) {
	    lnode = NODEIPTR(i, seg->y1 + 1, layer);
	    if (lnode->flags & NI_OFFSET_NS) {
	       dist = lnode->offset;
	       if (dist < 0) {
		  set_drc_blockage(i, seg->y1 + 1, layer);
	       }
	    }
	 }
      }
      if (seg->y1 > 0) {
	 sobs = OBSVAL(i, seg->y1 - 1, layer);
	 if ((sobs & OFFSET_TAP) && !(sobs & ROUTED_NET)) {
	    lnode = NODEIPTR(i, seg->y1 - 1, layer);
	    if (lnode->flags & NI_OFFSET_NS) {
	       dist = lnode->offset;
	       if (dist > 0) {
		  set_drc_blockage(i, seg->y1 - 1, layer);
	       }
	    }
	 }
      }
      if (i == seg->x2) break;
   }

   /* Check top of route for vertical routes */

   if (seg->y1 != seg->y2) {
      dir = OBSVAL(seg->x2, seg->y2, seg->layer) & (BLOCKED_MASK | PINOBSTRUCTMASK);
      OBSVAL(seg->x2, seg->y2, seg->layer) = netnum | dir;
      if (needblock[seg->layer] & ROUTEBLOCKY) {
         if (seg->y2 < (NumChannelsY - 1))
	    set_drc_blockage(seg->x2, seg->y2 + 1, seg->layer);
	 if (seg->y2 > 0)
	    set_drc_blockage(seg->x2, seg->y2 - 1, seg->layer);
      }
   }


   for (i = seg->y1; ; i += (seg->y2 > seg->y1) ? 1 : -1) {
      dir = OBSVAL(seg->x1, i, seg->layer) & (BLOCKED_MASK | PINOBSTRUCTMASK);
      OBSVAL(seg->x1, i, seg->layer) = netnum | dir;
      if (needblock[seg->layer] & ROUTEBLOCKX) {
	 if (seg->x1 < (NumChannelsX - 1))
	    set_drc_blockage(seg->x1 + 1, i, seg->layer);
	 if (seg->x1 > 0)
	    set_drc_blockage(seg->x1 - 1, i, seg->layer);
      }

      // Check position on each side for an offset tap on a different net, and
      // mark the position unroutable (see above).

      layer = (seg->layer == 0) ? 0 : seg->layer - 1;
      if (seg->x1 < (NumChannelsX - 1)) {
	 sobs = OBSVAL(seg->x1 + 1, i, layer);
	 if ((sobs & OFFSET_TAP) && !(sobs & ROUTED_NET)) {
	    lnode = NODEIPTR(seg->x1 + 1, i, layer);
	    if (lnode->flags & NI_OFFSET_EW) {
	       dist = lnode->offset;
	       if (dist < 0) {
		  set_drc_blockage(seg->x1 + 1, i, layer);
	       }
	    }
	 }
      }
      if (seg->x1 > 0) {
	 sobs = OBSVAL(seg->x1 - 1, i, layer);
	 if ((sobs & OFFSET_TAP) && !(sobs & ROUTED_NET)) {
	    lnode = NODEIPTR(seg->x1 - 1, i, layer);
	    if (lnode->flags & NI_OFFSET_EW) {
	       dist = lnode->offset;
	       if (dist > 0) {
		  set_drc_blockage(seg->x1 - 1, i, layer);
	       }
	    }
	 }
      }
      if (i == seg->y2) break;
   }

   /* Check end of route for horizontal routes */

   if (seg->x1 != seg->x2) {
      dir = OBSVAL(seg->x2, seg->y2, seg->layer) & (BLOCKED_MASK | PINOBSTRUCTMASK);
      OBSVAL(seg->x2, seg->y2, seg->layer) = netnum | dir;
      if (needblock[seg->layer] & ROUTEBLOCKX) {
	 if (seg->x2 < (NumChannelsX - 1))
	    set_drc_blockage(seg->x2 + 1, seg->y2, seg->layer);
	 if (seg->x2 > 0)
	    set_drc_blockage(seg->x2 - 1, seg->y2, seg->layer);
      }
   }
}

/*--------------------------------------------------------------*/
/* Set the endpoint information for the route.	Look at the	*/
/* first and last points of the route, determine if they	*/
/* connect to a node or another route, and set the "start" and	*/
/* "end" records of the route, and the flags accordingly.	*/
/*--------------------------------------------------------------*/

void
route_set_connections(net, route)
   NET   net;
   ROUTE route;
{
   SEG      seg, s;
   ROUTE    nr;
   NODEINFO lnode;
   u_char   found, match;
   int	    x, y;

   /* Reset start/end node flags */
   route->flags &= ~(RT_START_NODE | RT_END_NODE);

   /* Does first route segment connect to a node? */

   seg = route->segments;
   found = FALSE;
   if (seg->layer < Pinlayers) {
      lnode = NODEIPTR(seg->x1, seg->y1, seg->layer);
      if ((lnode != NULL) && (lnode->nodesav != NULL)) {
	 route->start.node = lnode->nodesav;
	 route->flags |= RT_START_NODE;
	 found = TRUE;
      }
   }

   /* For routes that were just imported from DEF, check if the	*/
   /* route overshot a pin terminal by rounding off a non-grid	*/
   /* coordinate.						*/

   if (!found && (route->flags & RT_CHECK) && (seg->layer < Pinlayers)) {
      if (seg->x1 == seg->x2) {
	 x = seg->x1;
	 if (seg->y1 < seg->y2)
	    y = seg->y1 + 1;
         else
	    y = seg->y1 - 1;
      }
      else {
	 y = seg->y1;
	 if (seg->x1 < seg->x2)
	    x = seg->x1 + 1;
         else
	    x = seg->x1 - 1;
      }
      lnode = NODEIPTR(x, y, seg->layer);
      if ((lnode != NULL) && (lnode->nodesav != NULL) &&
		(lnode->nodesav->netnum == net->netnum) &&
		((x != seg->x2) || (y != seg->y2))) {
	 route->start.node = lnode->nodesav;
	 route->flags |= RT_START_NODE;
	 found = TRUE;
	 /* Diagnostic */
	 Fprintf(stderr, "Coordinate %d %d corrected to %d %d\n",
		seg->x1, seg->y1, x, y);
	 seg->x1 = x;
	 seg->y1 = y;
      }
   }

   /* Does first route segment connect to a route? */

   if (!found) {
      for (nr = net->routes; nr; nr = nr->next) {
         if (nr == route) continue;
         for (s = nr->segments; s; s = s->next) {
	    match = FALSE;
	    if (seg->layer == s->layer) match = TRUE;
	    else if ((seg->segtype & ST_VIA) && ((seg->layer + 1) == s->layer))
	       match = TRUE;
	    else if ((s->segtype & ST_VIA) && ((s->layer + 1) == seg->layer))
	       match = TRUE;
	    if (!match) continue;
	    x = s->x1;
	    y = s->y1;
	    if (x == seg->x1 && y == seg->y1) {
	       found = TRUE;
	       route->start.route = nr;
	       break;
	    }
	    while (TRUE) {
	       if (s->x2 != s->x1) x += ((s->x2 > s->x1) ? 1 : -1);
	       if (s->y2 != s->y1) y += ((s->y2 > s->y1) ? 1 : -1);
	       if (x == seg->x1 && y == seg->y1) {
		  found = TRUE;
		  route->start.route = nr;
		  break;
	       }
	       if (x == s->x2 && y == s->y2) break;
	    }
	    if (found) break;
	 }
	 if (found) break;
      }
   }

   if (!found) {
      Fprintf(stderr, "Error:  Failure to find route start node/route on net %s!\n",
		net->netname);
   }

   /* Does last route segment connect to a node? */

   /* NOTE:  Avoid the case where the route is exactly one via connecting */
   /* a node to a route directly above, in which case the following code  */
   /* would flag the node twice, incorrectly.				  */

   found = FALSE;
   if ((seg->next != NULL) || !(seg->segtype & ST_VIA)) {
      for (; seg->next; seg = seg->next);
      if (seg->layer < Pinlayers) {
         lnode = NODEIPTR(seg->x2, seg->y2, seg->layer);
         if ((lnode != NULL) && (lnode->nodesav != NULL)) {
	    route->end.node = lnode->nodesav;
	    route->flags |= RT_END_NODE;
	    found = TRUE;
	 }
      }

      /* For routes that were just imported from DEF, check if	*/
      /* the route overshot a pin terminal by rounding off a	*/
      /* non-grid coordinate.					*/

      if (!found && (route->flags & RT_CHECK) && (seg->layer < Pinlayers)) {
         if (seg->x1 == seg->x2) {
	    x = seg->x2;
	    if (seg->y1 < seg->y2)
	       y = seg->y2 - 1;
            else
	       y = seg->y2 + 1;
         }
         else {
	    y = seg->y2;
	    if (seg->x1 < seg->x2)
	       x = seg->x2 - 1;
            else
	       x = seg->x2 + 1;
         }
         lnode = NODEIPTR(x, y, seg->layer);
         if ((lnode != NULL) && (lnode->nodesav != NULL) &&
			(lnode->nodesav->netnum == net->netnum) &&
			((x != seg->x1) || (y != seg->y1))) {
	    route->start.node = lnode->nodesav;
	    route->flags |= RT_END_NODE;
	    found = TRUE;
	    /* Diagnostic */
	    Fprintf(stderr, "Coordinate %d %d corrected to %d %d\n",
			seg->x2, seg->y2, x, y);
	    seg->x2 = x;
	    seg->y2 = y;
         }
      }
   }

   /* Does last route segment connect to a route? */

   if (!found) {
      for (nr = net->routes; nr; nr = nr->next) {
         if (nr == route) continue;
         for (s = nr->segments; s; s = s->next) {
	    match = FALSE;
	    if (seg->layer == s->layer) match = TRUE;
	    else if ((seg->segtype & ST_VIA) && ((seg->layer + 1) == s->layer))
	       match = TRUE;
	    else if ((s->segtype & ST_VIA) && ((s->layer + 1) == seg->layer))
	       match = TRUE;
	    if (!match) continue;
	    x = s->x1;
	    y = s->y1;
	    if (x == seg->x2 && y == seg->y2 && nr != route->start.route) {
	       found = TRUE;
	       route->end.route = nr;
	       break;
	    }
	    while (TRUE) {
	       if (s->x2 != s->x1) x += ((s->x2 > s->x1) ? 1 : -1);
	       if (s->y2 != s->y1) y += ((s->y2 > s->y1) ? 1 : -1);
	       if (x == seg->x2 && y == seg->y2 && nr != route->start.route) {
		  found = TRUE;
		  route->end.route = nr;
		  break;
	       }
	       if (x == s->x2 && y == s->y2) break;
	    }
	    if (found) break;
	 }
	 if (found) break;
      }
   }

   if (!found) {
      Fprintf(stderr, "Error:  Failure to find route end node/route on net %s!\n",
		net->netname);
   }

   /* Clear RT_CHECK flag after processing */
   if (route->flags & RT_CHECK) route->flags &= ~RT_CHECK;
}

/*--------------------------------------------------------------*/
/* commit_proute - turn the potential route into an actual	*/
/*		route by generating the route segments		*/
/*								*/
/*  ARGS:   route structure to fill;  "stage" is 1 if we're on	*/
/*	    second stage routing, in which case we fill the	*/
/*	    route structure but don't modify the Obs array.	*/
/*								*/
/*  RETURNS: 1 on success, 0 on stacked via failure, and 	*/
/*	    -1 on failure to find a terminal.  On a stacked	*/
/* 	    via failure, the route is committed anyway.		*/
/*								*/
/*  SIDE EFFECTS: Obs update, RT llseg added			*/
/*--------------------------------------------------------------*/

int commit_proute(ROUTE rt, GRIDP *ept, u_char stage)
{
   SEG  seg, lseg;
   NODEINFO lnode1, lnode2;
   int  lay2, rval;
   int  dx = -1, dy = -1, dl;
   u_int netnum, netobs1, netobs2, dir1, dir2;
   u_char first = (u_char)1;
   u_char dmask;
   u_char pflags, p2flags;
   PROUTE *Pr;
   POINT newlr, newlr2, lrtop, lrend, lrnext, lrcur, lrprev;

   if (Verbose > 1) {
      Flush(stdout);
      Fprintf(stdout, "\nCommit: TotalRoutes = %d\n", TotalRoutes);
   }

   netnum = rt->netnum;

   Pr = &OBS2VAL(ept->x, ept->y, ept->lay);
   if (!(Pr->flags & PR_COST)) {
      Fprintf(stderr, "commit_proute(): impossible - terminal is not routable!\n");
      return -1;
   }

   // Generate an indexed route, recording the series of predecessors and their
   // positions.

   lrtop = (POINT)malloc(sizeof(struct point_));
   lrtop->x1 = ept->x;
   lrtop->y1 = ept->y;
   lrtop->layer = ept->lay;
   lrtop->next = NULL;
   lrend = lrtop;

   while (1) {

      Pr = &OBS2VAL(lrend->x1, lrend->y1, lrend->layer);
      dmask = Pr->flags & PR_PRED_DMASK;
      if (dmask == PR_PRED_NONE) break;

      newlr = (POINT)malloc(sizeof(struct point_));
      newlr->x1 = lrend->x1;
      newlr->y1 = lrend->y1;
      newlr->layer = lrend->layer;
      lrend->next = newlr;
      newlr->next = NULL;

      switch (dmask) {
         case PR_PRED_N:
	    (newlr->y1)++;
	    break;
         case PR_PRED_S:
	    (newlr->y1)--;
	    break;
         case PR_PRED_E:
	    (newlr->x1)++;
	    break;
         case PR_PRED_W:
	    (newlr->x1)--;
	    break;
         case PR_PRED_U:
	    (newlr->layer)++;
	    break;
         case PR_PRED_D:
	    (newlr->layer)--;
	    break;
      }
      lrend = newlr;
   }
   lrend = lrtop;

   // TEST:  Walk through the solution, and look for stacked vias.  When
   // found, look for an alternative path that avoids the stack.

   rval = 1;
   if (StackedContacts < (Num_layers - 1)) {
      POINT lrppre;
      POINT a, b;
      PROUTE *pri, *pri2;
      int stacks = 1, stackheight;
      int cx, cy, cl;
      int mincost, minx = -1, miny = -1, collide, cost;

      while (stacks != 0) {	// Keep doing until all illegal stacks are gone
	 stacks = 0;
	 lrcur = lrend;
	 lrprev = lrend->next;

	 while (lrprev != NULL) {
	    lrppre = lrprev->next;
	    if (lrppre == NULL) break;
	    stackheight = 0;
	    /* Advance lrcur if jogs were inserted */
	    while (lrprev != lrcur->next) lrcur = lrcur->next;
	    a = lrcur;
	    b = lrprev;
	    while (a->layer != b->layer) {
	       stackheight++;
	       a = b;
	       b = a->next;
	       if (b == NULL) break;
	    }
	    collide = FALSE;
	    while (stackheight > StackedContacts) {	// Illegal stack found
	       stacks++;

	       // Try to move the second contact in the path
	       cx = lrprev->x1;
	       cy = lrprev->y1;
	       cl = lrprev->layer;
	       mincost = MAXRT;
	       dl = lrppre->layer;

	       // Check all four positions around the contact for the
	       // lowest cost, and make sure the position below that
	       // is available.

	       if (cx < NumChannelsX - 1) {
	          dx = cx + 1;	// Check to the right
	          pri = &OBS2VAL(dx, cy, cl);
	          pflags = pri->flags;
	          cost = pri->prdata.cost;
	          if (collide && !(pflags & (PR_COST | PR_SOURCE)) &&
				(pri->prdata.net < MAXNETNUM)) {
		     pflags = 0;
		     cost = ConflictCost;
	          }
	          if (pflags & PR_COST) {
		     pflags &= ~PR_COST;
		     if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE && cost < mincost) {
	                pri2 = &OBS2VAL(dx, cy, dl);
		        p2flags = pri2->flags;
		        if (p2flags & PR_COST) {
			   p2flags &= ~PR_COST;
		           if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		              mincost = cost;
		              minx = dx;
		              miny = cy;
			   }
		        }
		        else if (collide && !(p2flags & (PR_COST | PR_SOURCE)) &&
				(pri2->prdata.net < MAXNETNUM) &&
				((cost + ConflictCost) < mincost)) {
			   mincost = cost + ConflictCost;
			   minx = dx;
			   miny = dy;
		        }
		     }
	          }
	       }
	       if (cx > 0) {
	          dx = cx - 1;	// Check to the left
	          pri = &OBS2VAL(dx, cy, cl);
	          pflags = pri->flags;
	          cost = pri->prdata.cost;
	          if (collide && !(pflags & (PR_COST | PR_SOURCE)) &&
				(pri->prdata.net < MAXNETNUM)) {
		     pflags = 0;
		     cost = ConflictCost;
	          }
	          if (pflags & PR_COST) {
		     pflags &= ~PR_COST;
		     if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE && cost < mincost) {
	                pri2 = &OBS2VAL(dx, cy, dl);
		        p2flags = pri2->flags;
		        if (p2flags & PR_COST) {
			   p2flags &= ~PR_COST;
		           if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		              mincost = cost;
		              minx = dx;
		              miny = cy;
			   }
		        }
		        else if (collide && !(p2flags & (PR_COST | PR_SOURCE)) &&
				(pri2->prdata.net < MAXNETNUM) &&
				((cost + ConflictCost) < mincost)) {
			   mincost = cost + ConflictCost;
			   minx = dx;
			   miny = dy;
			}
		     }
		  }
	       }

	       if (cy < NumChannelsY - 1) {
	          dy = cy + 1;	// Check north
	          pri = &OBS2VAL(cx, dy, cl);
	          pflags = pri->flags;
	          cost = pri->prdata.cost;
	          if (collide && !(pflags & (PR_COST | PR_SOURCE)) &&
				(pri->prdata.net < MAXNETNUM)) {
		     pflags = 0;
		     cost = ConflictCost;
	          }
	          if (pflags & PR_COST) {
		     pflags &= ~PR_COST;
		     if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE && cost < mincost) {
	                pri2 = &OBS2VAL(cx, dy, dl);
		        p2flags = pri2->flags;
		        if (p2flags & PR_COST) {
			   p2flags &= ~PR_COST;
		           if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		              mincost = cost;
		              minx = cx;
		              miny = dy;
			   }
		        }
		        else if (collide && !(p2flags & (PR_COST | PR_SOURCE)) &&
				(pri2->prdata.net < MAXNETNUM) &&
				((cost + ConflictCost) < mincost)) {
			   mincost = cost + ConflictCost;
			   minx = dx;
			   miny = dy;
			}
		     }
		  }
	       }

	       if (cy > 0) {
	          dy = cy - 1;	// Check south
	          pri = &OBS2VAL(cx, dy, cl);
	          pflags = pri->flags;
	          cost = pri->prdata.cost;
	          if (collide && !(pflags & (PR_COST | PR_SOURCE)) &&
				(pri->prdata.net < MAXNETNUM)) {
		     pflags = 0;
		     cost = ConflictCost;
	          }
	          if (pflags & PR_COST) {
		     pflags &= ~PR_COST;
		     if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE && cost < mincost) {
	                pri2 = &OBS2VAL(cx, dy, dl);
		        p2flags = pri2->flags;
		        if (p2flags & PR_COST) {
		           p2flags &= ~PR_COST;
		           if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		              mincost = cost;
		              minx = cx;
		              miny = dy;
			   }
		        }
		        else if (collide && !(p2flags & (PR_COST | PR_SOURCE)) &&
				(pri2->prdata.net < MAXNETNUM) &&
				((cost + ConflictCost) < mincost)) {
			   mincost = cost + ConflictCost;
			   minx = dx;
			   miny = dy;
			}
		     }
		  }
	       }

	       // Was there an available route?  If so, modify
	       // records to route through this alternate path.  If not,
	       // then try to move the first contact instead.

	       if (mincost < MAXRT) {
	          pri = &OBS2VAL(minx, miny, cl);

		  newlr = (POINT)malloc(sizeof(struct point_));
		  newlr->x1 = minx;
		  newlr->y1 = miny;
		  newlr->layer = cl;

	          pri2 = &OBS2VAL(minx, miny, dl);

		  newlr2 = (POINT)malloc(sizeof(struct point_));
		  newlr2->x1 = minx;
		  newlr2->y1 = miny;
		  newlr2->layer = dl;

		  lrprev->next = newlr;
		  newlr->next = newlr2;

		  // Check if point at pri2 is equal to position of
		  // lrppre->next.  If so, bypass lrppre.

		  if ((lrnext = lrppre->next) != NULL) {
		     if (lrnext->x1 == minx && lrnext->y1 == miny &&
				lrnext->layer == dl) {
			newlr->next = lrnext;
			free(lrppre);
			free(newlr2);
			lrppre = lrnext;	// ?
		     }
		     else
		        newlr2->next = lrppre;
		  }
		  else
		     newlr2->next = lrppre;

		  break;	// Found a solution;  we're done.
	       }
	       else {

		  // If we couldn't offset lrprev position, then try
		  // offsetting lrcur.

	          cx = lrcur->x1;
	          cy = lrcur->y1;
	          cl = lrcur->layer;
	          mincost = MAXRT;
	          dl = lrprev->layer;

		  if (cx < NumChannelsX - 1) {
	             dx = cx + 1;	// Check to the right
	             pri = &OBS2VAL(dx, cy, cl);
	             pflags = pri->flags;
		     if (pflags & PR_COST) {
		        pflags &= ~PR_COST;
		        if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri->prdata.cost < mincost) {
	                   pri2 = &OBS2VAL(dx, cy, dl);
		           p2flags = pri2->flags;
			   if (p2flags & PR_COST) {
			      p2flags &= ~PR_COST;
		              if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		                 mincost = pri->prdata.cost;
		                 minx = dx;
		                 miny = cy;
			      }
			   }
		        }
		     }
	          }

		  if (cx > 0) {
	             dx = cx - 1;	// Check to the left
	             pri = &OBS2VAL(dx, cy, cl);
	             pflags = pri->flags;
		     if (pflags & PR_COST) {
		        pflags &= ~PR_COST;
		        if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri->prdata.cost < mincost) {
	                   pri2 = &OBS2VAL(dx, cy, dl);
		           p2flags = pri2->flags;
			   if (p2flags & PR_COST) {
			      p2flags &= ~PR_COST;
		              if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		                 mincost = pri->prdata.cost;
		                 minx = dx;
		                 miny = cy;
			      }
			   }
		        }
		     }
	          }

		  if (cy < NumChannelsY - 1) {
	             dy = cy + 1;	// Check north
	             pri = &OBS2VAL(cx, dy, cl);
	             pflags = pri->flags;
		     if (pflags & PR_COST) {
		        pflags &= ~PR_COST;
		        if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri->prdata.cost < mincost) {
	                   pri2 = &OBS2VAL(cx, dy, dl);
		           p2flags = pri2->flags;
			   if (p2flags & PR_COST) {
			      p2flags &= ~PR_COST;
		              if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		                 mincost = pri->prdata.cost;
		                 minx = cx;
		                 miny = dy;
			      }
			   }
		        }
		     }
	          }

		  if (cy > 0) {
	             dy = cy - 1;	// Check south
	             pri = &OBS2VAL(cx, dy, cl);
	             pflags = pri->flags;
		     if (pflags & PR_COST) {
		        pflags &= ~PR_COST;
		        if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri->prdata.cost < mincost) {
	                   pri2 = &OBS2VAL(cx, dy, dl);
		           p2flags = pri2->flags;
			   if (p2flags & PR_COST) {
			      p2flags &= ~PR_COST;
		              if ((p2flags & PR_PRED_DMASK) != PR_PRED_NONE &&
					pri2->prdata.cost < MAXRT) {
		                 mincost = pri->prdata.cost;
		                 minx = cx;
		                 miny = dy;
			      }
			   }
		        }
		     }
	          }

		  if (mincost < MAXRT) {
		     newlr = (POINT)malloc(sizeof(struct point_));
		     newlr->x1 = minx;
		     newlr->y1 = miny;
		     newlr->layer = cl;

		     newlr2 = (POINT)malloc(sizeof(struct point_));
		     newlr2->x1 = minx;
		     newlr2->y1 = miny;
		     newlr2->layer = dl;

		     // If newlr is a source or target, then make it
		     // the endpoint, because we have just moved the
		     // endpoint along the source or target, and the
		     // original endpoint position is not needed.

	             pri = &OBS2VAL(minx, miny, cl);
	             pri2 = &OBS2VAL(lrcur->x1, lrcur->y1, lrcur->layer);
		     if ((((pri->flags & PR_SOURCE) && (pri2->flags & PR_SOURCE)) ||
			 ((pri->flags & PR_TARGET) && (pri2->flags & PR_TARGET)))
                         && (lrcur == lrtop)
			) {
			lrtop = newlr;
			lrend = newlr;
			free(lrcur);
			lrcur = newlr;
		     }
		     else
		        lrcur->next = newlr;

		     newlr->next = newlr2;

		     // Check if point at pri2 is equal to position of
		     // lrprev->next.  If so, bypass lrprev.

		     if (lrppre->x1 == minx && lrppre->y1 == miny &&
				lrppre->layer == dl) {
			newlr->next = lrppre;
			free(lrprev);
			free(newlr2);
			lrprev = lrcur;
		     }
		     else
			newlr2->next = lrprev;
		
		     break;	// Found a solution;  we're done.
		  }
		  else if (stage == 0) {
		     // On the first stage, we call it an error and move
		     // on to the next net.  This is a bit conservative,
		     // but it works because failing to remove a stacked
		     // via is a rare occurrance.

		     if (Verbose > 0)
			Fprintf(stderr, "Failed to remove stacked via "
				"at grid point %d %d.\n",
				lrcur->x1, lrcur->y1);
		     stacks = 0;
		     rval = 0;
		     goto cleanup;
		  }
		  else {
		     if (collide == TRUE) {
		        Fprintf(stderr, "Failed to remove stacked via "
				"at grid point %d %d;  position may "
				"not be routable.\n",
				lrcur->x1, lrcur->y1);
			stacks = 0;
			rval = 0;
			goto cleanup;
		     }

		     // On the second stage, we will run through the
		     // search again, but allow overwriting other
		     // nets, which will be treated like other colliding
		     // nets in the regular route path search.

		     collide = TRUE;
		  }
	       }
	    }
	    lrcur = lrprev;
	    lrprev = lrppre;
	 }
      }
   }

   /* Handle minimum metal area rule for stacked contacts */

   else if (StackedContacts > 0) {

      /* Search for all stacked contacts.  For each via position inside	*/
      /* a stack, excluding top and bottom, determine if a minimum 	*/
      /* metal area is violated.  If so, then find an unused		*/
      /* neighboring grid and extend to that side.  If no unused	*/
      /* neighboring grids exist, then fail.				*/

      POINT lrppre;
      PROUTE *pri, *pri2;
      int violations = 1, checks;
      int i, cx, cy, cl, o, orient;
      int mincost, minx = -1, miny = -1, collide, cost;
      double min_area[MAX_LAYERS];
      u_char need_check[MAX_LAYERS];

      /* Register the minimum metal area rule for each via base layer */

      checks = 0;
      for (i = 0; i < Num_layers; i++) {
	 min_area[i] = LefGetRouteMinArea(i);
	 if (i == Num_layers - 1) {
	    if (min_area[i] <= LefGetViaWidth(i - 1, i, 0) * LefGetViaWidth(i - 1, i, 1))
	       need_check[i] = (u_char)0;
	    else {
	       need_check[i] = (u_char)1;
	       checks++;
	    }
	 }
	 else {
	    if (min_area[i] <= LefGetViaWidth(i, i, 0) * LefGetViaWidth(i, i, 1))
	       need_check[i] = (u_char)0;
	    else {
	       need_check[i] = (u_char)1;
	       checks++;
	    }
	 }
      }

      if (checks == 0) violations = 0;	// No minimum area violations possible.

      while (violations != 0) {	// Keep doing until all min. area violations are gone
	 violations = 0;
	 lrcur = lrend;
	 lrprev = lrend->next;

	 while (lrprev != NULL) {
	    lrppre = lrprev->next;
	    if (lrppre == NULL) break;
	    collide = FALSE;
	    while ((lrcur->layer != lrprev->layer) && (lrprev->layer != lrppre->layer)
			&& (need_check[lrprev->layer] == (u_char)1)) {
	       // Isolated via inside stack found
	
	       violations++;
	       cx = lrprev->x1;
	       cy = lrprev->y1;
	       cl = lrprev->layer;
	       mincost = MAXRT;

	       // Check all four positions around the contact for a
	       // free position to occupy.  Positions in the preferred
	       // direction of the route layer take precedence.

	       o = LefGetRouteOrientation(cl);
	       for (orient = 0; orient < 2; orient++) {
		  if (o == orient) {
		     if (cy < NumChannelsY - 1) {
			dy = cy + 1;	// Check north
			pri = &OBS2VAL(cx, dy, cl);
			pflags = pri->flags;
			cost = pri->prdata.cost;
			if (collide && !(pflags & (PR_COST | PR_SOURCE)) &&
					(pri->prdata.net < MAXNETNUM)) {
			   pflags = 0;
			   cost = ConflictCost;
			}
			if (pflags & PR_COST) {
			   pflags &= ~PR_COST;
			   if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
					cost < mincost) {
			      mincost = cost;
			      minx = cx;
			      miny = dy;
			   }
			}
		     }

		     if (cy > 0) {
			dy = cy - 1;	// Check south
			pri = &OBS2VAL(cx, dy, cl);
			pflags = pri->flags;
			cost = pri->prdata.cost;
			if (collide && !(pflags & (PR_COST | PR_SOURCE)) &&
					(pri->prdata.net < MAXNETNUM)) {
			   pflags = 0;
			   cost = ConflictCost;
			}
			if (pflags & PR_COST) {
			   pflags &= ~PR_COST;
			   if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
					cost < mincost) {
			      mincost = cost;
			      minx = cx;
			      miny = dy;
			   }
			}
		     }
		  }
		  else {
		     if (cx < NumChannelsX - 1) {
			dx = cx + 1;	// Check to the right
			pri = &OBS2VAL(dx, cy, cl);
			pflags = pri->flags;
			cost = pri->prdata.cost;
			if (collide && !(pflags & (PR_COST | PR_SOURCE)) &&
					(pri->prdata.net < MAXNETNUM)) {
			   pflags = 0;
			   cost = ConflictCost;
			}
			if (pflags & PR_COST) {
			   pflags &= ~PR_COST;
			   if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
					cost < mincost) {
			      mincost = cost;
			      minx = dx;
			      miny = cy;
			   }
			}
		     }

		     if (cx > 0) {
			dx = cx - 1;	// Check to the left
			pri = &OBS2VAL(dx, cy, cl);
			pflags = pri->flags;
			cost = pri->prdata.cost;
			if (collide && !(pflags & (PR_COST | PR_SOURCE)) &&
				(pri->prdata.net < MAXNETNUM)) {
			   pflags = 0;
			   cost = ConflictCost;
			}
			if (pflags & PR_COST) {
			   pflags &= ~PR_COST;
			   if ((pflags & PR_PRED_DMASK) != PR_PRED_NONE &&
					cost < mincost) {
			      mincost = cost;
			      minx = dx;
			      miny = cy;
			   }
			}
		     }
		  }
	       }

	       // Was there an available position?  If so, modify
	       // records to route to and from this position.

	       if (mincost < MAXRT) {

		  newlr = (POINT)malloc(sizeof(struct point_));
		  newlr->x1 = minx;
		  newlr->y1 = miny;
		  newlr->layer = cl;

		  newlr2 = (POINT)malloc(sizeof(struct point_));
		  newlr2->x1 = cx;
		  newlr2->y1 = cy;
		  newlr2->layer = cl;

		  lrprev->next = newlr;
		  newlr->next = newlr2;
		  newlr2->next = lrppre;

		  break;	/* Found a solution;  done. */
	       }

	       else if (stage == 0) {
		  // On the first stage, we call it an error and move
		  // on to the next net.  This is a bit conservative,
		  // but it happens rarely.

		  if (Verbose > 0)
		     Fprintf(stderr, "Failed to reserve sufficient area "
				"at grid point %d %d.\n",
				lrcur->x1, lrcur->y1);
		  violations = 0;
		  rval = 0;
		  goto cleanup;
	       }
	       else {
		  if (collide == TRUE) {
		     Fprintf(stderr, "Failed to reserve sufficient area "
				"at grid point %d %d;  position may "
				"not be routable.\n",
				lrcur->x1, lrcur->y1);
		     violations = 0;
		     rval = 0;
		     goto cleanup;
		  }

		  // On the second stage, we will run through the
		  // search again, but allow overwriting other
		  // nets, which will be treated like other colliding
		  // nets in the regular route path search.

		  collide = TRUE;
	       }
	    }
	    lrcur = lrprev;
	    lrprev = lrppre;
	 }
      }
   }

   /* Done handling stacked contacts */
 
   lrend = lrtop;
   lrcur = lrtop;
   lrprev = lrcur->next;
   lseg = (SEG)NULL;

   while (1) {
      seg = (SEG)malloc(sizeof(struct seg_));
      seg->next = NULL;

      seg->segtype = (lrcur->layer == lrprev->layer) ? ST_WIRE : ST_VIA;

      seg->x1 = lrcur->x1;
      seg->y1 = lrcur->y1;

      seg->layer = MIN(lrcur->layer, lrprev->layer);

      seg->x2 = lrprev->x1;
      seg->y2 = lrprev->y1;

      dx = seg->x2 - seg->x1;
      dy = seg->y2 - seg->y1;

      // segments are in order---place final segment at end of list
      if (rt->segments == NULL)
	 rt->segments = seg;
      else
	 lseg->next = seg;

      // Continue processing predecessors as long as the direction is the same,
      // so we get a single long wire segment.  This minimizes the number of
      // segments produced.  Vias have to be handled one at a time, as we make
      // no assumptions about stacked vias.

      if (seg->segtype & ST_WIRE) {
	 while ((lrnext = lrprev->next) != NULL) {
	    lrnext = lrprev->next;
	    if (((lrnext->x1 - lrprev->x1) == dx) &&
			((lrnext->y1 - lrprev->y1) == dy) &&
			(lrnext->layer == lrprev->layer)) {
	       lrcur = lrprev;
	       lrprev = lrnext;
	       seg->x2 = lrprev->x1;
	       seg->y2 = lrprev->y1;
	    }
	    else
	       break;
	 }
      }

      if (Verbose > 3) {
         Fprintf(stdout, "commit: index = %d, net = %d\n",
		Pr->prdata.net, netnum);

	 if (seg->segtype & ST_WIRE) {
            Fprintf(stdout, "commit: wire layer %d, (%d,%d) to (%d,%d)\n",
		seg->layer, seg->x1, seg->y1, seg->x2, seg->y2);
	 }
	 else {
            Fprintf(stdout, "commit: via %d to %d\n", seg->layer,
			seg->layer + 1);
	 }
	 Flush(stdout);
      }

      // now fill in the Obs structure with this route....

      lay2 = (seg->segtype & ST_VIA) ? seg->layer + 1 : seg->layer;

      netobs1 = OBSVAL(seg->x1, seg->y1, seg->layer);
      netobs2 = OBSVAL(seg->x2, seg->y2, lay2);

      lnode1 = (seg->layer < Pinlayers) ? NODEIPTR(seg->x1, seg->y1, seg->layer) : NULL;
      lnode2 = (lay2 < Pinlayers) ? NODEIPTR(seg->x2, seg->y2, lay2) : NULL;

      dir1 = netobs1 & PINOBSTRUCTMASK;
      dir2 = netobs2 & PINOBSTRUCTMASK;

      netobs1 &= NETNUM_MASK;
      netobs2 &= NETNUM_MASK;

      netnum |= ROUTED_NET;

      // Write back segment, but not on stage 2 or else the
      // collision information will be lost.  Stage 2 uses
      // writeback_route to call writeback_segment after the
      // colliding nets have been ripped up.

      if (stage == (u_char)0)
	 writeback_segment(seg, netnum);

      // If Obs shows this position as an obstruction, then this was a port with
      // no taps in reach of a grid point.  This will be dealt with by moving
      // the via off-grid and onto the port position in emit_routes().

      if (stage == (u_char)0) {
         if (first && dir1) {
	    first = (u_char)0;
         }
	 else if (first && dir2 && (seg->segtype & ST_VIA) && lrprev &&
			(lrprev->layer != lay2)) {
	    // This also applies to vias at the beginning of a route
	    // if the path goes down instead of up (can happen on pins,
	    // in particular)
	    OBSVAL(seg->x1, seg->y1, lay2) |= dir2;
	 }
      }

      // Keep stub information on obstructions that have been routed
      // over, so that in the rip-up stage, we can return them to obstructions.

      OBSVAL(seg->x1, seg->y1, seg->layer) |= dir1;
      OBSVAL(seg->x2, seg->y2, lay2) |= dir2;

      // An offset route end on the previous segment, if it is a via, needs
      // to carry over to this one, if it is a wire route.

      if (lseg && ((lseg->segtype & (ST_VIA | ST_OFFSET_END)) ==
			(ST_VIA | ST_OFFSET_END)))
	 if ((seg->segtype != ST_VIA) && (lnode1 != NULL))
	    if (((seg->x1 == seg->x2) && (lnode1->flags & NI_OFFSET_NS)) ||
		((seg->y1 == seg->y2) && (lnode1->flags & NI_OFFSET_EW)))
	       seg->segtype |= ST_OFFSET_START;

      // Check if the route ends are offset.  If so, add flags.  The segment
      // entries are integer grid points, so offsets need to be made when
      // the location is output.  This is only for offsets that are in the
      // same direction as the route, to make sure that the route reaches
      // the offset via, and does not extend past it.

      if (dir1 & OFFSET_TAP) {
	 if (lnode1 != NULL)
	    if (((seg->x1 == seg->x2) && (lnode1->flags & NI_OFFSET_NS)) ||
			((seg->y1 == seg->y2) && (lnode1->flags & NI_OFFSET_EW)))
	       seg->segtype |= ST_OFFSET_START;

	 // An offset on a via needs to be applied to the previous route
	 // segment as well, if that route is a wire, and the offset is
	 // in the same direction as the wire.

	 if (lseg && (seg->segtype & ST_VIA) && !(lseg->segtype & ST_VIA) &&
			(lnode2 != NULL))
	    if (((lseg->x1 == lseg->x2) && (lnode2->flags & NI_OFFSET_NS)) ||
		((lseg->y1 == lseg->y2) && (lnode2->flags & NI_OFFSET_EW)))
	       lseg->segtype |= ST_OFFSET_END;
      }

      if (dir2 & OFFSET_TAP) seg->segtype |= ST_OFFSET_END;

      lrend = lrcur;		// Save the last route position
      lrend->x1 = lrcur->x1;
      lrend->y1 = lrcur->y1;
      lrend->layer = lrcur->layer;

      lrcur = lrprev;		// Move to the next route position
      lrcur->x1 = seg->x2;
      lrcur->y1 = seg->y2;
      lrprev = lrcur->next;

      if (lrprev == NULL) {

         if (dir2 && (stage == (u_char)0)) {
	    OBSVAL(seg->x2, seg->y2, lay2) |= dir2;
         }
	 else if (dir1 && (seg->segtype & ST_VIA)) {
	    // This also applies to vias at the end of a route
	    OBSVAL(seg->x1, seg->y1, seg->layer) |= dir1;
	 }

	 // Before returning, set *ept to the endpoint
	 // position.  This is for diagnostic purposes only.
	 ept->x = lrend->x1;
	 ept->y = lrend->y1;
	 ept->lay = lrend->layer;

	 // Clean up allocated memory for the route. . .
	 while (lrtop != NULL) {
	    lrnext = lrtop->next;
	    free(lrtop);
	    lrtop = lrnext;
	 }
	 return rval;	// Success
      }
      lseg = seg;	// Move to next segment position
   }

cleanup:

   while (lrtop != NULL) {
      lrnext = lrtop->next;
      free(lrtop);
      lrtop = lrnext;
   }
   return 0;

} /* commit_proute() */

/*------------------------------------------------------*/
/* writeback_route() ---				*/
/*							*/
/*   This routine is the last part of the routine	*/
/*   above.  It copies the net defined by the segments	*/
/*   in the route structure "rt" into the Obs array.	*/
/*   This is used only for stage 2, when the writeback	*/
/*   is not done by commit_proute because we want to	*/
/*   rip up nets first, and also done prior to routing	*/
/*   for any pre-defined net routes.			*/
/*------------------------------------------------------*/

int writeback_route(ROUTE rt)
{
   SEG seg;
   int  lay2;
   u_int netnum, dir1, dir2;
   u_char first = (u_char)1;

   netnum = rt->netnum | ROUTED_NET;
   for (seg = rt->segments; seg; seg = seg->next) {

      /* Save stub route information at segment ends. 		*/
      /* NOTE:  Where segment end is a via, make sure we are	*/
      /* placing the segment end on the right metal layer!	*/

      lay2 = (seg->segtype & ST_VIA) ? seg->layer + 1 : seg->layer;

      dir1 = OBSVAL(seg->x1, seg->y1, seg->layer) & PINOBSTRUCTMASK;
      if (lay2 < Num_layers)
	 dir2 = OBSVAL(seg->x2, seg->y2, lay2) & PINOBSTRUCTMASK;
      else
	 dir2 = 0;

      writeback_segment(seg, netnum);

      if (first) {
	 first = (u_char)0;
	 if (dir1)
	    OBSVAL(seg->x1, seg->y1, seg->layer) |= dir1;
	 else if (dir2)
	    OBSVAL(seg->x2, seg->y2, lay2) |= dir2;
      }
      else if (!seg->next) {
	 if (dir1)
	    OBSVAL(seg->x1, seg->y1, seg->layer) |= dir1;
	 else if (dir2)
	    OBSVAL(seg->x2, seg->y2, lay2) |= dir2;
      }
   }
   return TRUE;
}

/*------------------------------------------------------*/
/* Writeback all routes belonging to a net		*/
/*------------------------------------------------------*/

int writeback_all_routes(NET net)
{
   ROUTE rt;
   int result = TRUE;

   for (rt = net->routes; rt; rt = rt->next) {
      if (writeback_route(rt) == FALSE)
	 result = FALSE;
   }
   return result;
}

/* end of maze.c */
