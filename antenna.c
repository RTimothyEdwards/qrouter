/*--------------------------------------------------------------*/
/*  antenna.c -- Compute the metal area to gate area for all	*/
/*  routes and determine where antenna violations occur.  Then,	*/
/*  resolve the violations by routing from each violation to an	*/
/*  antenna tap.						*/
/*								*/
/*  To be done:  If there are no antenna cells placed, or if	*/
/*  the antenna route fails, or if the antenna violation is	*/
/*  close to the limit, see if the route can be adjusted by	*/
/*  moving the route to a higher layer near the gate.		*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, May 2018                           	*/
/*--------------------------------------------------------------*/

#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* This entire file is dependent on the Tcl/Tk version */
#ifdef TCL_QROUTER
#include <tk.h>

#include "qrouter.h"
#include "qconfig.h"
#include "node.h"
#include "lef.h"
#include "def.h"

/* Node Hash Table routines taken from delay.c */
extern GATE FindGateNode(Tcl_HashTable *, NODE, int *);
extern void FreeNodeTable(Tcl_HashTable *);

/* Structure to hold information about an antenna error. */

typedef struct antennainfo_  *ANTENNAINFO;

struct antennainfo_ {
   ANTENNAINFO next;	/* Next antenna violation in the list.	*/
   NET net;		/* The net violating an antenna rule	*/
   NODE node;		/* A gate-end node that is in violation */
   int layer;		/* Uppermost metal layer of the antenna */
};

/* Keep the list as a global variable so it can be accessed	*/
/* from doroute() (in qrouter.c)				*/

ANTENNAINFO AntennaList;

/*--------------------------------------------------------------*/
/* Find free antenna cells, and collect all the antenna taps	*/
/* into a single net, much like VDD_NET or GND_NET.		*/
/*								*/
/* Return the number of free antenna taps available in the	*/
/* layout.							*/
/*--------------------------------------------------------------*/

int
find_free_antenna_taps(char *antennacell)
{
    int numtaps;
    GATE ginst;
    GATE gateginfo;
    int netnum, i;

    numtaps = 0;
    for (ginst = Nlgates; ginst; ginst = ginst->next) {
	gateginfo = ginst->gatetype;
	if (!strcasecmp(gateginfo->gatename, antennacell)) {
	    /* Find an unassigned node.  If there is not one,	*/
	    /* this is probably a routed (not free) cell.	*/
	    for (i = 0; i < ginst->nodes; i++) {
		netnum = ginst->netnum[i];
		if (netnum == 0) {
		    ginst->netnum[i] = ANTENNA_NET;
		    numtaps++;
		}		
	    }
	}
    }
    return numtaps;
}

/*--------------------------------------------------------------*/
/* States to track nodes as they are processed:			*/
/*								*/
/*	NOT_VISITED :  Node has not yet been processed.		*/
/*	VISITED :  Node was counted on this pass.		*/
/*	PROCESSED :  Node was counted on a previous pass.	*/
/*	ANCHOR :  Node is a source/drain connection.		*/
/*--------------------------------------------------------------*/

enum visit_states {NOT_VISITED = 0, VISITED, PROCESSED, ANCHOR};

/* Forward declarations */
float get_route_area_reverse(NET, ROUTE, int, u_char *, u_char, Tcl_HashTable *);
float get_route_area_forward(NET, ROUTE, int, u_char *, u_char, Tcl_HashTable *);
float get_route_area_reverse_fromseg(NET, ROUTE, SEG, int, u_char *, u_char,
		Tcl_HashTable *);

/*--------------------------------------------------------------*/
/* Determine the amount of metal in the route, starting at the	*/
/* route start point, and not moving past any point that is 	*/
/* above "layer".  Check all other unvisited routes in net to	*/
/* see if any connect to "rt".  If so, check if they connect	*/
/* to a point that is part of the subnet below or at "layer".	*/
/* If they do, recursively run get_route_area_forward on that	*/
/* route.  When done, return the total area of the subnet.	*/
/*--------------------------------------------------------------*/

float
get_route_area_forward_fromseg(NET net, ROUTE rt, SEG nseg, int layer,
		u_char *visited, u_char method, Tcl_HashTable *NodeTable)
{
    float area, length, width, thick;
    int x, y, l, compat;
    SEG seg, iseg, chkseg;
    ROUTE rt2;
    u_char found;

    if (rt->flags & RT_VISITED) return 0.0;
    rt->flags |= RT_VISITED;
    area = 0.0;

    /* If nseg is NULL then check from the beginning. */
    if (nseg == NULL) nseg = rt->segments;

    /* Check if the route beginning is a node */
    if (nseg == rt->segments) {
	if (rt->flags & RT_START_NODE) {
	    NODE node;
	    GATE g;
	    int i;

	    node = rt->start.node;
	    g = FindGateNode(NodeTable, node, &i);
	    if (g->area[i] == 0.0) {
		/* There's a diffusion diode here! */
		visited[node->nodenum] = ANCHOR;
		return 0.0;
	    } else {
		/* Add this node to the list of nodes with gates	*/
		/* attached to this antenna area.			*/
		visited[node->nodenum] = VISITED;
	    }
	}
    }

    for (seg = rt->segments; seg && (seg != nseg); seg = seg->next);
    if (seg == NULL) return 0.0;

    for (; seg; seg = seg->next) {

	/* Once the layer goes above the current check layer, the search stops. */
	if (seg->layer > layer) break;

	/* Vias don't contribute to area, at least for now. */
	if (seg->segtype & ST_VIA) continue;

	/* For non-cumulative methods, only count area for	*/
	/* those segments which are on the given check layer.	*/

	if ((method == CALC_AREA) || (method == CALC_SIDEAREA))
	    if (seg->layer != layer)
		continue;

	/* Note that one of x or y is zero, depending on segment orientation */
	x = (seg->x2 - seg->x1);
	y = (seg->y2 - seg->y1);
	if (x < 0) x = -x;
	if (y < 0) y = -y;

	/* Note that "l" is a unitless grid dimension */
	if (x == 0)
	    length = (float)y * (float)PitchY[layer];
	else
	    length = (float)x * (float)PitchX[layer];

	/* area is either the total top surface of the metal,	*/
	/* or the total side surface of the metal (in um^2)	*/

	width = LefGetRouteWidth(seg->layer);
	if ((method == CALC_AREA) || (method == CALC_AGG_AREA))
	    area += (float)(length * width);
	else if ((method == CALC_SIDEAREA) || (method == CALC_AGG_SIDEAREA)) {
	    thick = LefGetRouteThickness(seg->layer);
	    area += thick * 2.0 * (length + width);
	}
    }

    /* Check other routes for intersection with this route */

    for (rt2 = net->routes; rt2; rt2 = rt2->next) {
	if (rt2->flags & RT_VISITED) continue;

	if (!(rt2->flags & RT_START_NODE) && (rt2->start.route == rt)) {
	    /* The start point of rt2 connects somewhere on rt */
	    iseg = rt2->segments;
	    x = iseg->x1;
	    y = iseg->y1;
	    l = iseg->layer;
	}
	else if (!(rt2->flags & RT_END_NODE) && (rt2->end.route == rt)) {
	    /* The end point of rt2 connects somewhere on rt */
	    for (iseg = rt2->segments; iseg && iseg->next; iseg = iseg->next);
	    x = iseg->x2;
	    y = iseg->y2;
	    l = iseg->layer;
	}
	else
	    continue;

	/* Must determine if rt2 intersects rt within the antenna area */

	found = (u_char)0;
	for (chkseg = rt->segments; chkseg; chkseg = chkseg->next) {
	    if (chkseg->segtype & ST_WIRE) {
		if (iseg->segtype & ST_WIRE) {
		    compat = (l == chkseg->layer);
		}
		else {
		    compat = (l == chkseg->layer) || (l + 1 == chkseg->layer);
		}
	    }
	    else {
		if (iseg->segtype & ST_WIRE) {
		    compat = (l == chkseg->layer) || (l == chkseg->layer + 1);
		}
		else {
		    compat = (l == chkseg->layer) || (l == chkseg->layer + 1) ||
				(l + 1 == chkseg->layer);
		}
	    }
	    if (!compat) continue;

	    if (chkseg->segtype & ST_VIA) {
		if ((chkseg->x1 == x) && (chkseg->y1 == y)) {
		    found = (u_char)1;
		    break;
		}
	    }
	    else if (chkseg->x1 < chkseg->x2) {
		if (chkseg->y1 == y) {
		    if ((chkseg->x1 <= x) && (chkseg->x2 >= x)) {
			found = (u_char)1;
			break;
		    }
		}
	    }
	    else if (chkseg->x1 > chkseg->x2) {
		if (chkseg->y1 == y) {
		    if ((chkseg->x1 >= x) && (chkseg->x2 <= x)) {
			found = (u_char)1;
			break;
		    }
		}
	    }
	    else if (chkseg->y1 < chkseg->y2) {
		if (chkseg->x1 == x) {
		    if ((chkseg->y1 <= y) && (chkseg->y2 >= y)) {
			found = (u_char)1;
			break;
		    }
		}
	    }
	    else if (chkseg->y1 > chkseg->y2) {
		if (chkseg->x1 == x) {
		    if ((chkseg->y1 >= y) && (chkseg->y2 <= y)) {
			found = (u_char)1;
			break;
		    }
		}
	    }
	    if (chkseg == seg) break;
	}
	if (found == (u_char)1) {
	    if (rt2->start.route == rt)
		area += get_route_area_forward(net, rt2, layer, visited,
				method, NodeTable);
	    else
		area += get_route_area_reverse(net, rt2, layer, visited,
				method, NodeTable);
	}
    }

    /* The end of this route may be a node (so record it in visited) or	*/
    /* a route (so walk it).						*/

    if (seg == NULL) {	/* If seg != NULL then we didn't reach the route end */
	if (rt->flags & RT_END_NODE) {
	    NODE node;
	    GATE g;
	    int i;

	    node = rt->end.node;
	    g = FindGateNode(NodeTable, node, &i);
	    if (g->area[i] == 0.0) {
		/* There's a diffusion diode here! */
		visited[node->nodenum] = ANCHOR;
		return 0.0;
	    } else {
		/* Add this node to the list of nodes with gates	*/
		/* attached to this antenna area.			*/
		visited[node->nodenum] = VISITED;
	    }
	}
	else {
	    SEG rseg;

	    /* Back up seg to point to the last segment of the route */
	    for (seg = rt->segments; seg && seg->next; seg = seg->next);
	    x = seg->x2;
	    y = seg->y2;
	    l = seg->layer;

	    /* Find where on rt2 the segment lands, then search rt2 for	*/
	    /* antenna area forward and reverse from that segment.	*/

	    rt2 = rt->end.route;

	    for (rseg = rt2->segments; rseg; rseg = rseg->next) {
		if (rseg->segtype & ST_WIRE) {
		    if (seg->segtype & ST_WIRE) {
			compat = (l == rseg->layer);
		    }
		    else {
			compat = (l == rseg->layer) || (l + 1 == rseg->layer);
		    }
		}
		else {
		    if (seg->segtype & ST_WIRE) {
			compat = (l == rseg->layer) || (l == rseg->layer + 1);
		    }
		    else {
			compat = (l == rseg->layer) || (l == rseg->layer + 1) ||
				(l + 1 == rseg->layer);
		    }
		}
		if (compat) {
		    if (rseg->segtype & ST_VIA) {
			if ((rseg->x2 == seg->x2) && (rseg->y2 == seg->y2))
			    break;
		    }
		    else if (rseg->x1 < rseg->x2) {
			if (rseg->y2 == seg->y2) {
			    if ((rseg->x1 <= seg->x2) && (rseg->x2 >= seg->x2))
				break;
			}
		    }
		    else if (rseg->x1 > rseg->x2) {
			if (rseg->y2 == seg->y2) {
			    if ((rseg->x1 >= seg->x2) && (rseg->x2 <= seg->x2))
				break;
			}
		    }
		    else if (rseg->y1 < rseg->y2) {
			if (rseg->x2 == seg->x2) {
			    if ((rseg->y1 <= seg->y2) && (rseg->y2 >= seg->y2))
				break;
			}
		    }
		    else if (rseg->y1 > rseg->y2) {
			if (rseg->x2 == seg->x2) {
			    if ((rseg->y1 >= seg->y2) && (rseg->y2 <= seg->y2))
				break;
			}
		    }
		}
	    }
	    if (rseg == NULL) return;  /* This should not happen */

	    if (rseg->next != NULL)
	        area += get_route_area_forward_fromseg(net, rt2, rseg->next,
			layer, visited, method, NodeTable);
	    area += get_route_area_reverse_fromseg(net, rt2, rseg, layer,
			visited, method, NodeTable);
	}
    }
    return area;
}

float
get_route_area_forward(NET net, ROUTE rt, int layer, u_char *visited,
	u_char method, Tcl_HashTable *NodeTable)
{
    float area;

    area = get_route_area_forward_fromseg(net, rt, NULL, layer, visited,
		method, NodeTable);
    return area;
}

/*--------------------------------------------------------------*/
/* This is the same as get_route_area_forward_fromseg, but is	*/
/* searching the path from end to beginning, so reverse the	*/
/* route first and then call get_route_area_forward_fromseg().	*/
/*--------------------------------------------------------------*/

float
get_route_area_reverse_fromseg(NET net, ROUTE rt, SEG nseg, int layer,
	u_char *visited, u_char method, Tcl_HashTable *NodeTable)
{
    SEG seg, dseg, newseg, firstseg, saveseg;
    NODE savestartnode, saveendnode;
    float area;
    u_char saveflags;

    firstseg = NULL;

    /* Reverse the route */
    for (seg = rt->segments; seg; seg = seg->next) {
	newseg = (SEG)malloc(sizeof(struct seg_));
	newseg->layer = seg->layer;
	newseg->x1 = seg->x2;
	newseg->x2 = seg->x1;
	newseg->y1 = seg->y2;
	newseg->y2 = seg->y1;
	newseg->segtype = seg->segtype;
	newseg->next = firstseg;
	firstseg = newseg;
    }

    saveseg = rt->segments;

    /* Replace the route segment with the reversed route */
    rt->segments = firstseg;

    /* Reverse the endpoint information */
    savestartnode = rt->start.node;
    saveendnode = rt->end.node;
    rt->start.node = saveendnode;
    rt->end.node = savestartnode;

    /* Reverse the start/end flags */
    saveflags = rt->flags & (RT_START_NODE | RT_END_NODE);
    rt->flags &= ~(RT_START_NODE | RT_END_NODE);
    if (saveflags & RT_START_NODE) rt->flags |= RT_END_NODE;
    if (saveflags & RT_END_NODE) rt->flags |= RT_START_NODE;

    area = get_route_area_forward_fromseg(net, rt, nseg, layer, visited,
		method, NodeTable);

    /* Replace the route segment with the original route */
    rt->segments = saveseg;

    /* Revert the endpoint information */
    rt->start.node = savestartnode;
    rt->end.node = saveendnode;

    /* Revert the start/end flags */
    rt->flags &= ~(RT_START_NODE | RT_END_NODE);
    rt->flags |= saveflags;

    /* Free the reversed route */
    for (seg = firstseg; seg; ) {
	dseg = seg->next;
	free(seg);
	seg = dseg;
    }
    return area;
}

/*--------------------------------------------------------------*/
/* Walk a route in reverse from end to start.			*/
/*--------------------------------------------------------------*/

float
get_route_area_reverse(NET net, ROUTE rt, int layer, u_char *visited,
		u_char method, Tcl_HashTable *NodeTable)
{
    float area;
    area = get_route_area_reverse_fromseg(net, rt, NULL, layer, visited,
		method, NodeTable);
    return area;
}

/*--------------------------------------------------------------*/
/* Find all antenna violations at a specific metal layer	*/
/*--------------------------------------------------------------*/

int find_layer_antenna_violations(int layer, Tcl_HashTable *NodeTable)
{
    int numerrors, n, nn, numroutes, i, j, new, neterrors;
    u_char *visited, method;
    float antenna_ratio, thick;
    GATE g;
    NET net;
    ROUTE rt;
    NODEINFO nodeptr;
    NODE node, tnode;
    SEG seg;
    ANTENNAINFO newantenna;
    float gate_area, metal_area, ratio, save_gate, save_metal, max_ratio;

    numerrors = 0;

    /* Get the metal layer record for this layer and find the metal	*/
    /* area ratio limit and the method to be used for calculating	*/
    /* metal area.							*/

    method = LefGetRouteAntennaMethod(layer);
    if (method == CALC_NONE) return 0;	/* No antenna information in tech */
    antenna_ratio = (float)LefGetRouteAreaRatio(layer);
    thick = (float)LefGetRouteThickness(layer);
    if (((method == CALC_SIDEAREA) || (method == CALC_AGG_SIDEAREA)) && (thick == 0.0))
	return 0;	/* Insufficient antenna information in tech */

    /* Make a pass through all nets to find antenna violations */

    for (n = 0; n < Numnets; n++) {
	net = Nlnets[n];

	if ((net->netnum == VDD_NET) || (net->netnum == GND_NET) ||
		(net->netnum == ANTENNA_NET)) continue;

	/* Ignore nets with no routes */
	numroutes = 0;
	for (rt = net->routes; rt; rt = rt->next) numroutes++;
	if (numroutes == 0) continue;

	/* Consider each terminal as a separate sub-net calculation.	*/
	/* But if multiple terminals belong to the same sub-net, they	*/
	/* are marked visited and ignored on subsequent calculations.	*/

	visited = (u_char *)malloc(net->numnodes * sizeof(u_char));
	for (node = net->netnodes; node != NULL; node = node->next) {
	    nn = node->nodenum;
	    visited[nn] = NOT_VISITED;
	}

	/* Make a pass through all nodes of the net.  Where they are	*/
	/* not connected together at "layer", these are individual	*/
	/* sub-nets.							*/

	neterrors = 0;
	max_ratio = 0.0;	/* For diagnostics only */
	for (node = net->netnodes; node != NULL; node = node->next) {
	    nn = node->nodenum;
	    if (visited[nn] >= PROCESSED) continue; 	/* Already seen */

	    /* Find the gate area of this node */
	    g = FindGateNode(NodeTable, node, &i);
	    metal_area = 0.0;

	    if (g->area[i] == 0.0) {
		visited[nn] = ANCHOR;	/* Mark as S/D connection */
		continue;		/* No gate, so no violation */
	    }
	    else
		visited[nn] = VISITED;

	    /* Clear visited flags for routes */

	    for (rt = net->routes; rt; rt = rt->next)
		rt->flags &= ~RT_VISITED;

	    /* Find the route or routes that connect to this node */

	    for (rt = net->routes; rt; rt = rt->next) {
		if ((rt->flags & RT_START_NODE) && (rt->start.node == node)) {
		    metal_area += get_route_area_forward(net, rt, layer, visited,
				method, NodeTable);
		}
		else if ((rt->flags & RT_END_NODE) && (rt->end.node == node)) {
		    metal_area += get_route_area_reverse(net, rt, layer, visited,
				method, NodeTable);
		} 
		else continue;
	    }

	    /* Gate area is combined area of gates visited */

	    gate_area = 0.0;
	    for (tnode = net->netnodes; tnode != NULL; tnode = tnode->next) {
		j = tnode->nodenum;
		if (visited[j] == VISITED) {
		    g = FindGateNode(NodeTable, tnode, &i);
		    if (g->area[i] == 0.0) {
			visited[j] = ANCHOR;
			gate_area = 0.0;
			break;
		    }
		    else
			gate_area += g->area[i];
		}

	    }

	    if (gate_area > 0.0) {
		ratio = metal_area / gate_area;
		if (ratio > max_ratio) {
		    max_ratio = ratio;
		    save_gate = gate_area;
		    save_metal = metal_area;
		}

		if (ratio > antenna_ratio) {

		    /* Record and report the violation */

		    numerrors++;
		    neterrors++;
		    if (Verbose > 1) {
			Fprintf(stderr,
				"Antenna violation on node %d of net %s at metal%d\n",
				nn, net->netname, layer + 1);
		    }
		    if (Verbose > 2) {
			Fprintf(stderr, "Metal area = %f, Gate area = %f, Ratio = %f\n",
				metal_area, gate_area, ratio);
		    }
		    newantenna = (ANTENNAINFO)malloc(sizeof(struct antennainfo_));
		    newantenna->net = net;
		    newantenna->node = node;
		    newantenna->layer = layer;
		    newantenna->next = AntennaList;
		    AntennaList = newantenna;
		}
	    }

	    /* Mark gates as visited on previous pass */
	    for (tnode = net->netnodes; tnode != NULL; tnode = tnode->next) {
		j = tnode->nodenum;
		if (visited[j] == VISITED) visited[j] = PROCESSED;
	    }
	}
	free(visited);

	if (Verbose > 3) {
	    /* Diagnostic */
	    if (neterrors == 0) {
		if (max_ratio > 0.0)
		    Fprintf(stderr, "Worst case:  Metal area = %f, Gate area = %f, "
			"Ratio = %f\n", save_metal, save_gate, max_ratio);
	    }
	}

	/* Clear route visited flags */
	for (rt = net->routes; rt; rt = rt->next)
	    rt->flags &= ~RT_VISITED;
    }
    return numerrors;
}

/*--------------------------------------------------------------*/
/* Find all antenna violations					*/
/*--------------------------------------------------------------*/

int find_antenna_violations()
{
    int numerrors, layererrors;
    int layer, i, new;
    Tcl_HashTable NodeTable;
    Tcl_HashEntry *entry;
    GATE g;

    numerrors = 0;

    /* Build a hash table of nodes, so the gate area can be found	*/
    /* quickly for any node by hash lookup.				*/

    Tcl_InitHashTable(&NodeTable, TCL_ONE_WORD_KEYS);

    for (g = Nlgates; g; g = g->next) {
	for (i = 0; i < g->nodes; i++) {
	    GATENODE gn;
	    gn = (GATENODE)malloc(sizeof(struct gatenode_));
	    gn->idx = i;
	    gn->gate = g;
	    entry = Tcl_CreateHashEntry(&NodeTable, (char *)(*(g->noderec + i)), &new);
	    Tcl_SetHashValue(entry, gn);
	}
    }

    /* Working from the 1nd layer metal to the top, compute	*/
    /* route metal to gate area ratios.  Mark each one when	*/
    /* done, as an antenna violation that has been fixed at, 	*/
    /* say, metal2 can no longer be a violation on any higer	*/
    /* layer of metal.						*/

    for (layer = 0; layer < Num_layers; layer++) {
	layererrors = find_layer_antenna_violations(layer, &NodeTable);
	numerrors += layererrors;
	if (Verbose > 2) {
	    Fprintf(stdout, "Number of antenna errors on metal%d = %d\n",
			layer + 1, layererrors);
	}
    }

    FreeNodeTable(&NodeTable);
    Tcl_DeleteHashTable(&NodeTable);

    return numerrors;
}

/*--------------------------------------------------------------*/
/* This routine is similar to route_setup() for the normal	*/
/* stage routes, with changes for the antenna routing.		*/
/* Set the node in the "violation" record to source, and set	*/
/* all free antenna taps to destination.  Add existing routes	*/
/* to the source in the same manner as was used to find the	*/
/* antenna violation in the first place (this is a subnet of	*/
/* the complete net).  Disable the remainder of the net.	*/
/* Set all free antenna taps to the net number being routed,	*/
/* then route like stage 1 power routing.			*/
/*--------------------------------------------------------------*/

int antenna_setup(struct routeinfo_ *iroute, ANTENNAINFO violation)
{
    /* Work in progress */
    return 0;
}

/*--------------------------------------------------------------*/
/* Route from nets with antenna violations to the nearest	*/
/* routable antenna cell tap.					*/
/*								*/
/* This routine is essentially the same as doroute() but with	*/
/* some special handling related to the antenna taps, which	*/
/* have much in common with VDD and GND taps but significant	*/
/* differences as well.						*/
/*--------------------------------------------------------------*/

int doantennaroute(ANTENNAINFO violation)
{
    NET net;
    NODE node;
    ROUTE rt1;
    int layer, i, result;
    struct routeinfo_ iroute;

    net = violation->net;
    node = violation->node;
    layer = violation->layer;

    // Fill out route information record
    iroute.net = net;
    iroute.rt = NULL;
    for (i = 0; i < 6; i++)
	iroute.glist[i] = NULL;
    iroute.nsrc = node;
    iroute.nsrctap = iroute.nsrc->taps;
    iroute.maxcost = MAXRT;
    iroute.do_pwrbus = TRUE;
    iroute.pwrbus_src = 0;

    result = antenna_setup(&iroute, violation);

    rt1 = createemptyroute();
    rt1->netnum = net->netnum;
    iroute.rt = rt1;

    result = route_segs(&iroute, 0, (u_char)0);

    /* To do:  Handle failures, successes */

    return 0;
}

/*--------------------------------------------------------------*/
/* Top level routine called from tclqrouter.c			*/
/*--------------------------------------------------------------*/

void
resolve_antenna(char *antennacell)
{
    int numtaps, numerrors, result;
    ANTENNAINFO nextviolation;
    NET net;

    numtaps = find_free_antenna_taps(antennacell);
    if (Verbose > 3) {
	Fprintf(stdout, "Number of free antenna taps = %d\n", numtaps);
    }

    AntennaList = NULL;
    numerrors = find_antenna_violations();
    if (Verbose > 1) {
	Fprintf(stdout, "Total number of antenna errors = %d\n", numerrors);
    }
    if (numtaps < numerrors) {
	if (numtaps == 0)
	    Fprintf(stderr, "There are no antenna taps to use to correct "
			"antenna errors!\n");
	else {
	    Fprintf(stderr, "There are not enough antenna taps to use to "
			"correct antenna errors!\n");
	    Fprintf(stderr, "Number of errors = %d, number of taps = %d\n",
			numerrors, numtaps);
	    Fprintf(stderr, "Increate the amount of unallocated antenna cells"
			" in the design.\n");
	}
	/* To do:  Replace the error message with an ad-hoc solution to	*/
	/* pull routes up to a higher metal layer near the gate causing	*/
	/* the error.							*/
    }
    else if (numerrors > 0) {
	while (AntennaList != NULL) {
	    nextviolation = AntennaList->next;
	    net = nextviolation->net;

	    result = doantennaroute(nextviolation);

	    /* Free the error information */
	    free(AntennaList);
	    AntennaList = nextviolation;
	}
    }
}

#endif	/* TCL_QROUTER */

/* end of antenna.c */
