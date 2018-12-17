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
#include <sys/types.h>
#include <regex.h>

/* This entire file is dependent on the Tcl/Tk version */
#ifdef TCL_QROUTER
#include <tk.h>

#include "qrouter.h"
#include "qconfig.h"
#include "node.h"
#include "lef.h"
#include "def.h"
#include "point.h"

/* Node Hash Table routines taken from delay.c */
extern GATE FindGateNode(Tcl_HashTable *, NODE, int *);
extern void FreeNodeTable(Tcl_HashTable *);

extern int TotalRoutes;

/* Structure to hold information about an antenna error. */

typedef struct antennainfo_  *ANTENNAINFO;

struct antennainfo_ {
   ANTENNAINFO next;	/* Next antenna violation in the list.	*/
   NET net;		/* The net violating an antenna rule	*/
   NODE node;		/* A gate-end node that is in violation */
   ROUTE route;		/* A route that is part of the antenna	*/
   int layer;		/* Uppermost metal layer of the antenna */
};

/* Keep the list as a global variable so it can be accessed	*/
/* from doroute() (in qrouter.c)				*/

ANTENNAINFO AntennaList;

/*--------------------------------------------------------------*/
/* Regular expression matching of the given string in		*/
/* "antennacell" to the string "strtest".  If the regular	*/
/* expression matches and the result is in the first character	*/
/* position of the string, then return TRUE (match), otherwise	*/
/* return FALSE (no match).					*/
/*--------------------------------------------------------------*/

u_char
string_match(char *antennacell, char *strtest)
{
    regex_t regex;
    regmatch_t pmatch;
    int reti;

    /* Compile regular expression */
    reti = regcomp(&regex, antennacell, 0);
    if (reti) {
	/* Assume this is not a regular expression and just run */
	/* a straight string match.				*/
	if (!strcasecmp(antennacell, strtest))
	    return TRUE;
	else
	    return FALSE;
    }

    /* Execute regular expression */
    reti = regexec(&regex, strtest, 1, &pmatch, 0);
    regfree(&regex);

    if (!reti) {
	if (pmatch.rm_so == 0)	/* Must match beginning of string */
	    return TRUE;
	else
	    return FALSE;
    }
    else
	return FALSE;
}

/*--------------------------------------------------------------*/
/* Find free antenna cells, and collect all the antenna taps	*/
/* into a single net, much like VDD_NET or GND_NET.		*/
/*								*/
/* Return the number of free antenna taps available in the	*/
/* layout.							*/
/*								*/
/* If the name of the antennacell ends in '*', then assume a	*/
/* wildcard character and match to any string beginning with 	*/
/* the substring of antennacell.				*/
/*--------------------------------------------------------------*/

void
find_free_antenna_taps(char *antennacell)
{
    int numtaps;
    GATE ginst;
    GATE gateginfo;
    NODE noderec;
    int netnum, i;

    if (antennacell == NULL) {
	Fprintf(stderr, "No antenna cell defined!\n");
	return;
    }
    numtaps = 0;
    for (ginst = Nlgates; ginst; ginst = ginst->next) {
	gateginfo = ginst->gatetype;
	
	if (string_match(antennacell, gateginfo->gatename)) {
	    /* Find an unassigned node.  If there is not one,	*/
	    /* this is probably a routed (not free) cell.	*/
	    for (i = 0; i < ginst->nodes; i++) {
		netnum = ginst->netnum[i];
		noderec = ginst->noderec[i];
		if ((netnum == 0) && (noderec == NULL)) {
		    ginst->netnum[i] = ANTENNA_NET;
		    ginst->noderec[i] = (NODE)calloc(1, sizeof(struct node_));
		    ginst->noderec[i]->netnum = ANTENNA_NET;
		}
	    }
	}
    }
}

/*--------------------------------------------------------------*/
/* Similar to the routine above, but just count the free taps.	*/
/*--------------------------------------------------------------*/

int
count_free_antenna_taps(char *antennacell)
{
    int numtaps;
    GATE ginst;
    GATE gateginfo;
    int netnum, i;

    numtaps = 0;
    for (ginst = Nlgates; ginst; ginst = ginst->next) {
	gateginfo = ginst->gatetype;

	if (string_match(antennacell, gateginfo->gatename)) {
	    /* Find an unassigned node.  If there is not one,	*/
	    /* this is probably a routed (not free) cell.	*/
	    for (i = 0; i < ginst->nodes; i++) {
		netnum = ginst->netnum[i];
		if (netnum == ANTENNA_NET) 
		    numtaps++;
	    }
	}
    }
    return numtaps;
}

/*--------------------------------------------------------------*/
/* After routing, the free antenna taps are all marked with the	*/
/* net number of the net just routed.  To make them free again,	*/
/* change all but the one that was routed back to ANTENNA_NET.	*/
/* Identify the unused taps by finding the OBSVAL record with	*/
/* net set to netnum but not connected to the same node.	*/
/*--------------------------------------------------------------*/

void revert_antenna_taps(int netnum, NODE node)
{
    int x, y, lay;
    PROUTE *Pr;
    NODEINFO lnode = NULL;

    /* Clear all targets except for the one just routed */

    for (lay = 0; lay < Num_layers; lay++)
	for (x = 0; x < NumChannelsX; x++)
	    for (y = 0; y < NumChannelsY; y++)
		if ((OBSVAL(x, y, lay) & NETNUM_MASK) == netnum) {
		    Pr = &OBS2VAL(x, y, lay);
		    if (Pr->flags & PR_TARGET) {
			lnode = NODEIPTR(x, y, lay);
			if ((lnode == NULL) || (lnode->nodesav != node)) {
			    OBSVAL(x, y, lay) &= ~(NETNUM_MASK | ROUTED_NET);
			    OBSVAL(x, y, lay) |= ANTENNA_NET;
			}
		    }
		}
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
float get_route_area_reverse(NET, ROUTE, int, u_char *, u_char,
		Tcl_HashTable *, struct routeinfo_ *);
float get_route_area_forward(NET, ROUTE, int, u_char *, u_char,
		Tcl_HashTable *, struct routeinfo_ *);
float get_route_area_reverse_fromseg(NET, ROUTE, SEG, int, u_char *, u_char,
		Tcl_HashTable *, struct routeinfo_ *);

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
		u_char *visited, u_char method, Tcl_HashTable *NodeTable,
		struct routeinfo_ *iroute)
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

	    if (visited) {

		/* If more than one route is connected to the node,	*/
		/* then this node may have been visited already.	*/

		if (visited[node->nodenum] == NOT_VISITED) {
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
	    else if ((method == ANTENNA_ROUTE) && (iroute != NULL)) {
		set_node_to_net(node, PR_SOURCE, iroute->glist[0], iroute->bbox, 0);
	    }

	    /* Walk all other routes that start or end on this node */

	    for (rt2 = net->routes; rt2; rt2 = rt2->next) {
		if (rt2->flags & RT_VISITED) continue;

		if ((rt2->flags & RT_START_NODE) && (rt2->start.node == node)) {
		    /* The start point of rt2 connects to the same node */
		    area += get_route_area_forward(net, rt2, layer, visited,
				method, NodeTable, NULL);
		}
		else if ((rt2->flags & RT_END_NODE) && (rt2->end.node == node)) {
		    /* The end point of rt2 connects to the same node */
		    for (iseg = rt2->segments; iseg && iseg->next; iseg = iseg->next);
		    area += get_route_area_reverse(net, rt2, layer, visited,
				method, NodeTable, NULL);
		}
	    }
	}
    }

    for (seg = rt->segments; seg && (seg != nseg); seg = seg->next);
    if (seg == NULL) return 0.0;

    for (; seg; seg = seg->next) {

	/* Once the layer goes above the current check layer, the search stops. */
	if (method != ANTENNA_DISABLE)
	    if (seg->layer > layer) break;

	/* Vias don't contribute to area, at least for now. */
	if (seg->segtype & ST_VIA) continue;

	/* For non-cumulative methods, only count area for	*/
	/* those segments which are on the given check layer.	*/

	if ((method == CALC_AREA) || (method == CALC_SIDEAREA))
	    if (seg->layer != layer)
		continue;

	/* method ANTENNA_ROUTE indicates that this routine was	*/
	/* called as part of antenna routing.  So set up this	*/
	/* part of the route in a manner similar to the		*/
	/* set_route_to_net() routine.				*/

	if ((method == ANTENNA_ROUTE) && (iroute != NULL)) {
	    PROUTE *Pr;
	    POINT gpoint;

	    l = seg->layer;
	    x = seg->x1;
	    y = seg->y1;
	    while (1) {
		Pr = &OBS2VAL(x, y, l);
		Pr->flags = PR_SOURCE;
		Pr->prdata.cost = 0;

		if (~(Pr->flags & PR_ON_STACK)) {
		    Pr->flags |= PR_ON_STACK;
		    gpoint = allocPOINT();
		    gpoint->x1 = x;
		    gpoint->y1 = y;
		    gpoint->layer = l;
		    gpoint->next = iroute->glist[0];
		    iroute->glist[0] = gpoint;
		}

		if (x < iroute->bbox.x1) iroute->bbox.x1 = x;
		if (x > iroute->bbox.x2) iroute->bbox.x2 = x;
		if (y < iroute->bbox.y1) iroute->bbox.y1 = y;
		if (y > iroute->bbox.y2) iroute->bbox.y2 = y;

		// Move to next grid position in the segment
		if (x == seg->x2 && y == seg->y2) break;
		if (seg->x2 > seg->x1) x++;
		else if (seg->x2 < seg->x1) x--;
		if (seg->y2 > seg->y1) y++;
		else if (seg->y2 < seg->y1) y--;
	    }
	}
	else if (method == ANTENNA_DISABLE) {
	    PROUTE *Pr;

	    l = seg->layer;
	    x = seg->x1;
	    y = seg->y1;
	    while (1) {
		Pr = &OBS2VAL(x, y, l);
		Pr->prdata.net = MAXNETNUM;
		Pr->flags &= ~(PR_SOURCE | PR_TARGET | PR_COST);

		// Move to next grid position in the segment
		if (x == seg->x2 && y == seg->y2) break;
		if (seg->x2 > seg->x1) x++;
		else if (seg->x2 < seg->x1) x--;
		if (seg->y2 > seg->y1) y++;
		else if (seg->y2 < seg->y1) y--;
	    }
	}
	if ((method != ANTENNA_ROUTE) && (method != ANTENNA_DISABLE)) {

	    /* Note that one of x or y is zero, depending on segment orientation */
	    x = (seg->x2 - seg->x1);
	    y = (seg->y2 - seg->y1);
	    if (x < 0) x = -x;
	    if (y < 0) y = -y;

	    /* Note that "l" is a unitless grid dimension */
	    if (x == 0)
		length = (float)y * (float)PitchY;
	    else
		length = (float)x * (float)PitchX;

	    /* area is either the total top surface of the metal, */
	    /* or the total side surface of the metal (in um^2)	  */

	    width = LefGetRouteWidth(seg->layer);
	    if ((method == CALC_AREA) || (method == CALC_AGG_AREA))
		area += (float)(length * width);
	    else if ((method == CALC_SIDEAREA) || (method == CALC_AGG_SIDEAREA)) {
		thick = LefGetRouteThickness(seg->layer);
		area += thick * 2.0 * (length + width);
	    }
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
	    if (l > layer) continue;
	}
	else if (!(rt2->flags & RT_END_NODE) && (rt2->end.route == rt)) {
	    /* The end point of rt2 connects somewhere on rt */
	    for (iseg = rt2->segments; iseg && iseg->next; iseg = iseg->next);
	    x = iseg->x2;
	    y = iseg->y2;
	    l = iseg->layer;
	    if (l > layer) continue;
	}
	else
	    continue;

	/* Must determine if rt2 intersects rt within the antenna area */

	found = (u_char)0;
	for (chkseg = rt->segments; chkseg && chkseg != seg; chkseg = chkseg->next) {
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
	}
	if (found == (u_char)1) {
	    if (rt2->start.route == rt)
		area += get_route_area_forward(net, rt2, layer, visited,
				method, NodeTable, iroute);
	    else
		area += get_route_area_reverse(net, rt2, layer, visited,
				method, NodeTable, iroute);
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

	    /* Walk all other routes that start or end on this node */

	    for (rt2 = net->routes; rt2; rt2 = rt2->next) {
		if (rt2->flags & RT_VISITED) continue;

		if ((rt2->flags & RT_START_NODE) && (rt2->start.node == node)) {
		    /* The start point of rt2 connects to the same node */
		    area += get_route_area_forward(net, rt2, layer, visited,
				method, NodeTable, NULL);
		}
		else if ((rt2->flags & RT_END_NODE) && (rt2->end.node == node)) {
		    /* The end point of rt2 connects to the same node */
		    for (iseg = rt2->segments; iseg && iseg->next; iseg = iseg->next);
		    area += get_route_area_reverse(net, rt2, layer, visited,
				method, NodeTable, NULL);
		}
	    }

	    g = FindGateNode(NodeTable, node, &i);
	    if (g == NULL) {
		/* This should not happen */
	 	Fprintf(stderr, "Error: net %s route end marked as node, but"
			" no node found!\n", net->netname);
		return 0.0;
	    }
	    if (g->area[i] == 0.0) {
		/* There's a diffusion diode here! */
		if (visited) visited[node->nodenum] = ANCHOR;
		return 0.0;
	    } else {
		/* Add this node to the list of nodes with gates	*/
		/* attached to this antenna area.			*/
		if (visited) visited[node->nodenum] = VISITED;
	    }
	    if ((method == ANTENNA_ROUTE) && (iroute != NULL)) {
		set_node_to_net(node, PR_SOURCE, iroute->glist[0], iroute->bbox, 0);
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
	    if (rt2 == NULL) return;  /* This should not happen */

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
			layer, visited, method, NodeTable, iroute);
	    area += get_route_area_reverse_fromseg(net, rt2, rseg, layer,
			visited, method, NodeTable, iroute);
	}
    }
    return area;
}

/*--------------------------------------------------------------*/
/* Check route antenna forward from the beginning of the route.	*/
/*--------------------------------------------------------------*/

float
get_route_area_forward(NET net, ROUTE rt, int layer, u_char *visited,
	u_char method, Tcl_HashTable *NodeTable, struct routeinfo_ *iroute)
{
    float area;

    area = get_route_area_forward_fromseg(net, rt, NULL, layer, visited,
		method, NodeTable, iroute);
    return area;
}

/*--------------------------------------------------------------*/
/* This is the same as get_route_area_forward_fromseg, but is	*/
/* searching the path from end to beginning, so reverse the	*/
/* route first and then call get_route_area_forward_fromseg().	*/
/*--------------------------------------------------------------*/

float
get_route_area_reverse_fromseg(NET net, ROUTE rt, SEG nseg, int layer,
	u_char *visited, u_char method, Tcl_HashTable *NodeTable,
	struct routeinfo_ *iroute)
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
		method, NodeTable, iroute);

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
		u_char method, Tcl_HashTable *NodeTable,
		struct routeinfo_ *iroute)
{
    float area;
    area = get_route_area_reverse_fromseg(net, rt, NULL, layer, visited,
		method, NodeTable, iroute);
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
    ROUTE rt, saveroute;
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
		    saveroute = rt;
		    metal_area += get_route_area_forward(net, rt, layer, visited,
				method, NodeTable, NULL);
		}
		else if ((rt->flags & RT_END_NODE) && (rt->end.node == node)) {
		    saveroute = rt;
		    metal_area += get_route_area_reverse(net, rt, layer, visited,
				method, NodeTable, NULL);
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
		    newantenna->route = saveroute;
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
/* This routine is a combination of set_node_to_net(),		*/
/* set_routes_to_net(), and disable_node_nets() (see qrouter.c	*/
/* and maze.c), but walks the routes in the same manner used	*/
/* for finding the antenna violations.  Set the antenna part of	*/
/* the net as SOURCE, the free antenna taps as TARGET, and the	*/
/* non-antenna portion of the net to an unused net number,	*/
/* which can be converted back after routing.			*/
/*--------------------------------------------------------------*/

int set_antenna_to_net(int newflags, struct routeinfo_ *iroute,
		u_char stage, ANTENNAINFO violation, Tcl_HashTable *NodeTable)
{
    int x, y, lay, rval, layer;
    PROUTE *Pr;
    ROUTE rt, clrrt;
    NODE node;
    NET net;

    /* Set the node and connected antenna metal routes to PR_SOURCE.	*/

    rt = violation->route;
    node = violation->node;
    net = violation->net;
    layer = violation->layer;

    if ((rt->flags & RT_START_NODE) && (rt->start.node == node))
	get_route_area_forward(net, rt, layer, NULL, ANTENNA_ROUTE, NodeTable,
		iroute);
    else if ((rt->flags & RT_END_NODE) && (rt->end.node == node))
	get_route_area_reverse(net, rt, layer, NULL, ANTENNA_ROUTE, NodeTable,
		iroute);
    else {
	/* This should not happen */
	Fprintf(stderr, "Error:  Antenna route and node do not connect!\n");
	return 1;
    }

    /* Clear route visited flags for next pass */
    for (clrrt = iroute->net->routes; clrrt; clrrt = clrrt->next)
	clrrt->flags &= ~RT_VISITED;

    /* Disable the remainder of the route */

    if ((rt->flags & RT_START_NODE) && (rt->start.node == node))
	get_route_area_forward(net, rt, layer, NULL, ANTENNA_DISABLE, NodeTable,
		iroute);
    else if ((rt->flags & RT_END_NODE) && (rt->end.node == node))
	get_route_area_reverse(net, rt, layer, NULL, ANTENNA_DISABLE, NodeTable,
		iroute);
    else {
	/* This should not happen */
	Fprintf(stderr, "Error:  Antenna route and node do not connect!\n");
	return 1;
    }

    /* Done checking routes;  clear route visited flags */
    for (clrrt = iroute->net->routes; clrrt; clrrt = clrrt->next)
	clrrt->flags &= ~RT_VISITED;

    /* Set the antenna taps to the net number.		*/
    /* Routine is similar to set_powerbus_to_net().	*/

    rval = 0;
    for (lay = 0; lay < Num_layers; lay++)
	for (x = 0; x < NumChannelsX; x++)
	    for (y = 0; y < NumChannelsY; y++)
		if ((OBSVAL(x, y, lay) & NETNUM_MASK) == ANTENNA_NET) {
		    Pr = &OBS2VAL(x, y, lay);
		    // Skip locations that have been purposefully disabled
		    if (!(Pr->flags & PR_COST) && (Pr->prdata.net == MAXNETNUM))
			continue;
		    else if (!(Pr->flags & PR_SOURCE)) {
			Pr->flags |= (PR_TARGET | PR_COST);
			Pr->prdata.cost = MAXRT;
			rval = 1;
			OBSVAL(x, y, lay) &= ~NETNUM_MASK;
			OBSVAL(x, y, lay) |= net->netnum;
		    }
		}

    return rval;
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

int antenna_setup(struct routeinfo_ *iroute, ANTENNAINFO violation,
	Tcl_HashTable *NodeTable)
{
    int i, j, netnum, rval;
    PROUTE *Pr;

    for (i = 0; i < Num_layers; i++) {
	for (j = 0; j < NumChannelsX * NumChannelsY; j++) {
	    netnum = Obs[i][j] & (~BLOCKED_MASK);
	    Pr = &Obs2[i][j];
	    if (netnum != 0) {
		Pr->flags = 0;            // Clear all flags
		if (netnum == DRC_BLOCKAGE)
		    Pr->prdata.net = netnum;
		else
		    Pr->prdata.net = netnum & NETNUM_MASK;
	    } else {
		Pr->flags = PR_COST;              // This location is routable
		Pr->prdata.cost = MAXRT;
	    }
	}
    }

    // Fill out route information record

    iroute->net = violation->net;
    iroute->rt = NULL;
    for (i = 0; i < 6; i++)
	iroute->glist[i] = NULL;
    iroute->nsrc = violation->node;
    iroute->nsrctap = iroute->nsrc->taps;
    iroute->maxcost = MAXRT;
    iroute->do_pwrbus = TRUE;
    iroute->pwrbus_src = 0;

    iroute->bbox.x2 = iroute->bbox.y2 = 0;
    iroute->bbox.x1 = NumChannelsX;
    iroute->bbox.y1 = NumChannelsY;

    rval = set_antenna_to_net(PR_SOURCE, iroute, 0, violation, NodeTable);

    /* Unlikely that MASK_BBOX would be useful, since one does	*/
    /* not know if an antenna tap is inside the box or not.	*/
    /* Maybe if bounding box is expanded to encompass some	*/
    /* number of taps. . .					*/

    // if (maskMode == MASK_NONE)
	   fillMask((u_char)0);
    // else if (maskMode == MASK_BBOX)
    //	   createBboxMask(iroute->net, (u_char)Numpasses);

    iroute->maxcost = 20;
    return rval;
}

/*--------------------------------------------------------------*/
/* The simplest way to fix an antenna violation is to find	*/
/* a place in the antenna metal to break the antenna and pull	*/
/* it up to a higher level of metal.  Depending on the severity	*/
/* of the antenna violation, this may need to be done more than	*/
/* once.  If no place to break the antenna is found, return -1	*/
/* for failure.							*/
/*--------------------------------------------------------------*/

int simpleantennafix(ANTENNAINFO violation, Tcl_HashTable *NodeTable)
{
    return -1;		/* Antenna was not fixed */
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

int doantennaroute(ANTENNAINFO violation, Tcl_HashTable *NodeTable)
{
    NET net;
    NODE node;
    ROUTE rt1, lrt;
    int layer, i, result, savelayers;
    struct routeinfo_ iroute;

    net = violation->net;
    node = violation->node;
    layer = violation->layer;

    result = antenna_setup(&iroute, violation, NodeTable);

    rt1 = createemptyroute();
    rt1->netnum = net->netnum;
    iroute.rt = rt1;

    /* Force routing to be done at or below the antenna check layer.	*/

    savelayers = Num_layers;
    Num_layers = violation->layer + 1;

    result = route_segs(&iroute, 0, (u_char)0);

    Num_layers = savelayers;

    if (result < 0) {
	/* To do:  Handle failures? */
	Fprintf(stderr, "Antenna anchoring route failed.\n");
	free(rt1);
    }
    else {
	TotalRoutes++;
	if (net->routes) {
	    for (lrt = net->routes; lrt->next; lrt = lrt->next);
	    lrt->next = rt1;
	}
	else {
	    /* This should not happen */
	    Fprintf(stderr, "Error:  Net has no routes!\n");
	    net->routes = rt1;
	}
    }

    /* For power-bus-type routing, glist is not empty after routing */
    free_glist(&iroute);

    /* Put free taps back to ANTENNA_NET */
    revert_antenna_taps(net->netnum, rt1->start.node);

    return result;
}

/*--------------------------------------------------------------*/
/* Top level routine called from tclqrouter.c			*/
/*--------------------------------------------------------------*/

void
resolve_antenna(char *antennacell, u_char do_fix)
{
    FILE *fout;
    int numtaps, numerrors, numfixed, result;
    int layererrors;
    int layer, i, new;
    Tcl_HashTable NodeTable;
    Tcl_HashEntry *entry;
    GATE g;
    NET net;
    ROUTE rt;
    ANTENNAINFO nextviolation, FixedList = NULL, BadList = NULL;

    numtaps = count_free_antenna_taps(antennacell);
    if (Verbose > 3) {
	Fprintf(stdout, "Number of free antenna taps = %d\n", numtaps);
    }

    AntennaList = NULL;
    numerrors = 0;
    numfixed = 0;

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

	/* Fix the violations found on this layer before moving	*/
	/* on to the next layer.				*/

	while (AntennaList != NULL) {
	    nextviolation = AntennaList->next;
    
	    if (do_fix) {
		result = simpleantennafix(AntennaList, &NodeTable);
		if (result == 0) {
		    /* No antenna cell involved, so no backannotation	*/
		    /* required.  Remove the "route" record. */
		    AntennaList->route = NULL;
		}
		else
		    result = doantennaroute(AntennaList, &NodeTable);
		if (result >= 0) numfixed++;
	    }

	    /* Move the error information to either the Fixed or Bad lists */
	    if (result >= 0) {
		AntennaList->next = FixedList;
		FixedList = AntennaList;
		if (AntennaList->route != NULL) {
		    /* Replace the route record with the last route	*/
		    /* of the net, which was the route added to fix.	*/
		    /* If the net requires more than one antenna 	*/
		    /* anchor, then routes won't be confused.		*/
		    for (rt = AntennaList->net->routes; rt && rt->next; rt = rt->next);
		    AntennaList->route = rt;
		}
	    }
	    else {
		AntennaList->next = BadList;
		BadList = AntennaList;
	    }
	    AntennaList = nextviolation;
	}
    }

    if (Verbose > 0) {
	Fprintf(stdout, "Total number of antenna errors found = %d\n", numerrors);
	if (do_fix)
	    Fprintf(stdout, "Total number of antenna errors fixed = %d\n", numfixed);
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

    /* Output the violation lists.  The fixed violations need to be	*/
    /* known so that the additional connection to the net can be added	*/
    /* to the netlist for verification purposes.  The unfixed		*/
    /* violations need to be reported so they can be tracked down and	*/
    /* fixed by hand.							*/

    if ((FixedList != NULL) || (BadList != NULL))
	fout = fopen("antenna.out", "w");

    if (FixedList != NULL) {
	ROUTE rt;
	fprintf(fout, "Revised netlist: New antenna anchor connections\n");

	for (nextviolation = FixedList; nextviolation;
			nextviolation = nextviolation->next) {
	    // NOTE:  nextviolation->route was changed from the route that
	    // connects to the gate in violation, to the route that fixes
	    // the antenna error.
	    g = FindGateNode(&NodeTable, nextviolation->route->start.node, &i);
	    fprintf(fout, "Net=%s Instance=%s Cell=%s Pin=%s\n",
			nextviolation->net->netname, g->gatename,
			g->gatetype->gatename, g->gatetype->node[i]);
	}
	fprintf(fout, "\n");
    }

    if (BadList != NULL) {
	fprintf(fout, "Unfixed antenna errors:\n");

	for (nextviolation = BadList; nextviolation;
			nextviolation = nextviolation->next) {
	    g = FindGateNode(&NodeTable, nextviolation->node, &i);
	    fprintf(fout, "Net=%s Instance=%s Cell=%s Pin=%s error on Metal%d\n",
			nextviolation->net->netname,
			g->gatename, g->gatetype->gatename,
			g->gatetype->node[i], nextviolation->layer + 1);
	}
    }

    if ((FixedList != NULL) || (BadList != NULL)) fclose(fout);

    /* Free up the node hash table */

    FreeNodeTable(&NodeTable);
    Tcl_DeleteHashTable(&NodeTable);

    /* Free up the violation lists */

    while (FixedList != NULL) {
	nextviolation = FixedList->next;
	free(FixedList);
	FixedList = nextviolation;
    }
    while (BadList != NULL) {
	nextviolation = BadList->next;
	free(BadList);
	BadList = nextviolation;
    }
}

#endif	/* TCL_QROUTER */

/* end of antenna.c */
