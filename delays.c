/*--------------------------------------------------------------*/
/*  delays.c -- compute and write path delays from a routed	*/
/*  network.							*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, March 2017				*/
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

/*--------------------------------------------------------------*/
/* Find a node in the node list.				*/
/*--------------------------------------------------------------*/

/* Define record holding information pointing to a gate and the	*/
/* index into a specific node of that gate.			*/

typedef struct gatenode_ *GATENODE;

struct gatenode_ {
    GATE gate;
    int idx;
};

GATE
FindGateNode(Tcl_HashTable *NodeTable, NODE node, int *ridx)
{
    GATENODE gn;
    GATE g;
    Tcl_HashEntry *entry;

    entry = Tcl_FindHashEntry(NodeTable, (char *)node);
    if (entry) {
	gn = (GATENODE)Tcl_GetHashValue(entry);
	*ridx = gn->idx;
	return gn->gate;
    }
    return NULL;
}

/*--------------------------------------------------------------*/
/* Structure to hold information about endpoints of a route.	*/
/*--------------------------------------------------------------*/

typedef struct _endpointinfo {
    u_char flags;		/* flag bits (see below) */
    ROUTE route;		/* pointer to routed segments */
    ROUTE orig;			/* original pointer to routed segments */
    int startx;			/* values at segment start */
    int starty;
    int startl;
    u_char starttype;
    NODE startnode;
    int endx;			/* values at segment end */
    int endy;
    int endl;
    u_char endtype;
    NODE endnode;
    double res;			/* total resistance of segment	*/
    double cap;			/* total capacitance of segment */
    int *branching;		/* list of downstream segments	*/
				/* (last list item is -1)	*/
} endpointinfo;

/* endpointinfo flag definitions */
#define EPT_VISITED    0x01	/* 1 if endpoint has been visited */
#define EPT_DRIVER     0x02	/* 1 if endpoint is a driver */

/* Structure to hold R and C information for a path */

typedef struct _rcinfo {
    double res;
    double cap;
} rcinfo;

/* Structure to hold R and C information for a layer/via	  */
/* (viares is recorded for the layer number of the via bottom) */

typedef struct _lefrcinfo {
    double resx;	/* Resistance per track in X */
    double resy;	/* Resistance per track in Y */
    double capx;	/* Capacitance per track in X */
    double capy;	/* Capacitance per track in Y */
    double viares;	/* Resistance per via */
} lefrcinfo;

/* Forward declaration */

void walk_route(int, int, endpointinfo *, int, lefrcinfo *);

/*--------------------------------------------------------------*/
/* Add route information to the endpoint record showing where	*/
/* a route continues downstream.				*/
/*--------------------------------------------------------------*/

void
add_route_to_endpoint(endpointinfo *eptinfo, int eidx, int didx)
{
    int i;

    for (i = 0; i < 5; i++) {
	if (eptinfo[eidx].branching[i] == -1) {
	    eptinfo[eidx].branching[i] = didx;
	    if (i < 4) eptinfo[eidx].branching[i + 1] = -1;
	    break;
	}
    }
}

/*--------------------------------------------------------------*/
/* Check for a route segment that is downstream of the current	*/
/* segment (walkseg), and if found, process it.			*/
/* "end" is 0 if checking downstream of the driver node.  Every	*/
/* other check is from the end node of a route, and "end" is 1.	*/
/*--------------------------------------------------------------*/

void
check_downstream(SEG walkseg, endpointinfo *eptinfo, int eidx,
	int numroutes, lefrcinfo *lefrcvalues, u_char end)
{
    int i;
    int startcompat, endcompat;
    NODE nodeptr;

    /* At given segment "walkseg", find all routes that connect and walk them */

    for (i = 0; i < numroutes; i++) {
	if (eptinfo[i].flags & EPT_VISITED) continue;  /* already visited */

	/* Check wire/via layer compatibility */

	if (eptinfo[i].starttype & ST_WIRE) {
	    if (walkseg->segtype & ST_WIRE)
		startcompat = (walkseg->layer == eptinfo[i].startl);
	    else
		startcompat = (walkseg->layer == eptinfo[i].startl) ||
			(walkseg->layer + 1 == eptinfo[i].startl);
	}
	else {
	    if (walkseg->segtype & ST_WIRE)
		startcompat = (walkseg->layer == eptinfo[i].startl) ||
			(walkseg->layer == eptinfo[i].startl + 1);
	    else
		startcompat = (walkseg->layer == eptinfo[i].startl) ||
			(walkseg->layer == eptinfo[i].startl + 1) ||
			(walkseg->layer + 1 == eptinfo[i].startl);
	}

	if (eptinfo[i].endtype & ST_WIRE) {
	    if (walkseg->segtype & ST_WIRE)
		endcompat = (walkseg->layer == eptinfo[i].endl);
	    else
		endcompat = (walkseg->layer == eptinfo[i].endl) ||
			(walkseg->layer + 1 == eptinfo[i].endl);
	}
	else {
	    if (walkseg->segtype & ST_WIRE)
		endcompat = (walkseg->layer == eptinfo[i].endl) ||
			(walkseg->layer == eptinfo[i].endl + 1);
	    else
		endcompat = (walkseg->layer == eptinfo[i].endl) ||
			(walkseg->layer == eptinfo[i].endl + 1) ||
			(walkseg->layer + 1 == eptinfo[i].endl);
	}

	if ((walkseg->x2 == eptinfo[i].startx) &&
		(walkseg->y2 == eptinfo[i].starty) && startcompat) {
	    /* Watch for short via routes that are compatible	*/
	    /* on both start and end---walk from the higher	*/
	    /* side.						*/
	    int reverse = 0;
	    if ((eptinfo[i].startx == eptinfo[i].endx) &&
			(eptinfo[i].starty == eptinfo[i].endy) &&
			startcompat && endcompat)
		if (eptinfo[i].endl > eptinfo[i].startl)
		    reverse = 1;

	    /* Diagnostic */
	    /*
	    Fprintf(stdout, "Connects to %d, %d, %d\n",
			eptinfo[i].startx, eptinfo[i].starty, eptinfo[i].startl);
	    */
	    /* Recursive walk */
	    walk_route(i, reverse, eptinfo, numroutes, lefrcvalues);
	    add_route_to_endpoint(eptinfo, eidx, i);
	}
	else if ((walkseg->x2 == eptinfo[i].endx) &&
		(walkseg->y2 == eptinfo[i].endy) && endcompat) {
	    /* Diagnostic */
	    /*
	    Fprintf(stdout, "Connects to %d, %d, %d\n",
			eptinfo[i].endx, eptinfo[i].endy, eptinfo[i].endl);
	    */
	    /* If this is a node, output it now */
	    /* Recursive walk */
	    walk_route(i, 1, eptinfo, numroutes, lefrcvalues);
	    add_route_to_endpoint(eptinfo, eidx, i);
	}
    }

    /* If there is a node at the segment being checked, then walk any	*/
    /* path that connects to the same node.  This catches instances in	*/
    /* which a two paths may connect to a node at two different		*/
    /* locations.							*/

    nodeptr = (end == 0) ? eptinfo[eidx].startnode : eptinfo[eidx].endnode;

    if (nodeptr != NULL) {
	for (i = 0; i < numroutes; i++) {
	    if (eptinfo[i].flags & EPT_VISITED) continue;  /* already visited */
	    if (eptinfo[i].startnode == nodeptr) {
		walk_route(i, 0, eptinfo, numroutes, lefrcvalues);
		add_route_to_endpoint(eptinfo, eidx, i);
	    }
	    else if (eptinfo[i].endnode == nodeptr) {
		walk_route(i, 1, eptinfo, numroutes, lefrcvalues);
		add_route_to_endpoint(eptinfo, eidx, i);
	    }
	}
    }
}

/*--------------------------------------------------------------*/
/* Recursively walk a route to all endpoints, computing the	*/
/* path R and C values along the way.				*/
/*								*/
/* eidx is an index into eptinfo for the current route		*/
/* driverend is the upstream endpoint of that route		*/
/*	(0 = start of segment, 1 = end of segment).		*/
/* eptinfo contains the endpoints of all the routes.		*/
/* numroutes is te number of entries in eptinfo.		*/
/* delayFile is the output file to write to.			*/
/*								*/
/* Return the R and C values for the segment			*/
/*--------------------------------------------------------------*/

void
walk_route(int eidx, int driverend, endpointinfo *eptinfo,
		int numroutes, lefrcinfo *lefrcvalues)
{
    SEG firstseg, lastseg;
    SEG walkseg, newseg, testseg;
    SEG seg, nseg;
    GATE g;
    NODE node;
    int i;
    u_char f;
    ROUTE rt;

    eptinfo[eidx].flags |= EPT_VISITED;

    /* Always walk the segment from upstream to downstream.	*/
    /* If the upstream side is the end of the segment linked	*/
    /* list (driverend == 1), then replace the route with a	*/
    /* reversed copy.						*/
 
    rt = eptinfo[eidx].route;

    if (driverend == 1) {
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

	/* Delete the original route and replace it */
	for (seg = rt->segments; seg; ) {
	    nseg = seg->next;
	    free(seg);
	    seg = nseg;
	}
	rt->segments = firstseg;

	/* Everything in eptinfo related to start and end needs	*/
	/* to be swapped.					*/

	node = eptinfo[eidx].startnode;
	eptinfo[eidx].startnode = eptinfo[eidx].endnode;
	eptinfo[eidx].endnode = node;

	i = eptinfo[eidx].startx;
	eptinfo[eidx].startx = eptinfo[eidx].endx;
	eptinfo[eidx].endx = i;

	i = eptinfo[eidx].starty;
	eptinfo[eidx].starty = eptinfo[eidx].endy;
	eptinfo[eidx].endy = i;

	i = eptinfo[eidx].startl;
	eptinfo[eidx].startl = eptinfo[eidx].endl;
	eptinfo[eidx].endl = i;

	f = eptinfo[eidx].starttype;
	eptinfo[eidx].starttype = eptinfo[eidx].endtype;
	eptinfo[eidx].endtype = f;
    }
    else
	firstseg = rt->segments;

    /* Check for downstream nodes from the first route point, but only	*/
    /* if it is the driver.						*/

    if (eptinfo[eidx].flags & EPT_DRIVER)
	check_downstream(firstseg, eptinfo, eidx, numroutes, lefrcvalues, (u_char)0);

    /* Walk the route segment and accumulate R and C */

    eptinfo[eidx].res = 0.0;
    eptinfo[eidx].cap = 0.0;

    for (walkseg = firstseg; walkseg; walkseg = walkseg->next) {
	int rlength;

	/* Accumulate C and R */
	if (walkseg->segtype & ST_VIA) {
	    eptinfo[eidx].res += lefrcvalues[walkseg->layer].viares;
	}
	else if (walkseg->x1 == walkseg->x2) {  /* Vertical route */
	    rlength = (walkseg->y2 > walkseg->y1) ?
			(walkseg->y2 - walkseg->y1 + 1) :
			(walkseg->y1 - walkseg->y2 + 1);
			
	    eptinfo[eidx].res += lefrcvalues[walkseg->layer].resy * rlength;
	    eptinfo[eidx].cap += lefrcvalues[walkseg->layer].capy * rlength;
	}
	else {	/* Horizontal route */
	    rlength = (walkseg->x2 > walkseg->x1) ?
			(walkseg->x2 - walkseg->x1 + 1) :
			(walkseg->x1 - walkseg->x2 + 1);

	    eptinfo[eidx].res += lefrcvalues[walkseg->layer].resx * rlength;
	    eptinfo[eidx].cap += lefrcvalues[walkseg->layer].capx * rlength;
	}
	if (walkseg->next == NULL) lastseg = walkseg;
    }

    /* Check for downstream nodes from the last route point */
    check_downstream(lastseg, eptinfo, eidx, numroutes, lefrcvalues, (u_char)1);
}

/*--------------------------------------------------------------*/
/* Walk the sorted, directed routes and generate output.	*/
/*--------------------------------------------------------------*/

void
walk_route_output(endpointinfo *eptinfo, int eidx,
		Tcl_HashTable *NodeTable, FILE *delayFile)
{
    int d, i;
    NODE node;
    GATE g;

    /* Output information about self */

    fprintf(delayFile, "( %g %g ", eptinfo[eidx].res, eptinfo[eidx].cap);

    /* Count downstream nodes */

    for (d = 0; d < 5; d++)
	if (eptinfo[eidx].branching[d] == -1)
	    break;

    /* List of nodes and downstream routes follows */

    node = eptinfo[eidx].endnode;

    if (node != NULL) {
	/* Look up the gate */
	g = FindGateNode(NodeTable, node, &i);
	if (!strcmp(g->gatetype->node[i], "pin"))
	    fprintf(delayFile, "PIN/%s ", g->gatename);
	else
	    fprintf(delayFile, "%s/%s ", g->gatename, g->gatetype->node[i]);
	if (d > 0) fprintf(delayFile, ", ");
    }


    /* Output downstream nodes */
    for (i = 0; i < d; i++) {
	walk_route_output(eptinfo, eptinfo[eidx].branching[i],
			NodeTable, delayFile);
	if (i < (d - 1)) fprintf(delayFile, ", ");
    }

    /* End record */
    fprintf(delayFile, ") ");
}

/*--------------------------------------------------------------*/
/* Write an output file of the calculated R, C for every route	*/
/* branch.  Because the qrouter algorithm is agnostic about the	*/
/* direction of the signaling of routes, this has to be 	*/
/* discovered from the information at hand.  The routes for	*/
/* each net are reorganized into directed segments, and the	*/
/* whole directed tree walked from beginning to every endpoint.	*/
/* The routing algorithm is also unaware of any details of the	*/
/* nodes it routes to, so it is necessary to create a table of	*/
/* all nodes, referenced by the pointer address found in the	*/
/* nodeinfo array.						*/
/*--------------------------------------------------------------*/

int write_delays(char *filename)
{
    FILE *delayFile;
    NET net;
    ROUTE rt, nxroute;
    ROUTE droutes, newroute, lastroute;
    NODEINFO nodeptr;
    SEG seg, newseg, lastseg, nxseg;
    GATE g, drivergate;
    int i, j, n, new, driverend;
    int drivernodeidx, driveridx;
    int nroute, numroutes;
    endpointinfo *eptinfo;
    lefrcinfo *lefrcvalues;

    Tcl_HashTable NodeTable;
    Tcl_HashEntry *entry;

    if (!strcmp(filename, "stdout"))
	delayFile = stdout;
    else if (filename == NULL)
	delayFile = fopen(delayfilename, "w");
    else
	delayFile = fopen(filename, "w");

    if (!delayFile) {
	Fprintf(stderr, "write_delays():  Couldn't open output delay file.\n");
	return -1;
    }

    /* Build a hash table of nodes;  key = node record address,		*/
    /* record = pointer to gate and index of the node in its noderec.	*/

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

    /* Fill in the record of R and C values per layer, for efficiency */

    lefrcvalues = (lefrcinfo *)malloc(Num_layers * sizeof(lefrcinfo));
    for (i = 0; i < Num_layers; i++) {
	double areacap, edgecap;
	double respersq, respervia;
	double width, sqx, sqy;

	LefGetRouteRCvalues(i, &areacap, &edgecap, &respersq);
	width = LefGetRouteWidth(i);

	lefrcvalues[i].resx = (PitchX[i] / width) * respersq;
	lefrcvalues[i].resy = (PitchY[i] / width) * respersq;

	lefrcvalues[i].capx = (PitchX[i] * width) * areacap + (PitchX[i] * edgecap);
	lefrcvalues[i].capy = (PitchY[i] * width) * areacap + (PitchY[i] * edgecap);

	if (i < (Num_layers - 1))
	    LefGetViaResistance(i, &(lefrcvalues[i].viares));
	else
	    lefrcvalues[i].viares = 0.0;	/* Not used */
    }

    /* Each net is output independently.  Loop through all nets. */

    for (n = 0; n < Numnets; n++) {
	net = Nlnets[n];

	if ((net->netnum == VDD_NET) || (net->netnum == GND_NET)) continue;

	/* Count number of net routes */
	numroutes = 0;
	for (rt = net->routes; rt; rt = rt->next) numroutes++;
	if (numroutes == 0) continue;	/* Ignore nets with no routes */

	/* Marked as one driver node.  Not handling more than one driver yet. */
        fprintf(delayFile, "%s 1", net->netname);

	/* Determine the driver node, as determined by the node with	*/
	/* LEF direction 'OUTPUT'.  					*/
	/* (For now, if a net has multiple tristate drivers, just use	*/
	/* the first one and treat the rest as receivers.)		*/

	/* Allocate space for endpoint info */
	eptinfo = (endpointinfo *)malloc(numroutes * sizeof(endpointinfo));
	
	/* Fill in initial endpoint information */
	nroute = 0;
	for (rt = net->routes; rt; rt = rt->next) {
	    eptinfo[nroute].route = rt;
	    eptinfo[nroute].orig = rt;
	    eptinfo[nroute].flags = (u_char)0;
	    eptinfo[nroute].branching = NULL;
	    eptinfo[nroute].startnode = NULL;
	    eptinfo[nroute].endnode = NULL;

	    /* Segment start */
	    seg = rt->segments;
            if (seg != NULL) {
		eptinfo[nroute].startx = seg->x1;
		eptinfo[nroute].starty = seg->y1;
		eptinfo[nroute].startl = seg->layer;
		eptinfo[nroute].starttype = seg->segtype;
		eptinfo[nroute].res = 0.0;
		eptinfo[nroute].cap = 0.0;
	    }

	    /* Segment end */
	    for (seg = rt->segments; seg && seg->next; seg = seg->next);
	    if (seg != NULL) {
		eptinfo[nroute].endx = seg->x2;
		eptinfo[nroute].endy = seg->y2;
		eptinfo[nroute].endl = seg->layer;
		eptinfo[nroute].endtype = seg->segtype;
	    }
	    nroute++;
	}

	/* Copy net->routes into droutes, and update eptinfo */

	droutes = (ROUTE)NULL;
	lastroute = (ROUTE)NULL;
	i = 0;
	for (rt = net->routes; rt; rt = rt->next) {
	    newroute = (ROUTE)malloc(sizeof(struct route_));
	    newroute->next = NULL;
	    if (lastroute == NULL)
		droutes = newroute;
	    else
		lastroute->next = newroute; 
	    lastroute = newroute;
	    newroute->segments = NULL;
	    newroute->start.route = NULL;
	    newroute->end.route = NULL;
	    newroute->flags = (u_char)0;
	    newroute->netnum = rt->netnum;
	    eptinfo[i].route = newroute;

	    lastseg = (SEG)NULL;
	    for (seg = rt->segments; seg; seg = seg->next) {
		newseg = (SEG)malloc(sizeof(struct seg_));
		if (lastseg == NULL)
		    newroute->segments = newseg;
		else
		    lastseg->next = newseg;
		lastseg = newseg;
		newseg->x1 = seg->x1;
		newseg->x2 = seg->x2;
		newseg->y1 = seg->y1;
		newseg->y2 = seg->y2;
		newseg->layer = seg->layer;
		newseg->segtype = seg->segtype;
		newseg->next = (SEG)NULL;
	    }
	    i++;
	}

	/* Check each point of each route against the endpoints of the	*/
	/* other routes, and break routes at connection points, so that	*/
	/* each route is an independent segment for calculating R, C.	*/

	j = 0;
	for (rt = droutes; rt; rt = rt->next) {
	    ROUTE testroute;
	    int startx, starty, startl, starttype;
	    int endx, endy, endl, endtype;
	    int brkx, brky, brki, startcompat, endcompat;
	    int initial, final;
	    int x1, y1, x2, y2;

	    /* Check all segments (but not the endpoints) */
	    for (seg = rt->segments; seg; seg = seg->next) {
		initial = (seg == rt->segments) ? 1 : 0;
		final = (seg->next == NULL) ? 1 : 0;

		if (initial && (seg->segtype & ST_VIA)) continue;
		if (final && (seg->segtype & ST_VIA)) continue;

		x1 = seg->x1;
		x2 = seg->x2;
		y1 = seg->y1;
		y2 = seg->y2;

		if (initial) {
		    if (y1 == y2) {
			if (x1 > x2)
			    x1--;
			else if (x1 < x2)
			    x1++;
			else
			    continue;	/* shouldn't happen */
		    }
		    else {
			if (y1 > y2)
			    y1--;
			else if (y1 < y2)
			    y1++;
			else
			    continue;	/* shouldn't happen */
		    }
		}
		if (final) {
		    if (y1 == y2) {
			if (x1 > x2)
			    x2++;
			else if (x1 < x2)
			    x2--;
			else
			    continue;	/* shouldn't happen */
		    }
		    else {
			if (y1 > y2)
			    y2++;
			else if (y1 < y2)
			    y2--;
			else
			    continue;	/* shouldn't happen */
		    }
		}

		/* Compare against endpoints of all other routes */
		brki = -1;
		for (i = 0; i < numroutes; i++) {
		    if (eptinfo[i].route == rt) continue;
		    if (eptinfo[i].endl == -2) continue;

		    testroute = eptinfo[i].orig;
		    if ((!(testroute->flags & RT_START_NODE)) &&
				(testroute->start.route == eptinfo[j].orig)) {
			/* Nothing */
		    }
		    else if ((!(testroute->flags & RT_END_NODE)) &&
				(testroute->end.route == eptinfo[j].orig)) {
			/* Nothing */
		    }
		    else
			continue;	/* Not a connected route */

		    /* Check for start/end points connecting on same layer */
		    startx = eptinfo[i].startx;
		    starty = eptinfo[i].starty;
		    startl = eptinfo[i].startl;
		    starttype = eptinfo[i].starttype;
		    endx = eptinfo[i].endx;
		    endy = eptinfo[i].endy;
		    endl = eptinfo[i].endl;
		    endtype = eptinfo[i].endtype;

		    /* Check various combinations of wire and via layers */
		    if (seg->segtype & ST_WIRE) {
			if (starttype & ST_WIRE)
			    startcompat = (startl == seg->layer);
			else
			    startcompat = (startl == seg->layer)
					|| (startl + 1 == seg->layer);

			if (endtype & ST_WIRE)
			    endcompat = (endl == seg->layer);
			else
			    endcompat = (endl == seg->layer)
					|| (endl + 1 == seg->layer);
		    }
		    else {
			if (starttype & ST_WIRE)
			    startcompat = (startl == seg->layer)
					|| (startl == seg->layer + 1);
			else
			    startcompat = (startl == seg->layer)
					|| (startl == seg->layer + 1)
					|| (startl + 1 == seg->layer);

			if (endtype & ST_WIRE)
			    endcompat = (endl == seg->layer)
					|| (endl == seg->layer + 1);
			else
			    endcompat = (endl == seg->layer)
					|| (endl == seg->layer + 1)
					|| (endl + 1 == seg->layer);
		    }

		    if (x1 == x2) {
			if (startcompat && (startx == x1)) {
			    if (y1 > y2) {
				if (starty >= y2 &&
					starty <= y1) {
				    brkx = startx;
				    brky = starty;
				    y2 = brky;
				    brki = i;
				}
			    }
			    else {
				if (starty >= y1 &&
					starty <= y2) {
				    brkx = startx;
				    brky = starty;
				    y2 = brky;
				    brki = i;
				}
			    }
			}
			if (endcompat && (endx == x2)) {
			    if (y1 > y2) {
				if (endy >= y2 &&
					endy <= y1) {
				    brkx = endx;
				    brky = endy;
				    y2 = brky;
				    brki = i;
				}
			    }
			    else {
				if (endy >= y1 &&
					endy <= y2) {
				    brkx = endx;
				    brky = endy;
				    y2 = brky;
				    brki = i;
				}
			    }
			}
		    }
		    else if (y1 == y2) {
			if (startcompat && (starty == y1)) {
			    if (x1 > x2) {
				if (startx >= x2 &&
					startx <= x1) {
				    brkx = startx;
				    brky = starty;
				    x2 = brkx;
				    brki = i;
				}
			    }
			    else {
				if (startx >= x1 &&
					startx <= x2) {
				    brkx = startx;
				    brky = starty;
				    x2 = brkx;
				    brki = i;
				}
			    }
			}
			if (endcompat && (endy == y2)) {
			    if (x1 > x2) {
				if (endx >= x2 &&
					endx <= x1) {
				    brkx = endx;
				    brky = endy;
				    x2 = brkx;
				    brki = i;
				}
			    }
			    else {
				if (endx >= x1 &&
					endx <= x2) {
				    brkx = endx;
				    brky = endy;
				    x2 = brkx;
				    brki = i;
				}
			    }
			}
		    }
		}
		if ((brki >= 0) && (eptinfo[brki].endl != -2)) {
		    /* Disable this endpoint so it is not checked again */
		    eptinfo[brki].endl = -2;

		    /* Break route at this point */
		    /* Make a copy of the segment where the break occurs */
		    newroute = (ROUTE)malloc(sizeof(struct route_));
		    newseg = (SEG)malloc(sizeof(struct seg_));
		    newseg->segtype = seg->segtype;
		    newseg->x1 = brkx;
		    newseg->y1 = brky;
		    newseg->x2 = seg->x2;
		    newseg->y2 = seg->y2;
		    newseg->layer = seg->layer;

		    newseg->next = seg->next;
		    seg->next = NULL;
		    seg->x2 = brkx;
		    seg->y2 = brky;

		    newroute->segments = newseg;
		    newroute->netnum = rt->netnum;
		    newroute->flags = (u_char)0;
		    newroute->next = rt->next;
		    rt->next = newroute;

		    newroute->start.route = NULL;
		    newroute->end.route = NULL;

		    /* Update eptinfo[j].route to point to new route */
		    eptinfo[j].route = newroute;

		    /* Next loop ends list of segs and moves to next route */
		    /* which is still the same original route, so adjust j */
		    /* index so that it still refers to the correct	   */
		    /* eptinfo entry.					   */
		    j--;
		}
	    }
	    j++;
	}

	/* Regenerate endpoint information */
	free(eptinfo);
	numroutes = 0;
	for (rt = droutes; rt; rt = rt->next) numroutes++;
	eptinfo = (endpointinfo *)malloc(numroutes * sizeof(endpointinfo));

	/* Determine the driver and fill in endpoint information */
	nroute = 0;
	drivergate = NULL;
	drivernodeidx = -1;
	driveridx = -1;
	for (rt = droutes; rt; rt = rt->next) {
	    eptinfo[nroute].route = rt;
	    eptinfo[nroute].flags = (u_char)0;
	    /* Segment start */
	    seg = rt->segments;
	    if (seg == NULL) {
		eptinfo[nroute].route = NULL;
		eptinfo[nroute].startnode = NULL;
		eptinfo[nroute].endnode = NULL;
		eptinfo[nroute].branching = NULL;
		nroute++;
		continue;
	    }
	    eptinfo[nroute].startx = seg->x1;
	    eptinfo[nroute].starty = seg->y1;
	    eptinfo[nroute].startl = seg->layer;
	    eptinfo[nroute].starttype = seg->segtype;
	    nodeptr = (seg->layer < Pinlayers) ?
			NODEIPTR(seg->x1, seg->y1, seg->layer) : NULL;
	    eptinfo[nroute].startnode = nodeptr ? nodeptr->nodesav : NULL;
	    /* In a 3D grid there can be at most 5 downstream branches	*/
	    /* from a single point.					*/
	    eptinfo[nroute].branching = (int *)malloc(5 * sizeof(int));
	    eptinfo[nroute].branching[0] = -1;
	    eptinfo[nroute].res = 0.0;
	    eptinfo[nroute].cap = 0.0;

	    /* Look up node */
	    if (nodeptr) {
		g = FindGateNode(&NodeTable, nodeptr->nodesav, &i);
		if (g && (g->gatetype->direction[i] == PORT_CLASS_OUTPUT)) {
		    drivernodeidx = i;
		    driveridx = nroute;
		    drivergate = g;
		    driverend = 0;
		}
		else if (g && (g->gatetype->direction[i] != PORT_CLASS_INPUT)) {
		    if (drivernodeidx == -1) {
			drivernodeidx = i;
			driveridx = nroute;
			drivergate = g;
			driverend = 0;
		    }
		}
		else if (g == NULL) {
		    /* should not happen? */
		    if (nodeptr->nodesav->netname == NULL)
			Fprintf(stderr, "Cannot find recorded node of netnum %d\n",
				nodeptr->nodesav->netnum);
		    else
			Fprintf(stderr, "Cannot find recorded node of net %s\n",
				nodeptr->nodesav->netname);
		}
	    }

	    /* Segment end */
	    lastseg = NULL;
	    for (seg = rt->segments; seg && seg->next; seg = seg->next)
		lastseg = seg;
	    eptinfo[nroute].endx = seg->x2;
	    eptinfo[nroute].endy = seg->y2;
	    eptinfo[nroute].endl = seg->layer;
	    eptinfo[nroute].endtype = seg->segtype;
	    nodeptr = (seg->layer < Pinlayers) ?
			NODEIPTR(seg->x2, seg->y2, seg->layer) : NULL;
	    eptinfo[nroute].endnode = nodeptr ? nodeptr->nodesav : NULL;

	    /* Look up node */
	    if (nodeptr) {
		g = FindGateNode(&NodeTable, nodeptr->nodesav, &i);
		if (g && (g->gatetype->direction[i] == PORT_CLASS_OUTPUT)) {
		    drivernodeidx = i;
		    driveridx = nroute;
		    drivergate = g;
		    driverend = 1;
		}
		else if (g && (g->gatetype->direction[i] != PORT_CLASS_INPUT)) {
		    if (drivernodeidx == -1) {
			drivernodeidx = i;
			driveridx = nroute;
			drivergate = g;
			driverend = 1;
		    }
		}
		else if (g == NULL) {
		    /* should not happen? */
		    if (nodeptr->nodesav->netname == NULL)
			Fprintf(stderr, "Cannot find recorded node of netnum %d\n",
				nodeptr->nodesav->netnum);
		    else
			Fprintf(stderr, "Cannot find recorded node of net %s\n",
				nodeptr->nodesav->netname);
		}
	    }
	    nroute++;
	}

	/* Start with net driver node, start generating output */

	if ((drivernodeidx != -1) && (driveridx != -1)) {

	    eptinfo[driveridx].flags |= EPT_DRIVER;

	    /* Diagnostic, for debugging */
	    /*
	    Fprintf(stdout, "Walking net %s.\n", net->netname);
	    Fprintf(stdout, "Has %d nodes.\n", net->numnodes);
	    Fprintf(stdout, "After segmenting, has %d routes.\n", numroutes);
	    Fprintf(stdout, "Driver node %s/%s\n",
			drivergate->gatename,
			drivergate->gatetype->node[drivernodeidx]);
	    */
	    if (!strcmp(drivergate->gatetype->node[drivernodeidx], "pin"))
		fprintf(delayFile, " PIN/%s %d ",
			drivergate->gatename, net->numnodes - 1);
	    else
		fprintf(delayFile, " %s/%s %d ",
			drivergate->gatename,
			drivergate->gatetype->node[drivernodeidx],
			net->numnodes - 1);

	    /* Walk the route and organize from driver to terminals and */
	    /* accumulate resistance and capacitance of each segment	*/

	    walk_route(driveridx, driverend, eptinfo, nroute, lefrcvalues);

	    /* Diagnostic:  There should be no unhandled segments if	*/
	    /* everything went right.					*/

	    for (i = 0; i < numroutes; i++) {
		if ((eptinfo[i].flags & EPT_VISITED) == (u_char)0) {
		    Fprintf(stderr, "Route segment %d was not walked!\n", i);
		}
	    }

	    walk_route_output(eptinfo, driveridx, &NodeTable, delayFile);
	    fprintf(delayFile, "\n");	/* End of net output */
	}
	else {
	    if (net->netname == NULL)
		Fprintf(stderr, "No driver for netnum %d\n", net->netnum);
	    else
		Fprintf(stderr, "No driver for net %s\n", net->netname);
	}

	/* Free up allocated information */

	for (rt = droutes; rt; ) {
	    for (seg = rt->segments; seg; ) {
		nxseg = seg->next;
		free(seg);
		seg = nxseg;
	    }
	    nxroute = rt->next;
	    free(rt);
	    rt = nxroute;
	}
	for (i = 0; i < nroute; i++)
	    if (eptinfo[i].branching != NULL)
		free(eptinfo[i].branching);
	free(eptinfo);
    }
    fclose(delayFile);

    free(lefrcvalues);

    Tcl_DeleteHashTable(&NodeTable);

    return 0;
}

#endif	/* TCL_QROUTER */

/* end of delays.c */
