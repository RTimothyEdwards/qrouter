/*--------------------------------------------------------------*/
/*  qrouter.c -- general purpose autorouter                     */
/*  Reads LEF libraries and DEF netlists, and generates an	*/
/*  annotated DEF netlist as output.				*/
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

int  TotalRoutes = 0;

NET     *Nlnets;	// list of nets in the design
NET	CurNet;		// current net to route, used by 2nd stage
STRING  DontRoute;      // a list of nets not to route (e.g., power)
STRING  CriticalNet;    // list of critical nets to route first
GATE    GateInfo;       // standard cell macro information
GATE	PinMacro;	// macro definition for a pin
GATE    Nlgates;	// gate instance information
NETLIST FailedNets;	// list of nets that failed to route

u_int    *Obs[MAX_LAYERS];      // net obstructions in layer
PROUTE   *Obs2[MAX_LAYERS];     // used for pt->pt routes on layer
float    *Obsinfo[MAX_LAYERS];  // temporary array used for detailed obstruction info
NODEINFO *Nodeinfo[MAX_LAYERS]; // nodes and stub information is here. . .
DSEG      UserObs;		// user-defined obstruction layers

u_int     progress[3];		// analysis of behavior

u_char needblock[MAX_LAYERS];

char *vddnet = NULL;
char *gndnet = NULL;
char *antenna_cell = NULL;

int    Numnets = 0;
int    Pinlayers = 0;
u_int  minEffort = 0;	// Minimum effort applied from command line.
u_char Verbose = 3;	// Default verbose level
u_char forceRoutable = FALSE;
u_char maskMode = MASK_AUTO;
u_char mapType = MAP_OBSTRUCT | DRAW_ROUTES;
u_char ripLimit = 10;	// Fail net rather than rip up more than
			// this number of other nets.

char *DEFfilename = NULL;
char *delayfilename = NULL;

ScaleRec Scales;	// record of input and output scales

/*--------------------------------------------------------------*/
/* Check track pitch and set the number of channels (may be	*/
/* called from DefRead)						*/
/*--------------------------------------------------------------*/

int set_num_channels(void)
{
    int i, glimitx, glimity;
    NET net;
    NODE node;
    DPOINT ctap, ltap, ntap;

    if (NumChannelsX != 0) return 0;	/* Already been called */

    if (PitchX == 0.0) {
	Fprintf(stderr, "Have a 0 pitch for X direction.  Exit.\n");
	return (-3);
    }
    else if (PitchY == 0.0) {
	Fprintf(stderr, "Have a 0 pitch for Y direction.  Exit.\n");
	return (-3);
    }

    NumChannelsX = (int)(1.5 + (Xupperbound - Xlowerbound) / PitchX);
    NumChannelsY = (int)(1.5 + (Yupperbound - Ylowerbound) / PitchY);
    if ((Verbose > 1) || (NumChannelsX <= 0))
	Fprintf(stdout, "Number of x channels is %d\n", NumChannelsX);
    if ((Verbose > 1) || (NumChannelsY <= 0))
	Fprintf(stdout, "Number of y channels is %d\n", NumChannelsY);
	
    if (NumChannelsX <= 0) {
	Fprintf(stderr, "Something wrong with x bounds.\n");
	return(-3);
    }
    if (NumChannelsY <= 0) {
	Fprintf(stderr, "Something wrong with y bounds.\n");
	return(-3);
    }
    Flush(stdout);

    // Go through all nodes and remove any tap or extend entries that are
    // out of bounds.

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	for (node = net->netnodes; node != NULL; node = node->next) {

	    ltap = NULL;
	    for (ctap = node->taps; ctap != NULL; ) {
		ntap = ctap->next;
		glimitx = NumChannelsX;
		glimity = NumChannelsY;
		if (ctap->gridx < 0 || ctap->gridx >= glimitx ||
				ctap->gridy < 0 || ctap->gridy >= glimity) {
		    /* Remove ctap */
		    if (ltap == NULL)
			node->taps = ntap;
		    else
			ltap->next = ntap;
		}
		else
		    ltap = ctap;
		ctap = ntap;
	    }

	    ltap = NULL;
	    for (ctap = node->extend; ctap != NULL; ) {
		ntap = ctap->next;
		glimitx = NumChannelsX;
		glimity = NumChannelsY;
		if (ctap->gridx < 0 || ctap->gridx >= glimitx ||
				ctap->gridy < 0 || ctap->gridy >= glimity) {
		    /* Remove ctap */
		    if (ltap == NULL)
			node->taps = ntap;
		    else
			ltap->next = ntap;
		}
		else
		    ltap = ctap;
		ctap = ntap;
	    }
	}
    }

    if (recalc_spacing()) draw_layout();
    return 0;
}

/*--------------------------------------------------------------*/
/* Allocate the Obs[] array (may be called from DefRead)	*/
/*--------------------------------------------------------------*/

int allocate_obs_array(void)
{
   int i;

   if (Obs[0] != NULL) return 0;	/* Already been called */

   for (i = 0; i < Num_layers; i++) {
      Obs[i] = (u_int *)calloc(NumChannelsX * NumChannelsY,
			sizeof(u_int));
      if (!Obs[i]) {
	 Fprintf(stderr, "Out of memory 4.\n");
	 return(4);
      }
   }
   return 0;
}

/*--------------------------------------------------------------*/
/* countlist ---						*/
/*   Count the number of entries in a simple linked list	*/
/*--------------------------------------------------------------*/

int
countlist(NETLIST net)
{
   NETLIST nptr = net;
   int count = 0;

   while (nptr != NULL) {
      count++;
      nptr = nptr->next;
   }
   return count;
}

/*--------------------------------------------------------------*/
/* runqrouter - main program entry point, parse command line	*/
/*								*/
/*   ARGS: argc (count) argv, command line 			*/
/*   RETURNS: to OS						*/
/*   SIDE EFFECTS: 						*/
/*--------------------------------------------------------------*/

int
runqrouter(int argc, char *argv[])
{
   int	i;
   FILE *configFILEptr, *infoFILEptr;
   static char configdefault[] = CONFIGFILENAME;
   char *configfile = configdefault;
   char *infofile = NULL;
   char *dotptr;
   char *Filename = NULL;
   u_char readconfig = FALSE;
   u_char doscript = FALSE;
    
   Scales.iscale = 1;
   Scales.mscale = 100;

   /* Parse arguments */

   for (i = 0; i < argc; i++) {
      char optc, argsep = '\0';
      char *optarg = NULL;

      if (*argv[i] == '-') {

	 /* 1st pass---look for which options require an argument */
	 optc = *(argv[i] + 1);

	 switch (optc) {
	    case 'c':
	    case 'i':
	    case 'e':
	    case 'k':
	    case 'v':
	    case 'd':
	    case 'p':
	    case 'g':
	    case 'r':
	    case 's':
	       argsep = *(argv[i] + 2);
	       if (argsep == '\0') {
		  i++;
	          if (i < argc) {
	             optarg = argv[i];
		     if (*optarg == '-') {
		        Fprintf(stderr, "Option -%c needs an argument.\n", optc);
		        Fprintf(stderr, "Option not handled.\n");
		        continue;
		     }
	          }
	          else {
		     Fprintf(stderr, "Option -%c needs an argument.\n", optc);
		     Fprintf(stderr, "Option not handled.\n");
		     continue;
		  }
	       }
	       else
		  optarg = argv[i] + 2;
	 }

	 /* Now handle each option individually */

	 switch (optc) {
	    case 'c':
	       configfile = strdup(optarg);
	       break;
	    case 'v':
	       Verbose = atoi(optarg);
	       break;
	    case 'i':
	       infofile = strdup(optarg);
	       break;
	    case 'd':
	       if (delayfilename != NULL) free(delayfilename);
	       delayfilename = strdup(optarg);
	       break;
	    case 'p':
	       vddnet = strdup(optarg);
	       break;
	    case 'g':
	       gndnet = strdup(optarg);
	       break;
	    case 's':
	       // The "-s" argument is not handled here but is used
	       // to avoid generating a warning message.
	       doscript = TRUE;
	       break;
	    case 'r':
	       if (sscanf(optarg, "%d", &Scales.iscale) != 1) {
		   Fprintf(stderr, "Bad resolution scalefactor \"%s\", "
			"integer expected.\n", optarg);
		   Scales.iscale = 1;
	       }
	       break;
	    case 'h':
	       helpmessage();
	       return 1;
	       break;
	    case 'f':
	       forceRoutable = TRUE;
	       break;
	    case 'k':
	       Fprintf(stdout, "Option \"k\" deprecated.  Use \"effort\""
			" in stage2 or stage3 command or -e option\n");
	       minEffort = 100 * atoi(optarg);
	       break;
	    case 'e':
	       minEffort = atoi(optarg);
	       break;
	    case 'n':
	       /* Ignore '-noc' or '-nog', handled elsewhere */
	       break;
	    case '\0':
	       /* Ignore '-' */
	       break;
	    case '-':
	       /* Ignore '--' */
	       break;
	    default:
	       Fprintf(stderr, "Bad option -%c, ignoring.\n", optc);
	 }
      }
      else {
	 /* Not an option or an option argument, so treat as a filename */
	 Filename = strdup(argv[i]);
      }
   }

   if (infofile != NULL) {
      infoFILEptr = fopen(infofile, "w" );
      free(infofile);
   }
   else {
      infoFILEptr = NULL;
#ifndef TCL_QROUTER
      fprintf(stdout, "Qrouter detail maze router version %s.%s\n", VERSION, REVISION);
#endif
   }

   if (!doscript) {
      configFILEptr = fopen(configfile, "r");

      if (configFILEptr) {
          read_config(configFILEptr, (infoFILEptr == NULL) ? FALSE : TRUE);
          readconfig = TRUE;
      }
      else {
         if (configfile != configdefault)
	    Fprintf(stderr, "Could not open %s\n", configfile );
         else
	    Fprintf(stdout, "No .cfg file specified, continuing without.\n");
      }
      if (configfile != configdefault) free(configfile);
   }

   if (infoFILEptr != NULL) {

      /* Print qrouter name and version number at the top */
#ifdef TCL_QROUTER
      fprintf(infoFILEptr, "qrouter %s.%s.T\n", VERSION, REVISION);
#else
      fprintf(infoFILEptr, "qrouter %s.%s\n", VERSION, REVISION);
#endif
      /* Output database units expected by the technology LEF file */
      /* Note that this comes from MANUFACTURINGGRID, not UNITS DATABASE */
      fprintf(infoFILEptr, "units scale %d\n", Scales.mscale);

      /* Resolve base horizontal and vertical pitches. */
      post_config(TRUE);

      /* Print information about route layers, and exit */
      for (i = 0; i < Num_layers; i++) {
	 double pitch, width;
	 int vnum, hnum;
	 int o = LefGetRouteOrientation(i);
	 char *layername = LefGetRouteName(i);

	 check_variable_pitch(i, &hnum, &vnum);
	 if (vnum > 1 && hnum == 1) hnum++;	// see note in node.c
	 if (hnum > 1 && vnum == 1) vnum++;
		
	 if (layername != NULL) {
	    pitch = (o == 1) ? PitchY : PitchX,
	    width = LefGetRouteWidth(i);
	    if (pitch == 0.0 || width == 0.0) continue;
	    fprintf(infoFILEptr, "%s %g %g %g %s",
		layername, pitch,
		LefGetRouteOffset(i), width,
		(o == 1) ? "horizontal" : "vertical");
	    if (o == 1 && vnum > 1)
	       fprintf(infoFILEptr, " %d", vnum);
	    else if (o == 0 && hnum > 1)
	       fprintf(infoFILEptr, " %d", hnum);
	    fprintf(infoFILEptr, "\n");
	 }
      }

      fclose(infoFILEptr);
      return 1;
   }

   if (Filename != NULL) {

      /* process last non-option string */

      dotptr = strrchr(Filename, '.');
      if (dotptr != NULL) *dotptr = '\0';
      if (DEFfilename != NULL) free(DEFfilename);
      DEFfilename = (char *)malloc(strlen(Filename) + 5);
      sprintf(DEFfilename, "%s.def", Filename);
   }
   else if (readconfig) {
      Fprintf(stdout, "No netlist file specified, continuing without.\n");

      // Print help message but continue normally.
      helpmessage();
   }

   Obs[0] = (u_int *)NULL;
   NumChannelsX = 0;	// This is so we can check if NumChannelsX/Y were
			// set from within DefRead() due to reading in
			// existing nets.

   Scales.oscale = 1.0;
   return 0;
}

/*--------------------------------------------------------------*/
/* remove_from_failed ---					*/
/*								*/
/* Remove one net from the list of failing nets.  If "net" was	*/
/* in the list FailedNets, then return TRUE, otherwise return	*/
/* FALSE.							*/
/*--------------------------------------------------------------*/

u_char remove_from_failed(NET net)
{
    NETLIST nl, lastnl;

    lastnl = (NETLIST)NULL;
    for (nl = FailedNets; nl; nl = nl->next) {
	if (nl->net == net) {
	    if (lastnl == NULL)
		FailedNets = nl->next;
	    else
		lastnl->next = nl->next;
	    free(nl);
	    return TRUE;
	}
	lastnl = nl;
    }
    return FALSE; 
}

/*--------------------------------------------------------------*/
/* remove_failed ---						*/
/*								*/
/* Free up memory in the list of route failures.		*/
/*--------------------------------------------------------------*/

void remove_failed()
{
    NETLIST nl;
    while (FailedNets) {
	nl = FailedNets;
	FailedNets = FailedNets->next;
	free(nl);
    }
}

/*--------------------------------------------------------------*/
/* Remove the first (top) route record from a net		*/
/*--------------------------------------------------------------*/

void remove_top_route(NET net)
{
    ROUTE rt;
    SEG seg;

    rt = net->routes;
    net->routes = net->routes->next;
    while (rt->segments) {
	seg = rt->segments;
	rt->segments = rt->segments->next;
	    free(seg);
    }
    free(rt);
}

/*--------------------------------------------------------------*/
/* reinitialize ---						*/
/*								*/
/* Free up memory in preparation for reading another DEF file	*/
/*--------------------------------------------------------------*/

static void reinitialize()
{
    int i, j;
    NETLIST nl;
    NET net;
    ROUTE rt;
    SEG seg;
    DSEG obs, tap;
    NODE node;
    GATE gate;
    DPOINT dpt;

    // Free up all of the matrices

    for (i = 0; i < Pinlayers; i++) {
	for (j = 0; j < NumChannelsX * NumChannelsY; j++)
	    if (Nodeinfo[i][j])
		free(Nodeinfo[i][j]);
	free(Nodeinfo[i]);
	Nodeinfo[i] = NULL;
    }
    for (i = 0; i < Num_layers; i++) {
	free(Obs2[i]);
	free(Obs[i]);

	Obs2[i] = NULL;
	Obs[i] = NULL;
    }
    if (RMask != NULL) {
	free(RMask);
	RMask = NULL;
    }

    // Free the netlist of failed nets (if there is one)

    remove_failed();

    // Free all net and route information

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	while (net->noripup) {
	    nl = net->noripup;
	    net->noripup = net->noripup->next;
	    free(nl);
	}
	while (net->routes)
            remove_top_route(net);

	while (net->netnodes) {
	    node = net->netnodes;
	    net->netnodes = net->netnodes->next;

	    while (node->taps) {
		dpt = node->taps;
		node->taps = node->taps->next;
		free(dpt);
	    }
	    while (node->extend) {
		dpt = node->extend;
		node->extend = node->extend->next;
		free(dpt);
	    }
	    // Note: node->netname is not allocated
	    // but copied from net record
	    free(node);
	}
	free (net->netname);
	free (net);
    }
    free(Nlnets);
    Nlnets = NULL;
    Numnets = 0;

    // Free all gates information

    while (Nlgates) {
	gate = Nlgates;
	Nlgates = Nlgates->next;
	while (gate->obs) {
	    obs = gate->obs;
	    gate->obs = gate->obs->next;
	    free(obs);
	}
	for (i = 0; i < gate->nodes; i++) {
	    while (gate->taps[i]) {
	        tap = gate->taps[i];
		gate->taps[i] = gate->taps[i]->next;
		free(tap);
	    }
	    // Note: gate->node[i] is not allocated
	    // but copied from cell record in GateInfo
	    // Likewise for gate->noderec[i]
	}

	free(gate->gatename);
    }
    Nlgates = NULL;
}

/*--------------------------------------------------------------*/
/* apply_drc_blocks() ---					*/
/*								*/
/* Use via and route width and spacing information to determine	*/
/* if blockages are needed in tracks adjacent to routed		*/
/* segments to avoid causing DRC errors in the output.		*/
/*								*/
/* If layer == -1, then determine values normally for all	*/
/* route layers.  If layer >= 0, determine values for specified	*/
/* layer only.  If via_except > 0, then adjust the value for	*/
/* a DRC violating distance for vias in adjacent tracks by that	*/
/* amount (in microns).  If route_except > 0, then adjust the	*/
/* value for a DRC violating distance between a via and a route	*/
/* in adjacent tracks by that amount.				*/
/*--------------------------------------------------------------*/

void apply_drc_blocks(int layer, double via_except, double route_except)
{
   int i;
   double sreq1, sreq2, sreq2t;

   // Fill in needblock bit fields, which are used by commit_proute
   // when route layers are too large for the grid size, and grid points
   // around a route need to be marked as blocked whenever something is
   // routed on those layers.

   // "ROUTEBLOCK" is set if the spacing is violated between a normal
   // route and an adjacent via.  "VIABLOCK" is set if the spacing is
   // violated between two adjacent vias.  It may be helpful to define
   // a third category which is route-to-route spacing violation.

   // There are up to four different via types per base layer with
   // different geometries based on the permutation of rotations of
   // the top and bottom layers, so we only register blocking behavior
   // if all of the via types will generate spacing violations.

   for (i = 0; i < Num_layers; i++) {
      if ((layer >= 0) && (i != layer)) continue;

      needblock[i] = FALSE;

      sreq1 = LefGetRouteSpacing(i);
      if (i < Num_layers - 1) {
         sreq2 = LefGetXYViaWidth(i, i, 0, 0) + sreq1;
         sreq2t = LefGetXYViaWidth(i, i, 0, 1) + sreq1;
         if (sreq2t < sreq2) sreq2 = sreq2t;
         sreq2t = LefGetXYViaWidth(i, i, 0, 2) + sreq1;
         if (sreq2t < sreq2) sreq2 = sreq2t;
         sreq2t = LefGetXYViaWidth(i, i, 0, 3) + sreq1;
         if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2 -= via_except;
         if ((sreq2 - EPS) > PitchX) needblock[i] |= VIABLOCKX;
      }
      if (i != 0) {
	 sreq2 = LefGetXYViaWidth(i - 1, i, 0, 0) + sreq1;
	 sreq2t = LefGetXYViaWidth(i - 1, i, 0, 1) + sreq1;
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2t = LefGetXYViaWidth(i - 1, i, 0, 2) + sreq1;
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2t = LefGetXYViaWidth(i - 1, i, 0, 3) + sreq1;
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2 -= via_except;
         if ((sreq2 - EPS) > PitchX) needblock[i] |= VIABLOCKX;
      }

      if (i < Num_layers - 1) {
         sreq2 = LefGetXYViaWidth(i, i, 1, 0) + sreq1;
         sreq2t = LefGetXYViaWidth(i, i, 1, 1) + sreq1;
         if (sreq2t < sreq2) sreq2 = sreq2t;
         sreq2t = LefGetXYViaWidth(i, i, 1, 2) + sreq1;
         if (sreq2t < sreq2) sreq2 = sreq2t;
         sreq2t = LefGetXYViaWidth(i, i, 1, 3) + sreq1;
         if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2 -= via_except;
         if ((sreq2 - EPS) > PitchY) needblock[i] |= VIABLOCKY;
      }
      if (i != 0) {
	 sreq2 = LefGetXYViaWidth(i - 1, i, 1, 0) + sreq1;
	 sreq2t = LefGetXYViaWidth(i - 1, i, 1, 1) + sreq1;
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2t = LefGetXYViaWidth(i - 1, i, 1, 2) + sreq1;
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2t = LefGetXYViaWidth(i - 1, i, 1, 3) + sreq1;
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2 -= via_except;
         if ((sreq2 - EPS) > PitchY) needblock[i] |= VIABLOCKY;
      }

      sreq1 += 0.5 * LefGetRouteWidth(i);

      if (i < Num_layers - 1) {
         sreq2 = sreq1 + 0.5 * LefGetXYViaWidth(i, i, 0, 0);
         sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i, i, 0, 1);
         if (sreq2t < sreq2) sreq2 = sreq2t;
         sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i, i, 0, 2);
         if (sreq2t < sreq2) sreq2 = sreq2t;
         sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i, i, 0, 3);
         if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2 -= route_except;
         if ((sreq2 - EPS) > PitchX) needblock[i] |= ROUTEBLOCKX;
      }
      if (i != 0) {
	 sreq2 = sreq1 + 0.5 * LefGetXYViaWidth(i - 1, i, 0, 0);
	 sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i - 1, i, 0, 1);
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i - 1, i, 0, 2);
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i - 1, i, 0, 3);
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2 -= route_except;
         if ((sreq2 - EPS) > PitchX) needblock[i] |= ROUTEBLOCKX;
      }

      if (i < Num_layers - 1) {
         sreq2 = sreq1 + 0.5 * LefGetXYViaWidth(i, i, 1, 0);
         sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i, i, 1, 1);
         if (sreq2t < sreq2) sreq2 = sreq2t;
         sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i, i, 1, 2);
         if (sreq2t < sreq2) sreq2 = sreq2t;
         sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i, i, 1, 3);
         if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2 -= route_except;
         if ((sreq2 - EPS) > PitchY) needblock[i] |= ROUTEBLOCKY;
      }
      if (i != 0) {
	 sreq2 = sreq1 + 0.5 * LefGetXYViaWidth(i - 1, i, 1, 0);
	 sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i - 1, i, 1, 1);
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i - 1, i, 1, 2);
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2t = sreq1 + 0.5 * LefGetXYViaWidth(i - 1, i, 1, 3);
	 if (sreq2t < sreq2) sreq2 = sreq2t;
	 sreq2 -= route_except;
         if ((sreq2 - EPS) > PitchY) needblock[i] |= ROUTEBLOCKY;
      }
   }
}

/*--------------------------------------------------------------*/
/* remove_tap_blocks						*/
/*								*/
/* Qrouter avoids routing directly over a tap point, blocking	*/
/* it, if there is a Nodeinfo[][]->nodeloc entry present.	*/
/* Remove this entry to remove the blockage (it can be		*/
/* replaced if needed by copying back the Nodeinfo[][]->nodesav	*/
/* pointer)							*/
/*--------------------------------------------------------------*/

void
remove_tap_blocks(int netnum)
{
    int i, j;
    NODE node;

    for (i = 0; i < Pinlayers; i++) {
	for (j = 0; j < NumChannelsX * NumChannelsY; j++) {
	    if (Nodeinfo[i][j]) {
		node = Nodeinfo[i][j]->nodeloc;
		if (node != (NODE)NULL)
		    if (node->netnum == netnum)
			Nodeinfo[i][j]->nodeloc = (NODE)NULL;
	    }
        }
    }
}

/*--------------------------------------------------------------*/
/* post_def_setup ---						*/
/*								*/
/* Things to do after a DEF file has been read in, and the size	*/
/* of the layout, components, and nets are known.		*/
/*--------------------------------------------------------------*/

static int post_def_setup()
{
   NET net;
   ROUTE rt;
   int i;

   if (DEFfilename == NULL) {
      Fprintf(stderr, "No DEF file read, nothing to set up.\n");
      return 1;
   }
   else {
      if (Num_layers <= 0) {
         Fprintf(stderr, "No routing layers defined, nothing to do.\n");
         return 1;
      }
   }

   for (i = 0; i < Numnets; i++) {
      net = Nlnets[i];
      find_bounding_box(net);
      defineRouteTree(net);
   }

   create_netorder(0);		// Choose ordering method (0 or 1)

   set_num_channels();		// If not called from DefRead()
   allocate_obs_array();	// If not called from DefRead()

   initMask();

   for (i = 0; i < Num_layers; i++) {

      Obsinfo[i] = (float *)calloc(NumChannelsX * NumChannelsY,
			sizeof(float));
      if (!Obsinfo[i]) {
	 fprintf(stderr, "Out of memory 5.\n");
	 exit(5);
      }

      Nodeinfo[i] = (NODEINFO *)calloc(NumChannelsX * NumChannelsY,
			sizeof(NODEINFO));
      if (!Nodeinfo[i]) {
	 fprintf( stderr, "Out of memory 6.\n");
	 exit(6);
      }
   }
   Flush(stdout);

   if (Verbose > 1)
      Fprintf(stderr, "Diagnostic: memory block is %d bytes\n",
		(int)sizeof(u_int) * NumChannelsX * NumChannelsY);

   /* Be sure to create obstructions from gates first, since we don't	*/
   /* want improperly defined or positioned obstruction layers to over-	*/
   /* write our node list.						*/

#ifdef TCL_QROUTER
   find_free_antenna_taps(antenna_cell);
#endif
   expand_tap_geometry();
   clip_gate_taps();
   create_obstructions_from_gates();
   create_obstructions_inside_nodes();
   create_obstructions_outside_nodes();
   tap_to_tap_interactions();
   create_obstructions_from_variable_pitch();
   adjust_stub_lengths();
   find_route_blocks();
   count_reachable_taps();
   count_pinlayers();
   
   // If any nets are pre-routed, calculate route endpoints, and
   // place those routes.

   for (i = 0; i < Numnets; i++) {
      net = Nlnets[i];
      for (rt = net->routes; rt; rt = rt->next)
	 route_set_connections(net, rt);
      writeback_all_routes(net);
   }

   // Remove the Obsinfo array, which is no longer needed, and allocate
   // the Obs2 array for costing information

   for (i = 0; i < Num_layers; i++) free(Obsinfo[i]);

   for (i = 0; i < Num_layers; i++) {
      Obs2[i] = (PROUTE *)calloc(NumChannelsX * NumChannelsY,
			sizeof(PROUTE));
      if (!Obs2[i]) {
         fprintf( stderr, "Out of memory 9.\n");
         exit(9);
      }
   }

   // Remove tap blocks from power, ground, and antenna nets, as these
   // can take up large areas of the layout and will cause serious issues
   // with routability if left blocked.

   remove_tap_blocks(VDD_NET);
   remove_tap_blocks(GND_NET);
   remove_tap_blocks(ANTENNA_NET);

   // Now we have netlist data, and can use it to get a list of nets.

   FailedNets = (NETLIST)NULL;
   Flush(stdout);
   if (Verbose > 0)
      Fprintf(stdout, "There are %d nets in this design.\n", Numnets);

   return 0;
}

/*--------------------------------------------------------------*/
/* read_def ---							*/
/*								*/
/* Read in the DEF file in DEFfilename				*/
/* Return 0 on success, 1 on fatal error in DEF file.		*/
/*--------------------------------------------------------------*/

int read_def(char *filename)
{
   float oscale;
   double precis;
   int result;

   if ((filename == NULL) && (DEFfilename == NULL)) {
      Fprintf(stderr, "No DEF file specified, nothing to read.\n");
      return 1;
   }
   else if (filename != NULL) {
      if (DEFfilename != NULL) {
	  reinitialize();
	  free(DEFfilename);
      }
      DEFfilename = strdup(filename);
   }
   else reinitialize();

   oscale = (float)0.0;
   result = DefRead(DEFfilename, &oscale);
   precis = Scales.mscale / (double)oscale;	// from LEF manufacturing grid
   if (precis < 1.0) precis = 1.0;
   precis *= (double)Scales.iscale;	// user-defined extra scaling

   Scales.iscale = (int)(precis + 0.5);
   Scales.oscale = (double)(Scales.iscale * oscale);

   if (Verbose > 0)
      Fprintf(stdout, "Output scale = microns / %g, precision %g\n",
		Scales.oscale / (double)Scales.iscale,
		1.0 / (double)Scales.iscale);

   post_def_setup();
   return result;
}

/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/

int dofirststage(u_char graphdebug, int debug_netnum)
{
   int i, failcount, remaining, result;
   NET net;
   NETLIST nl;

   // Clear the lists of failed routes, in case first
   // stage is being called more than once.

   if (debug_netnum <= 0) remove_failed();

   // Now find and route all the nets

   remaining = Numnets;
 
   for (i = (debug_netnum >= 0) ? debug_netnum : 0; i < Numnets; i++) {

      net = getnettoroute(i);
      if ((net != NULL) && (net->netnodes != NULL)) {
	 result = doroute(net, FALSE, graphdebug);
	 if (result == 0) {
	    remaining--;
	    if (Verbose > 0)
	       Fprintf(stdout, "Finished routing net %s\n", net->netname);
	    Fprintf(stdout, "Nets remaining: %d\n", remaining);
	    Flush(stdout);
	 }
	 else {
	    if (Verbose > 0)
	       Fprintf(stdout, "Failed to route net %s\n", net->netname);
	 }
      }
      else {
	 if (net && (Verbose > 0)) {
	    Fprintf(stdout, "Nothing to do for net %s\n", net->netname);
	 }
	 remaining--;
      }
      if (debug_netnum >= 0) break;
   }
   failcount = countlist(FailedNets);
   if (debug_netnum >= 0) return failcount;

   if (Verbose > 0) {
      Flush(stdout);
      Fprintf(stdout, "\n----------------------------------------------\n");
      Fprintf(stdout, "Progress: ");
      Fprintf(stdout, "Stage 1 total routes completed: %d\n", TotalRoutes);
   }
   if (FailedNets == (NETLIST)NULL)
      Fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL)
          Fprintf(stdout, "Failed net routes: %d\n", failcount);
   }
   if (Verbose > 0)
      Fprintf(stdout, "----------------------------------------------\n");

   return failcount;
}

/*--------------------------------------------------------------*/
/* getnettoroute - get a net to route				*/
/*								*/
/*   ARGS: 							*/
/*   RETURNS: 							*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

NET getnettoroute(int order)
{
   NET net;

   net = Nlnets[order]; 
   if (net == NULL) return NULL;
  
   if (net->flags & NET_IGNORED) return NULL;
   if (net->numnodes >= 2) return net;

   // Qrouter will route power and ground nets even if the
   // standard cell power and ground pins are not listed in
   // the nets section.  Because of this, it is okay to have
   // only one node.

   if ((net->numnodes == 1) && (net->netnum == VDD_NET ||
		net->netnum == GND_NET || net->netnum == ANTENNA_NET))
      return net;

   if (Verbose > 3) {
      Flush(stdout);
      Fprintf(stderr, "getnettoroute():  Fell through\n");
   }
   return NULL;

} /* getnettoroute() */

/*--------------------------------------------------------------*/
/* Find all routes that collide with net "net", remove them	*/
/* from the Obs[] matrix, append them to the FailedNets list,	*/
/* and then write the net "net" back to the Obs[] matrix.	*/
/*								*/
/* Return the number of nets ripped up				*/
/*--------------------------------------------------------------*/

static int ripup_colliding(NET net, u_char onlybreak)
{
    NETLIST nl, nl2, fn;
    int ripped;

    // Analyze route for nets with which it collides

    nl = find_colliding(net, &ripped);

    // "ripLimit" limits the number of collisions so that the
    // router avoids ripping up huge numbers of nets, which can
    // cause the number of failed nets to keep increasing.

    if (ripped > ripLimit) {
	while (nl) {
	    nl2 = nl->next;
	    free(nl);
	    nl = nl2;
	}
	return -1;
    }

    // Remove the colliding nets from the route grid and append
    // them to FailedNets.

    ripped = 0;
    while(nl) {
	ripped++;
	nl2 = nl->next;
	if (Verbose > 0)
            Fprintf(stdout, "Ripping up blocking net %s\n", nl->net->netname);
	if (ripup_net(nl->net, TRUE, onlybreak, FALSE) == TRUE) { 
	    for (fn = FailedNets; fn && fn->next != NULL; fn = fn->next);
	    if (fn)
		fn->next = nl;
	    else
		FailedNets = nl;

	    // Add nl->net to "noripup" list for this net, so it won't be
	    // routed over again by the net.  Avoids infinite looping in
	    // the second stage.

	    fn = (NETLIST)malloc(sizeof(struct netlist_));
	    fn->next = net->noripup;
	    net->noripup = fn;
	    fn->net = nl->net;
	}

	nl->next = (NETLIST)NULL;
	nl = nl2;
     }
     return ripped;
}

/*--------------------------------------------------------------*/
/* Do a second-stage route (rip-up and re-route) of a single	*/
/* net "net".							*/
/*--------------------------------------------------------------*/

int route_net_ripup(NET net, u_char graphdebug, u_char onlybreak)
{
    int result;
    NETLIST nl, nl2;

    // Find the net in the Failed list and remove it.
    if (FailedNets) {
	if (FailedNets->net == net) {
	    nl2 = FailedNets;
	    FailedNets = FailedNets->next;
	    free(nl2);
	}
	else {
	    for (nl = FailedNets; nl->next; nl = nl->next) {
		if (nl->next->net == net)
		    break;
	    }
	    nl2 = nl->next;
	    nl->next = nl2->next;
	    free(nl2);
	}
    }

    result = doroute(net, TRUE, graphdebug);
    if (result != 0) {
	if (net->noripup != NULL) {
	    if ((net->flags & NET_PENDING) == 0) {
		// Clear this net's "noripup" list and try again.

		while (net->noripup) {
		    nl = net->noripup->next;
		    free(net->noripup);
		    net->noripup = nl;
		}
		result = doroute(net, TRUE, graphdebug);
		net->flags |= NET_PENDING;	// Next time we abandon it.
	    }
	}
    }
    if (result != 0)
	result = ripup_colliding(net, onlybreak);

    return result;
}

/*--------------------------------------------------------------*/
/* dosecondstage() ---						*/
/*								*/
/* Second stage:  Rip-up and reroute failing nets.		*/
/* Method:							*/
/* 1) Route a failing net with stage = 1 (other nets become	*/
/*    costs, not blockages, no copying to Obs)			*/
/* 2) If net continues to fail, flag it as unroutable and	*/
/*    remove it from the list.					*/
/* 3) Otherwise, determine the nets with which it collided.	*/
/* 4) Remove all of the colliding nets, and add them to the	*/
/*    FailedNets list						*/
/* 5) Route the original failing net.				*/
/* 6) Continue until all failed nets have been processed.	*/
/*								*/
/* Return value:  The number of failing nets			*/
/*--------------------------------------------------------------*/

int
dosecondstage(u_char graphdebug, u_char singlestep, u_char onlybreak, u_int effort)
{
   int failcount, result, i;
   NET net;
   NETLIST nl, nl2;
   NETLIST Abandoned;	// Abandoned routes---not even trying any more.
   ROUTE rt, rt2;
   SEG seg;
   u_int loceffort = (effort > minEffort) ? effort : minEffort;

   fillMask((u_char)0);
   Abandoned = NULL;
   for (i = 0; i < 3; i++) progress[i] = 0;
   
   // Clear the "noripup" field from all of the failed nets, in case
   // the second stage route is being repeated.

   for (nl2 = FailedNets; nl2; nl2 = nl2->next) {
       net = nl2->net;
       while (net->noripup) {
          nl = net->noripup->next;
          free(net->noripup);
          net->noripup = nl;
       }
       net->flags &= ~NET_PENDING;
   }

   while (FailedNets != NULL) {

      // Diagnostic:  how are we doing?
      failcount = countlist(FailedNets);
      if (Verbose > 1) Fprintf(stdout, "------------------------------\n");
      Fprintf(stdout, "Nets remaining: %d\n", failcount);
      if (Verbose > 1) Fprintf(stdout, "------------------------------\n");

      net = FailedNets->net;

      // Remove this net from the fail list
      nl2 = FailedNets;
      FailedNets = FailedNets->next;
      free(nl2);

      // Keep track of which routes existed before the call to doroute().
      for (rt = net->routes; rt && rt->next; rt = rt->next);

      if (Verbose > 2)
	 Fprintf(stdout, "Routing net %s with collisions\n", net->netname);
      Flush(stdout);

      result = doroute(net, TRUE, graphdebug);

      if (result != 0) {
	 if (net->noripup != NULL) {
	    if ((net->flags & NET_PENDING) == 0) {
	       // Clear this net's "noripup" list and try again.

	       while (net->noripup) {
	          nl = net->noripup->next;
	          free(net->noripup);
	          net->noripup = nl;
	       }
	       result = doroute(net, TRUE, graphdebug);
	       net->flags |= NET_PENDING;	// Next time we abandon it.
	    }
	 }
      }

      if (result == 0) {

         // Find nets that collide with "net" and remove them, adding them
         // to the end of the FailedNets list.

	 // If the number of nets to be ripped up exceeds "ripLimit",
	 // then treat this as a route failure, and don't rip up any of
	 // the colliding nets.

	 result = ripup_colliding(net, onlybreak);
	 if (result > 0) result = 0;
      }

      if (result != 0) {

	 // Complete failure to route, even allowing collisions.
	 // Abandon routing this net.

	 if (Verbose > 0)
	    Fprintf(stdout, "Failure on net %s:  Abandoning for now.\n",
			net->netname);

	 // Add the net to the "abandoned" list
	 nl = (NETLIST)malloc(sizeof(struct netlist_));
	 nl->net = net;
	 nl->next = Abandoned;
	 Abandoned = nl;

	 while (FailedNets && (FailedNets->net == net)) {
	    nl = FailedNets->next;
	    free(FailedNets);
	    FailedNets = nl;
	 }

	 // Remove routing information for all new routes that have
	 // not been copied back into Obs[].
	 if (rt == NULL) {
	    rt = net->routes;
	    net->routes = NULL;		// remove defunct pointer
	 }
	 else {
	    rt2 = rt->next;
	    rt->next = NULL;
	    rt = rt2;
	 }
	 while (rt != NULL) {
	    rt2 = rt->next;
            while (rt->segments) {
              seg = rt->segments->next;
              free(rt->segments);
              rt->segments = seg;
            }
	    free(rt);
	    rt = rt2;
	 }

	 // Remove both routing information and remove the route from
	 // Obs[] for all parts of the net that were previously routed

	 ripup_net(net, TRUE, FALSE, FALSE);	// Remove routing information from net
	 continue;
      }

      // Write back the original route to the grid array
      writeback_all_routes(net);

      // Evaluate progress by counting the total number of remaining
      // routes in the last (effort) cycles.  progress[2]->progress[1]
      // is a progression from oldest to newest number of remaining
      // routes.  Calculate the slope of this line and declare an end
      // to this 2nd stage route if the slope falls to zero.

      progress[1] += failcount;
      progress[0]++;
      if (progress[0] > loceffort) {
	 if ((progress[2] > 0) && (progress[2] <= progress[1])) {
	    Fprintf(stderr, "\nNo progress at level of effort %d;"
			" ending 2nd stage.\n", loceffort);
	    break;
	 }
	 progress[2] = progress[1];
	 progress[1] = progress[0] = 0;
      }
      if (singlestep && (FailedNets != NULL)) return countlist(FailedNets);
   }

   // If the list of abandoned nets is non-null, attach it to the
   // end of the failed nets list.

   if (Abandoned != NULL) {
      if (FailedNets == NULL) {
	 FailedNets = Abandoned;
	 Abandoned = NULL;
      }
      else {
	 for (nl = FailedNets; nl->next; nl = nl->next);
	 nl->next = Abandoned;
	 Abandoned = NULL;
      }
   }

   if (Verbose > 0) {
      Flush(stdout);
      Fprintf(stdout, "\n----------------------------------------------\n");
      Fprintf(stdout, "Progress: ");
      Fprintf(stdout, "Stage 2 total routes completed: %d\n", TotalRoutes);
   }
   if (FailedNets == (NETLIST)NULL) {
      failcount = 0;
      Fprintf(stdout, "No failed routes!\n");
   }
   else {
      failcount = countlist(FailedNets);
      if (FailedNets != (NETLIST)NULL)
          Fprintf(stdout, "Failed net routes: %d\n", failcount);
   }
   if (Verbose > 0)
      Fprintf(stdout, "----------------------------------------------\n");

   return failcount;
}

/*--------------------------------------------------------------*/
/* 3rd stage routing (cleanup).  Rip up each net in turn and	*/
/* reroute it.  With all of the crossover costs gone, routes	*/
/* should be much better than the 1st stage.  Any route that	*/
/* existed before it got ripped up should by definition be	*/
/* routable.							*/
/*--------------------------------------------------------------*/

int dothirdstage(u_char graphdebug, int debug_netnum, u_int effort)
{
   int i, failcount, remaining, result, maskSave;
   u_char failed;
   NET net;
   ROUTE rt;
   NETLIST nl;
   u_int loceffort = (effort > minEffort) ? effort : minEffort;

   // Now find and route all the nets

   for (i = 0; i < 3; i++) progress[i] = 0;
   remaining = Numnets;
 
   for (i = (debug_netnum >= 0) ? debug_netnum : 0; i < Numnets; i++) {

      net = getnettoroute(i);
      failed = remove_from_failed(net);
      if ((net != NULL) && (net->netnodes != NULL)) {

	 // Simple optimization:  If every route has four or fewer
	 // segments, then rerouting is almost certainly a waste of
	 // time.

	 if (!failed) {
	    for (rt = net->routes; rt; rt = rt->next) {
	       int j;
	       SEG seg = rt->segments;
	       for (j = 0; j < 3; j++) {
	          if (seg->next == NULL) break;
	          seg = seg->next;
	       }
	       if (j == 3) break;
	    }
	    if (rt == NULL) {
	       if (Verbose > 0)
	          Fprintf(stdout, "Keeping route for net %s\n", net->netname);
	       remaining--;
	       continue;
	    }
	 }

	 setBboxCurrent(net);
	 ripup_net(net, FALSE, FALSE, TRUE);	/* retain = TRUE */
	 // Set aside routes in case of failure.
         rt = net->routes;
	 net->routes = NULL;

	 // set mask mode to BBOX, if auto
	 maskSave = maskMode;
	 if (maskMode == MASK_AUTO) maskMode = MASK_BBOX;
	 result = doroute(net, FALSE, graphdebug);
	 maskMode = maskSave;
	 if (result == 0) {
	    if (Verbose > 0)
	       Fprintf(stdout, "Finished routing net %s\n", net->netname);
	    remaining--;
	    Fprintf(stdout, "Nets remaining: %d\n", remaining);
	    Flush(stdout);
	    remove_routes(rt, FALSE);	/* original is no longer needed */
	 }
	 else if (!failed) {
	    if (Verbose > 0)
	       Fprintf(stdout, "Failed to route net %s; restoring original\n",
			net->netname);

	    ripup_net(net, TRUE, FALSE, TRUE);	// Remove routes from Obs array
	    remove_routes(net->routes, FALSE);	/* should be NULL already */
	    net->routes = rt;
	    writeback_all_routes(net);	/* restore the original */
	    remaining--;
	    /* Pull net from FailedNets, since we restored it. */
	    if (FailedNets && (FailedNets->net == net)) {
	       nl = FailedNets->next;
	       free(FailedNets);
	       FailedNets = nl;
	    }
	 }
	 else {
	    if (Verbose > 0)
	       Fprintf(stdout, "Failed to route net %s.\n", net->netname);
	 }
      }
      else {
	 if (net && (Verbose > 0)) {
	    Fprintf(stdout, "Nothing to do for net %s\n", net->netname);
	 }
	 remaining--;
      }
      if (debug_netnum >= 0) break;

      /* Progress analysis (see 2nd stage).  Normally, the 3rd	 */
      /* stage is run only after all nets have been successfully */	
      /* routed.  However, there is no guarantee of this, so it	 */
      /* is necessary to anticipate convergence issues.		 */

      progress[1] += failcount;
      progress[0]++;
      if (progress[0] > loceffort) {
	 if ((progress[2] > 0) && (progress[2] < progress[1])) {
	    Fprintf(stderr, "\nNo progress at level of effort %d;"
			" ending 3rd stage.\n", loceffort);
	    break;
	 }
	 progress[2] = progress[1];
	 progress[1] = progress[0] = 0;
      }
   }
   failcount = countlist(FailedNets);
   if (debug_netnum >= 0) return failcount;

   if (Verbose > 0) {
      Flush(stdout);
      Fprintf(stdout, "\n----------------------------------------------\n");
      Fprintf(stdout, "Progress: ");
      Fprintf(stdout, "Stage 3 total routes completed: %d\n", TotalRoutes);
   }
   if (FailedNets == (NETLIST)NULL)
      Fprintf(stdout, "No failed routes!\n");
   else {
      if (FailedNets != (NETLIST)NULL)
          Fprintf(stdout, "Failed net routes: %d\n", failcount);
   }
   if (Verbose > 0)
      Fprintf(stdout, "----------------------------------------------\n");

   return failcount;
}

/*--------------------------------------------------------------*/
/* Free memory of an iroute glist and clear the Obs2		*/
/* PR_ON_STACK flag for each location in the list.		*/
/*--------------------------------------------------------------*/

void
free_glist(struct routeinfo_ *iroute)
{
   POINT gpoint;
   PROUTE *Pr;
   int i;
   
   for (i = 0; i < 6; i++) {
      while (iroute->glist[i]) {
         gpoint = iroute->glist[i];
         iroute->glist[i] = iroute->glist[i]->next;
         Pr = &OBS2VAL(gpoint->x1, gpoint->y1, gpoint->layer);
         Pr->flags &= ~PR_ON_STACK;
         freePOINT(gpoint);
      }
   }
}

/*--------------------------------------------------------------*/
/* doroute - basic route call					*/
/*								*/
/*	stage = 0 is normal routing				*/
/*	stage = 1 is the rip-up and reroute stage		*/
/*								*/
/*   ARGS: two nodes to be connected				*/
/*   RETURNS: 0 on success, -1 on failure			*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

int doroute(NET net, u_char stage, u_char graphdebug)
{
  ROUTE rt1, lrt;
  NETLIST nlist;
  int result, lastlayer, unroutable, i;
  struct routeinfo_ iroute;

  if (!net) {
     Fprintf(stderr, "doroute():  no net to route.\n");
     return 0;
  }

  CurNet = net;				// Global, used by 2nd stage

  // Fill out route information record
  iroute.net = net;
  iroute.rt = NULL;
  for (i = 0; i < 6; i++)
     iroute.glist[i] = NULL;
  iroute.nsrc = NULL;
  iroute.nsrctap = NULL;
  iroute.maxcost = MAXRT;
  iroute.do_pwrbus = FALSE;
  iroute.pwrbus_src = 0;

  lastlayer = -1;

  /* Set up Obs2[] matrix for first route */

  result = route_setup(&iroute, stage);
  unroutable = result - 1;
  if (graphdebug) highlight_mask();

  // Keep going until we are unable to route to a terminal

  while (net && (result > 0)) {

     if (graphdebug) highlight_source();
     if (graphdebug) highlight_dest();
     if (graphdebug)
	for (i = 0; i < 6; i++)
	    highlight_starts(iroute.glist[i]);

     rt1 = createemptyroute();
     rt1->netnum = net->netnum;
     iroute.rt = rt1;

     if (Verbose > 3) {
        Fprintf(stdout,"doroute(): added net %d path start %d\n", 
	       net->netnum, net->netnodes->nodenum);
     }

     result = route_segs(&iroute, stage, graphdebug);

     if (result < 0) {		// Route failure.

	// If we failed this on the last round, then stop
	// working on this net and move on to the next.
	if (FailedNets && (FailedNets->net == net)) break;

	nlist = (NETLIST)malloc(sizeof(struct netlist_));
	nlist->net = net;
	nlist->next = FailedNets;
	FailedNets = nlist;
	free(rt1);
     }
     else {

        TotalRoutes++;

        if (net->routes) {
           for (lrt = net->routes; lrt->next; lrt = lrt->next);
	   lrt->next = rt1;
        }
        else {
	   net->routes = rt1;
        }
        draw_net(net, TRUE, &lastlayer);
     }

     // For power routing, clear the list of existing pending route
     // solutions---they will not be relevant.

     if (iroute.do_pwrbus) free_glist(&iroute);

     /* Set up for next route and check if routing is done */
     result = next_route_setup(&iroute, stage);
  }

  /* Finished routing (or error occurred) */
  free_glist(&iroute);

  /* Route failure due to no taps or similar error---Log it */
  if ((result < 0) || (unroutable > 0)) {
     if ((FailedNets == NULL) || (FailedNets->net != net)) {
	nlist = (NETLIST)malloc(sizeof(struct netlist_));
	nlist->net = net;
	nlist->next = FailedNets;
	FailedNets = nlist;
     }
  }
  return (unroutable > 0) ? -1 : result;
  
} /* doroute() */

/*--------------------------------------------------------------*/
/* Catch-all routine when no tap points are found.  This is a	*/
/* common problem when the technology is not set up correctly	*/
/* and it's helpful to have all these error conditions pass	*/
/* to a single subroutine.					*/
/*--------------------------------------------------------------*/

static void unable_to_route(char *netname, NODE node, unsigned char forced)
{
    if (node)
	Fprintf(stderr, "Node %s of net %s has no tap points---",
		print_node_name(node), netname);
    else
	Fprintf(stderr, "Node of net %s has no tap points---",
		netname);

    if (forced)
	Fprintf(stderr, "forcing a tap point.\n");
    else
	Fprintf(stderr, "unable to route!\n");
}

/*--------------------------------------------------------------*/
/* next_route_setup --						*/
/*								*/
/*--------------------------------------------------------------*/

static int next_route_setup(struct routeinfo_ *iroute, u_char stage)
{
  ROUTE rt;
  NODE node;
  int  i, j;
  int  rval, result;

  if (iroute->do_pwrbus == TRUE) {

     iroute->pwrbus_src++;
     iroute->nsrc = iroute->nsrc->next;
     rval = -2;
     while (rval == -2) {
	if ((iroute->pwrbus_src > iroute->net->numnodes) || (iroute->nsrc == NULL)) {
	    result = 0;
	    break;
	}
	else {
	    result = set_powerbus_to_net(iroute->nsrc->netnum);
	    clear_target_node(iroute->nsrc);
	    rval = set_node_to_net(iroute->nsrc, PR_SOURCE,
			&iroute->glist[0], &iroute->bbox, stage);
	    if (rval == -2) {
		if (forceRoutable) {
		    make_routable(iroute->nsrc);
		}
		else {
		    iroute->pwrbus_src++;
		    iroute->nsrc = iroute->nsrc->next;
		}
		unable_to_route(iroute->net->netname, iroute->nsrc, forceRoutable);
	    }
	    else if (rval < 0) return -1;
	}
     }
  }
  else {

     for (rt = iroute->net->routes; (rt && rt->next); rt = rt->next);

     // Set positions on last route to PR_SOURCE
     if (rt) {
	result = set_route_to_net(iroute->net, rt, PR_SOURCE,
			&iroute->glist[0], &iroute->bbox, stage);

        if (result == -2) {
	   unable_to_route(iroute->net->netname, NULL, 0);
           return -1;
	}
     }
     else return -1;

     result = (count_targets(iroute->net) == 0) ? 0 : 1;
  }

  // Check for the possibility that there is already a route to the target

  if (!result) {

     // Remove nodes of the net from Nodeinfo.nodeloc so that they will not be
     // used for crossover costing of future routes.
     remove_tap_blocks(iroute->net->netnum);
     free_glist(iroute);
     return 0;
  }

  if (!iroute->do_pwrbus) {

     // If any target is found during the search, but is not the
     // target that is chosen for the minimum-cost path, then it
     // will be left marked "processed" and never visited again.
     // Make sure this doesn't happen by clearing the "processed"
     // flag from all such target nodes, and placing the positions
     // on the stack for processing again.

     clear_non_source_targets(iroute->net, &iroute->glist[0]);
  }

  if (Verbose > 1) {
     Fprintf(stdout, "netname = %s, route number %d\n",
		iroute->net->netname, TotalRoutes );
     Flush(stdout);
  }

  if (iroute->maxcost > 2)
      iroute->maxcost >>= 1;	// Halve the maximum cost from the last run

  return 1;		// Successful setup
}

/*--------------------------------------------------------------*/
/* route_setup --						*/
/*								*/
/*--------------------------------------------------------------*/

static int route_setup(struct routeinfo_ *iroute, u_char stage)
{
  int  i, j;
  u_int netnum, dir;
  int  result, rval, unroutable;
  NODE node;
  NODEINFO lnode;
  PROUTE *Pr;

  // Make Obs2[][] a copy of Obs[][].  Convert pin obstructions to
  // terminal positions for the net being routed.

  for (i = 0; i < Num_layers; i++) {
      for (j = 0; j < NumChannelsX * NumChannelsY; j++) {
	  netnum = Obs[i][j] & (~BLOCKED_MASK);
	  Pr = &Obs2[i][j];
	  if (netnum != 0) {
	      Pr->flags = 0;		// Clear all flags
	      if ((netnum & DRC_BLOCKAGE) == DRC_BLOCKAGE)
	         Pr->prdata.net = DRC_BLOCKAGE;
	      else
	         Pr->prdata.net = netnum & NETNUM_MASK;
	   } else {
	      Pr->flags = PR_COST;		// This location is routable
	      Pr->prdata.cost = MAXRT;
	   }
      }
  }

  if (iroute->net->netnum == VDD_NET || iroute->net->netnum == GND_NET ||
		iroute->net->netnum == ANTENNA_NET) {
     // The normal method of selecting source and target is not amenable
     // to power bus routes.  Instead, we use the global standard cell
     // power rails as the target, and each net in sequence becomes the
     // sole source node
     
     iroute->do_pwrbus = TRUE;
     iroute->nsrc = find_unrouted_node(iroute->net);
     result = (iroute->nsrc == NULL) ? 0 : 1;
  }
  else {
     iroute->do_pwrbus = FALSE;
     if (iroute->net->netnodes != NULL)
	 iroute->nsrc = iroute->net->netnodes;
     else {
	 Fprintf(stderr, "Net %s has no nodes, unable to route!\n",
			iroute->net->netname);
	 return -1;
     }
     result = 1;
  }

  // We start at the node referenced by the route structure, and flag all
  // of its taps as PR_SOURCE, as well as all connected routes.

  unroutable = 0;

  if (result) {
     iroute->bbox.x2 = iroute->bbox.y2 = 0;
     iroute->bbox.x1 = NumChannelsX;
     iroute->bbox.y1 = NumChannelsY;

     if (iroute->do_pwrbus == FALSE) {

	// Set node to PR_SOURCE
	rval = set_node_to_net(iroute->nsrc, PR_SOURCE,
		&iroute->glist[0], &iroute->bbox, stage);

        if (rval == -2) {
	   unable_to_route(iroute->net->netname, NULL, 0);
           return -1;
        }

        // Set associated routes to PR_SOURCE (okay to fail)
        rval = set_routes_to_net(iroute->nsrc, iroute->net, PR_SOURCE,
		&iroute->glist[0], &iroute->bbox, stage);

        // Now search for all other nodes on the same net that have not
        // yet been routed, and flag all of their taps as PR_TARGET

        result = 0;
        for (node = iroute->net->netnodes; node; node = node->next) {
	   if (node == iroute->nsrc) continue;
           rval = set_node_to_net(node, PR_TARGET, NULL, &iroute->bbox, stage);
           if (rval == 0) {
	      result = 1;
           }
           else if (rval == -2) {
	      if (forceRoutable) make_routable(node);
	      unable_to_route(iroute->net->netname, node, forceRoutable);
	      if (result == 0) result = -1;
	      unroutable++;
	      break;
           }
	   else if (rval == 1) continue;	/* This node was part of source */

	   // And add associated routes
	   rval = set_routes_to_net(node, iroute->net, PR_TARGET, NULL,
			&iroute->bbox, stage);
           if (rval == 0) result = 1;	/* (okay to fail) */
        }

        /* If there's only one node and it's not routable, then fail. */
        if (result == -1) return -1;
     }
     else {	/* Do this for power bus connections */

        while(1) {
           rval = set_node_to_net(iroute->nsrc, PR_SOURCE,
			&iroute->glist[0], &iroute->bbox, stage);
	   if (rval == -2) {
	      iroute->nsrc = iroute->nsrc->next;
	      if (iroute->nsrc == NULL) break;
	   }
	   else break;
        }
        if (rval == -2) {
           if (forceRoutable) make_routable(iroute->net->netnodes);
	   unable_to_route(iroute->net->netname, iroute->nsrc, forceRoutable);
           return -1;
        }

        /* Set all nodes that are NOT nsrc to an unused net number */
        for (node = iroute->net->netnodes; node; node = node->next) {
	   if (node != iroute->nsrc) {
	      disable_node_nets(node);
	   }
        }
        set_powerbus_to_net(iroute->nsrc->netnum);
     }
  }

  if (!result) {
     // Remove nodes of the net from Nodeinfo.nodeloc so that they will not be
     // used for crossover costing of future routes.
     remove_tap_blocks(iroute->net->netnum);
     free_glist(iroute);
     return 0;
  }

  // Generate a search area mask representing the "likely best route".
  if ((iroute->do_pwrbus == FALSE) && (maskMode == MASK_AUTO)) {
     if (stage == 0)
	createMask(iroute->net, MASK_SMALL, (u_char)Numpasses);
     else
	createMask(iroute->net, MASK_LARGE, (u_char)Numpasses);
  }
  else if ((iroute->do_pwrbus == TRUE) || (maskMode == MASK_NONE))
     fillMask((u_char)0);
  else if (maskMode == MASK_BBOX)
     createBboxMask(iroute->net, (u_char)Numpasses);
  else
     createMask(iroute->net, maskMode, (u_char)Numpasses);

  // Heuristic:  Set the initial cost beyond which we stop searching.
  // This value is twice the cost of a direct route across the
  // maximum extent of the source to target, divided by the square
  // root of the number of nodes in the net.  We purposely set this
  // value low.  It has a severe impact on the total run time of the
  // algorithm.  If the initial max cost is so low that no route can
  // be found, it will be doubled on each pass.

  if (iroute->do_pwrbus)
     iroute->maxcost = 20;	// Maybe make this SegCost * row height?
  else {
     iroute->maxcost = 1 + 2 * MAX((iroute->bbox.x2 - iroute->bbox.x1),
		(iroute->bbox.y2 - iroute->bbox.y1))
		* SegCost + (int)stage * ConflictCost;
     iroute->maxcost /= (iroute->nsrc->numnodes - 1);
  }

  netnum = iroute->net->netnum;

  iroute->nsrctap = iroute->nsrc->taps;
  if (iroute->nsrctap == NULL) iroute->nsrctap = iroute->nsrc->extend;
  if (iroute->nsrctap == NULL) {
     unable_to_route(iroute->net->netname, iroute->nsrc, 0);
     return -1;
  }

  if (Verbose > 2) {
     Fprintf(stdout, "Source node @ %gum %gum layer=%d grid=(%d %d)\n",
	  iroute->nsrctap->x, iroute->nsrctap->y, iroute->nsrctap->layer,
	  iroute->nsrctap->gridx, iroute->nsrctap->gridy);
  }

  if (Verbose > 1) {
     Fprintf(stdout, "netname = %s, route number %d\n", iroute->net->netname,
		TotalRoutes );
     Flush(stdout);
  }

  // Successful setup, although if nodes were marked unroutable,
  // this information is passed back;  routing will proceed for
  // all routable nodes and the net will be then marked as
  // abandoned.

  return (unroutable + 1);
}

/*--------------------------------------------------------------*/
/* route_segs - detailed route from node to node using onestep	*/
/*	method   						*/
/*								*/
/*   ARGS: ROUTE, ready to add segments to do route		*/
/*   RETURNS: NULL if failed, manhattan distance if success	*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

int route_segs(struct routeinfo_ *iroute, u_char stage, u_char graphdebug)
{
  POINT gpoint, gunproc, newpt;
  int  i, o;
  int  pass, maskpass;
  u_int forbid;
  GRIDP best, curpt;
  int rval;
  u_char first = TRUE;
  u_char check_order[6];
  u_char max_reached;
  u_char conflict;
  u_char predecessor;
  PROUTE *Pr;

  best.cost = MAXRT;
  best.x = 0;
  best.y = 0;
  best.lay = 0;
  gunproc = (POINT)NULL;
  maskpass = 0;
  
  for (pass = 0; pass < Numpasses; pass++) {

    max_reached = FALSE;
    if (!first && (Verbose > 2)) {
       Fprintf(stdout, "\n");
       first = TRUE;
    }
    if (Verbose > 2) {
       Fprintf(stdout, "Pass %d", pass + 1);
       Fprintf(stdout, " (maxcost is %d)\n", iroute->maxcost);
    }

    while (TRUE) {

      // Check priority stack and move down if 1st priorty is empty
      while (iroute->glist[0] == NULL) {
	 for (i = 0; i < 5; i++)
	    iroute->glist[i] = iroute->glist[i + 1];
	 iroute->glist[5] = NULL;
	 if ((iroute->glist[0] == NULL) && (iroute->glist[1] == NULL) &&
		(iroute->glist[2] == NULL) && (iroute->glist[3] == NULL) &&
		(iroute->glist[4] == NULL))
	    break;
      }
      gpoint = iroute->glist[0];
      if (gpoint == NULL) break;

      iroute->glist[0] = gpoint->next;

      // Stop-gap:  Needs to be investigated.  Occasional gpoint has
      // large (random?) value for y1.  Suggests a memory leak.  Only
      // seen occurring during doantennaroute().  Check using valgrind.

      if ((gpoint->x1 < 0) || (gpoint->y1 < 0) ||
		(gpoint->x1 > NumChannelsX) ||
		(gpoint->y1 > NumChannelsY)) {
         Fprintf(stderr, "Internal memory error!\n");
	 freePOINT(gpoint);
	 continue;
      }

      curpt.x = gpoint->x1;
      curpt.y = gpoint->y1;
      curpt.lay = gpoint->layer;

      if (graphdebug) highlight(curpt.x, curpt.y);
	
      Pr = &OBS2VAL(curpt.x, curpt.y, curpt.lay);

      // ignore grid positions that have already been processed
      if (Pr->flags & PR_PROCESSED) {
	 Pr->flags &= ~PR_ON_STACK;
	 freePOINT(gpoint);
	 continue;
      }

      if (Pr->flags & PR_COST)
	 curpt.cost = Pr->prdata.cost;	// Route points, including target
      else
	 curpt.cost = 0;			// For source tap points

      // if the grid position is the destination, save the position and
      // cost if minimum.

      if (Pr->flags & PR_TARGET) {

 	 if (curpt.cost < best.cost) {
	    if (first) {
	       if (Verbose > 2)
		  Fprintf(stdout, "Found a route of cost ");
	       first = FALSE;
	    }
	    else if (Verbose > 2) {
	       Fprintf(stdout, "|");
	       Fprintf(stdout, "%d", curpt.cost);
	       Flush(stdout);
	    }

	    // This position may be on a route, not at a terminal, so
	    // record it.
	    best.x = curpt.x;
	    best.y = curpt.y;
	    best.lay = curpt.lay;
	    best.cost = curpt.cost;

	    // If a complete route has been found, then there's no point
	    // in searching paths with a greater cost than this one.
	    if (best.cost < iroute->maxcost) iroute->maxcost = best.cost;
	 }

         // Don't continue processing from the target
	 Pr->flags |= PR_PROCESSED;
	 Pr->flags &= ~PR_ON_STACK;
	 freePOINT(gpoint);
	 continue;
      }

      if (curpt.cost < MAXRT) {

	 // Severely limit the search space by not processing anything that
	 // is not under the current route mask, which identifies a narrow
	 // "best route" solution.

	 if (RMASK(curpt.x, curpt.y) > (u_char)maskpass) {
	    gpoint->next = gunproc;
	    gunproc = gpoint;
	    continue;
	 }

         // Quick check:  Limit maximum cost to limit search space
         // Move the point onto the "unprocessed" stack and we'll pick up
         // from this point on the next pass, if needed.

         if (curpt.cost > iroute->maxcost) {
	    max_reached = TRUE;
	    gpoint->next = gunproc;
	    gunproc = gpoint;
	    continue;
	 }
      }
      Pr->flags &= ~PR_ON_STACK;
      freePOINT(gpoint);

      // check east/west/north/south, and bottom to top

      // 1st optimization:  Direction of route on current layer is preferred.
      o = LefGetRouteOrientation(curpt.lay);
      forbid = OBSVAL(curpt.x, curpt.y, curpt.lay) & BLOCKED_MASK;

      // To reach otherwise unreachable taps, allow searching on blocked
      // paths but with a high cost.
      conflict = (forceRoutable) ? PR_CONFLICT : PR_NO_EVAL;

      if (o == 1) {		// horizontal routes---check EAST and WEST first
	 check_order[0] = EAST  | ((forbid & BLOCKED_E) ? conflict : 0);
	 check_order[1] = WEST  | ((forbid & BLOCKED_W) ? conflict : 0);
	 check_order[2] = UP    | ((forbid & BLOCKED_U) ? conflict : 0);
	 check_order[3] = DOWN  | ((forbid & BLOCKED_D) ? conflict : 0);
	 check_order[4] = NORTH | ((forbid & BLOCKED_N) ? conflict : 0);
	 check_order[5] = SOUTH | ((forbid & BLOCKED_S) ? conflict : 0);
      }
      else {			// vertical routes---check NORTH and SOUTH first
	 check_order[0] = NORTH | ((forbid & BLOCKED_N) ? conflict : 0);
	 check_order[1] = SOUTH | ((forbid & BLOCKED_S) ? conflict : 0);
	 check_order[2] = UP    | ((forbid & BLOCKED_U) ? conflict : 0);
	 check_order[3] = DOWN  | ((forbid & BLOCKED_D) ? conflict : 0);
	 check_order[4] = EAST  | ((forbid & BLOCKED_E) ? conflict : 0);
	 check_order[5] = WEST  | ((forbid & BLOCKED_W) ? conflict : 0);
      }

      // Check order is from 0 (1st priority) to 5 (last priority).  However, this
      // is a stack system, so the last one placed on the stack is the first to be
      // pulled and processed.  Therefore we evaluate and drop positions to check
      // on the stack in reverse order (5 to 0).

      for (i = 5; i >= 0; i--) {
	 predecessor = 0;
	 switch (check_order[i]) {
	    case EAST | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case EAST:
	       predecessor |= PR_PRED_W;
               if ((curpt.x + 1) < NumChannelsX) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist[i];
         	     iroute->glist[i] = gpoint;
                  }
               }
	       break;

	    case WEST | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case WEST:
	       predecessor |= PR_PRED_E;
               if ((curpt.x - 1) >= 0) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist[i];
         	     iroute->glist[i] = gpoint;
                  }
               }
	       break;
         
	    case SOUTH | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case SOUTH:
	       predecessor |= PR_PRED_N;
               if ((curpt.y - 1) >= 0) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist[i];
         	     iroute->glist[i] = gpoint;
                   }
               }
	       break;

	    case NORTH | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case NORTH:
	       predecessor |= PR_PRED_S;
               if ((curpt.y + 1) < NumChannelsY) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist[i];
         	     iroute->glist[i] = gpoint;
                  }
               }
	       break;
      
	    case DOWN | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case DOWN:
	       predecessor |= PR_PRED_U;
               if (curpt.lay > 0) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist[i];
         	     iroute->glist[i] = gpoint;
         	  }
               }
	       break;
         
	    case UP | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case UP:
	       predecessor |= PR_PRED_D;
               if (curpt.lay < (Num_layers - 1)) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist[i];
         	     iroute->glist[i] = gpoint;
         	  }
               }
	       break;
            }
         }

      // Mark this node as processed
      Pr->flags |= PR_PROCESSED;

    } // while stack is not empty

    free_glist(iroute);

    // If we found a route, save it and return

    if (best.cost <= iroute->maxcost) {
	curpt.x = best.x;
	curpt.y = best.y;
	curpt.lay = best.lay;
	if ((rval = commit_proute(iroute->rt, &curpt, stage)) != 1) break;
	if (Verbose > 2) {
	   Fprintf(stdout, "\nCommit to a route of cost %d\n", best.cost);
	   Fprintf(stdout, "Between positions (%d %d) and (%d %d)\n",
			best.x, best.y, curpt.x, curpt.y);
	}
	route_set_connections(iroute->net, iroute->rt);
	goto done;	/* route success */
    }

    // Continue loop to next pass if any positions were ignored due to
    // masking or due to exceeding maxcost.

    // If the cost of the route exceeded maxcost at one or more locations,
    // then increase maximum cost for next pass.

    if (max_reached == TRUE) {
       iroute->maxcost <<= 1;
       // Cost overflow;  we're probably completely hosed long before this.
       if (iroute->maxcost > MAXRT) break;
    }
    else
       maskpass++;			// Increase the mask size

    if (gunproc == NULL) break;		// route failure not due to limiting
					// search to maxcost or to masking

    // Regenerate the stack of unprocessed nodes
    iroute->glist[0] = gunproc;
    gunproc = NULL;
    
  } // pass
  
  if (!first && (Verbose > 2)) {
     Fprintf(stdout, "\n");
     Flush(stdout);
  }
  if (Verbose > 1) {
     Fprintf(stderr, "Fell through %d passes\n", pass);
  }
  if (!iroute->do_pwrbus && (Verbose > 2)) {
     Fprintf(stderr, "(%g,%g) net=%s\n",
		iroute->nsrctap->x, iroute->nsrctap->y, iroute->net->netname);
  }
  rval = -1;

done:

  // Regenerate the stack of unprocessed nodes
  if (gunproc != NULL) iroute->glist[0] = gunproc;
  return rval;
  
} /* route_segs() */

/*--------------------------------------------------------------*/
/* createemptyroute - begin a ROUTE structure			*/
/*								*/
/*   ARGS: a nodes						*/
/*   RETURNS: ROUTE calloc'd and ready to begin			*/
/*   SIDE EFFECTS: 						*/
/*   AUTHOR and DATE: steve beccue      Fri Aug 8		*/
/*--------------------------------------------------------------*/

ROUTE createemptyroute(void)
{
   ROUTE rt;

   rt = (ROUTE)calloc(1, sizeof(struct route_));
   rt->netnum = 0;
   rt->segments = (SEG)NULL;
   rt->flags = (u_char)0;
   rt->next = (ROUTE)NULL;
   rt->start.route = (ROUTE)NULL;
   rt->end.route = (ROUTE)NULL;
   return rt;

} /* createemptyroute(void) */

/*--------------------------------------------------------------*/
/* helpmessage - tell user how to use the program		*/
/*								*/
/*   ARGS: none.						*/
/*   RETURNS: nothing.						*/
/*   SIDE EFFECTS: 						*/
/*								*/
/* NOTES:							*/
/* 1) "qrouter -v0 -h" prints only the version number and exits	*/
/* 2) Tcl-Tk based version adds ".T" to the end to alert tools	*/
/*    attempting to query the capabilities of qrouter of the	*/
/*    availability of the scripting.				*/
/*								*/
/*--------------------------------------------------------------*/

static void helpmessage(void)
{
    if (Verbose > 0) {
	Fprintf(stdout, "qrouter - maze router by Tim Edwards\n\n");
	Fprintf(stdout, "usage:  qrouter [-switches] design_name\n\n");
	Fprintf(stdout, "switches:\n");
	Fprintf(stdout, "\t-c <file>\t\t\tConfiguration file name if not route.cfg.\n");
	Fprintf(stdout, "\t-d <file>\t\t\tGenerate delay information output.\n");
	Fprintf(stdout, "\t-v <level>\t\t\tVerbose output level.\n");
	Fprintf(stdout, "\t-i <file>\t\t\tPrint route names and pitches and exit.\n");
	Fprintf(stdout, "\t-p <name>\t\t\tSpecify global power bus name.\n");
	Fprintf(stdout, "\t-g <name>\t\t\tSpecify global ground bus name.\n");
	Fprintf(stdout, "\t-r <value>\t\t\tForce output resolution scale.\n");
	Fprintf(stdout, "\t-f       \t\t\tForce all pins to be routable.\n");
	Fprintf(stdout, "\t-e <level>\t\t\tLevel of effort to keep trying.\n");
	Fprintf(stdout, "\n");
    }
#ifdef TCL_QROUTER
    Fprintf(stdout, "%s.%s.T\n", VERSION, REVISION);
#else
    Fprintf(stdout, "%s.%s\n", VERSION, REVISION);
#endif

} /* helpmessage() */

/* end of qrouter.c */
