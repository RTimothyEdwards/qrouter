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
#include "lef.h"
#include "def.h"
#include "graphics.h"

int  Pathon = -1;
int  TotalRoutes = 0;

NET     *Nlnets;	// list of nets in the design
NET	CurNet;		// current net to route, used by 2nd stage
STRING  DontRoute;      // a list of nets not to route (e.g., power)
STRING  CriticalNet;    // list of critical nets to route first
GATE    GateInfo;       // standard cell macro information
GATE	PinMacro;	// macro definition for a pin
GATE    Nlgates;	// gate instance information
NETLIST FailedNets;	// list of nets that failed to route

u_char   *RMask;    	        // mask out best area to route
u_int    *Obs[MAX_LAYERS];      // net obstructions in layer
PROUTE   *Obs2[MAX_LAYERS];     // used for pt->pt routes on layer
float    *Obsinfo[MAX_LAYERS];  // temporary array used for detailed obstruction info
NODEINFO *Nodeinfo[MAX_LAYERS]; // nodes and stub information is here. . .
DSEG      UserObs;		// user-defined obstruction layers

u_char needblock[MAX_LAYERS];

char *vddnet = NULL;
char *gndnet = NULL;

int    Numnets = 0;
int    Pinlayers = 0;
u_char Verbose = 3;	// Default verbose level
u_char keepTrying = (u_char)0;
u_char forceRoutable = FALSE;
u_char highOverhead = FALSE;
u_char maskMode = MASK_AUTO;
u_char mapType = MAP_OBSTRUCT | DRAW_ROUTES;
u_char ripLimit = 10;	// Fail net rather than rip up more than
			// this number of other nets.

char *DEFfilename = NULL;
char *delayfilename = NULL;

ScaleRec Scales;	// record of input and output scales

/* Prototypes for some local functions */
static void initMask(void);
static void fillMask(u_char value);
static int next_route_setup(struct routeinfo_ *iroute, u_char stage);
static int route_setup(struct routeinfo_ *iroute, u_char stage);
static int route_segs(struct routeinfo_ *iroute, u_char stage, u_char graphdebug);
static ROUTE createemptyroute(void);
static void emit_routes(char *filename, double oscale, int iscale);
static void helpmessage(void);


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

    if (NumChannelsX[0] != 0) return 0;	/* Already been called */

    for (i = 0; i < Num_layers; i++) {
	if (PitchX[i] == 0.0 || PitchY[i] == 0.0) {
	    Fprintf(stderr, "Have a 0 pitch for layer %d (of %d).  "
			"Exit.\n", i + 1, Num_layers);
	    return (-3);
	}
	NumChannelsX[i] = (int)(1.5 + (Xupperbound - Xlowerbound) / PitchX[i]);
	NumChannelsY[i] = (int)(1.5 + (Yupperbound - Ylowerbound) / PitchY[i]);
	if ((Verbose > 1) || (NumChannelsX[i] <= 0))
	    Fprintf(stdout, "Number of x channels for layer %d is %d\n",
				i, NumChannelsX[i]);
	if ((Verbose > 1) || (NumChannelsY[i] <= 0))
	    Fprintf(stdout, "Number of y channels for layer %d is %d\n",
				i, NumChannelsY[i]);
	
	if (NumChannelsX[i] <= 0) {
	    Fprintf(stderr, "Something wrong with layer %d x bounds.\n", i);
	    return(-3);
	}
	if (NumChannelsY[i] <= 0) {
	    Fprintf(stderr, "Something wrong with layer %d y bounds.\n", i);
	    return(-3);
	}
	Flush(stdout);
    }

    // Go through all nodes and remove any tap or extend entries that are
    // out of bounds.

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	for (node = net->netnodes; node != NULL; node = node->next) {

	    ltap = NULL;
	    for (ctap = node->taps; ctap != NULL; ) {
		ntap = ctap->next;
		glimitx = NumChannelsX[ctap->layer];
		glimity = NumChannelsY[ctap->layer];
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
		glimitx = NumChannelsX[ctap->layer];
		glimity = NumChannelsY[ctap->layer];
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
      Obs[i] = (u_int *)calloc(NumChannelsX[i] * NumChannelsY[i],
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
	    case 'k':
	    case 'v':
	    case 'd':
	    case 'p':
	    case 'g':
	    case 'r':
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
	    case 'm':
	       highOverhead = TRUE;
	       break;
	    case 'k':
	       keepTrying = (u_char)atoi(optarg);
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

   if (infoFILEptr != NULL) {

      /* Print qrouter name and version number at the top */
#ifdef TCL_QROUTER
      fprintf(infoFILEptr, "qrouter %s.%s.T\n", VERSION, REVISION);
#else
      fprintf(infoFILEptr, "qrouter %s.%s\n", VERSION, REVISION);
#endif

      /* Resolve pitches.  This is normally done after reading	*/
      /* the DEF file, but the info file is usually generated	*/
      /* from LEF layer information only, in order to get the	*/
      /* values needed to write the DEF file tracks.		*/

      for (i = 0; i < Num_layers; i++) {
	 int o = LefGetRouteOrientation(i);

	 /* Set PitchX and PitchY from route info as	*/
	 /* check_variable_pitch needs the values	*/

	 if (o == 1)
	    PitchY[i] = LefGetRoutePitch(i);
	 else
	    PitchX[i] = LefGetRoutePitch(i);
      }

      /* Resolve pitch information similarly to post_config() */

      for (i = 1; i < Num_layers; i++) {
	 int o = LefGetRouteOrientation(i);

	 if ((o == 1) && (PitchY[i - 1] == 0))
	    PitchY[i - 1] = PitchY[i];
	 else if ((o == 0) && (PitchX[i - 1] == 0))
	    PitchX[i - 1] = PitchX[i];
      }

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
	    pitch = (o == 1) ? PitchY[i] : PitchX[i],
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
   NumChannelsX[0] = 0;	// This is so we can check if NumChannelsX/Y were
			// set from within DefRead() due to reading in
			// existing nets.

   Scales.oscale = 1.0;
   return 0;
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
	for (j = 0; j < NumChannelsX[i] * NumChannelsY[i]; j++)
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

    while (FailedNets) {
	nl = FailedNets;
	FailedNets = FailedNets->next;
	free(nl);
    }

    // Free all net and route information

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	while (net->noripup) {
	    nl = net->noripup;
	    net->noripup = net->noripup->next;
	    free(nl);
	}
	while (net->routes) {
	    rt = net->routes;
	    net->routes = net->routes->next;
	    while (rt->segments) {
		seg = rt->segments;
		rt->segments = rt->segments->next;
		free(seg);
	    }
	    free(rt);
	}
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
/* post_def_setup ---						*/
/*								*/
/* Things to do after a DEF file has been read in, and the size	*/
/* of the layout, components, and nets are known.		*/
/*--------------------------------------------------------------*/

static int post_def_setup()
{
   NET net;
   int i;
   double sreq1, sreq2;

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

      Obsinfo[i] = (float *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(float));
      if (!Obsinfo[i]) {
	 fprintf(stderr, "Out of memory 5.\n");
	 exit(5);
      }

      Nodeinfo[i] = (NODEINFO *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(NODEINFO));
      if (!Nodeinfo[i]) {
	 fprintf( stderr, "Out of memory 6.\n");
	 exit(6);
      }
   }
   Flush(stdout);

   if (Verbose > 1)
      Fprintf(stderr, "Diagnostic: memory block is %d bytes\n",
		(int)sizeof(u_int) * NumChannelsX[0] * NumChannelsY[0]);

   /* Be sure to create obstructions from gates first, since we don't	*/
   /* want improperly defined or positioned obstruction layers to over-	*/
   /* write our node list.						*/

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
   
   // If any nets are pre-routed, place those routes.

   for (i = 0; i < Numnets; i++) {
      net = Nlnets[i];
      writeback_all_routes(net);
   }

   // Remove the Obsinfo array, which is no longer needed, and allocate
   // the Obs2 array for costing information

   for (i = 0; i < Num_layers; i++) free(Obsinfo[i]);

   for (i = 0; i < Num_layers; i++) {
      Obs2[i] = (PROUTE *)calloc(NumChannelsX[i] * NumChannelsY[i],
			sizeof(PROUTE));
      if (!Obs2[i]) {
         fprintf( stderr, "Out of memory 9.\n");
         exit(9);
      }
   }

   // Fill in needblock bit fields, which are used by commit_proute
   // when route layers are too large for the grid size, and grid points
   // around a route need to be marked as blocked whenever something is
   // routed on those layers.

   // "ROUTEBLOCK" is set if the spacing is violated between a normal
   // route and an adjacent via.  "VIABLOCK" is set if the spacing is
   // violated between two adjacent vias.  It may be helpful to define
   // a third category which is route-to-route spacing violation.

   for (i = 0; i < Num_layers; i++) {
      needblock[i] = (u_char)0;
      sreq1 = LefGetRouteSpacing(i);

      sreq2 = LefGetViaWidth(i, i, 0) + sreq1;
      if ((sreq2 - EPS) > PitchX[i]) needblock[i] |= VIABLOCKX;
      if (i != 0) {
	 sreq2 = LefGetViaWidth(i - 1, i, 0) + sreq1;
         if ((sreq2 - EPS) > PitchX[i]) needblock[i] |= VIABLOCKX;
      }

      sreq2 = LefGetViaWidth(i, i, 1) + sreq1;
      if ((sreq2 - EPS) > PitchY[i]) needblock[i] |= VIABLOCKY;
      if (i != 0) {
	 sreq2 = LefGetViaWidth(i - 1, i, 1) + sreq1;
         if ((sreq2 - EPS) > PitchY[i]) needblock[i] |= VIABLOCKY;
      }

      sreq1 += 0.5 * LefGetRouteWidth(i);

      sreq2 = sreq1 + 0.5 * LefGetViaWidth(i, i, 0);
      if ((sreq2 - EPS) > PitchX[i]) needblock[i] |= ROUTEBLOCKX;
      if (i != 0) {
	 sreq2 = sreq1 + 0.5 * LefGetViaWidth(i - 1, i, 0);
         if ((sreq2 - EPS) > PitchX[i]) needblock[i] |= ROUTEBLOCKX;
      }

      sreq2 = sreq1 + 0.5 * LefGetViaWidth(i, i, 1);
      if ((sreq2 - EPS) > PitchY[i]) needblock[i] |= ROUTEBLOCKY;
      if (i != 0) {
	 sreq2 = sreq1 + 0.5 * LefGetViaWidth(i - 1, i, 1);
         if ((sreq2 - EPS) > PitchY[i]) needblock[i] |= ROUTEBLOCKY;
      }
   }

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
/*--------------------------------------------------------------*/

void read_def(char *filename)
{
   double oscale, precis;

   if ((filename == NULL) && (DEFfilename == NULL)) {
      Fprintf(stderr, "No DEF file specified, nothing to read.\n");
      return;
   }
   else if (filename != NULL) {
      if (DEFfilename != NULL) {
	  reinitialize();
	  free(DEFfilename);
      }
      DEFfilename = strdup(filename);
   }
   else reinitialize();

   oscale = (double)DefRead(DEFfilename);
   precis = Scales.mscale / oscale;	// from LEF manufacturing grid
   if (precis < 1.0) precis = 1.0;
   precis *= (double)Scales.iscale;	// user-defined extra scaling

   Scales.iscale = (int)(precis + 0.5);
   Scales.oscale = (double)(Scales.iscale * oscale);

   if (Verbose > 0)
      Fprintf(stdout, "Output scale = microns / %g, precision %g\n",
		Scales.oscale / (double)Scales.iscale,
		1.0 / (double)Scales.iscale);

   post_def_setup();
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

   if (debug_netnum <= 0) {
      while (FailedNets) {
         nl = FailedNets->next;
         free(FailedNets);
         FailedNets = nl;
      }
   }

   // Now find and route all the nets

   remaining = Numnets;
 
   for (i = (debug_netnum >= 0) ? debug_netnum : 0; i < Numnets; i++) {

      net = getnettoroute(i);
      if ((net != NULL) && (net->netnodes != NULL)) {
	 result = doroute(net, (u_char)0, graphdebug);
	 if (result == 0) {
	    remaining--;
	    if (Verbose > 0)
	       Fprintf(stdout, "Finished routing net %s\n", net->netname);
	    Fprintf(stdout, "Nets remaining: %d\n", remaining);
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

         fprintf(cmd, "%s %g ( %g %g ) ", CIFLayer[layer],
			invscale * (int)(oscale * wvia + 0.5),
			invscale * x, invscale * y);
      }
      else
         fprintf(cmd, "%s ( %g %g ) ", CIFLayer[layer], invscale * x, invscale * y);
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
	fprintf(cmd, "%g ", invscale * x);
    else
	fprintf(cmd, "* ");

    if (horizontal)
	fprintf(cmd, "* ");
    else
	fprintf(cmd, "%g ", invscale * y);

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

    if ((ViaPattern == VIA_PATTERN_NONE) || (ViaY[layer] == NULL))
	s = ViaX[layer];
    else if (ViaPattern == VIA_PATTERN_NORMAL)
	s = (checkersign == 0) ?  ViaX[layer] : ViaY[layer];
    else
	s = (checkersign == 0) ?  ViaY[layer] : ViaX[layer];

    if (Pathon <= 0) {
       if (Pathon == -1)
	  fprintf(cmd, "+ ROUTED ");
       else 
	  fprintf(cmd, "\n  NEW ");
       fprintf(cmd, "%s ( %g %g ) ", CIFLayer[layer], invscale * x, invscale * y);
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
		net->netnum == GND_NET))
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

static int ripup_colliding(NET net)
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
	if (ripup_net(nl->net, (u_char)1) == TRUE) { 
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

int route_net_ripup(NET net, u_char graphdebug)
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

    result = doroute(net, (u_char)1, graphdebug);
    if (result != 0) {
	if (net->noripup != NULL) {
	    if ((net->flags & NET_PENDING) == 0) {
		// Clear this net's "noripup" list and try again.

		while (net->noripup) {
		    nl = net->noripup->next;
		    free(net->noripup);
		    net->noripup = nl;
		}
		result = doroute(net, (u_char)1, graphdebug);
		net->flags |= NET_PENDING;	// Next time we abandon it.
	    }
	}
    }
    if (result != 0)
	result = ripup_colliding(net);

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
dosecondstage(u_char graphdebug, u_char singlestep)
{
   int failcount, origcount, result, maxtries;
   NET net;
   NETLIST nl, nl2;
   NETLIST Abandoned;	// Abandoned routes---not even trying any more.
   ROUTE rt, rt2;
   SEG seg;

   origcount = countlist(FailedNets);
   if (FailedNets)
      maxtries = TotalRoutes + ((origcount < 20) ? 20 : origcount) * 8;
   else
      maxtries = 0;

   fillMask((u_char)0);
   Abandoned = NULL;

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

      result = doroute(net, (u_char)1, graphdebug);

      if (result != 0) {
	 if (net->noripup != NULL) {
	    if ((net->flags & NET_PENDING) == 0) {
	       // Clear this net's "noripup" list and try again.

	       while (net->noripup) {
	          nl = net->noripup->next;
	          free(net->noripup);
	          net->noripup = nl;
	       }
	       result = doroute(net, (u_char)1, graphdebug);
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

	 result = ripup_colliding(net);
	 if (result > 0) result = 0;
      }

      if (result != 0) {

	 // Complete failure to route, even allowing collisions.
	 // Abandon routing this net.

	 if (Verbose > 0) {
	    Flush(stdout);
	    Fprintf(stderr, "----------------------------------------------\n");
	    Fprintf(stderr, "Complete failure on net %s:  Abandoning.\n",
			net->netname);
	    Fprintf(stderr, "----------------------------------------------\n");
	 }

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

	 ripup_net(net, (u_char)1);	// Remove routing information from net
	 continue;
      }

      // Write back the original route to the grid array
      writeback_all_routes(net);

      // Failsafe---if we have been looping enough times to exceed
      // maxtries (which is set to 8 route attempts per original failed
      // net), then we check progress.  If we have reduced the number
      // of failed nets by half or more, then we have an indication of
      // real progress, and will continue.  If not, we give up.  Qrouter
      // is almost certainly hopelessly stuck at this point.

      if (TotalRoutes >= maxtries) {
	 if (failcount <= (origcount / 2)) {
	    maxtries = TotalRoutes + failcount * 8;
	    origcount = failcount;
	 }
	 else if (keepTrying == 0) {
	    Fprintf(stderr, "\nQrouter is stuck, abandoning remaining routes.\n");
	    break;
	 }
	 else {
	    keepTrying--;
	    Fprintf(stderr, "\nQrouter is stuck, but I was told to keep trying.\n");
	    maxtries = TotalRoutes + failcount * 8;
	    origcount = failcount;
	 }
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

int dothirdstage(u_char graphdebug, int debug_netnum)
{
   int i, failcount, remaining, result;
   NET net;
   NETLIST nl;

   // Clear the lists of failed routes, in case first
   // stage is being called more than once.

   if (debug_netnum <= 0) {
      while (FailedNets) {
         nl = FailedNets->next;
         free(FailedNets);
         FailedNets = nl;
      }
   }

   // Now find and route all the nets

   remaining = Numnets;
 
   for (i = (debug_netnum >= 0) ? debug_netnum : 0; i < Numnets; i++) {

      net = getnettoroute(i);
      if ((net != NULL) && (net->netnodes != NULL)) {
	 ripup_net(net, (u_char)0);
	 result = doroute(net, (u_char)0, graphdebug);
	 if (result == 0) {
	    remaining--;
	    if (Verbose > 0)
	       Fprintf(stdout, "Finished routing net %s\n", net->netname);
	    Fprintf(stdout, "Nets remaining: %d\n", remaining);
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
/* initMask() ---						*/
/*--------------------------------------------------------------*/

static void initMask(void)
{
   RMask = (u_char *)calloc(NumChannelsX[0] * NumChannelsY[0],
			sizeof(u_char));
   if (!RMask) {
      fprintf(stderr, "Out of memory 3.\n");
      exit(3);
   }
}

/*--------------------------------------------------------------*/
/* Fill mask around the area of a vertical line			*/
/*--------------------------------------------------------------*/

static void
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
   if (gx2 >= NumChannelsX[0]) gx2 = NumChannelsX[0] - 1;
   if (gy1 < 0) gy1 = 0;
   if (gy2 >= NumChannelsY[0]) gy2 = NumChannelsY[0] - 1;

   for (i = gx1; i <= gx2; i++)
      for (j = gy1; j <= gy2; j++)
	 RMASK(i, j) = (u_char)0;

   for (v = 1; v < halo; v++) {
      if (gx1 > 0) gx1--;
      if (gx2 < NumChannelsX[0] - 1) gx2++;
      if (y1 > y2) {
         if (gy1 < NumChannelsY[0] - 1) gy1++;
         if (gy2 < NumChannelsY[0] - 1) gy2++;
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

static void
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
   if (gx2 >= NumChannelsX[0]) gx2 = NumChannelsX[0] - 1;
   if (gy1 < 0) gy1 = 0;
   if (gy2 >= NumChannelsY[0]) gy2 = NumChannelsY[0] - 1;

   for (i = gx1; i <= gx2; i++)
      for (j = gy1; j <= gy2; j++)
	 RMASK(i, j) = (u_char)0;

   for (v = 1; v < halo; v++) {
      if (gy1 > 0) gy1--;
      if (gy2 < NumChannelsY[0] - 1) gy2++;
      if (x1 > x2) {
         if (gx1 < NumChannelsX[0] - 1) gx1++;
         if (gx2 < NumChannelsX[0] - 1) gx2++;
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
/* createBboxMask() ---						*/
/*								*/
/* Create mask limiting the area to search for routing		*/
/*								*/
/* The bounding box mask generates an area including the	*/
/* bounding box as defined in the net record, includes all pin	*/
/* positions in the mask, and increases the mask area by one	*/
/* route track for each pass, up to "halo".			*/
/*--------------------------------------------------------------*/

static void createBboxMask(NET net, u_char halo)
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
	if (gx1 >= 0 && gx1 < NumChannelsX[0])
           for (j = ymin - i; j <= ymax + i; j++)
	      if (j >= 0 && j < NumChannelsY[0])
		 RMASK(gx1, j) = (u_char)i;

	gx2 = xmax + i;
	if (gx2 >= 0 && gx2 < NumChannelsX[0])
           for (j = ymin - i; j <= ymax + i; j++)
	      if (j >= 0 && j < NumChannelsY[0])
		 RMASK(gx2, j) = (u_char)i;

	gy1 = ymin - i;
	if (gy1 >= 0 && gy1 < NumChannelsY[0])
           for (j = xmin - i; j <= xmax + i; j++)
	      if (j >= 0 && j < NumChannelsX[0])
		 RMASK(j, gy1) = (u_char)i;

	gy2 = ymax + i;
	if (gy2 >= 0 && gy2 < NumChannelsY[0])
           for (j = xmin - i; j <= xmax + i; j++)
	      if (j >= 0 && j < NumChannelsX[0])
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

static int analyzeCongestion(int ycent, int ymin, int ymax, int xmin, int xmax)
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

static void createMask(NET net, u_char slack, u_char halo)
{
  NODE n1, n2;
  DPOINT dtap;
  int i, j, orient;
  int dx, dy, gx1, gx2, gy1, gy2;
  int xcent, ycent, xmin, ymin, xmax, ymax;

  fillMask((u_char)halo);

  xmin = net->xmin;
  xmax = net->xmax;
  ymin = net->ymin;
  ymax = net->ymax;

  xcent = net->trunkx;
  ycent = net->trunky;

  orient = 0;

  // Construct the trunk line mask

  if (!(net->flags & NET_VERTICAL_TRUNK) || (net->numnodes == 2)) {
     // Horizontal trunk
     orient |= 1;

     ycent = analyzeCongestion(net->trunky, ymin, ymax, xmin, xmax);
     ymin = ymax = ycent;

     for (i = xmin - slack; i <= xmax + slack; i++) {
	if (i < 0 || i >= NumChannelsX[0]) continue;
	for (j = ycent - slack; j <= ycent + slack; j++) {
	   if (j < 0 || j >= NumChannelsY[0]) continue;
	   RMASK(i, j) = (u_char)0;
	}
     }

     for (i = 1; i < halo; i++) {
	gy1 = ycent - slack - i;
	gy2 = ycent + slack + i;
        for (j = xmin - slack - i; j <= xmax + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsX[0]) continue;
	   if (gy1 >= 0)
	      RMASK(j, gy1) = (u_char)i;
	   if (gy2 < NumChannelsY[0])
	      RMASK(j, gy2) = (u_char)i;
	}
	gx1 = xmin - slack - i;
	gx2 = xmax + slack + i;
        for (j = ycent - slack - i; j <= ycent + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsY[0]) continue;
	   if (gx1 >= 0)
	      RMASK(gx1, j) = (u_char)i;
	   if (gx2 < NumChannelsX[0])
	      RMASK(gx2, j) = (u_char)i;
	}
     }
  }
  if ((net->flags & NET_VERTICAL_TRUNK) || (net->numnodes == 2)) {
     // Vertical trunk
     orient |= 2;
     xmin = xmax = xcent;

     for (i = xcent - slack; i <= xcent + slack; i++) {
	if (i < 0 || i >= NumChannelsX[0]) continue;
	for (j = ymin - slack; j <= ymax + slack; j++) {
	   if (j < 0 || j >= NumChannelsY[0]) continue;
	   RMASK(i, j) = (u_char)0;
	}
     }

     for (i = 1; i < halo; i++) {
	gx1 = xcent - slack - i;
	gx2 = xcent + slack + i;
        for (j = ymin - slack - i; j <= ymax + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsY[0]) continue;
	   if (gx1 >= 0)
	      RMASK(gx1, j) = (u_char)i;
	   if (gx2 < NumChannelsX[0])
	      RMASK(gx2, j) = (u_char)i;
	}
	gy1 = ymin - slack - i;
	gy2 = ymax + slack + i;
        for (j = xcent - slack - i; j <= xcent + slack + i; j++) {
	   if (j < 0 || j >= NumChannelsX[0]) continue;
	   if (gy1 >= 0)
	      RMASK(j, gy1) = (u_char)i;
	   if (gy2 < NumChannelsY[0])
	      RMASK(j, gy2) = (u_char)i;
	}
     }
  }
     
  // Construct the branch line masks

  for (n1 = net->netnodes; n1; n1 = n1->next) {
     dtap = (n1->taps == NULL) ? n1->extend : n1->taps;
     if (!dtap) continue;

     if (orient | 1) 	// Horizontal trunk, vertical branches
	create_vbranch_mask(n1->branchx, n1->branchy, ycent, slack, halo);
     if (orient | 2) 	// Vertical trunk, horizontal branches
	create_hbranch_mask(n1->branchy, n1->branchx, xcent, slack, halo);
  }

  // Look for branches that are closer to each other than to the
  // trunk line.  If any are found, make a cross-connection between
  // the branch end that is closer to the trunk and the branch that
  // is its nearest neighbor.

  if (orient | 1) {	// Horizontal trunk, vertical branches
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
  if (orient | 2) {		// Vertical trunk, horizontal branches
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
			xmin, ymin, xmax, ymax);
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

static void fillMask(u_char value) {
   memset((void *)RMask, (int)value,
		(size_t)(NumChannelsX[0] * NumChannelsY[0]
		* sizeof(u_char)));
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

   while (iroute->glist) {
      gpoint = iroute->glist;
      iroute->glist = iroute->glist->next;
      Pr = &OBS2VAL(gpoint->x1, gpoint->y1, gpoint->layer);
      Pr->flags &= ~PR_ON_STACK;
      freePOINT(gpoint);
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
  int result, lastlayer, unroutable;
  struct routeinfo_ iroute;

  if (!net) {
     Fprintf(stderr, "doroute():  no net to route.\n");
     return 0;
  }

  CurNet = net;				// Global, used by 2nd stage

  // Fill out route information record
  iroute.net = net;
  iroute.rt = NULL;
  iroute.glist = NULL;
  iroute.nsrc = NULL;
  iroute.nsrctap = NULL;
  iroute.maxcost = MAXRT;
  iroute.do_pwrbus = (u_char)0;
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
     if (graphdebug) highlight_starts(iroute.glist);

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
  return result;
  
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
	    rval = set_node_to_net(iroute->nsrc, PR_SOURCE, &iroute->glist,
			&iroute->bbox, stage);
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
	result = set_route_to_net(iroute->net, rt, PR_SOURCE, &iroute->glist,
			&iroute->bbox, stage);

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

     for (i = 0; i < Pinlayers; i++) {
	for (j = 0; j < NumChannelsX[i] * NumChannelsY[i]; j++) {
	   if (Nodeinfo[i][j]) {
	      node = Nodeinfo[i][j]->nodeloc;
	      if (node != (NODE)NULL)
	         if (node->netnum == iroute->net->netnum)
		    Nodeinfo[i][j]->nodeloc = (NODE)NULL;
	   }
        }
     }

     free_glist(iroute);
     return 0;
  }

  if (!iroute->do_pwrbus) {

     // If any target is found during the search, but is not the
     // target that is chosen for the minimum-cost path, then it
     // will be left marked "processed" and never visited again.
     // Make sure this doesn't happen my clearing the "processed"
     // flag from all such target nodes, and placing the positions
     // on the stack for processing again.

     clear_non_source_targets(iroute->net, &iroute->glist);
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
      for (j = 0; j < NumChannelsX[i] * NumChannelsY[i]; j++) {
	  netnum = Obs[i][j] & (~BLOCKED_MASK);
	  Pr = &Obs2[i][j];
	  if (netnum != 0) {
	      Pr->flags = 0;		// Clear all flags
	      if (netnum == DRC_BLOCKAGE)
	         Pr->prdata.net = netnum;
	      else
	         Pr->prdata.net = netnum & NETNUM_MASK;
	   } else {
	      Pr->flags = PR_COST;		// This location is routable
	      Pr->prdata.cost = MAXRT;
	   }
      }
  }

  if (iroute->net->netnum == VDD_NET || iroute->net->netnum == GND_NET) {
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
     iroute->bbox.x1 = NumChannelsX[0];
     iroute->bbox.y1 = NumChannelsY[0];

     while(1) {
        rval = set_node_to_net(iroute->nsrc, PR_SOURCE, &iroute->glist,
			&iroute->bbox, stage);
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

     if (iroute->do_pwrbus == FALSE) {

        // Set associated routes to PR_SOURCE
        rval = set_routes_to_net(iroute->net, PR_SOURCE, &iroute->glist,
		&iroute->bbox, stage);

        if (rval == -2) {
	   unable_to_route(iroute->net->netname, NULL, 0);
           return -1;
        }

        // Now search for all other nodes on the same net that have not
        // yet been routed, and flag all of their taps as PR_TARGET

        result = 0;
        for (node = iroute->net->netnodes; node; node = node->next) {
	   if (node == iroute->nsrc) continue;
           rval = set_node_to_net(node, PR_TARGET, NULL,
			&iroute->bbox, stage);
           if (rval == 0) {
	      result = 1;
           }
           else if (rval == -2) {
	      if (forceRoutable) make_routable(node);
	      unable_to_route(iroute->net->netname, node, forceRoutable);
	      if (result == 0) result = -1;
	      unroutable++;
           }
        }

        /* If there's only one node and it's not routable, then fail. */
        if (result == -1) return -1;
     }
     else {	/* Do this for power bus connections */

        /* Set all nodes that are NOT nsrc to an unused net number */
        for (node = iroute->net->netnodes; node; node = node->next) {
	   if (node != iroute->nsrc) {
	      disable_node_nets(node);
	   }
        }
        set_powerbus_to_net(iroute->nsrc->netnum);
     }
  }

  // Check for the possibility that there is already a route to the target

  if (!result) {
     // Remove nodes of the net from Nodeinfo.nodeloc so that they will not be
     // used for crossover costing of future routes.

     for (i = 0; i < Pinlayers; i++) {
	for (j = 0; j < NumChannelsX[i] * NumChannelsY[i]; j++) {
	   if (Nodeinfo[i][j]) {
	      iroute->nsrc = Nodeinfo[i][j]->nodeloc;
	      if (iroute->nsrc != (NODE)NULL)
	         if (iroute->nsrc->netnum == iroute->net->netnum)
		    Nodeinfo[i][j]->nodeloc = (NODE)NULL;
	   }
        }
     }

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

static int route_segs(struct routeinfo_ *iroute, u_char stage, u_char graphdebug)
{
  POINT gpoint, gunproc, newpt;
  int  i, o;
  int  pass, maskpass;
  u_int forbid;
  GRIDP best, curpt;
  int rval;
  u_char first = (u_char)1;
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

    max_reached = (u_char)0;
    if (!first && (Verbose > 2)) {
       Fprintf(stdout, "\n");
       first = (u_char)1;
    }
    if (Verbose > 2) {
       Fprintf(stdout, "Pass %d", pass + 1);
       Fprintf(stdout, " (maxcost is %d)\n", iroute->maxcost);
    }

    while ((gpoint = iroute->glist) != NULL) {

      iroute->glist = gpoint->next;

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
	       first = (u_char)0;
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
	    max_reached = (u_char)1;
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
               if ((curpt.x + 1) < NumChannelsX[curpt.lay]) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
                  }
               }
	       break;

	    case WEST | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case WEST:
	       predecessor |= PR_PRED_E;
               if ((curpt.x - 1) >= 0) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
                  }
               }
	       break;
         
	    case SOUTH | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case SOUTH:
	       predecessor |= PR_PRED_N;
               if ((curpt.y - 1) >= 0) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
                   }
               }
	       break;

	    case NORTH | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case NORTH:
	       predecessor |= PR_PRED_S;
               if ((curpt.y + 1) < NumChannelsY[curpt.lay]) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
                  }
               }
	       break;
      
	    case DOWN | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case DOWN:
	       predecessor |= PR_PRED_U;
               if (curpt.lay > 0) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
         	  }
               }
	       break;
         
	    case UP | PR_CONFLICT:
	       predecessor = PR_CONFLICT;
	    case UP:
	       predecessor |= PR_PRED_D;
               if (curpt.lay < (Num_layers - 1)) {
         	  if ((gpoint = eval_pt(&curpt, predecessor, stage)) != NULL) {
         	     gpoint->next = iroute->glist;
         	     iroute->glist = gpoint;
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
	goto done;	/* route success */
    }

    // Continue loop to next pass if any positions were ignored due to
    // masking or due to exceeding maxcost.

    // If the cost of the route exceeded maxcost at one or more locations,
    // then increase maximum cost for next pass.

    if (max_reached == (u_char)1) {
       iroute->maxcost <<= 1;
       // Cost overflow;  we're probably completely hosed long before this.
       if (iroute->maxcost > MAXRT) break;
    }
    else
       maskpass++;			// Increase the mask size

    if (gunproc == NULL) break;		// route failure not due to limiting
					// search to maxcost or to masking

    // Regenerate the stack of unprocessed nodes
    iroute->glist = gunproc;
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
  if (gunproc != NULL) iroute->glist = gunproc;
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

static ROUTE createemptyroute(void)
{
   ROUTE rt;

   rt = (ROUTE)calloc(1, sizeof(struct route_));
   rt->netnum = 0;
   rt->segments = (SEG)NULL;
   rt->flags = (u_char)0;
   rt->next = (ROUTE)NULL;
   return rt;

} /* createemptyroute(void) */

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
/*--------------------------------------------------------------*/

static void cleanup_net(NET net)
{
   SEG segf, segl, seg;
   ROUTE rt, rt2;
   NODEINFO lnode;
   int lf, ll, lf2, ll2;
   u_char fcheck, lcheck;
   u_char xcheckf, ycheckf, xcheckl, ycheckl; 

   lf = ll = lf2 = ll2 = -1;

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
		  offset2 = lnode2->offset;
	       }
	       else {
		  lnode2 = NODEIPTR(seg->x2, seg->y2, seg->layer);
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
			pathstart(Cmd, seg->layer, x, y, special, oscale, invscale,
				horizontal);
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

    fdef = fopen(filename, "r");
    if (fdef == NULL) {
	if (strchr(filename, '.') == NULL) {
	    char *extfilename = malloc(strlen(filename) + 5);
	    sprintf(extfilename, "%s.def", filename);
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
	else
	    Cmd = fopen(filename, "w");
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

    // NOTE:  May want to remove this message.  It may merely reflect
    // that the DEF file defined one or more SPECIALNETS.

    if (numnets != Numnets) {
      	Flush(stdout);
	Fprintf(stderr, "emit_routes():  DEF file has %d nets, but we want"
		" to write %d\n", numnets, Numnets);
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
	Fprintf(stdout, "\t-k <level>\t\t\tLevel of effort to keep trying.\n");
	Fprintf(stdout, "\n");
    }
#ifdef TCL_QROUTER
    Fprintf(stdout, "%s.%s.T\n", VERSION, REVISION);
#else
    Fprintf(stdout, "%s.%s\n", VERSION, REVISION);
#endif

} /* helpmessage() */

/* end of qrouter.c */
