/*--------------------------------------------------------------*/
/* qrouter.h -- general purpose autorouter                     	*/
/*--------------------------------------------------------------*/
/* Written by Steve Beccue, 2003				*/
/* Modified by Tim Edwards 2011-2013				*/
/*--------------------------------------------------------------*/

#ifndef QROUTER_H

#define OGRID(x, y) ((int)((x) + ((y) * NumChannelsX)))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define ABSDIFF(x, y) (((x) > (y)) ? ((x) - (y)) : ((y) - (x)))

#define EPS	1e-4		// For handling round-off error;  value
				// is in um (so 1e-4 = 0.1nm), should be
				// smaller than any feature size.

// For handling round-off error on signed values.  REPS(x) is x + EPS when
// x is positive, and x - EPS when x is negative.

#define REPS(x)	(((x) < 0) ? ((x) - EPS) : ((x) + EPS))

#define TRUE    1
#define FALSE   0

#ifndef _SYS_TYPES_H
#ifndef u_char
typedef unsigned char  u_char;
#endif
#ifndef u_short
typedef unsigned short u_short;
#endif
#ifndef u_int
typedef unsigned int   u_int;
#endif
#ifndef u_long
typedef unsigned long  u_long;
#endif
#endif /* _SYS_TYPES_H */

/* Compare functions aren't defined in the Mac's standard library */
#if defined(__APPLE__)
typedef int (*__compar_fn_t)(const void*, const void*);
#endif

/* Maximum number of route layers */
#define MAX_LAYERS    12

/* Maximum number of all defined layers.  Since masterslice and	*/
/* overlap types are ignored, this just includes all the cuts.	*/
#define MAX_TYPES    (MAX_LAYERS * 2 - 1)

/* Cell name (and other names) max length */
#define MAX_NAME_LEN    1024

/* Max reasonable line length */
#define MAX_LINE_LEN    2048

/* Default configuration filename */
#define  CONFIGFILENAME        "route.cfg" 

// define possible gate orientations

#define MNONE	0
#define MX	1
#define MY	2

// define search directions

#define NORTH	(u_char)1
#define SOUTH	(u_char)2
#define EAST	(u_char)3
#define WEST	(u_char)4
#define UP	(u_char)5
#define DOWN	(u_char)6

// define types of via checkerboard patterns
#define VIA_PATTERN_NONE	-1
#define VIA_PATTERN_NORMAL	 0
#define VIA_PATTERN_INVERT	 1

// linked list structure for holding a list of char * strings

typedef struct linkedstring_ *LinkedStringPtr;

typedef struct linkedstring_ {
   char *name;
   LinkedStringPtr next;
} LinkedString;

// structure holding input and output scalefactors

typedef struct scalerec_ {
   int    iscale;
   int	  mscale;
   double oscale;
} ScaleRec;

// define a structure containing x, y, and layer

typedef struct gridp_ GRIDP;

struct gridp_ {
   int x;
   int y;
   int lay;
   u_int cost;
};

typedef struct proute_ PROUTE;

struct proute_ {        // partial route
   u_short flags; 	// values PR_PROCESSED and PR_CONFLICT, and others
   union {
      u_int cost;	// cost of route coming from predecessor
      u_int net;	// net number at route point
   } prdata;
};

// Bit values for "flags" in PROUTE

#define PR_PRED_DMASK	0x007		// Mask for directional bits

#define PR_PRED_NONE	0x000		// This node does not have a predecessor
#define PR_PRED_N	0x001		// Predecessor is north
#define PR_PRED_S	0x002		// Predecessor is south
#define PR_PRED_E	0x003		// Predecessor is east
#define PR_PRED_W	0x004		// Predecessor is west
#define PR_PRED_U	0x005		// Predecessor is up
#define PR_PRED_D	0x006		// Predecessor is down

#define PR_PROCESSED	0x008		// Tag to avoid visiting more than once
#define PR_NO_EVAL	0x008		// Used only for making calls to eval_pt()
#define PR_CONFLICT	0x010		// Two nets collide here during stage 2
#define PR_SOURCE	0x020		// This is a source node
#define PR_TARGET	0x040		// This is a target node
#define PR_COST		0x080		// if 1, use prdata.cost, not prdata.net
#define PR_ON_STACK	0x100		// if 1, position has been recorded for
					// pending evalutaion

// Linked string list

typedef struct string_ *STRING;

struct string_ {
   STRING next;
   char *name;
};

/* Path segment information */

#define  ST_WIRE		0x01
#define	 ST_VIA			0x02
#define	 ST_OFFSET_START	0x04	/* (x1, y1) is offset from grid */
#define	 ST_OFFSET_END		0x08	/* (x2, y2) is offset from grid */
#define  ST_SPECIAL		0x10	/* wide metal (special net)	*/
#define  ST_MINMETAL		0x20	/* segment for min metal area	*/

typedef struct seg_ *SEG;

struct seg_ {
   SEG next;
   int layer;
   int x1, y1, x2, y2;
   u_char segtype;
};

/* DSEG is like a SEG, but coordinates are in microns (therefore type double)	*/
/* Used for gate node and obstruction positions.				*/

typedef struct dseg_ *DSEG;

struct dseg_ {
   DSEG   next;
   int    layer;
   double x1, y1, x2, y2;
};


/* POINT is an integer point in three dimensions (layer giving the	*/
/* vertical dimension).							*/

typedef struct point_ *POINT;

struct point_ {
  POINT next; 
  int layer;
  int x1, y1;
};

/* DPOINT is a point location with  coordinates given *both* as an	*/
/* integer (for the grid-based routing) and as a physical dimension	*/
/* (microns).								*/

typedef struct dpoint_ *DPOINT;

struct dpoint_ {
   DPOINT next;
   int layer;
   double x, y;
   int gridx, gridy;
};

typedef struct route_ *ROUTE;
typedef struct node_ *NODE;

struct route_ {
   ROUTE  next;
   int    netnum;
   SEG    segments;
   union {
      ROUTE route;
      NODE  node;     
   } start;
   union {
      ROUTE route;
      NODE  node;     
   } end;
   u_char flags;         // See below for flags
};

/* Definitions for flags in struct route_ */

#define RT_OUTPUT	0x01	// Route has been output
#define RT_STUB		0x02	// Route has at least one stub route
#define RT_START_NODE	0x04	// Route starts on a node
#define RT_END_NODE	0x08	// Route ends on a node
#define RT_VISITED	0x10	// Flag for recursive search
#define RT_RIP		0x20	// Flag for route rip-up
#define RT_CHECK	0x40	// Route from DEF file needs checking

/* Structure used to hold nodes, saved nodes, and stub/offset info */

typedef struct nodeinfo_ *NODEINFO;

struct nodeinfo_ {
   NODE  nodesav;
   NODE  nodeloc;
   float stub;		// Stub route to node
   float offset;	// Tap offset
   u_char flags;
};

/* Definitions for flags in stuct nodeinfo_ */

#define NI_STUB_NS	 0x01	// Stub route north(+)/south(-)
#define NI_STUB_EW	 0x02	// Stub route east(+)/west(-)
#define NI_STUB_MASK	 0x03	// Stub route mask (N/S + E/W)
#define NI_OFFSET_NS	 0x04	// Tap offset north(+)/south(-)
#define NI_OFFSET_EW	 0x08	// Tap offset east(+)/west(-)
#define NI_OFFSET_MASK   0x0c	// Tap offset mask (N/S + E/W)
#define NI_NO_VIAX   	 0x10	// Via in ViaX array is prohibited
#define NI_NO_VIAY   	 0x20	// Via in ViaY array is prohibited
#define NI_VIA_X 	 0x40	// Placed via is oriented horizontally
#define NI_VIA_Y 	 0x80	// Placed via is oriented vertically

struct node_ {
  NODE    next;
  int     nodenum;		// node ordering within its net
  DPOINT  taps;			// point position for node taps
  DPOINT  extend;		// point position within halo of the tap
  char    *netname;   		// name of net this node belongs to
  u_char  numtaps;		// number of actual reachable taps
  int     netnum;               // number of net this node belongs to
  int     numnodes;		// number of nodes on this net
  int	  branchx;		// position of the node branch in x
  int	  branchy;		// position of the node branch in y
};

// these are instances of gates in the netlist.  The description of a 
// given gate (the macro) is held in GateInfo.  The same structure is
// used for both the macro and the instance records.

typedef struct gate_ *GATE;

struct gate_ {
    GATE next;
    char *gatename;	// Name of instance
    GATE  gatetype;	// Pointer to macro record
    int   nodes;        // number of nodes on this gate
    char **node;	// names of the pins on this gate
    int   *netnum;	// net number connected to each pin
    NODE  *noderec;	// node record for each pin
    float *area;	// gate area for each pin
    u_char *direction;	// port direction (input, output, etc.)	
    DSEG  *taps;	// list of gate node locations and layers
    DSEG   obs;		// list of obstructions in gate
    double width, height;
    double placedX;                 
    double placedY;
    int orient;
};

// Define record holding information pointing to a gate and the
// index into a specific node of that gate.

typedef struct gatenode_ *GATENODE;

struct gatenode_ {
    GATE gate;
    int idx;
};

// Structure for a network to be routed

typedef struct net_ *NET;
typedef struct netlist_ *NETLIST;

struct net_ {
   int  netnum;		// a unique number for this net
   char *netname;
   NODE netnodes;	// list of nodes connected to the net
   int  numnodes;	// number of nodes connected to the net
   u_char flags;	// flags for this net (see below)
   int  netorder;	// to be assigned by route strategy (critical
			// nets first, then order by number of nodes).
   int  xmin, ymin;	// Bounding box lower left corner
   int  xmax, ymax;	// Bounding box upper right corner
   int  trunkx;		// X position of the net's trunk line (see flags)
   int  trunky;		// Y position of the net's trunk line (see flags)
   NETLIST noripup;	// list of nets that have been ripped up to
			// route this net.  This will not be allowed
			// a second time, to avoid looping.
   ROUTE   routes;	// routes for this net
};

// Flags used by NET "flags" record

#define NET_PENDING  		1	// pending being placed on "abandoned" list
#define NET_CRITICAL 		2	// net is in CriticalNet list
#define NET_IGNORED  		4	// net is ignored by router
#define NET_STUB     		8	// Net has at least one stub
#define NET_VERTICAL_TRUNK	16	// Trunk line is (preferred) vertical

// List of nets, used to maintain a list of failed routes

struct netlist_ {
   NETLIST next;
   NET net;
};

// A structure to hold information about source and target nets for
// a route, to be passed between the route setup and execution stages

struct routeinfo_ {
   NET net;
   ROUTE rt;
   POINT glist[6];	/* Lists of points by priority 0 to 5 */
   NODE nsrc;
   DPOINT nsrctap;
   int maxcost;
   u_char do_pwrbus;
   int pwrbus_src;
   struct seg_ bbox;
};

#define MAXRT		10000000		// "Infinite" route cost

// The following values are added to the Obs[] structure for unobstructed
// route positions close to a terminal, but not close enough to connect
// directly.  They describe which direction to go to reach the terminal.
// The Stub[] vector indicates the distance needed to reach the terminal.
// The OFFSET_TAP flag marks a position that is inside a terminal but
// which needs to be adjusted in one direction to avoid a close obstruction.
// The Stub[] vector indicates the distance needed to avoid the obstruction.
//
// The maximum number of nets must not overrun the area used by flags, so
// the maximum number of nets is 0x3fffff, or 4,194,303 nets

#define OFFSET_TAP	((u_int)0x80000000)  // tap position needs to be offset
#define STUBROUTE	((u_int)0x40000000)  // route stub to reach terminal
#define PINOBSTRUCTMASK	((u_int)0xc0000000)  // either offset tap or stub route
#define NO_NET		((u_int)0x20000000)  // indicates a non-routable obstruction
#define ROUTED_NET	((u_int)0x10000000)  // indicates position occupied by a routed

#define BLOCKED_N	((u_int)0x08000000)  // grid point cannot be routed from the N
#define BLOCKED_S	((u_int)0x04000000)  // grid point cannot be routed from the S
#define BLOCKED_E	((u_int)0x02000000)  // grid point cannot be routed from the E
#define BLOCKED_W	((u_int)0x01000000)  // grid point cannot be routed from the W
#define BLOCKED_U	((u_int)0x00800000)  // grid point cannot be routed from top
#define BLOCKED_D	((u_int)0x00400000)  // grid point cannot be routed from bottom
#define BLOCKED_MASK	((u_int)0x0fc00000)
#define OBSTRUCT_MASK	((u_int)0x0000000f)  // with NO_NET, directional obstruction
#define OBSTRUCT_N	((u_int)0x00000008)  // Tells where the obstruction is
#define OBSTRUCT_S	((u_int)0x00000004)  // relative to the grid point.  Nodeinfo
#define OBSTRUCT_E	((u_int)0x00000002)  // offset contains distance to grid point
#define OBSTRUCT_W	((u_int)0x00000001)
#define MAX_NETNUMS	((u_int)0x003fffff)  // Maximum net number

#define NETNUM_MASK	((u_int)0x203fffff)  // Mask for the net number field
					     // (includes NO_NET)
#define ROUTED_NET_MASK ((u_int)0x303fffff)  // Mask for the net number field
					     // (includes NO_NET and ROUTED_NET)
#define DRC_BLOCKAGE	(NO_NET | ROUTED_NET) // Special case

// Map and draw modes
#define MAP_NONE	0x0	// No map (blank background)
#define MAP_OBSTRUCT	0x1	// Make a map of obstructions and pins
#define MAP_CONGEST	0x2	// Make a map of congestion
#define MAP_ESTIMATE	0x3	// Make a map of estimated congestion
#define MAP_MASK	0x3

#define DRAW_NONE	0x0	// Draw only the background map
#define DRAW_ROUTES	0x4	// Draw routes on top of background map
#define DRAW_UNROUTED   0x8     // Draw unrouted nets on top of background map
#define DRAW_MASK	0xc

// Mask types (values other than 255 are interpreted as "slack" value)
#define MASK_MINIMUM	(u_char)0	// No slack
#define MASK_SMALL	(u_char)1	// Slack of +/-1
#define MASK_MEDIUM	(u_char)2	// Slack of +/-2
#define MASK_LARGE	(u_char)4	// Slack of +/-4
#define MASK_AUTO       (u_char)253	// Choose best mask type
#define MASK_BBOX       (u_char)254	// Mask is simple bounding box
#define MASK_NONE	(u_char)255	// No mask used

// Definitions of bits in needblock
#define ROUTEBLOCKX	(u_char)1	// Block adjacent routes in X
#define ROUTEBLOCKY	(u_char)2	// Block adjacent routes in Y
#define VIABLOCKX	(u_char)4	// Block adjacent vias in X
#define VIABLOCKY	(u_char)8	// Block adjacent vias in Y

// Numnets + MIN_NET_NUMBER is guaranteed to be greater than the highest
// number assigned to a net.
#define MAXNETNUM	(Numnets + MIN_NET_NUMBER)

/* Global variables */

extern STRING  DontRoute;
extern STRING  CriticalNet;
extern NET     CurNet;
extern NETLIST FailedNets;	// nets that have failed the first pass
extern char    *DEFfilename;
extern char    *delayfilename;
extern ScaleRec Scales;

extern GATE   GateInfo;		// standard cell macro information
extern GATE   PinMacro;		// macro definition for a pin
extern GATE   Nlgates;
extern NET    *Nlnets;

extern u_char *RMask;
extern u_int  *Obs[MAX_LAYERS];		// obstructions by layer, y, x
extern PROUTE *Obs2[MAX_LAYERS]; 	// working copy of Obs 
extern float  *Obsinfo[MAX_LAYERS];	// temporary detailed obstruction info
extern NODEINFO *Nodeinfo[MAX_LAYERS];	// stub route distances to pins and
					// pointers to node structures.

#define NODEIPTR(x, y, l) (Nodeinfo[l][OGRID(x, y)])
#define OBSINFO(x, y, l) (Obsinfo[l][OGRID(x, y)])
#define OBSVAL(x, y, l)  (Obs[l][OGRID(x, y)])
#define OBS2VAL(x, y, l) (Obs2[l][OGRID(x, y)])

#define RMASK(x, y)      (RMask[OGRID(x, y)])
#define CONGEST(x, y)	 (Congestion[OGRID(x, y)])

extern DSEG  UserObs;			// user-defined obstruction layers

extern u_char needblock[MAX_LAYERS];

extern int    Numnets;
extern int    Pinlayers;		// Number of layers containing pin info.

extern u_char Verbose;
extern u_char forceRoutable;
extern u_char maskMode;
extern u_char mapType;
extern u_char ripLimit;

extern char *vddnet;
extern char *gndnet;
extern char *antenna_cell;

/* Tcl output to console handling */

#ifdef TCL_QROUTER
   #define Fprintf tcl_printf
   #define Flush   tcl_stdflush
   #define Vprintf tcl_vprintf
#else
   #define Fprintf fprintf
   #define Flush   fflush
   #define Vprintf vfprintf
#endif

/* Function prototypes */

static int next_route_setup(struct routeinfo_ *iroute, u_char stage);
static int route_setup(struct routeinfo_ *iroute, u_char stage);
int route_segs(struct routeinfo_ *iroute, u_char stage, u_char graphdebug);
ROUTE createemptyroute(void);
static void helpmessage(void);

int    set_num_channels(void);
int    allocate_obs_array(void);
int    countlist(NETLIST net);
int    runqrouter(int argc, char *argv[]);
void   remove_failed();
void   apply_drc_blocks(int, double, double);
void   remove_top_route(NET net);

int    read_def(char *filename);

#ifdef TCL_QROUTER
int    write_delays(char *filename);
int    write_spef(char *filename);
#endif

int    dofirststage(u_char graphdebug, int debug_netnum);
int    dosecondstage(u_char graphdebug, u_char singlestep,
		u_char onlybreak, u_int effort);
int    dothirdstage(u_char graphdebug, int debug_netnum, u_int effort);

int    doroute(NET net, u_char stage, u_char graphdebug);
NET    getnettoroute(int order);
int    route_net_ripup(NET net, u_char graphdebug, u_char onlybreak);

#ifdef TCL_QROUTER
void   tcl_printf(FILE *, const char *, ...);
void   tcl_stdflush(FILE *);
void   tcl_vprintf(FILE *, const char *, va_list);
#endif

#define QROUTER_H
#endif 

/* end of qrouter.h */
