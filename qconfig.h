/*--------------------------------------------------------------*/
/* qconfig.h -- general purpose autorouter                     	*/
/* configuration file read/writer       			*/
/*--------------------------------------------------------------*/
/* Written by Steve Beccue, 2003				*/
/*--------------------------------------------------------------*/
/* Modified by Tim Edwards, June 2011, to separate Pitch 	*/
/* information in X and Y dimensions.				*/
/*--------------------------------------------------------------*/

#ifndef QCONFIG_H

extern int     Num_layers;

extern double  PathWidth[MAX_LAYERS];    // width of the paths
extern int     GDSLayer[MAX_TYPES];     // GDS layer number 
extern int     GDSCommentLayer;          // for dummy wires, etc.
extern char    CIFLayer[MAX_TYPES][50]; // CIF layer name 
extern double  PitchX;       		// base horizontal wire pitch
extern double  PitchY;       		// base vertical wire pitch
extern int     NumChannelsX;
extern int     NumChannelsY;
extern int     Vert[MAX_LAYERS];        // 1 if verticle, 0 if horizontal
extern int     Numpasses;               // number of times to iterate in route_segs
extern char    StackedContacts;	  	// Number of vias that can be stacked together

extern double  Xlowerbound;  // Bounding Box of routes
extern double  Xupperbound;      
extern double  Ylowerbound;
extern double  Yupperbound;      

extern int     SegCost;
extern int     ViaCost;
extern int     JogCost;
extern int     XverCost;
extern int     BlockCost;
extern int     OffsetCost;
extern int     ConflictCost;

// If vias are non-square, then they can have up to four orientations,
// with the top and/or bottom metal layers oriented with the longest
// dimension along either the X or the Y axis.

extern char    *ViaXX[MAX_LAYERS];	// Top and bottom horizontal
extern char    *ViaXY[MAX_LAYERS];	// Bottom horizontal, top vertical
extern char    *ViaYX[MAX_LAYERS];	// Bottom vertial, top horizontal
extern char    *ViaYY[MAX_LAYERS];	// Top and bottom vertical

int  read_config(FILE *configfileptr, int is_info);
void post_config(u_char noprint);

#define QCONFIG_H
#endif 

/* end of qconfig.h */
