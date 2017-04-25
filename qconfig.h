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
extern int     GDSLayer[MAX_LAYERS];     // GDS layer number 
extern int     GDSCommentLayer;          // for dummy wires, etc.
extern char    CIFLayer[MAX_LAYERS][50]; // CIF layer name 
extern double  PitchX[MAX_LAYERS];       // horizontal wire pitch of layer
extern double  PitchY[MAX_LAYERS];       // vertical wire pitch of layer
extern int     NumChannelsX[MAX_LAYERS];
extern int     NumChannelsY[MAX_LAYERS];
extern int     Vert[MAX_LAYERS];        // 1 if verticle, 0 if horizontal
extern int     Numpasses;               // number of times to iterate in route_segs
extern char    StackedContacts;	  	// Number of vias that can be stacked together
extern char    ViaPattern;		// Type of via patterning to use

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

extern char    *ViaX[MAX_LAYERS];
extern char    *ViaY[MAX_LAYERS];

int  read_config(FILE *configfileptr, int is_info);
void post_config(void);

#define QCONFIG_H
#endif 

/* end of qconfig.h */
