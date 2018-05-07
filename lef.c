/*
 * lef.c --      
 *
 * This module incorporates the LEF/DEF format for standard-cell routing
 * route.
 *
 * Version 0.1 (September 26, 2003):  LEF input handling.  Includes creation
 * of cells from macro statements, handling of pins, ports, obstructions, and
 * associated geometry.
 *
 * Written by Tim Edwards, Open Circuit Design
 * Modified June 2011 for use with qrouter.
 *
 * It is assumed that the "route.cfg" file has been called prior to this, and
 * so the basic linked lists have been created.  The contents of the LEF file
 * will override anything in the route.cfg file, allowing the route.cfg file
 * to contain a minimum of information, but also allowing for the possibility
 * that there is no LEF file for the technology, and that all such information
 * is in the route.cfg file.
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <sys/time.h>
#include <math.h>

#include "qrouter.h"
#include "node.h"
#include "qconfig.h"
#include "maze.h"
#include "lef.h"

/* ---------------------------------------------------------------------*/

/* Current line number for reading */
int lefCurrentLine = 0;

/* Information about routing layers */
LefList LefInfo = NULL;

/* Information about what vias to use */
LinkedStringPtr AllowedVias = NULL;

/* Gate information is in the linked list GateInfo, imported */

/*---------------------------------------------------------
 * Lookup --
 *	Searches a table of strings to find one that matches a given
 *	string.  It's useful mostly for command lookup.
 *
 *	Only the portion of a string in the table up to the first
 *	blank character is considered significant for matching.
 *
 * Results:
 *	If str is the same as
 *      or an unambiguous abbreviation for one of the entries
 *	in table, then the index of the matching entry is returned.
 *	If str is not the same as any entry in the table, but 
 *      an abbreviation for more than one entry, 
 *	then -1 is returned.  If str doesn't match any entry, then
 *	-2 is returned.  Case differences are ignored.
 *
 * NOTE:  
 *      Table entries need no longer be in alphabetical order
 *      and they need not be lower case.  The irouter command parsing
 *      depends on these features.
 *
 * Side Effects:
 *	None.
 *---------------------------------------------------------
 */

int
Lookup(str, table)
    char *str;			/* Pointer to a string to be looked up */
    char *(table[]);		/* Pointer to an array of string pointers
				 * which are the valid commands.  
				 * The end of
				 * the table is indicated by a NULL string.
				 */
{
    int match = -2;	/* result, initialized to -2 = no match */
    int pos;
    int ststart = 0;

    /* search for match */
    for (pos=0; table[pos] != NULL; pos++)
    {
	char *tabc = table[pos];
	char *strc = &(str[ststart]);
	while (*strc!='\0' && *tabc!=' ' &&
	    ((*tabc==*strc) ||
	     (isupper(*tabc) && islower(*strc) && (tolower(*tabc)== *strc))||
	     (islower(*tabc) && isupper(*strc) && (toupper(*tabc)== *strc)) ))
	{
	    strc++;
	    tabc++;
	}


	if (*strc=='\0') 
	{
	    /* entry matches */
	    if(*tabc==' ' || *tabc=='\0')
	    {
		/* exact match - record it and terminate search */
		match = pos;
		break;
	    }    
	    else if (match == -2)
	    {
		/* inexact match and no previous match - record this one 
		 * and continue search */
		match = pos;
	    }	
	    else
	    {
		/* previous match, so string is ambiguous unless exact
		 * match exists.  Mark ambiguous for now, and continue
		 * search.
		 */
		match = -1;
	    }
	}
    }
    return(match);
}

/*
 * ----------------------------------------------------------------------------
 * LookupFull --
 *
 * Look up a string in a table of pointers to strings.  The last
 * entry in the string table must be a NULL pointer.
 * This is much simpler than Lookup() in that it does not
 * allow abbreviations.  It does, however, ignore case.
 *
 * Results:
 *	Index of the name supplied in the table, or -1 if the name
 *	is not found.
 *
 * Side effects:
 *	None.
 *
 * ----------------------------------------------------------------------------
 */

int
LookupFull(name, table)
    char *name;
    char **table;
{
    char **tp;

    for (tp = table; *tp; tp++)
    {
	if (strcmp(name, *tp) == 0)
	    return (tp - table);
	else
	{
	    char *sptr, *tptr;
	    for (sptr = name, tptr = *tp; ((*sptr != '\0') && (*tptr != '\0'));
			sptr++, tptr++)
		if (toupper(*sptr) != toupper(*tptr))
		    break;
	    if ((*sptr == '\0') && (*tptr == '\0'))
		return (tp - table);
	}
    }

    return (-1);
}


/*
 *------------------------------------------------------------
 *
 * LefNextToken --
 *
 *	Move to the next token in the stream input.
 *	If "ignore_eol" is FALSE, then the end-of-line character
 *	"\n" will be returned as a token when encountered.
 *	Otherwise, end-of-line will be ignored.
 *
 * Results:
 *	Pointer to next token to parse
 *
 * Side Effects:
 *	May read a new line from the specified file.
 *
 * Warnings:
 *	The return result of LefNextToken will be overwritten by
 *	subsequent calls to LefNextToken if more than one line of
 *	input is parsed.
 *
 *------------------------------------------------------------
 */

char *
LefNextToken(FILE *f, u_char ignore_eol)
{
    static char line[LEF_LINE_MAX + 2];	/* input buffer */
    static char *nexttoken = NULL;	/* pointer to next token */
    static char *curtoken;		/* pointer to current token */
    static char eol_token='\n';

    /* Read a new line if necessary */

    if (nexttoken == NULL)
    {
	for(;;)
	{
	    if (fgets(line, LEF_LINE_MAX + 1, f) == NULL) return NULL;
	    lefCurrentLine++;
	    curtoken = line;
	    while (isspace(*curtoken) && (*curtoken != '\n') && (*curtoken != '\0'))
		curtoken++;		/* skip leading whitespace */

	    if ((*curtoken != '#') && (*curtoken != '\n') && (*curtoken != '\0'))
	    {
		nexttoken = curtoken;
		break;
	    }
	}
	if (!ignore_eol)
	    return &eol_token;
    }
    else
	curtoken = nexttoken;

    /* Find the next token; set to NULL if none (end-of-line). */
    /* Treat quoted material as a single token */

    if (*nexttoken == '\"') {
	nexttoken++;
	while (((*nexttoken != '\"') || (*(nexttoken - 1) == '\\')) &&
		(*nexttoken != '\0')) {
	    if (*nexttoken == '\n') { 	
		if (fgets(nexttoken + 1, LEF_LINE_MAX -
				(size_t)(nexttoken - line), f) == NULL)
		    return NULL;
	    }
	    nexttoken++;	/* skip all in quotes (move past current token) */
	}
	if (*nexttoken == '\"')
	    nexttoken++;
    }
    else {
	while (!isspace(*nexttoken) && (*nexttoken != '\0') && (*nexttoken != '\n'))
	    nexttoken++;	/* skip non-whitespace (move past current token) */
    }

    /* Terminate the current token */
    if (*nexttoken != '\0') *nexttoken++ = '\0';

    while (isspace(*nexttoken) && (*nexttoken != '\0') && (*nexttoken != '\n'))
	nexttoken++;	/* skip any whitespace */

    if ((*nexttoken == '#') || (*nexttoken == '\n') || (*nexttoken == '\0'))
	nexttoken = NULL;

    return curtoken;
}

/*
 *------------------------------------------------------------
 *
 * LefError --
 *
 *	Print an error message (via fprintf) giving the line
 *	number of the input file on which the error occurred.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Prints to the output (stderr).
 *
 *------------------------------------------------------------
 */

void
LefError(char *fmt, ...)
{  
    static int errors = 0;
    va_list args;

    if (Verbose == 0) return;

    if (fmt == NULL)  /* Special case:  report any errors and reset */
    {
	if (errors)
	{
	    Fprintf(stdout, "LEF Read: encountered %d error%s total.\n",
			errors, (errors == 1) ? "" : "s");
	    errors = 0;
	}
	return;
    }

    if (errors < LEF_MAX_ERRORS)
    {
	Fprintf(stderr, "LEF Read, Line %d: ", lefCurrentLine);
	va_start(args, fmt);
	Vprintf(stderr, fmt, args);
	va_end(args);
	Flush(stderr);
    }
    else if (errors == LEF_MAX_ERRORS)
	Fprintf(stderr, "LEF Read:  Further errors will not be reported.\n");

    errors++;
}

/*
 *------------------------------------------------------------
 *
 * LefParseEndStatement --
 *
 *	Check if the string passed in "lineptr" contains the
 *	appropriate matching keyword.  Sections in LEF files
 *	should end with "END (keyword)" or "END".  To check
 *	against the latter case, "match" should be NULL.
 *
 * Results:
 *	TRUE if the line matches the expected end statement,
 *	FALSE if not. 
 *
 * Side effects:
 *	None.
 *
 *------------------------------------------------------------
 */

u_char
LefParseEndStatement(FILE *f, char *match)
{
    char *token;
    int keyword;
    char *match_name[2];

    match_name[0] = match;
    match_name[1] = NULL;

    token = LefNextToken(f, (match == NULL) ? FALSE : TRUE);
    if (token == NULL)
    {
	LefError("Bad file read while looking for END statement\n");
	return FALSE;
    }

    /* END or ENDEXT */
    if ((*token == '\n') && (match == NULL)) return TRUE;

    /* END <section_name> */
    else {
	keyword = LookupFull(token, match_name);
	if (keyword == 0)
	    return TRUE;
	else
	    return FALSE;
    }
}

/*
 *------------------------------------------------------------
 *
 * LefSkipSection --
 *
 *	Skip to the "END" record of a LEF input section
 *	String "section" must follow the "END" statement in
 *	the file to be considered a match;  however, if
 *	section = NULL, then "END" must have no strings
 *	following.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Reads input from the specified file.  Prints an
 *	error message if the expected END record cannot
 *	be found.
 *
 *------------------------------------------------------------
 */

void
LefSkipSection(FILE *f, char *section)
{
    char *token;
    int keyword;
    static char *end_section[] = {
	"END",
	"ENDEXT",
	NULL
    };

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	if ((keyword = Lookup(token, end_section)) == 0)
	{
	    if (LefParseEndStatement(f, section))
		return;
	}
	else if (keyword == 1)
	{
	    if (!strcmp(section, "BEGINEXT"))
		return;
	}
    }

    LefError("Section %s has no END record!\n", section);
    return;
}

/*
 *------------------------------------------------------------
 *
 * lefFindCell --
 *
 * 	"name" is the name of the cell to search for.
 *	Returns the GATE entry for the cell from the GateInfo
 *	list.
 *
 *------------------------------------------------------------
 */

GATE
lefFindCell(char *name)
{
    GATE gateginfo;

    for (gateginfo = GateInfo; gateginfo; gateginfo = gateginfo->next) { 
	if (!strcasecmp(gateginfo->gatename, name))
	    return gateginfo;
    }
    return (GATE)NULL;
}

/*
 *------------------------------------------------------------
 *
 * LefLower --
 *
 *	Convert a token in a LEF or DEF file to all-lowercase.
 *
 *------------------------------------------------------------
 */

char *
LefLower(char *token)
{
    char *tptr;

    for (tptr = token; *tptr != '\0'; tptr++)
	*tptr = tolower(*tptr);

    return token;
}

/*
 *------------------------------------------------------------
 * LefRedefined --
 *
 *	In preparation for redefining a LEF layer, we need
 *	to first check if there are multiple names associated
 *	with the lefLayer entry.  If so, split the entry into
 *	two copies, so that the redefinition doesn't affect
 *	the other LEF names.
 *
 * Results:
 *	Pointer to a lefLayer, which may or may not be the
 *	same one presented to the subroutine.
 *	
 * Side Effects:
 *	May add an entry to the list of LEF layers.
 *
 *------------------------------------------------------------
 */

LefList
LefRedefined(LefList lefl, char *redefname)
{
    LefList slef, newlefl;
    char *altName;
    int records;
    DSEG drect;

    /* check if more than one entry points to the same	*/
    /* lefLayer record.	 If so, we will also record the	*/
    /* name of the first type that is not the same as	*/
    /* "redefname".					*/

    records = 0;
    altName = NULL;

    for (slef = LefInfo; slef; slef = slef->next) {
	if (slef == lefl)
	    records++;
	if (altName == NULL)
	    if (strcmp(slef->lefName, redefname))
		altName = (char *)slef->lefName;
    }
    if (records == 1)
    {
	/* Only one name associated with the record, so	*/
	/* just clear all the allocated information.	*/

        while (lefl->info.via.lr) {
	   drect = lefl->info.via.lr->next;
	   free(lefl->info.via.lr);
	   lefl->info.via.lr = drect;
	}
	newlefl = lefl;
    }
    else
    {
	slef = LefFindLayer(redefname);

	newlefl = (LefList)malloc(sizeof(lefLayer));
	newlefl->lefName = strdup(newlefl->lefName);

	newlefl->next = LefInfo;
	LefInfo = newlefl;

	/* If the canonical name of the original entry	*/
	/* is "redefname", then change it.		*/

	if (!strcmp(slef->lefName, redefname))
	    if (altName != NULL)
		slef->lefName = altName;
    }
    newlefl->type = -1;
    newlefl->obsType = -1;
    newlefl->info.via.area.x1 = 0.0;
    newlefl->info.via.area.x2 = 0.0;
    newlefl->info.via.area.y1 = 0.0;
    newlefl->info.via.area.y2 = 0.0;
    newlefl->info.via.area.layer = -1;
    newlefl->info.via.cell = (GATE)NULL;
    newlefl->info.via.lr = (DSEG)NULL;

    return newlefl;
}

/*
 *------------------------------------------------------------
 * Find a layer record in the list of layers
 *------------------------------------------------------------
 */

LefList
LefFindLayer(char *token)
{
    LefList lefl, rlefl;
   
    if (token == NULL) return NULL;
    rlefl = (LefList)NULL;
    for (lefl = LefInfo; lefl; lefl = lefl->next) {
	if (!strcmp(lefl->lefName, token)) {
	   rlefl = lefl;
	   break;
	}
    }
    return rlefl;
}
	
/*
 *------------------------------------------------------------
 * Find a layer record in the list of layers, by layer number
 *------------------------------------------------------------
 */

LefList
LefFindLayerByNum(int layer)
{
    LefList lefl, rlefl;
   
    rlefl = (LefList)NULL;
    for (lefl = LefInfo; lefl; lefl = lefl->next) {
	if (lefl->type == layer) {
	   rlefl = lefl;
	   break;
	}
    }
    return rlefl;
}
	
/*
 *------------------------------------------------------------
 * Find a layer record in the list of layers, and return the
 * layer number.
 *------------------------------------------------------------
 */

int
LefFindLayerNum(char *token)
{
    LefList lefl;

    lefl = LefFindLayer(token);
    if (lefl)
	return lefl->type;
    else
	return -1;
}

/*
 *---------------------------------------------------------------
 * Find the maximum routing layer number defined by the LEF file
 *---------------------------------------------------------------
 */

int
LefGetMaxLayer(void)
{
    int maxlayer = -1;
    LefList lefl;

    for (lefl = LefInfo; lefl; lefl = lefl->next) {
	if (lefl->type > maxlayer)
	    maxlayer = lefl->type;
    }
    return (maxlayer + 1);
}

/*
 *------------------------------------------------------------
 * Return the route keepout area, defined as the route space
 * plus 1/2 the route width.  This is the distance outward
 * from an obstruction edge within which one cannot place a
 * route.
 *
 * If no route layer is defined, then we pick up the value
 * from information in the route.cfg file (if any).  Here
 * we define it as the route pitch less 1/2 the route width,
 * which is the same as above if the route pitch has been
 * chosen for minimum spacing.
 *
 * If all else fails, return zero.
 *------------------------------------------------------------
 */

double
LefGetRouteKeepout(int layer)
{
    LefList lefl;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    return lefl->info.route.width / 2.0
		+ lefl->info.route.spacing->spacing;
	}
    }
    return MIN(PitchX[layer], PitchY[layer]) - PathWidth[layer] / 2.0;
}

/*
 *------------------------------------------------------------
 * Similar routine to the above.  Return the route width for
 * a route layer.  Return value in microns.  If there is no
 * LEF file information about the route width, then return
 * half of the minimum route pitch.
 *------------------------------------------------------------
 */

double
LefGetRouteWidth(int layer)
{
    LefList lefl;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    return lefl->info.route.width;
	}
    }
    return MIN(PitchX[layer], PitchY[layer]) / 2.0;
}

/*
 *------------------------------------------------------------
 * Similar routine to the above.  Return the route offset for
 * a route layer.  Return value in microns.  If there is no
 * LEF file information about the route offset, then return
 * half of the minimum route pitch.
 *------------------------------------------------------------
 */

double
LefGetRouteOffset(int layer)
{
    LefList lefl;
    u_char o;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    o = lefl->info.route.hdirection;
            if (o == TRUE)
	        return lefl->info.route.offsety;
	    else
	        return lefl->info.route.offsetx;
	}
    }
    return MIN(PitchX[layer], PitchY[layer]) / 2.0;
}

double
LefGetRouteOffsetX(int layer)
{
    LefList lefl;
    u_char o;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    return lefl->info.route.offsetx;
	}
    }
    return PitchX[layer] / 2.0;
}

double
LefGetRouteOffsetY(int layer)
{
    LefList lefl;
    u_char o;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    return lefl->info.route.offsety;
	}
    }
    return PitchY[layer] / 2.0;
}

/*
 *------------------------------------------------------------
 * Determine and return the width of a via.  The first layer
 * is the base (lower) layer of the via (e.g., layer 0, or
 * metal1, for via12).  The second layer is the layer for
 * which we want the width rule (e.g., 0 or 1, for metal1
 * or metal2).  If dir = 0, return the side-to-side width,
 * otherwise, return the top-to-bottom width.  This accounts
 * for non-square vias.
 *
 * Note that Via rectangles are stored with x2 dimensions
 * because the center can be on a half-grid position; so,
 * return half the value obtained.
 *
 * This routine always uses a horizontally oriented via if
 * available.  See the specific LefGetXYViaWidth() routine
 * for differentiation between via orientations.
 *------------------------------------------------------------
 */

double
LefGetViaWidth(int base, int layer, int dir)
{
   return LefGetXYViaWidth(base, layer, dir, 0);
}

/*
 *------------------------------------------------------------
 * The base routing used by LefGetViaWidth(), with an
 * additional argument that specifies which via orientation
 * to use, if an alternative orientation is available.  This
 * is necessary for doing checkerboard via patterning and
 * for certain standard cells with ports that do not always
 * fit one orientation of via.
 *------------------------------------------------------------
 */

double
LefGetXYViaWidth(int base, int layer, int dir, int orient)
{
    DSEG lrect;
    LefList lefl;
    double width;
    char **viatable;

    viatable = (orient == 1) ? ViaY : ViaX;

    lefl = LefFindLayer(*(viatable + base));
    if (!lefl) {
	viatable = (orient == 1) ? ViaX : ViaY;
	lefl = LefFindLayer(*(viatable + base));
	viatable = (orient == 1) ? ViaY : ViaX;
    }
    if (!lefl) {
	if (base == (Num_layers - 1))
	    lefl = LefFindLayer(*(viatable + base - 1));
    }
    if (lefl) {
	if (lefl->lefClass == CLASS_VIA) {
	    if (lefl->info.via.area.layer == layer) {
	       if (dir)
		  width = lefl->info.via.area.y2 - lefl->info.via.area.y1;
	       else
		  width = lefl->info.via.area.x2 - lefl->info.via.area.x1;
	       return width / 2.0;
	    }
	    for (lrect = lefl->info.via.lr; lrect; lrect = lrect->next) {
	       if (lrect->layer == layer) {
		  if (dir)
		     width = lrect->y2 - lrect->y1;
		  else
		     width = lrect->x2 - lrect->x1;
	          return width / 2.0;
	       }
	    }
	}
    }
    return MIN(PitchX[layer], PitchY[layer]) / 2.0;	// Best guess
}

/*
 *------------------------------------------------------------
 * And another such routine, for route spacing (minimum width)
 *------------------------------------------------------------
 */

double
LefGetRouteSpacing(int layer)
{
    LefList lefl;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    if (lefl->info.route.spacing)
		return lefl->info.route.spacing->spacing;
	    else
		return 0.0;
	}
    }
    return MIN(PitchX[layer], PitchY[layer]) / 2.0;
}

/*
 *------------------------------------------------------------
 * Find route spacing to a metal layer of specific width
 *------------------------------------------------------------
 */

double
LefGetRouteWideSpacing(int layer, double width)
{
    LefList lefl;
    lefSpacingRule *srule;
    double spacing;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    // Prepare a default in case of bad values
	    spacing = lefl->info.route.spacing->spacing;
	    for (srule = lefl->info.route.spacing; srule; srule = srule->next) {
		if (srule->width > width) break;
		spacing = srule->spacing;
	    }
	    return spacing;
	}
    }
    return MIN(PitchX[layer], PitchY[layer]) / 2.0;
}

/*
 *-----------------------------------------------------------------
 * Get the route pitch in the preferred direction for a given layer
 *-----------------------------------------------------------------
 */

double
LefGetRoutePitch(int layer)
{
    LefList lefl;
    u_char o;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    o = lefl->info.route.hdirection;
            if (o == TRUE)
		return lefl->info.route.pitchy;
	    else
		return lefl->info.route.pitchx;
	}
    }
    return MIN(PitchX[layer], PitchY[layer]);
}

/*
 *------------------------------------------------------------
 * Get the route pitch in X for a given layer
 *------------------------------------------------------------
 */

double
LefGetRoutePitchX(int layer)
{
    LefList lefl;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    return lefl->info.route.pitchx;
	}
    }
    return PitchX[layer];
}

/*
 *------------------------------------------------------------
 * Get the route pitch in Y for a given layer
 *------------------------------------------------------------
 */

double
LefGetRoutePitchY(int layer)
{
    LefList lefl;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    return lefl->info.route.pitchy;
	}
    }
    return PitchY[layer];
}

/*
 *------------------------------------------------------------
 * Get the route name for a given layer
 *------------------------------------------------------------
 */

char *
LefGetRouteName(int layer)
{
    LefList lefl;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    return lefl->lefName;
	}
    }
    return NULL;
}

/*
 *------------------------------------------------------------
 * Get the route orientation for the given layer,
 * where the result is 1 for horizontal, 0 for vertical, and
 * -1 if the layer is not found.
 *------------------------------------------------------------
 */

int
LefGetRouteOrientation(int layer)
{
    LefList lefl;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    return (int)lefl->info.route.hdirection;
	}
    }
    return -1;
}

/*
 *------------------------------------------------------------
 * Get the route resistance and capacitance information.
 * Fill in the pointer values with the relevant information.
 * Return 0 on success, -1 if the layer is not found.
 *------------------------------------------------------------
 */

int
LefGetRouteRCvalues(int layer, double *areacap, double *edgecap,
	double *respersq)
{
    LefList lefl;

    lefl = LefFindLayerByNum(layer);
    if (lefl) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    *areacap = (double)lefl->info.route.areacap;
	    *edgecap = (double)lefl->info.route.edgecap;
	    *respersq = (double)lefl->info.route.respersq;
	    return 0;
	}
    }
    return -1;
}

/*
 *------------------------------------------------------------
 * Get resistance per via for a via layer.
 * Return 0 on success, -1 if the layer is not found.
 * Fill in the pointer value with the resistance.
 *------------------------------------------------------------
 */

int
LefGetViaResistance(int layer, double *respervia)
{
    DSEG lrect;
    LefList lefl;
    double width;
    char **viatable = ViaX;

    lefl = LefFindLayer(*(viatable + layer));
    if (!lefl) {
	viatable = ViaY;
	lefl = LefFindLayer(*(viatable + layer));
    }
    if (lefl) {
	if (lefl->lefClass == CLASS_VIA) {
	    *respervia = (double)lefl->info.via.respervia;
	    return 0;
	}
    }
    return -1;
}

/*
 *------------------------------------------------------------
 * LefReadLayers --
 *
 *	Read a LEF "LAYER" record from the file.
 *	If "obstruct" is TRUE, returns the layer mapping
 *	for obstruction geometry as defined in the
 *	technology file (if it exists), and up to two
 *	types are returned (the second in the 3rd argument
 *	pointer).
 *
 * Results:
 *	Returns layer number or -1 if no matching type is found.
 *
 * Side Effects:
 *	Reads input from file f;
 *
 *------------------------------------------------------------
 */

int
LefReadLayers(f, obstruct, lreturn)
    FILE *f;
    u_char obstruct;
    int *lreturn;
{
    char *token;
    int curlayer = -1;
    LefList lefl = NULL;

    token = LefNextToken(f, TRUE);
    if (*token == ';')
    {
	LefError("Bad Layer statement\n");
	return -1;
    }
    else
    {
	lefl = LefFindLayer(token);
	if (lefl)
	{
	    if (obstruct)
	    {
		/* Use the obstruction type, if it is defined */
		curlayer = lefl->obsType;
		if ((curlayer < 0) && (lefl->lefClass != CLASS_IGNORE))
		    curlayer = lefl->type;
		else if (lefl->lefClass == CLASS_VIA)
		    if (lreturn) *lreturn = lefl->info.via.obsType;
	    }
	    else
	    {
		if (lefl->lefClass != CLASS_IGNORE)
		    curlayer = lefl->type;
	    }
	}
	if ((curlayer < 0) && ((!lefl) || (lefl->lefClass != CLASS_IGNORE)))
	{
	    /* CLASS_VIA in lefl record is a cut, and the layer */
	    /* geometry is ignored for the purpose of routing.	*/

	    if (lefl->lefClass != CLASS_VIA)
		LefError("Don't know how to parse layer \"%s\"\n", token);
	}
    }
    return curlayer;
}

/*
 *------------------------------------------------------------
 * LefReadLayer --
 *
 *	Read a LEF "LAYER" record from the file.
 *	If "obstruct" is TRUE, returns the layer mapping
 *	for obstruction geometry as defined in the
 *	technology file (if it exists).
 *
 * Results:
 *	Returns a layer number or -1 if no match is found.
 *
 * Side Effects:
 *	Reads input from file f;
 *
 *------------------------------------------------------------
 */

int
LefReadLayer(FILE *f, u_char obstruct)
{
    return LefReadLayers(f, obstruct, (int *)NULL);
}

/*
 *------------------------------------------------------------
 * LefReadRect --
 *
 *	Read a LEF "RECT" record from the file, and
 *	return a Rect in micron coordinates.
 *
 * Results:
 *	Returns a pointer to a Rect containing the micron
 *	coordinates, or NULL if an error occurred.
 *
 * Side Effects:
 *	Reads input from file f;
 *
 * Note:
 *	LEF/DEF does NOT define a RECT record as having (...)
 *	pairs, only routes.  However, at least one DEF file
 *	contains this syntax, so it is checked.
 *
 *------------------------------------------------------------
 */

DSEG
LefReadRect(FILE *f, int curlayer, float oscale)
{
    char *token;
    float llx, lly, urx, ury;
    static struct dseg_ paintrect;
    u_char needMatch = FALSE;

    token = LefNextToken(f, TRUE);
    if (*token == '(')
    {
	token = LefNextToken(f, TRUE);
	needMatch = TRUE;
    }
    if (!token || sscanf(token, "%f", &llx) != 1) goto parse_error;
    token = LefNextToken(f, TRUE);
    if (!token || sscanf(token, "%f", &lly) != 1) goto parse_error;
    token = LefNextToken(f, TRUE);
    if (needMatch)
    {
	if (*token != ')') goto parse_error;
	else token = LefNextToken(f, TRUE);
	needMatch = FALSE;
    }
    if (*token == '(')
    {
	token = LefNextToken(f, TRUE);
	needMatch = TRUE;
    }
    if (!token || sscanf(token, "%f", &urx) != 1) goto parse_error;
    token = LefNextToken(f, TRUE);
    if (!token || sscanf(token, "%f", &ury) != 1) goto parse_error;
    if (needMatch)
    {
	token = LefNextToken(f, TRUE);
	if (*token != ')') goto parse_error;
    }
    if (curlayer < 0) {
	/* Issue warning but keep geometry with negative layer number */
	LefError("No layer defined for RECT.\n");
    }

    /* Scale coordinates (microns to centimicrons)	*/
		
    paintrect.x1 = llx / oscale;
    paintrect.y1 = lly / oscale;
    paintrect.x2 = urx / oscale;
    paintrect.y2 = ury / oscale;
    paintrect.layer = curlayer;
    return (&paintrect);

parse_error:
    LefError("Bad port geometry: RECT requires 4 values.\n");
    return (DSEG)NULL;
}

/*
 *------------------------------------------------------------
 * Support routines for polygon reading
 *------------------------------------------------------------
 */

#define HEDGE 0		/* Horizontal edge */
#define REDGE 1		/* Rising edge */
#define FEDGE -1	/* Falling edge */

/*
 *------------------------------------------------------------
 * lefLowX ---
 *
 *	Sort routine to find the lowest X coordinate between
 *	two DPOINT structures passed from qsort()
 *------------------------------------------------------------
 */

int
lefLowX(DPOINT *a, DPOINT *b)
{
    DPOINT p = *a;
    DPOINT q = *b;

    if (p->x < q->x)
	return (-1);
    if (p->x > q->x)
	return (1);
    return (0);
}

/*
 *------------------------------------------------------------
 * lefLowY ---
 *
 *	Sort routine to find the lowest Y coordinate between
 *	two DPOINT structures passed from qsort()
 *------------------------------------------------------------
 */

int
lefLowY(DPOINT *a, DPOINT *b)
{
    DPOINT p = *a;
    DPOINT q = *b;

    if (p->y < q->y)
	return (-1);
    if (p->y > q->y)
	return (1);
    return (0);
}

/*
 *------------------------------------------------------------
 * lefOrient ---
 *
 *	Assign a direction to each of the edges in a polygon.
 *
 * Note that edges have been sorted, but retain the original
 * linked list pointers, from which we can determine the
 * path orientation
 *
 *------------------------------------------------------------
 */

char
lefOrient(DPOINT *edges, int nedges, int *dir)
{
    int n;
    DPOINT p, q;

    for (n = 0; n < nedges; n++)
    {
	p = edges[n];
	q = edges[n]->next;

	if (p->y == q->y)
	{
	    dir[n] = HEDGE;
	    continue;
	}
	if (p->x == q->x)
	{
	    if (p->y < q->y)
	    {
		dir[n] = REDGE;
		continue;
	    }
	    if (p->y > q->y)
	    {
		dir[n] = FEDGE;
		continue;
	    }
	    /* Point connects to itself */
	    dir[n] = HEDGE;
	    continue;
	}
	/* It's not Manhattan, folks. */
	return (FALSE);
    }
    return (TRUE);
}

/*
 *------------------------------------------------------------
 * lefCross ---
 *
 *	See if an edge crosses a particular area.
 *	Return TRUE if edge if vertical and if it crosses the
 *	y-range defined by ybot and ytop.  Otherwise return
 *	FALSE.
 *------------------------------------------------------------
 */

char
lefCross(DPOINT edge, int dir, double ybot, double ytop)
{
    double ebot, etop;

    switch (dir)
    {
	case REDGE:
	    ebot = edge->y;
	    etop = edge->next->y;
	    return (ebot <= ybot && etop >= ytop);

	case FEDGE:
	    ebot = edge->next->y;
	    etop = edge->y;
	    return (ebot <= ybot && etop >= ytop);
    }
    return (FALSE);
}

	
/*
 *------------------------------------------------------------
 * LefPolygonToRects --
 *
 *	Convert Geometry information from a POLYGON statement
 *	into rectangles.  NOTE:  For now, this routine
 *	assumes that all points are Manhattan.  It will flag
 *	non-Manhattan geometry
 *
 *	the DSEG pointed to by rectListPtr is updated by
 *	having the list of rectangles appended to it.
 *
 *------------------------------------------------------------
 */

void
LefPolygonToRects(DSEG *rectListPtr, DPOINT pointlist)
{
   DPOINT ptail, p, *pts, *edges;
   DSEG rtail, rex, new;
   int npts = 0;
   int *dir;
   int curr, wrapno, n;
   double xbot, xtop, ybot, ytop;

   if (pointlist == NULL) return;

   /* Close the path by duplicating 1st point if necessary */

   for (ptail = pointlist; ptail->next; ptail = ptail->next);

   if ((ptail->x != pointlist->x) || (ptail->y != pointlist->y))
   {
	p = (DPOINT)malloc(sizeof(struct dpoint_));
	p->x = pointlist->x;
	p->y = pointlist->y;
	p->layer = pointlist->layer;
	p->next = NULL;
	ptail->next = p;
    }

    // To do:  Break out non-manhattan parts here.
    // See CIFMakeManhattanPath in magic-8.0

    rex = NULL;
    for (p = pointlist; p->next; p = p->next, npts++);
    pts = (DPOINT *)malloc(npts * sizeof(DPOINT));
    edges = (DPOINT *)malloc(npts * sizeof(DPOINT));
    dir = (int *)malloc(npts * sizeof(int));
    npts = 0;

    for (p = pointlist; p->next; p = p->next, npts++)
    {
	// pts and edges are two lists of pointlist entries
	// that are NOT linked lists and can be shuffled
	// around by qsort().  The linked list "next" pointers
	// *must* be retained.

	pts[npts] = p;
	edges[npts] = p;
    }

    if (npts < 4)
    {
	LefError("Polygon with fewer than 4 points.\n");
	goto done;
    }

    /* Sort points by low y, edges by low x */
    qsort((char *)pts, npts, (int)sizeof(DPOINT), (__compar_fn_t)lefLowY);
    qsort((char *)edges, npts, (int)sizeof(DPOINT), (__compar_fn_t)lefLowX);

    /* Find out which direction each edge points */

    if (!lefOrient(edges, npts, dir))
    {
	LefError("I can't handle non-manhattan polygons!\n");
	goto done;
    }

    /* Scan the polygon from bottom to top.  At each step, process
     * a minimum-sized y-range of the polygon (i.e., a range such that
     * there are no vertices inside the range).  Use wrap numbers
     * based on the edge orientations to determine how much of the
     * x-range for this y-range should contain material.
     */

    for (curr = 1; curr < npts; curr++)
    {
	/* Find the next minimum-sized y-range. */

	ybot = pts[curr - 1]->y;
	while (ybot == pts[curr]->y)
	    if (++curr >= npts) goto done;
	ytop = pts[curr]->y;

	/* Process all the edges that cross the y-range, from left
	 * to right.
	 */

	for (wrapno = 0, n = 0; n < npts; n++)
	{
	    if (wrapno == 0) xbot = edges[n]->x;
	    if (!lefCross(edges[n], dir[n], ybot, ytop))
		continue;
	    wrapno += (dir[n] == REDGE) ? 1 : -1;
	    if (wrapno == 0)
	    {
		xtop = edges[n]->x;
		if (xbot == xtop) continue;
		new = (DSEG)malloc(sizeof(struct dseg_));
		new->x1 = xbot;
		new->x2 = xtop;
		new->y1 = ybot;
		new->y2 = ytop;
		new->layer = edges[n]->layer;
		new->next = rex;
		rex = new;
	    }
	}
    }

done:
    free(edges);
    free(dir);
    free(pts);

    if (*rectListPtr == NULL)
	*rectListPtr = rex;
    else
    {
	for (rtail = *rectListPtr; rtail->next; rtail = rtail->next);
	rtail->next = rex;
    }
}

/*
 *------------------------------------------------------------
 * LefReadPolygon --
 *
 *	Read Geometry information from a POLYGON statement
 *
 *------------------------------------------------------------
 */

DPOINT
LefReadPolygon(FILE *f, int curlayer, float oscale)
{
    DPOINT plist = NULL, newPoint;
    char *token;
    double px, py;

    while (1)
    {
	token = LefNextToken(f, TRUE);
	if (token == NULL || *token == ';') break;
	if (sscanf(token, "%lg", &px) != 1)
	{
	    LefError("Bad X value in polygon.\n");
	    LefEndStatement(f);
	    break;
	}

	token = LefNextToken(f, TRUE);
	if (token == NULL || *token == ';')
	{
	    LefError("Missing Y value in polygon point!\n");
	    break;
	}
	if (sscanf(token, "%lg", &py) != 1)
	{
	    LefError("Bad Y value in polygon.\n");
	    LefEndStatement(f);
	    break;
	}

	newPoint = (DPOINT)malloc(sizeof(struct dpoint_));
	newPoint->x = px / (double)oscale;
	newPoint->y = py / (double)oscale;
	newPoint->layer = curlayer;
	newPoint->next = plist;
	plist = newPoint;
    }

    return plist;
}

/*
 *------------------------------------------------------------
 * LefReadGeometry --
 *
 *	Read Geometry information from a LEF file.
 *	Used for PORT records and OBS statements.
 *
 * Results:
 *	Returns a linked list of all areas and types
 *	painted.
 *
 * Side Effects:
 *	Reads input from file f;
 *	Paints into the GATE lefMacro.
 *
 *------------------------------------------------------------
 */

enum lef_geometry_keys {LEF_LAYER = 0, LEF_WIDTH, LEF_PATH,
	LEF_RECT, LEF_POLYGON, LEF_VIA, LEF_PORT_CLASS,
	LEF_GEOMETRY_END};

DSEG
LefReadGeometry(GATE lefMacro, FILE *f, float oscale)
{
    int curlayer = -1, otherlayer = -1;

    char *token;
    int keyword;
    DSEG rectList = (DSEG)NULL;
    DSEG paintrect, newRect;
    DPOINT pointlist;

    static char *geometry_keys[] = {
	"LAYER",
	"WIDTH",
	"PATH",
	"RECT",
	"POLYGON",
	"VIA",
	"CLASS",
	"END",
	NULL
    };

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, geometry_keys);
	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in LEF file; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}
	switch (keyword)
	{
	    case LEF_LAYER:
		curlayer = LefReadLayers(f, FALSE, &otherlayer);
		LefEndStatement(f);
		break;
	    case LEF_WIDTH:
		LefEndStatement(f);
		break;
	    case LEF_PATH:
		LefEndStatement(f);
		break;
	    case LEF_RECT:
		paintrect = (curlayer < 0) ? NULL : LefReadRect(f, curlayer, oscale);
		if (paintrect)
		{
		    /* Remember the area and layer */
		    newRect = (DSEG)malloc(sizeof(struct dseg_));
		    *newRect = *paintrect;
		    newRect->next = rectList;
		    rectList = newRect;
		}
		LefEndStatement(f);
		break;
	    case LEF_POLYGON:
		pointlist = LefReadPolygon(f, curlayer, oscale);
		LefPolygonToRects(&rectList, pointlist);
		break;
	    case LEF_VIA:
		LefEndStatement(f);
		break;
	    case LEF_PORT_CLASS:
		LefEndStatement(f);
		break;
	    case LEF_GEOMETRY_END:
		if (!LefParseEndStatement(f, NULL))
		{
		    LefError("Geometry (PORT or OBS) END statement missing.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == LEF_GEOMETRY_END) break;
    }
    return rectList;
}

/*
 *------------------------------------------------------------
 * LefReadPort --
 *
 *	A wrapper for LefReadGeometry, which adds a label
 *	to the last rectangle generated by the geometry
 *	parsing.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Reads input from file f;
 *	Paints into the GATE lefMacro.
 *
 *------------------------------------------------------------
 */

void
LefReadPort(lefMacro, f, pinName, pinNum, pinDir, pinUse, oscale)
    GATE lefMacro;
    FILE *f;
    char *pinName;
    int pinNum, pinDir, pinUse;
    float oscale;
{
    DSEG rectList, rlist;

    rectList = LefReadGeometry(lefMacro, f, oscale);

    if (pinNum >= 0) {
        int nodealloc, orignodes;

	if (lefMacro->nodes <= pinNum) {
            orignodes = lefMacro->nodes;
	    lefMacro->nodes = (pinNum + 1);
            nodealloc = lefMacro->nodes / 10;
            if (nodealloc > (orignodes / 10)) {
		nodealloc++;
		lefMacro->taps = (DSEG *)realloc(lefMacro->taps,
			nodealloc * 10 * sizeof(DSEG));
		lefMacro->noderec = (NODE *)realloc(lefMacro->noderec,
			nodealloc * 10 * sizeof(NODE));
		lefMacro->direction = (u_char *)realloc(lefMacro->direction,
			nodealloc * 10 * sizeof(u_char));
		lefMacro->netnum = (int *)realloc(lefMacro->netnum,
			nodealloc * 10 * sizeof(int));
		lefMacro->node = (char **)realloc(lefMacro->node,
			nodealloc * 10 * sizeof(char *));
            } 
        }
	lefMacro->taps[pinNum] = rectList;
	lefMacro->noderec[pinNum] = NULL;
	lefMacro->direction[pinNum] = pinDir;
	lefMacro->netnum[pinNum] = -1;
        if (pinName != NULL)
            lefMacro->node[pinNum] = strdup(pinName);
        else
	    lefMacro->node[pinNum] = NULL;
    }
    else {
       while (rectList) {
	  rlist = rectList->next;
	  free(rectList);
	  rectList = rlist;
       }
    }
}

/*
 *------------------------------------------------------------
 * LefReadPin --
 *
 *	Read a PIN statement from a LEF file.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Reads input from file f;
 *	Paints into the GATE lefMacro.
 *
 *------------------------------------------------------------
 */

enum lef_pin_keys {LEF_DIRECTION = 0, LEF_USE, LEF_PORT, LEF_CAPACITANCE,
	LEF_ANTENNADIFF, LEF_ANTENNAGATE, LEF_ANTENNAMOD,
	LEF_ANTENNAPAR, LEF_ANTENNAPARSIDE, LEF_ANTENNAMAX, LEF_ANTENNAMAXSIDE,
	LEF_SHAPE, LEF_NETEXPR, LEF_PIN_END};

void
LefReadPin(lefMacro, f, pinname, pinNum, oscale)
   GATE lefMacro;
   FILE *f;
   char *pinname;
   int pinNum;
   float oscale;
{
    char *token;
    int keyword, subkey;
    int pinDir = PORT_CLASS_DEFAULT;
    int pinUse = PORT_USE_DEFAULT;

    static char *pin_keys[] = {
	"DIRECTION",
	"USE",
	"PORT",
	"CAPACITANCE",
	"ANTENNADIFFAREA",
	"ANTENNAGATEAREA",
	"ANTENNAMODEL",
	"ANTENNAPARTIALMETALAREA",
	"ANTENNAPARTIALMETALSIDEAREA",
	"ANTENNAMAXAREACAR",
	"ANTENNAMAXSIDEAREACAR",
	"SHAPE",
	"NETEXPR",
	"END",
	NULL
    };

    static char *pin_classes[] = {
	"DEFAULT",
	"INPUT",
	"OUTPUT",
	"OUTPUT TRISTATE",
	"INOUT",
	"FEEDTHRU",
	NULL
    };

    static int lef_class_to_bitmask[] = {
	PORT_CLASS_DEFAULT,
	PORT_CLASS_INPUT,
	PORT_CLASS_OUTPUT,
	PORT_CLASS_TRISTATE,
	PORT_CLASS_BIDIRECTIONAL,
	PORT_CLASS_FEEDTHROUGH
    };

    static char *pin_uses[] = {
	"DEFAULT",
	"SIGNAL",
	"ANALOG",
	"POWER",
	"GROUND",
	"CLOCK",
	"TIEOFF",
	"ANALOG",
	"SCAN",
	"RESET",
	NULL
    };

    static int lef_use_to_bitmask[] = {
	PORT_USE_DEFAULT,
	PORT_USE_SIGNAL,
	PORT_USE_ANALOG,
	PORT_USE_POWER,
	PORT_USE_GROUND,
	PORT_USE_CLOCK
    };

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, pin_keys);
	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in LEF file; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}
	switch (keyword)
	{
	    case LEF_DIRECTION:
		token = LefNextToken(f, TRUE);
		subkey = Lookup(token, pin_classes);
		if (subkey < 0)
		    LefError("Improper DIRECTION statement\n");
		else
		    pinDir = lef_class_to_bitmask[subkey];
		LefEndStatement(f);
		break;
	    case LEF_USE:
		token = LefNextToken(f, TRUE);
		subkey = Lookup(token, pin_uses);
		if (subkey < 0)
		    LefError("Improper USE statement\n");
		else
		    pinUse = lef_use_to_bitmask[subkey];
		LefEndStatement(f);
		break;
	    case LEF_PORT:
		LefReadPort(lefMacro, f, pinname, pinNum, pinDir, pinUse, oscale);
		break;
	    case LEF_CAPACITANCE:
	    case LEF_ANTENNADIFF:
	    case LEF_ANTENNAGATE:
	    case LEF_ANTENNAMOD:
	    case LEF_ANTENNAPAR:
	    case LEF_ANTENNAPARSIDE:
	    case LEF_ANTENNAMAX:
	    case LEF_ANTENNAMAXSIDE:
	    case LEF_NETEXPR:
	    case LEF_SHAPE:
		LefEndStatement(f);	/* Ignore. . . */
		break;
	    case LEF_PIN_END:
		if (!LefParseEndStatement(f, pinname))
		{
		    LefError("Pin END statement missing.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == LEF_PIN_END) break;
    }
}

/*
 *------------------------------------------------------------
 * LefEndStatement --
 *
 *	Read file input to EOF or a ';' token (end-of-statement)
 *	If we encounter a quote, make sure we don't terminate
 *	the statement on a semicolon that is part of the
 *	quoted material.
 *
 *------------------------------------------------------------
 */

void
LefEndStatement(FILE *f)
{
    char *token;

    while ((token = LefNextToken(f, TRUE)) != NULL)
	if (*token == ';') break;
}

/*
 *------------------------------------------------------------
 *
 * LefReadMacro --
 *
 *	Read in a MACRO section from a LEF file.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Creates a new cell definition in the database.
 *
 *------------------------------------------------------------
 */

enum lef_macro_keys {LEF_CLASS = 0, LEF_SIZE, LEF_ORIGIN,
	LEF_SYMMETRY, LEF_SOURCE, LEF_SITE, LEF_PIN, LEF_OBS,
	LEF_TIMING, LEF_FOREIGN, LEF_MACRO_END};

void
LefReadMacro(f, mname, oscale)
    FILE *f;			/* LEF file being read	*/
    char *mname;		/* name of the macro 	*/
    float oscale;		/* scale factor to um, usually 1 */
{
    GATE lefMacro, altMacro;
    char *token, tsave[128];
    int keyword, pinNum;
    float x, y;
    u_char has_size, is_imported = FALSE;
    struct dseg_ lefBBox;

    static char *macro_keys[] = {
	"CLASS",
	"SIZE",
	"ORIGIN",
	"SYMMETRY",
	"SOURCE",
	"SITE",
	"PIN",
	"OBS",
	"TIMING",
	"FOREIGN",
	"END",
	NULL
    };

    /* Start by creating a new celldef */

    lefMacro = (GATE)NULL;
    for (altMacro = GateInfo; altMacro; altMacro = altMacro->next)
    {
	if (!strcmp(altMacro->gatename, mname)) {
	    lefMacro = altMacro;
	    break;
	}
    }

    while (lefMacro)
    {
	int suffix;
	char newname[256];

	altMacro = lefMacro;
	for (suffix = 1; altMacro != NULL; suffix++)
	{
	    sprintf(newname, "%250s_%d", mname, suffix);
	    for (altMacro = GateInfo; altMacro; altMacro = altMacro->next)
		if (!strcmp(altMacro->gatename, newname))
		    break;
	}
	LefError("Cell \"%s\" was already defined in this file.  "
		"Renaming original cell \"%s\"\n", mname, newname);

	lefMacro->gatename = strdup(newname);
	lefMacro = lefFindCell(mname);
    }

    // Create the new cell
    lefMacro = (GATE)malloc(sizeof(struct gate_));
    lefMacro->gatename = strdup(mname);
    lefMacro->gatetype = NULL;
    lefMacro->width = 0.0;
    lefMacro->height = 0.0;
    lefMacro->placedX = 0.0;
    lefMacro->placedY = 0.0;
    lefMacro->obs = (DSEG)NULL;
    lefMacro->next = GateInfo;
    lefMacro->nodes = 0;
    lefMacro->orient = 0;
    // Allocate memory for up to 10 pins initially
    lefMacro->taps = (DSEG *)malloc(10 * sizeof(DSEG));
    lefMacro->noderec = (NODE *)malloc(10 * sizeof(NODE));
    lefMacro->direction = (u_char *)malloc(10 * sizeof(u_char));
    lefMacro->netnum = (int *)malloc(10 * sizeof(int));
    lefMacro->node = (char **)malloc(10 * sizeof(char *));
    // Fill in 1st entry
    lefMacro->taps[0] = NULL;
    lefMacro->noderec[0] = NULL;
    lefMacro->node[0] = NULL;
    lefMacro->netnum[0] = -1;
    GateInfo = lefMacro;


    /* Initial values */
    pinNum = 0;
    has_size = FALSE;
    lefBBox.x2 = lefBBox.x1 = 0.0;
    lefBBox.y2 = lefBBox.y1 = 0.0;

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, macro_keys);
	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in LEF file; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}
	switch (keyword)
	{
	    case LEF_CLASS:
		token = LefNextToken(f, TRUE);
		if (*token != '\n')
		    // DBPropPut(lefMacro, "LEFclass", token);
		    ;
		LefEndStatement(f);
		break;
	    case LEF_SIZE:
		token = LefNextToken(f, TRUE);
		if (!token || sscanf(token, "%f", &x) != 1) goto size_error;
		token = LefNextToken(f, TRUE);		/* skip keyword "BY" */
		if (!token) goto size_error;
		token = LefNextToken(f, TRUE);
		if (!token || sscanf(token, "%f", &y) != 1) goto size_error;

		lefBBox.x2 = x + lefBBox.x1;
		lefBBox.y2 = y + lefBBox.y1;
		has_size = TRUE;
		LefEndStatement(f);
		break;
size_error:
		LefError("Bad macro SIZE; requires values X BY Y.\n");
		LefEndStatement(f);
		break;
	    case LEF_ORIGIN:
		token = LefNextToken(f, TRUE);
		if (!token || sscanf(token, "%f", &x) != 1) goto origin_error;
		token = LefNextToken(f, TRUE);
		if (!token || sscanf(token, "%f", &y) != 1) goto origin_error;

		lefBBox.x1 = -x;
		lefBBox.y1 = -y;
		if (has_size)
		{
		    lefBBox.x2 += lefBBox.x1;
		    lefBBox.y2 += lefBBox.y1;
		}
		LefEndStatement(f);
		break;
origin_error:
		LefError("Bad macro ORIGIN; requires 2 values.\n");
		LefEndStatement(f);
		break;
	    case LEF_SYMMETRY:
		token = LefNextToken(f, TRUE);
		if (*token != '\n')
		    // DBPropPut(lefMacro, "LEFsymmetry", token + strlen(token) + 1);
		    ;
		LefEndStatement(f);
		break;
	    case LEF_SOURCE:
		token = LefNextToken(f, TRUE);
		if (*token != '\n')
		    // DBPropPut(lefMacro, "LEFsource", token);
		    ;
		LefEndStatement(f);
		break;
	    case LEF_SITE:
		token = LefNextToken(f, TRUE);
		if (*token != '\n')
		    // DBPropPut(lefMacro, "LEFsite", token);
		    ;
		LefEndStatement(f);
		break;
	    case LEF_PIN:
		token = LefNextToken(f, TRUE);
		/* Diagnostic */
		/*
		Fprintf(stdout, "   Macro defines pin %s\n", token);
		*/
		sprintf(tsave, "%.127s", token);
		if (is_imported)
		    LefSkipSection(f, tsave);
		else
		    LefReadPin(lefMacro, f, tsave, pinNum++, oscale);
		break;
	    case LEF_OBS:
		/* Diagnostic */
		/*
		Fprintf(stdout, "   Macro defines obstruction\n");
		*/
		if (is_imported)
		    LefSkipSection(f, NULL);
		else 
		    lefMacro->obs = LefReadGeometry(lefMacro, f, oscale);
		break;
	    case LEF_TIMING:
		LefSkipSection(f, macro_keys[LEF_TIMING]);
		break;
	    case LEF_FOREIGN:
		LefEndStatement(f);
		break;
	    case LEF_MACRO_END:
		if (!LefParseEndStatement(f, mname))
		{
		    LefError("Macro END statement missing.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == LEF_MACRO_END) break;
    }

    /* Finish up creating the cell */

    if (lefMacro) {
	if (has_size) {
	    lefMacro->width = (lefBBox.x2 - lefBBox.x1);
	    lefMacro->height = (lefBBox.y2 - lefBBox.y1);

	    /* "placed" for macros (not instances) corresponds to the	*/
	    /* cell origin.						*/

	    lefMacro->placedX = lefBBox.x1;
	    lefMacro->placedY = lefBBox.y1;
	}
	else {
	    LefError("Gate %s has no size information!\n", lefMacro->gatename);
	}
    }
}

/*
 *------------------------------------------------------------
 *
 * LefAddViaGeometry --
 *
 *	Read in geometry for a VIA section from a LEF or DEF
 *	file.
 *
 *	f		LEF file being read
 *	lefl		pointer to via info
 *	curlayer	current tile type
 *	oscale		output scaling
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Adds to the lefLayer record for a via definition.
 *
 *------------------------------------------------------------
 */

void
LefAddViaGeometry(FILE *f, LefList lefl, int curlayer, float oscale)
{
    DSEG currect;
    DSEG viarect;

    /* Rectangles for vias are read in units of 1/2 lambda */
    currect = LefReadRect(f, curlayer, (oscale / 2));
    if (currect == NULL) return;

    /* First rect goes into info.via.area, others go into info.via.lr */
    if (lefl->info.via.area.layer < 0)
    {
	lefl->info.via.area = *currect;
    }
    else 
    {
	viarect = (DSEG)malloc(sizeof(struct dseg_));
	*viarect = *currect;
	viarect->next = lefl->info.via.lr;
	lefl->info.via.lr = viarect;
    }
}

/*
 *------------------------------------------------------------
 *
 * LefReadLayerSection --
 *
 *	Read in a LAYER, VIA, or VIARULE section from a LEF file.
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Adds to the LEF layer info hash table.
 *
 *------------------------------------------------------------
 */

enum lef_layer_keys {LEF_LAYER_TYPE=0, LEF_LAYER_WIDTH,
	LEF_LAYER_MAXWIDTH, LEF_LAYER_AREA,
	LEF_LAYER_SPACING, LEF_LAYER_SPACINGTABLE,
	LEF_LAYER_PITCH, LEF_LAYER_DIRECTION, LEF_LAYER_OFFSET,
	LEF_LAYER_WIREEXT,
	LEF_LAYER_RES, LEF_LAYER_CAP, LEF_LAYER_EDGECAP,
	LEF_LAYER_THICKNESS, LEF_LAYER_HEIGHT,
	LEF_LAYER_MINDENSITY, LEF_LAYER_ANTENNADIFF,
	LEF_LAYER_ANTENNASIDE,
	LEF_VIA_DEFAULT, LEF_VIA_LAYER, LEF_VIA_RECT,
	LEF_VIA_ENCLOSURE, LEF_VIA_PREFERENCLOSURE,
	LEF_VIARULE_OVERHANG,
	LEF_VIARULE_METALOVERHANG, LEF_VIARULE_VIA,
	LEF_VIARULE_GENERATE, LEF_LAYER_END};

enum lef_spacing_keys {LEF_SPACING_RANGE=0, LEF_END_LAYER_SPACING};

void
LefReadLayerSection(f, lname, mode, lefl)
    FILE *f;			/* LEF file being read	  */
    char *lname;		/* name of the layer 	  */
    int mode;			/* layer, via, or viarule */
    LefList lefl;		/* pointer to layer info  */
{
    char *token, *tp;
    int keyword, typekey = -1, entries, i;
    struct seg_ viaArea;
    int curlayer = -1;
    double dvalue, oscale;
    lefSpacingRule *newrule = NULL, *testrule;

    /* These are defined in the order of CLASS_* in lefInt.h */
    static char *layer_type_keys[] = {
	"ROUTING",
	"CUT",
	"MASTERSLICE",
	"OVERLAP",
	NULL
    };

    static char *layer_keys[] = {
	"TYPE",
	"WIDTH",
	"MAXWIDTH",
	"AREA",
	"SPACING",
	"SPACINGTABLE",
	"PITCH",
	"DIRECTION",
	"OFFSET",
	"WIREEXTENSION",
	"RESISTANCE",
	"CAPACITANCE",
	"EDGECAPACITANCE",
	"THICKNESS",
	"HEIGHT",
	"MINIMUMDENSITY",
	"ANTENNADIFFAREARATIO",
	"ANTENNASIDEAREARATIO",
	"DEFAULT",
	"LAYER",
	"RECT",
	"ENCLOSURE",
	"PREFERENCLOSURE",
	"OVERHANG",
	"METALOVERHANG",
	"VIA",
	"GENERATE",
	"END",
	NULL
    };

    static char *spacing_keys[] = {
	"RANGE",
	";",
	NULL
    };

    /* Database is assumed to be in microns.			*/
    /* If not, we need to parse the UNITS record, which is	*/
    /* currently ignored.					*/

    oscale = 1;
    viaArea.x1 = viaArea.x2 = 0;
    viaArea.y1 = viaArea.y2 = 0;
    viaArea.layer = -1;

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, layer_keys);
	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in LEF file; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}
	switch (keyword)
	{
	    case LEF_LAYER_TYPE:
		token = LefNextToken(f, TRUE);
		if (*token != '\n')
		{
		    typekey = Lookup(token, layer_type_keys);
		    if (typekey < 0)
			LefError("Unknown layer type \"%s\" in LEF file; "
				"ignoring.\n", token);
		}
		if (lefl->lefClass == CLASS_IGNORE) {
		    lefl->lefClass = typekey;
		    if (typekey == CLASS_ROUTE) {

			lefl->info.route.width = 0.0;
			lefl->info.route.spacing = NULL;
			lefl->info.route.pitchx = 0.0;
			lefl->info.route.pitchy = 0.0;
			// Use -1.0 as an indication that offset has not
			// been specified and needs to be set to default.
			lefl->info.route.offsetx = -1.0;
			lefl->info.route.offsety = -1.0;
			lefl->info.route.hdirection = (u_char)0;

			/* A routing type has been declared.  Assume	*/
			/* this takes the name "metal1", "M1", or some	*/
			/* variant thereof.				*/

		        for (tp = lefl->lefName; *tp != '\0'; tp++) {
			    if (*tp >= '0' && *tp <= '9') {
				sscanf(tp, "%d", &lefl->type);

				/* "metal1", e.g., is assumed to be layer #0 */
				/* This may not be a proper assumption, always */

				lefl->type--;
				break;
			    }
			}

			/* This is probably some special non-numerical 	*/
			/* name for a top metal layer.  Take a stab at	*/
			/* it, defining it to be the next layer up from	*/
			/* whatever the previous topmost route layer	*/
			/* was.  This should work unless the LEF file	*/
			/* is really weirdly written.			*/

			if (lefl->type < 0) {
			    lefl->type = LefGetMaxLayer();
			}
		    }
		    else if (typekey == CLASS_VIA) {
			lefl->info.via.area.x1 = 0.0;
			lefl->info.via.area.y1 = 0.0;
			lefl->info.via.area.x2 = 0.0;
			lefl->info.via.area.y2 = 0.0;
			lefl->info.via.area.layer = -1;
			lefl->info.via.cell = (GATE)NULL;
			lefl->info.via.lr = (DSEG)NULL;
		    }
		}
		else if (lefl->lefClass != typekey) {
		    LefError("Attempt to reclassify layer %s from %s to %s\n",
				lname, layer_type_keys[lefl->lefClass],
				layer_type_keys[typekey]);
		}
		LefEndStatement(f);
		break;
	    case LEF_LAYER_WIDTH:
		token = LefNextToken(f, TRUE);
		sscanf(token, "%lg", &dvalue);
		lefl->info.route.width = dvalue / (double)oscale;
		LefEndStatement(f);
		break;
	    case LEF_LAYER_MAXWIDTH:
		// Not handled.
		LefEndStatement(f);
		break;
	    case LEF_LAYER_AREA:
		// Not handled, but needed!
		LefEndStatement(f);
		break;
	    case LEF_LAYER_SPACING:
		token = LefNextToken(f, TRUE);
		sscanf(token, "%lg", &dvalue);
		token = LefNextToken(f, TRUE);
		typekey = Lookup(token, spacing_keys);

		newrule = (lefSpacingRule *)malloc(sizeof(lefSpacingRule));

		// If no range specified, then the rule goes in front
		if (typekey != LEF_SPACING_RANGE) {
		    newrule->spacing = dvalue / (double)oscale;
		    newrule->width = 0.0;
		    newrule->next = lefl->info.route.spacing;
		    lefl->info.route.spacing = newrule;
		}
		else {
		    // Get range minimum, ignore range maximum, and sort
		    // the spacing order.
		    newrule->spacing = dvalue / (double)oscale;
		    token = LefNextToken(f, TRUE);
		    sscanf(token, "%lg", &dvalue);
		    newrule->width = dvalue / (double)oscale;
		    for (testrule = lefl->info.route.spacing; testrule;
				testrule = testrule->next)
			if (testrule->next == NULL || testrule->next->width >
				newrule->width)
			    break;

		    if (!testrule) {
			newrule->next = NULL;
			lefl->info.route.spacing = newrule;
		    }
		    else {
			newrule->next = testrule->next;
			testrule->next = newrule;
		    }
		    token = LefNextToken(f, TRUE);
		    typekey = Lookup(token, spacing_keys);
		}
		if (typekey != LEF_END_LAYER_SPACING)
		    LefEndStatement(f);
		break;

	    case LEF_LAYER_SPACINGTABLE:
		// Use the values for the maximum parallel runlength
		token = LefNextToken(f, TRUE);	// "PARALLELRUNLENTTH"
		entries = 0;
		while (1) {
		    token = LefNextToken(f, TRUE);
		    if (*token == ';' || !strcmp(token, "WIDTH"))
			break;
		    else
			entries++;
		}
		if (*token != ';')
		    newrule = (lefSpacingRule *)malloc(sizeof(lefSpacingRule));

		while (*token != ';') {
		    token = LefNextToken(f, TRUE);	// Minimum width value
		    sscanf(token, "%lg", &dvalue);
		    newrule->width = dvalue / (double)oscale;

		    for (i = 0; i < entries; i++) {
			token = LefNextToken(f, TRUE);	// Spacing value
		    }
		    sscanf(token, "%lg", &dvalue);
		    newrule->spacing = dvalue / (double)oscale;
		    token = LefNextToken(f, TRUE);

		    for (testrule = lefl->info.route.spacing; testrule;
				testrule = testrule->next)
			if (testrule->next == NULL || testrule->next->width >
				newrule->width)
			    break;

		    if (!testrule) {
			newrule->next = NULL;
			lefl->info.route.spacing = newrule;
		    }
		    else {
			newrule->next = testrule->next;
			testrule->next = newrule;
		    }
		    token = LefNextToken(f, TRUE);
		    if (strcmp(token, "WIDTH")) break;
		}
		break;
	    case LEF_LAYER_PITCH:
		token = LefNextToken(f, TRUE);
		sscanf(token, "%lg", &dvalue);
		lefl->info.route.pitchx = dvalue / (double)oscale;

		token = LefNextToken(f, TRUE);
		if (token && (*token != ';')) {
		    sscanf(token, "%lg", &dvalue);
		    lefl->info.route.pitchy = dvalue / (double)oscale;
		    LefEndStatement(f);
		}
		else {
		    lefl->info.route.pitchy = lefl->info.route.pitchx;
		}

		/* Offset default is 1/2 the pitch.  Offset is		*/
		/* intialized to -1 to tell whether or not the value	*/
		/* has been set by an OFFSET statement.			*/
		if (lefl->info.route.offsetx < 0.0)
		    lefl->info.route.offsetx = lefl->info.route.pitchx / 2.0;
		if (lefl->info.route.offsety < 0.0)
		    lefl->info.route.offsety = lefl->info.route.pitchy / 2.0;
		break;
	    case LEF_LAYER_DIRECTION:
		token = LefNextToken(f, TRUE);
		LefLower(token);
		lefl->info.route.hdirection = (token[0] == 'h') ? TRUE : FALSE;
		LefEndStatement(f);
		break;
	    case LEF_LAYER_OFFSET:
		token = LefNextToken(f, TRUE);
		sscanf(token, "%lg", &dvalue);
		lefl->info.route.offsetx = dvalue / (double)oscale;

		token = LefNextToken(f, TRUE);
		if (token && (*token != ';')) {
		    sscanf(token, "%lg", &dvalue);
		    lefl->info.route.offsety = dvalue / (double)oscale;
		    LefEndStatement(f);
		}
		else {
		    lefl->info.route.offsety = lefl->info.route.offsetx;
		}
		break;
	    case LEF_LAYER_RES:
		token = LefNextToken(f, TRUE);
		if (lefl->lefClass == CLASS_ROUTE) {
		    if (!strcmp(token, "RPERSQ")) {
			token = LefNextToken(f, TRUE);
			sscanf(token, "%lg", &dvalue);
			// Units are ohms per square
			lefl->info.route.respersq = dvalue;
		    }
		}
		else if (lefl->lefClass == CLASS_VIA) {
		    sscanf(token, "%lg", &dvalue);
		    lefl->info.via.respervia = dvalue;	// Units ohms
		}
		LefEndStatement(f);
		break;
	    case LEF_LAYER_CAP:
		token = LefNextToken(f, TRUE);
		if (lefl->lefClass == CLASS_ROUTE) {
		    if (!strcmp(token, "CPERSQDIST")) {
			token = LefNextToken(f, TRUE);
			sscanf(token, "%lg", &dvalue);
			// Units are pF per squared unit length
			lefl->info.route.areacap = dvalue / 
				((double)oscale * (double)oscale);
		    }
		}
		LefEndStatement(f);
		break;
	    case LEF_LAYER_EDGECAP:
		token = LefNextToken(f, TRUE);
		if (lefl->lefClass == CLASS_ROUTE) {
		    sscanf(token, "%lg", &dvalue);
		    // Units are pF per unit length
		    lefl->info.route.edgecap = dvalue / (double)oscale;
		}
		LefEndStatement(f);
		break;
	    case LEF_LAYER_THICKNESS:
	    case LEF_LAYER_HEIGHT:
	    case LEF_LAYER_MINDENSITY:
	    case LEF_LAYER_ANTENNADIFF:
	    case LEF_LAYER_ANTENNASIDE:
	    case LEF_LAYER_WIREEXT:
		/* Not specifically handling these */
		LefEndStatement(f);
		break;
	    case LEF_VIA_DEFAULT:
	    case LEF_VIARULE_GENERATE:
		/* Do nothing; especially, don't look for end-of-statement! */
		break;
	    case LEF_VIA_LAYER:
		curlayer = LefReadLayer(f, FALSE);
		LefEndStatement(f);
		break;
	    case LEF_VIA_RECT:
		if (curlayer >= 0)
		    LefAddViaGeometry(f, lefl, curlayer, oscale);
		LefEndStatement(f);
		break;
	    case LEF_VIA_ENCLOSURE:
	    case LEF_VIA_PREFERENCLOSURE:
	    case LEF_VIARULE_OVERHANG:
	    case LEF_VIARULE_METALOVERHANG:
		/* Ignoring this:  Need to handle via generates */
		LefEndStatement(f);
		break;
	    case LEF_VIARULE_VIA:
		LefEndStatement(f);
		break;
	    case LEF_LAYER_END:
		if (!LefParseEndStatement(f, lname))
		{
		    LefError("Layer END statement missing.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == LEF_LAYER_END) break;
    }
}

/*----------------------------------------------------------------*/
/* This routine runs through all the defined vias, from last to	  */
/* first defined.  Check the X vs. Y dimension of the base layer. */
/* If X is longer, save as ViaX.  If Y is longer, save as ViaY.   */
/* If there is an AllowedVias list, then only assign vias that	  */
/* are in the list.						  */
/*----------------------------------------------------------------*/

void
LefAssignLayerVias()
{
    LefList lefl;
    int layer;
    double xydiff;
    DSEG grect;
    LinkedStringPtr viaName;
    char *newViaX[MAX_LAYERS];
    char *newViaY[MAX_LAYERS];

    for (layer = 0; layer < MAX_LAYERS; layer++) {
	newViaX[layer] = newViaY[layer] = NULL;
    }

    for (lefl = LefInfo; lefl; lefl = lefl->next) {
	if (lefl->lefClass == CLASS_VIA) {
	    if (lefl->info.via.lr) {
		layer = MAX_LAYERS;
		if (lefl->info.via.area.layer >= 0) {
		   layer = lefl->info.via.area.layer;
		   xydiff = (lefl->info.via.area.x2 - lefl->info.via.area.x1) -
			(lefl->info.via.area.y2 - lefl->info.via.area.y1);
		}

		for (grect = lefl->info.via.lr; grect; grect = grect->next) {
		    if (grect->layer >= 0 && grect->layer < layer) {
			layer = grect->layer;
			xydiff = (grect->x2 - grect->x1) - (grect->y2 - grect->y1);
		    }
		}
		if (layer < MAX_LAYERS) {
		    /* Assign only to layers in AllowedVias, if it is non-NULL */
		    if (AllowedVias != NULL) {
			for (viaName = AllowedVias; viaName; viaName = viaName->next) {
			    if (!strcmp(viaName->name, lefl->lefName))
				break;
			}
			if (viaName == NULL) continue;
		    }
		    if (xydiff > -EPS) {
			if (newViaX[layer] != NULL) free(newViaX[layer]);
			newViaX[layer] = strdup(lefl->lefName);      
		    }
		    else {
			if (newViaY[layer] != NULL) free(newViaY[layer]);
			newViaY[layer] = strdup(lefl->lefName);
		    }
		}
	    }
	}
    }

    /* Copy newViaX and newViaY back into viaX and viaY, making */
    /* sure that at least one entry exists for each layer.	*/

    /* At this time, only ViaX[] reports values back in		*/
    /* LefGetViaWidth(), so make sure that if there is only one	*/
    /* allowed via for a layer, it is copied into the ViaX	*/
    /* array, regardless of its orientation.			*/

    for (layer = 0; layer < MAX_LAYERS; layer++) {
	if ((newViaX[layer] == NULL) && (newViaY[layer] == NULL))
	    continue;
	if (ViaX[layer] != NULL) free(ViaX[layer]);
	if (ViaY[layer] != NULL) free(ViaY[layer]);

	if (newViaX[layer] != NULL)
	    ViaX[layer] = strdup(newViaX[layer]);
	else
	    ViaX[layer] = strdup(newViaY[layer]);
	if (newViaY[layer] != NULL)
	    ViaY[layer] = strdup(newViaY[layer]);
    }

    for (layer = 0; layer < MAX_LAYERS; layer++) {
	if (newViaX[layer] != NULL) free(newViaX[layer]);
	if (newViaY[layer] != NULL) free(newViaY[layer]);
    }
}

/*
 *------------------------------------------------------------
 *
 * LefRead --
 *
 *	Read a .lef file and generate all routing configuration
 *	structures and values from the LAYER, VIA, and MACRO sections
 *
 * Results:
 *	None.
 *
 * Side Effects:
 *	Many.  Cell definitions are created and added to
 *	the GateInfo database.
 *
 *------------------------------------------------------------
 */

enum lef_sections {LEF_VERSION = 0,
	LEF_BUSBITCHARS, LEF_DIVIDERCHAR, LEF_MANUFACTURINGGRID,
	LEF_USEMINSPACING, LEF_CLEARANCEMEASURE,
	LEF_NAMESCASESENSITIVE, LEF_PROPERTYDEFS, LEF_UNITS,
	LEF_SECTION_LAYER, LEF_SECTION_VIA, LEF_SECTION_VIARULE,
	LEF_SECTION_SPACING, LEF_SECTION_SITE, LEF_PROPERTY,
	LEF_NOISETABLE, LEF_CORRECTIONTABLE, LEF_IRDROP,
	LEF_ARRAY, LEF_SECTION_TIMING, LEF_EXTENSION, LEF_MACRO,
	LEF_END};

int
LefRead(inName)
    char *inName;
{
    FILE *f;
    char filename[256];
    char *token;
    char tsave[128];
    int keyword, layer;
    int oprecis = 100;	// = 1 / manufacturing grid (microns)
    float oscale;
    double xydiff, ogrid;
    LefList lefl;
    DSEG grect;
    GATE gateginfo;

    static char *sections[] = {
	"VERSION",
	"BUSBITCHARS",
	"DIVIDERCHAR",
	"MANUFACTURINGGRID",
	"USEMINSPACING",
	"CLEARANCEMEASURE",
	"NAMESCASESENSITIVE",
	"PROPERTYDEFINITIONS",
	"UNITS",
	"LAYER",
	"VIA",
	"VIARULE",
	"SPACING",
	"SITE",
	"PROPERTY",
	"NOISETABLE",
	"CORRECTIONTABLE",
	"IRDROP",
	"ARRAY",
	"TIMING",
	"BEGINEXT",
	"MACRO",
	"END",
	NULL
    };

    if (!strrchr(inName, '.'))
	sprintf(filename, "%s.lef", inName);
    else
	strcpy(filename, inName);

    f = fopen(filename, "r");

    if (f == NULL)
    {
	Fprintf(stderr, "Cannot open input file: ");
	perror(filename);
	return 0;
    }

    if (Verbose > 0) {
	Fprintf(stdout, "Reading LEF data from file %s.\n", filename);
	Flush(stdout);
    }

    oscale = 1;

    while ((token = LefNextToken(f, TRUE)) != NULL)
    {
	keyword = Lookup(token, sections);
	if (keyword < 0)
	{
	    LefError("Unknown keyword \"%s\" in LEF file; ignoring.\n", token);
	    LefEndStatement(f);
	    continue;
	}
	switch (keyword)
	{
	    case LEF_VERSION:
	    case LEF_BUSBITCHARS:
	    case LEF_DIVIDERCHAR:
	    case LEF_CLEARANCEMEASURE:
	    case LEF_USEMINSPACING:
	    case LEF_NAMESCASESENSITIVE:
		LefEndStatement(f);
		break;
	    case LEF_MANUFACTURINGGRID:
		token = LefNextToken(f, TRUE);
		if (sscanf(token, "%lg", &ogrid) == 1)
		    oprecis = (int)((1.0 / ogrid) + 0.5);
		LefEndStatement(f);
		break;
	    case LEF_PROPERTYDEFS:
		LefSkipSection(f, sections[LEF_PROPERTYDEFS]);
		break;
	    case LEF_UNITS:
		LefSkipSection(f, sections[LEF_UNITS]);
		break;

	    case LEF_SECTION_VIA:
	    case LEF_SECTION_VIARULE:
		token = LefNextToken(f, TRUE);
		sprintf(tsave, "%.127s", token);

		lefl = LefFindLayer(token);
		if (lefl == NULL)
		{
		    lefl = (LefList)calloc(1, sizeof(lefLayer));
		    lefl->type = -1;
		    lefl->obsType = -1;
		    lefl->lefClass = CLASS_VIA;
		    lefl->info.via.area.x1 = 0.0;
		    lefl->info.via.area.y1 = 0.0;
		    lefl->info.via.area.x2 = 0.0;
		    lefl->info.via.area.y2 = 0.0;
		    lefl->info.via.area.layer = -1;
		    lefl->info.via.cell = (GATE)NULL;
		    lefl->info.via.lr = (DSEG)NULL;
		    lefl->lefName = strdup(token);

		    lefl->next = LefInfo;
		    LefInfo = lefl;

		    LefReadLayerSection(f, tsave, keyword, lefl);
		}
		else if (keyword == LEF_SECTION_VIARULE)
		    /* If we've already seen this via, don't reprocess. */
		    /* This deals with VIA followed by VIARULE.  We	*/
		    /* really ought to have special processing for the	*/
		    /* VIARULE section. . .				*/
		    LefSkipSection(f, tsave);
		else
		{
		    LefError("Warning:  Cut type \"%s\" redefined.\n", token);
		    lefl = LefRedefined(lefl, token);
		    LefReadLayerSection(f, tsave, keyword, lefl);
		}
		break;

	    case LEF_SECTION_LAYER:
		token = LefNextToken(f, TRUE);
		sprintf(tsave, "%.127s", token);
	
		lefl = LefFindLayer(token);	
		if (lefl == (LefList)NULL)
		{
		    lefl = (LefList)malloc(sizeof(lefLayer));
		    lefl->type = -1;
		    lefl->obsType = -1;
		    lefl->lefClass = CLASS_IGNORE;	/* For starters */
		    lefl->lefName = strdup(token);
		    lefl->next = LefInfo;
		    LefInfo = lefl;
		}
		else
		{
		    if (lefl && lefl->type < 0)
		    {
			LefError("Layer %s is only defined for obstructions!\n", token);
			LefSkipSection(f, tsave);
			break;
		    }
		}
		LefReadLayerSection(f, tsave, keyword, lefl);
		break;

	    case LEF_SECTION_SPACING:
		LefSkipSection(f, sections[LEF_SECTION_SPACING]);
		break;
	    case LEF_SECTION_SITE:
		token = LefNextToken(f, TRUE);
		if (Verbose > 0)
		    Fprintf(stdout, "LEF file:  Defines site %s (ignored)\n", token);
		sprintf(tsave, "%.127s", token);
		LefSkipSection(f, tsave);
		break;
	    case LEF_PROPERTY:
		LefSkipSection(f, NULL);
		break;
	    case LEF_NOISETABLE:
		LefSkipSection(f, sections[LEF_NOISETABLE]);
		break;
	    case LEF_CORRECTIONTABLE:
		LefSkipSection(f, sections[LEF_CORRECTIONTABLE]);
		break;
	    case LEF_IRDROP:
		LefSkipSection(f, sections[LEF_IRDROP]);
		break;
	    case LEF_ARRAY:
		LefSkipSection(f, sections[LEF_ARRAY]);
		break;
	    case LEF_SECTION_TIMING:
		LefSkipSection(f, sections[LEF_SECTION_TIMING]);
		break;
	    case LEF_EXTENSION:
		LefSkipSection(f, sections[LEF_EXTENSION]);
		break;
	    case LEF_MACRO:
		token = LefNextToken(f, TRUE);
		/* Diagnostic */
		/*
		Fprintf(stdout, "LEF file:  Defines new cell %s\n", token);
		*/
		sprintf(tsave, "%.127s", token);
		LefReadMacro(f, tsave, oscale);
		break;
	    case LEF_END:
		if (!LefParseEndStatement(f, "LIBRARY"))
		{
		    LefError("END statement out of context.\n");
		    keyword = -1;
		}
		break;
	}
	if (keyword == LEF_END) break;
    }
    if (Verbose > 0) {
	Fprintf(stdout, "LEF read: Processed %d lines.\n", lefCurrentLine);
	LefError(NULL);	/* print statement of errors, if any */
    }

    /* Cleanup */
    if (f != NULL) fclose(f);

    /* Make sure that the gate list has one entry called "pin" */

    for (gateginfo = GateInfo; gateginfo; gateginfo = gateginfo->next)
	if (!strcasecmp(gateginfo->gatename, "pin"))
	    break;

    if (!gateginfo) {
	/* Add a new GateInfo entry for pseudo-gate "pin" */
	gateginfo = (GATE)malloc(sizeof(struct gate_));
	gateginfo->gatetype = NULL;
	gateginfo->gatename = (char *)malloc(4);
	strcpy(gateginfo->gatename, "pin");
	gateginfo->width = 0.0;
	gateginfo->height = 0.0;
	gateginfo->placedX = 0.0;
	gateginfo->placedY = 0.0;
	gateginfo->nodes = 1;

        gateginfo->taps = (DSEG *)malloc(sizeof(DSEG));
        gateginfo->noderec = (NODE *)malloc(sizeof(NODE));
        gateginfo->direction = (u_char *)malloc(sizeof(u_char));
        gateginfo->netnum = (int *)malloc(sizeof(int));
        gateginfo->node = (char **)malloc(sizeof(char *));

	grect = (DSEG)malloc(sizeof(struct dseg_));
	grect->x1 = grect->x2 = 0.0;
	grect->y1 = grect->y2 = 0.0;
	grect->next = (DSEG)NULL;
	gateginfo->obs = (DSEG)NULL;
	gateginfo->next = GateInfo;
	gateginfo->taps[0] = grect;
        gateginfo->noderec[0] = NULL;
        gateginfo->netnum[0] = -1;
	gateginfo->node[0] = strdup("pin");
	GateInfo = gateginfo;
    }
    PinMacro = gateginfo;

    /* Work through all of the defined layers, and copy the names into	*/
    /* the strings used for route output, overriding any information	*/
    /* that may have been in the route.cfg file.			*/

    for (lefl = LefInfo; lefl; lefl = lefl->next) {
	if (lefl->lefClass == CLASS_ROUTE) {
	    strcpy(CIFLayer[lefl->type], lefl->lefName);
	}
    }
    LefAssignLayerVias();

    return oprecis;
}
