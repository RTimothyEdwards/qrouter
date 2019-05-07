/*--------------------------------------------------------------*/
/* tclqrouter.c:						*/
/*	Tcl routines for qrouter command-line functions		*/
/* Copyright (c) 2013  Tim Edwards, Open Circuit Design, Inc.	*/
/*--------------------------------------------------------------*/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include <tk.h>

#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>

#include "qrouter.h"
#include "maze.h"
#include "qconfig.h"
#include "lef.h"
#include "def.h"
#include "graphics.h"
#include "node.h"
#include "tkSimple.h"

/* Global variables */

Tcl_HashTable QrouterTagTable;
Tcl_Interp *qrouterinterp;
Tcl_Interp *consoleinterp;

int stepnet = -1;
int batchmode = 0;

/* Command structure */

typedef struct {
   const char	*cmdstr;
   int		(*func)(ClientData clientData, Tcl_Interp *interp,
                        int objc, Tcl_Obj *CONST objv[]);
} cmdstruct;

/* Forward declarations of commands */

static int qrouter_map(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_start(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_stage1(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_stage2(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_stage3(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_cleanup(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_writedef(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_writefailed(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_writedelays(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_antenna(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_readdef(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_readlef(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_readconfig(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_failing(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_cost(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_tag(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_remove(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_obs(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_layerinfo(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_priority(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_ignore(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_via(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_resolution(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_congested(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_layers(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_drc(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_passes(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_vdd(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_gnd(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_verbose(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_print(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);
static int qrouter_quit(
    ClientData clientData, Tcl_Interp *interp,
    int objc, Tcl_Obj *CONST objv[]);

static cmdstruct qrouter_commands[] =
{
   {"tag", qrouter_tag},
   {"start", qrouter_start},
   {"stage1", qrouter_stage1},
   {"stage2", qrouter_stage2},
   {"stage3", qrouter_stage3},
   {"cleanup", qrouter_cleanup},
   {"write_def", qrouter_writedef},
   {"read_def", qrouter_readdef},
   {"read_lef", qrouter_readlef},
   {"read_config", qrouter_readconfig},
   {"write_delays", qrouter_writedelays},
   {"antenna", qrouter_antenna},
   {"write_failed", qrouter_writefailed},
   {"layer_info", qrouter_layerinfo},
   {"obstruction", qrouter_obs},
   {"ignore", qrouter_ignore},
   {"priority", qrouter_priority},
   {"via", qrouter_via},
   {"resolution", qrouter_resolution},
   {"congested", qrouter_congested},
   {"layers", qrouter_layers},
   {"drc", qrouter_drc},
   {"passes", qrouter_passes},
   {"vdd", qrouter_vdd},
   {"gnd", qrouter_gnd},
   {"failing", qrouter_failing},
   {"remove", qrouter_remove},
   {"cost", qrouter_cost},
   {"map", qrouter_map},
   {"verbose", qrouter_verbose},
   {"redraw", redraw},
   {"print", qrouter_print},
   {"quit", qrouter_quit},
   {"", NULL}  /* sentinel */
};

/*-----------------------*/
/* Tcl 8.4 compatibility */
/*-----------------------*/

#ifndef CONST84
#define CONST84
#endif

/*----------------------------------------------------------------------*/
/* Deal with systems which don't define va_copy().			*/
/*----------------------------------------------------------------------*/

#ifndef HAVE_VA_COPY
  #ifdef HAVE___VA_COPY
    #define va_copy(a, b) __va_copy(a, b)
  #else
    #define va_copy(a, b) a = b
  #endif
#endif

#ifdef ASG
   extern int SetDebugLevel(int *level);
#endif

/*----------------------------------------------------------------------*/
/* Reimplement strdup() to use Tcl_Alloc().				*/
/*----------------------------------------------------------------------*/

char *Tcl_Strdup(const char *s)
{
   char *snew;
   int slen;

   slen = 1 + strlen(s);
   snew = Tcl_Alloc(slen);
   if (snew != NULL)
      memcpy(snew, s, slen);

   return snew;
}

/*----------------------------------------------------------------------*/
/* Reimplement vfprintf() as a call to Tcl_Eval().			*/
/*									*/
/* Since the string goes through the interpreter, we need to escape	*/
/* various characters like brackets, braces, dollar signs, etc., that	*/
/* will otherwise be modified by the interpreter.			*/
/*----------------------------------------------------------------------*/

void tcl_vprintf(FILE *f, const char *fmt, va_list args_in)
{
   va_list args;
   static char outstr[128] = "puts -nonewline std";
   char *outptr, *bigstr = NULL, *finalstr = NULL;
   int i, nchars, escapes = 0;

   /* If we are printing an error message, we want to bring attention	*/
   /* to it by mapping the console window and raising it, as necessary.	*/
   /* I'd rather do this internally than by Tcl_Eval(), but I can't	*/
   /* find the right window ID to map!					*/

   if ((f == stderr) && (consoleinterp != qrouterinterp)) {
      Tk_Window tkwind;
      tkwind = Tk_MainWindow(consoleinterp);
      if ((tkwind != NULL) && (!Tk_IsMapped(tkwind)))
         Tcl_Eval(consoleinterp, "wm deiconify .\n");
      Tcl_Eval(consoleinterp, "raise .\n");
   }

   strcpy (outstr + 19, (f == stderr) ? "err \"" : "out \"");
   outptr = outstr;

   /* This mess circumvents problems with systems which do not have	*/
   /* va_copy() defined.  Some define __va_copy();  otherwise we must	*/
   /* assume that args = args_in is valid.				*/

   va_copy(args, args_in);
   nchars = vsnprintf(outptr + 24, 102, fmt, args);
   va_end(args);

   if (nchars >= 102) {
      va_copy(args, args_in);
      bigstr = Tcl_Alloc(nchars + 26);
      strncpy(bigstr, outptr, 24);
      outptr = bigstr;
      vsnprintf(outptr + 24, nchars + 2, fmt, args);
      va_end(args);
    }
    else if (nchars == -1) nchars = 126;

    for (i = 24; *(outptr + i) != '\0'; i++) {
       if (*(outptr + i) == '\"' || *(outptr + i) == '[' ||
	  	*(outptr + i) == ']' || *(outptr + i) == '\\' ||
		*(outptr + i) == '$')
	  escapes++;
    }

    if (escapes > 0) {
      finalstr = Tcl_Alloc(nchars + escapes + 26);
      strncpy(finalstr, outptr, 24);
      escapes = 0;
      for (i = 24; *(outptr + i) != '\0'; i++) {
	  if (*(outptr + i) == '\"' || *(outptr + i) == '[' ||
	    		*(outptr + i) == ']' || *(outptr + i) == '\\' ||
			*(outptr + i) == '$') {
	     *(finalstr + i + escapes) = '\\';
	     escapes++;
	  }
	  *(finalstr + i + escapes) = *(outptr + i);
      }
      outptr = finalstr;
    }

    *(outptr + 24 + nchars + escapes) = '\"';
    *(outptr + 25 + nchars + escapes) = '\0';

    Tcl_Eval(consoleinterp, outptr);

    if (bigstr != NULL) Tcl_Free(bigstr);
    if (finalstr != NULL) Tcl_Free(finalstr);
}
    
/*------------------------------------------------------*/
/* Console output flushing which goes along with the	*/
/* routine tcl_vprintf() above.				*/
/*------------------------------------------------------*/

void tcl_stdflush(FILE *f)
{   
   Tcl_SavedResult state;
   static char stdstr[] = "::flush stdxxx";
   char *stdptr = stdstr + 11;
    
   Tcl_SaveResult(qrouterinterp, &state);
   strncpy(stdptr, (f == stderr) ? "err" : "out", 3);
   Tcl_Eval(qrouterinterp, stdstr);
   Tcl_RestoreResult(qrouterinterp, &state);
}

/*----------------------------------------------------------------------*/
/* Reimplement fprintf() as a call to Tcl_Eval().			*/
/*----------------------------------------------------------------------*/

void tcl_printf(FILE *f, const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  tcl_vprintf(f, format, ap);
  va_end(ap);
}

/*----------------------------------------------------------------------*/
/* Implement tag callbacks on functions					*/
/* Find any tags associated with a command and execute them.		*/
/*----------------------------------------------------------------------*/

int QrouterTagCallback(Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
    int objidx, result = TCL_OK;
    char *postcmd, *substcmd, *newcmd, *sptr, *sres;
    char *croot = Tcl_GetString(objv[0]);
    Tcl_HashEntry *entry;
    Tcl_SavedResult state;
    int reset = FALSE;
    int i, llen;

    entry = Tcl_FindHashEntry(&QrouterTagTable, croot);
    postcmd = (entry) ? (char *)Tcl_GetHashValue(entry) : NULL;

    if (postcmd)
    {
	substcmd = (char *)Tcl_Alloc(strlen(postcmd) + 1);
	strcpy(substcmd, postcmd);
	sptr = substcmd;

	/*--------------------------------------------------------------*/
	/* Parse "postcmd" for Tk-substitution escapes			*/
	/* Allowed escapes are:						*/
	/* 	%W	substitute the tk path of the calling window	*/
	/*	%r	substitute the previous Tcl result string	*/
	/*	%R	substitute the previous Tcl result string and	*/
	/*		reset the Tcl result.				*/
	/*	%[0-5]  substitute the argument to the original command	*/
	/*	%N	substitute all arguments as a list		*/
	/*	%%	substitute a single percent character		*/
	/*	%*	(all others) no action: print as-is.		*/
	/*--------------------------------------------------------------*/

	while ((sptr = strchr(sptr, '%')) != NULL)
	{
	    switch (*(sptr + 1))
	    {
		case 'W': {
		    char *tkpath = NULL;
		    Tk_Window tkwind = Tk_MainWindow(interp);
		    if (tkwind != NULL) tkpath = Tk_PathName(tkwind);
		    if (tkpath == NULL)
			newcmd = (char *)Tcl_Alloc(strlen(substcmd));
		    else
			newcmd = (char *)Tcl_Alloc(strlen(substcmd) + strlen(tkpath));

		    strcpy(newcmd, substcmd);

		    if (tkpath == NULL)
			strcpy(newcmd + (int)(sptr - substcmd), sptr + 2);
		    else
		    {
			strcpy(newcmd + (int)(sptr - substcmd), tkpath);
			strcat(newcmd, sptr + 2);
		    }
		    Tcl_Free(substcmd);
		    substcmd = newcmd;
		    sptr = substcmd;
		    } break;

		case 'R':
		    reset = TRUE;
		case 'r':
		    sres = (char *)Tcl_GetStringResult(interp);
		    newcmd = (char *)Tcl_Alloc(strlen(substcmd)
				+ strlen(sres) + 1);
		    strcpy(newcmd, substcmd);
		    sprintf(newcmd + (int)(sptr - substcmd), "\"%s\"", sres);
		    strcat(newcmd, sptr + 2);
		    Tcl_Free(substcmd);
		    substcmd = newcmd;
		    sptr = substcmd;
		    break;

		case '0': case '1': case '2': case '3': case '4': case '5':
		    objidx = (int)(*(sptr + 1) - '0');
		    if ((objidx >= 0) && (objidx < objc))
		    {
		        newcmd = (char *)Tcl_Alloc(strlen(substcmd)
				+ strlen(Tcl_GetString(objv[objidx])));
		        strcpy(newcmd, substcmd);
			strcpy(newcmd + (int)(sptr - substcmd),
				Tcl_GetString(objv[objidx]));
			strcat(newcmd, sptr + 2);
			Tcl_Free(substcmd);
			substcmd = newcmd;
			sptr = substcmd;
		    }
		    else if (objidx >= objc)
		    {
		        newcmd = (char *)Tcl_Alloc(strlen(substcmd) + 1);
		        strcpy(newcmd, substcmd);
			strcpy(newcmd + (int)(sptr - substcmd), sptr + 2);
			Tcl_Free(substcmd);
			substcmd = newcmd;
			sptr = substcmd;
		    }
		    else sptr++;
		    break;

		case 'N':
		    llen = 1;
		    for (i = 1; i < objc; i++)
		       llen += (1 + strlen(Tcl_GetString(objv[i])));
		    newcmd = (char *)Tcl_Alloc(strlen(substcmd) + llen);
		    strcpy(newcmd, substcmd);
		    strcpy(newcmd + (int)(sptr - substcmd), "{");
		    for (i = 1; i < objc; i++) {
		       strcat(newcmd, Tcl_GetString(objv[i]));
		       if (i < (objc - 1))
			  strcat(newcmd, " ");
		    }
		    strcat(newcmd, "}");
		    strcat(newcmd, sptr + 2);
		    Tcl_Free(substcmd);
		    substcmd = newcmd;
		    sptr = substcmd;
		    break;

		case '%':
		    newcmd = (char *)Tcl_Alloc(strlen(substcmd) + 1);
		    strcpy(newcmd, substcmd);
		    strcpy(newcmd + (int)(sptr - substcmd), sptr + 1);
		    Tcl_Free(substcmd);
		    substcmd = newcmd;
		    sptr = substcmd;
		    break;

		default:
		    break;
	    }
	}

	/* Fprintf(stderr, "Substituted tag callback is \"%s\"\n", substcmd); */
	/* Flush(stderr); */

	Tcl_SaveResult(interp, &state);
	result = Tcl_Eval(interp, substcmd);
	if ((result == TCL_OK) && (reset == FALSE))
	    Tcl_RestoreResult(interp, &state);
	else
	    Tcl_DiscardResult(&state);

	Tcl_Free(substcmd);
    }
    return result;
}

/*--------------------------------------------------------------*/
/* Add a command tag callback					*/
/*--------------------------------------------------------------*/

static int
qrouter_tag(ClientData clientData,
            Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
    Tcl_HashEntry *entry;
    char *hstring;
    int new;

    if (objc != 2 && objc != 3)
	return TCL_ERROR;

    entry = Tcl_CreateHashEntry(&QrouterTagTable, Tcl_GetString(objv[1]), &new);
    if (entry == NULL) return TCL_ERROR;

    hstring = (char *)Tcl_GetHashValue(entry);
    if (objc == 2)
    {
	Tcl_SetResult(interp, hstring, NULL);
	return TCL_OK;
    }

    if (strlen(Tcl_GetString(objv[2])) == 0)
    {
	Tcl_DeleteHashEntry(entry);
    }
    else
    {
	hstring = Tcl_Strdup(Tcl_GetString(objv[2]));
	Tcl_SetHashValue(entry, hstring);
    }
    return TCL_OK;
}

/*--------------------------------------------------------------*/
/* Initialization procedure for Tcl/Tk				*/
/*--------------------------------------------------------------*/

int
Qrouter_Init(Tcl_Interp *interp)
{
   int cmdidx;
   char command[256];
   char version_string[20];
   Tk_Window tktop;
   char *nullgvar;

   /* Interpreter sanity checks */
   if (interp == NULL) return TCL_ERROR;

   /* Remember the interpreter */
   qrouterinterp = interp;

   if (Tcl_InitStubs(interp, "8.5", 0) == NULL) return TCL_ERROR;

   strcpy(command, "qrouter::");
   
   /* NOTE:  Qrouter makes calls to Tk routines that may or may not	*/
   /* exist, depending on whether qrouter was called with or without	*/
   /* graphics.  We depend on the Tcl/Tk stubs methods to allow 	*/
   /* qrouter to run without linking to Tk libraries.			*/

   nullgvar = (char *)Tcl_GetVar(interp, "no_graphics_mode", TCL_GLOBAL_ONLY);
   if ((nullgvar == NULL) || !strcasecmp(nullgvar, "false")) {
      if (Tk_InitStubs(interp, "8.5", 0) == NULL) return TCL_ERROR;
      tktop = Tk_MainWindow(interp);
      batchmode = 0;
   }
   else {
      tktop = NULL;
      batchmode = 1;
   }

   /* Create all of the commands (except "simple") */

   for (cmdidx = 0; qrouter_commands[cmdidx].func != NULL; cmdidx++) {
      sprintf(command + 9, "%s", qrouter_commands[cmdidx].cmdstr);
      Tcl_CreateObjCommand(interp, command,
		(Tcl_ObjCmdProc *)qrouter_commands[cmdidx].func,
		(ClientData)tktop, (Tcl_CmdDeleteProc *) NULL);
   }

   if (tktop != NULL) {
      Tcl_CreateObjCommand(interp, "simple",
		(Tcl_ObjCmdProc *)Tk_SimpleObjCmd,
		(ClientData)tktop, (Tcl_CmdDeleteProc *) NULL);
   }

   Tcl_Eval(interp, "lappend auto_path .");

   sprintf(version_string, "%s", VERSION);
   Tcl_SetVar(interp, "QROUTER_VERSION", version_string, TCL_GLOBAL_ONLY);

   Tcl_Eval(interp, "namespace eval qrouter namespace export *");
   Tcl_PkgProvide(interp, "Qrouter", version_string);

   /* Initialize the console interpreter, if there is one. */

   if ((consoleinterp = Tcl_GetMaster(interp)) == NULL)
      consoleinterp = interp;

   /* Initialize the command tag table */

   Tcl_InitHashTable(&QrouterTagTable, TCL_STRING_KEYS);

   return TCL_OK;
}

/*------------------------------------------------------*/
/* Command "start"					*/
/*------------------------------------------------------*/

static int
qrouter_start(ClientData clientData, Tcl_Interp *interp,
              int objc, Tcl_Obj *CONST objv[])
{
    int i, result, argc;
    char *scriptfile = NULL;
    char **argv;

    /* For compatibility with the original C code, convert Tcl	*/
    /* object arguments to strings.  Handle "-s <name>",	*/
    /* which is not handled by runqrouter(), and source the	*/
    /* script <name> between runqrouter() and read_def().	*/

    argv = (char **)malloc((objc - 1) * sizeof(char *));
    argc = 0;
    for (i = 1; i < objc; i++) {
	if (!strcmp(Tcl_GetString(objv[i]), "-s"))
	    scriptfile = strdup(Tcl_GetString(objv[i + 1]));
	argv[argc++] = strdup(Tcl_GetString(objv[i]));
    }

    result = runqrouter(argc, argv);
    if ((result == 0) && (batchmode == 0)) GUI_init(interp);

    for (i = 0; i < argc; i++)
        free(argv[i]);
    free(argv);

    if (scriptfile != NULL) {

	/* First check that the script file exists.  If not,	*/
	/* then generate an error here.				*/

	FILE *scriptf = fopen(scriptfile, "r");
	if (scriptf == NULL) {
	    Fprintf(stderr, "Script file \"%s\" unavaliable or unreadable.\n",
			scriptfile);
	    Tcl_SetResult(interp, "Script file unavailable or unreadable.", NULL);
	    result = TCL_ERROR;
	}
	else {
	    fclose(scriptf);
	    result = Tcl_EvalFile(interp, scriptfile);
	}

	/* The script file should determine whether or not to	*/
	/* exit by including the "quit" command.  But if there	*/
	/* is an error in the script, then always quit.		*/

	/* If tkcon console is in use and there is an error in	*/
	/* the script, then print the error message to the	*/
	/* terminal, not the console, or else it vanishes.	*/

	if (result != TCL_OK) {
	    if (consoleinterp == interp)
		Fprintf(stderr, "Script file \"%s\" failed with result \'%s\'\n",
			scriptfile, Tcl_GetStringResult(interp));
	    else
		fprintf(stderr, "Script file \"%s\" failed with result \'%s\'\n",
			scriptfile, Tcl_GetStringResult(interp));
	    free(scriptfile);
	    /* Make sure Tcl has generated all output */
	    while (Tcl_DoOneEvent(TCL_DONT_WAIT) != 0);
	    /* And exit gracefully */
	    qrouter_quit(clientData, interp, 1, objv);
	}
	else
	    free(scriptfile);
    }

    if ((DEFfilename != NULL) && (Nlgates == NULL)) {
	read_def(NULL);
	draw_layout();
    }

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command: qrouter_quit				*/
/*							*/
/* Call tkcon's exit routine, which will make sure	*/
/* the history file is updated before final exit.	*/
/*------------------------------------------------------*/

int
qrouter_quit(ClientData clientData, Tcl_Interp *interp,
	int objc, Tcl_Obj *CONST objv[])
{
    if (objc != 1) {
	Tcl_WrongNumArgs(interp, 1, objv, "(no arguments)");
	return TCL_ERROR;
    }

    /* Free up failed net list */
    remove_failed();

    /* Should be doing other cleanup tasks here. . . */

    if (consoleinterp == interp)
	Tcl_Exit(TCL_OK);
    else
	Tcl_Eval(interp, "catch {tkcon eval exit}\n");

    return TCL_OK;       /* Not reached */
}

/*------------------------------------------------------*/
/* Command "map"					*/
/*							*/
/* Specify what to draw in the graphics window		*/
/*							*/
/*	map obstructions    draw routes (normal)	*/
/*	map congestion	    draw actual congestion	*/
/*	map estimate	    draw estimated congestion	*/
/*	map none	    route background is plain	*/
/*	map routes	    draw routes over map	*/
/*	map noroutes	    don't draw routes over map	*/
/*	map unrouted	    draw unrouted nets over map	*/
/*	map nounrouted	    don't draw unrouted nets	*/
/*------------------------------------------------------*/

static int
qrouter_map(ClientData clientData, Tcl_Interp *interp,
            int objc, Tcl_Obj *CONST objv[])
{
    int idx, result;

    static char *subCmds[] = {
	"obstructions", "congestion", "estimate", "none",
	"routes", "noroutes", "unrouted", "nounrouted", NULL
    };
    enum SubIdx {
	ObsIdx, CongIdx, EstIdx, NoneIdx, RouteIdx, NoRouteIdx, UnroutedIdx, NoUnroutedIdx
    };
   
    if (objc != 2) {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }
    else if ((result = Tcl_GetIndexFromObj(interp, objv[1],
		(CONST84 char **)subCmds, "option", 0, &idx)) != TCL_OK)
	return result;

    switch (idx) {
	case ObsIdx:
	    if ((mapType & MAP_MASK) != MAP_OBSTRUCT) {
		mapType &= ~MAP_MASK;
		mapType |= MAP_OBSTRUCT;
		draw_layout();
	    }
	    break;
	case CongIdx:
	    if ((mapType & MAP_MASK) != MAP_CONGEST) {
		mapType &= ~MAP_MASK;
		mapType |= MAP_CONGEST;
		draw_layout();
	    }
	    break;
	case EstIdx:
	    if ((mapType & MAP_MASK) != MAP_ESTIMATE) {
		mapType &= ~MAP_MASK;
		mapType |= MAP_ESTIMATE;
		draw_layout();
	    }
	    break;
	case NoneIdx:
	    if ((mapType & MAP_MASK) != MAP_NONE) {
		mapType &= ~MAP_MASK;
		mapType |= MAP_NONE;
		draw_layout();
	    }
	    break;
	case RouteIdx:
            mapType |= DRAW_ROUTES;
            draw_layout();
	    break;
	case NoRouteIdx:
            mapType &= ~DRAW_ROUTES;
            draw_layout();
	    break;
        case UnroutedIdx:
            mapType |= DRAW_UNROUTED;
            draw_layout();
            break;
        case NoUnroutedIdx:
            mapType &= ~DRAW_UNROUTED;
            draw_layout();
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Find the net with number "number" in the list of	*/
/* nets and return a pointer to it.			*/
/*							*/
/* NOTE:  This could be hashed like net names, but is	*/
/* only used in one place, and router performance does	*/
/* not depend on it.					*/
/*------------------------------------------------------*/

NET LookupNetNr(int number)
{
    NET net;
    int i;

    for (i = 0; i < Numnets; i++) {
       net = Nlnets[i];
       if (net->netnum == number)
	  return net;
    }
    return NULL;
}

/*------------------------------------------------------*/
/* Command "stage1"					*/
/*							*/
/* Execute stage1 routing.  This works through the	*/
/* entire netlist, routing as much as possible but not	*/
/* doing any rip-up and re-route.  Nets that fail to	*/
/* route are put in the "FailedNets" list.		*/
/*							*/
/* The interpreter result is set to the number of	*/
/* failed routes at the end of the first stage.		*/
/*							*/
/* Options:						*/
/*							*/
/*  stage1 debug	Draw the area being searched in	*/
/*			real-time.  This slows down the	*/
/*			algorithm and is intended only	*/
/*			for diagnostic use.		*/
/*  stage1 step		Single-step stage one.		*/
/*  stage1 mask none	Don't limit the search area	*/
/*  stage1 mask auto	Select the mask automatically	*/
/*  stage1 mask bbox	Use the net bbox as a mask	*/
/*  stage1 mask <value> Set the mask size to <value>,	*/
/*			an integer typ. 0 and up.	*/
/*  stage1 route <net>	Route net named <net> only.	*/
/*							*/
/*  stage1 force	Force a terminal to be routable	*/
/*------------------------------------------------------*/

static int
qrouter_stage1(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    u_char dodebug;
    u_char dostep;
    u_char saveForce, saveOverhead;
    int i, idx, idx2, val, result, failcount = 0;
    NET net = NULL;

    static char *subCmds[] = {
	"debug", "mask", "route", "force", "step", NULL
    };
    enum SubIdx {
	DebugIdx, MaskIdx, RouteIdx, ForceIdx, StepIdx
    };
   
    static char *maskSubCmds[] = {
	"none", "auto", "bbox", NULL
    };
    enum maskSubIdx {
	NoneIdx, AutoIdx, BboxIdx
    };

    // Command defaults

    dodebug = FALSE;
    dostep = FALSE;
    maskMode = MASK_AUTO;	// Mask mode is auto unless specified

    // Save these global defaults in case they are locally changed
    saveForce = forceRoutable;

    if (objc >= 2) {
	for (i = 1; i < objc; i++) {

	    if ((result = Tcl_GetIndexFromObj(interp, objv[i],
			(CONST84 char **)subCmds, "option", 0, &idx))
			!= TCL_OK)
		return result;

	    switch (idx) {
		case DebugIdx:
		    dodebug = TRUE;
		    break;

		case StepIdx:
		    dostep = TRUE;
		    break;

		case ForceIdx:
		    forceRoutable = TRUE;
		    break;

		case RouteIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "route ?net?");
			return TCL_ERROR;
		    }
		    i++;
		    net = DefFindNet(Tcl_GetString(objv[i]));
		    if (net == NULL) {
			Tcl_SetResult(interp, "No such net", NULL);
			return TCL_ERROR;
		    }
		    break;

		case MaskIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "mask ?type?");
			return TCL_ERROR;
		    }
		    i++;
		    if ((result = Tcl_GetIndexFromObj(interp, objv[i],
				(CONST84 char **)maskSubCmds, "type", 0,
				&idx2)) != TCL_OK) {
			Tcl_ResetResult(interp);
			result = Tcl_GetIntFromObj(interp, objv[i], &val);
			if (result != TCL_OK) return result;
			else if (val < 0 || val > 200) {
			    Tcl_SetResult(interp, "Bad mask value", NULL);
			    return TCL_ERROR;
			}
			maskMode = (u_char)val;
		    }
		    else {
			switch(idx2) {
			    case NoneIdx:
				maskMode = MASK_NONE;
				break;
			    case AutoIdx:
				maskMode = MASK_AUTO;
				break;
			    case BboxIdx:
				maskMode = MASK_BBOX;
				break;
			}
		    }
		    break;
	    }
	}
    }

    if (dostep == FALSE) stepnet = -1;
    else stepnet++;

    if (net == NULL)
	failcount = dofirststage(dodebug, stepnet);
    else {
	if ((net != NULL) && (net->netnodes != NULL)) {
	    result = doroute(net, (u_char)0, dodebug);
	    failcount = (result == 0) ? 0 : 1;

	    /* Remove from FailedNets list if routing	*/
	    /* was successful				*/

	    if (result == 0 && FailedNets != NULL) {
		NETLIST fnet, lnet = NULL;
		for (fnet = FailedNets; fnet != NULL; fnet = fnet->next) {
		    if (fnet->net == net) {
			if (lnet == NULL)
			    FailedNets = fnet->next;
			else
			    lnet->next = fnet->next;
			free(fnet);
			break;
		    }
		    lnet = fnet;
		}
	    }
	}
    }
    Tcl_SetObjResult(interp, Tcl_NewIntObj(failcount));

    if (stepnet >= (Numnets - 1)) stepnet = -1;

    // Restore global defaults in case they were locally changed
    forceRoutable = saveForce;

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "stage2"					*/
/*							*/
/* Execute stage2 routing.  This stage works through	*/
/* the "FailedNets" list, routing with collisions, and	*/
/* then ripping up the colliding nets and appending	*/
/* them to the "FailedNets" list.			*/
/*							*/
/* The interpreter result is set to the number of	*/
/* failed routes at the end of the second stage.	*/
/*							*/
/* Options:						*/
/*							*/
/*  stage2 debug	Draw the area being searched in	*/
/*			real-time.  This slows down the	*/
/*			algorithm and is intended only	*/
/*			for diagnostic use.		*/
/*  stage2 step		Single-step stage two		*/
/*  stage2 mask none	Don't limit the search area	*/
/*  stage2 mask auto	Select the mask automatically	*/
/*  stage2 mask bbox	Use the net bbox as a mask	*/
/*  stage2 mask <value> Set the mask size to <value>,	*/
/*			an integer typ. 0 and up.	*/
/*  stage2 limit <n>	Fail route if solution collides	*/
/*			with more than <n> nets.	*/
/*  stage2 route <net>	Route net named <net> only.	*/
/*							*/
/*  stage2 force	Force a terminal to be routable	*/
/*  stage2 break	Only rip up colliding segment	*/
/*  stage2 effort <n>	Level of effort (default 100)	*/
/*------------------------------------------------------*/

static int
qrouter_stage2(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    u_int  effort;
    u_char dodebug;
    u_char dostep;
    u_char onlybreak;
    u_char saveForce, saveOverhead;
    int i, idx, idx2, val, result, failcount;
    NET net = NULL;

    static char *subCmds[] = {
	"debug", "mask", "limit", "route", "force", "tries", "step",
	"break", "effort", NULL
    };
    enum SubIdx {
	DebugIdx, MaskIdx, LimitIdx, RouteIdx, ForceIdx, TriesIdx, StepIdx,
	BreakIdx, EffortIdx
    };
   
    static char *maskSubCmds[] = {
	"none", "auto", "bbox", NULL
    };
    enum maskSubIdx {
	NoneIdx, AutoIdx, BboxIdx
    };

    // Command defaults

    dodebug = FALSE;
    dostep = FALSE;
    onlybreak = FALSE;
    maskMode = MASK_AUTO;	// Mask mode is auto unless specified
    // Save these global defaults in case they are locally changed
    saveForce = forceRoutable;
    ripLimit = 10;		// Rip limit is 10 unless specified
    effort = 100;		// Moderate to high effort

    if (objc >= 2) {
	for (i = 1; i < objc; i++) {

	    if ((result = Tcl_GetIndexFromObj(interp, objv[i],
			(CONST84 char **)subCmds, "option", 0, &idx))
			!= TCL_OK)
		return result;

	    switch (idx) {
		case DebugIdx:
		    dodebug = TRUE;
		    break;

		case StepIdx:
		    dostep = TRUE;
		    break;

		case BreakIdx:
		    onlybreak = TRUE;
		    break;
	
		case ForceIdx:
		    forceRoutable = TRUE;
		    break;

		case EffortIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "effort ?num?");
			return TCL_ERROR;
		    }
		    i++;
		    result = Tcl_GetIntFromObj(interp, objv[i], &val);
		    if (result != TCL_OK) return result;
		    effort = (u_int)val;
		    break;

		case TriesIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "tries ?num?");
			return TCL_ERROR;
		    }
		    i++;
		    result = Tcl_GetIntFromObj(interp, objv[i], &val);
		    if (result != TCL_OK) return result;
		    Tcl_SetResult(interp, "\"tries\" deprecated, "
				"use \"effort\" instead.", NULL);
		    effort = (u_char)val * 100;
		    break;
	
		case RouteIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "route ?net?");
			return TCL_ERROR;
		    }
		    i++;
		    net = DefFindNet(Tcl_GetString(objv[i]));
		    if (net == NULL) {
			Tcl_SetResult(interp, "No such net", NULL);
			return TCL_ERROR;
		    }
		    break;

		case LimitIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "limit ?num?");
			return TCL_ERROR;
		    }
		    i++;
		    result = Tcl_GetIntFromObj(interp, objv[i], &val);
		    if (result != TCL_OK) return result;
		    ripLimit = (u_char)val;
		    break;
	
		case MaskIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "mask ?type?");
			return TCL_ERROR;
		    }
		    i++;
		    if ((result = Tcl_GetIndexFromObj(interp, objv[i],
				(CONST84 char **)maskSubCmds, "type", 0,
				&idx2)) != TCL_OK) {
			Tcl_ResetResult(interp);
			result = Tcl_GetIntFromObj(interp, objv[i], &val);
			if (result != TCL_OK) return result;
			else if (val < 0 || val > 200) {
			    Tcl_SetResult(interp, "Bad mask value", NULL);
			    return TCL_ERROR;
			}
			maskMode = (u_char)val;
		    }
		    else {
			switch(idx2) {
			    case NoneIdx:
				maskMode = MASK_NONE;
				break;
			    case AutoIdx:
				maskMode = MASK_AUTO;
				break;
			    case BboxIdx:
				maskMode = MASK_BBOX;
				break;
			}
		    }
		    break;
	    }
	}
    }

    if (net == NULL)
	failcount = dosecondstage(dodebug, dostep, onlybreak, effort);
    else
	failcount = route_net_ripup(net, dodebug, onlybreak);
    Tcl_SetObjResult(interp, Tcl_NewIntObj(failcount));

    draw_layout();

    // Restore global defaults in case they were locally changed
    forceRoutable = saveForce;

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "stage3"					*/
/*							*/
/* Execute stage3 routing.  This works through the	*/
/* entire netlist, ripping up each route in turn and	*/
/* re-routing it.  					*/
/*							*/
/* The interpreter result is set to the number of	*/
/* failed routes at the end of the first stage.		*/
/*							*/
/* Options:						*/
/*							*/
/*  stage3 debug	Draw the area being searched in	*/
/*			real-time.  This slows down the	*/
/*			algorithm and is intended only	*/
/*			for diagnostic use.		*/
/*  stage3 step		Single-step stage three.	*/
/*  stage3 mask none	Don't limit the search area	*/
/*  stage3 mask auto	Select the mask automatically	*/
/*  stage3 mask bbox	Use the net bbox as a mask	*/
/*  stage3 mask <value> Set the mask size to <value>,	*/
/*			an integer typ. 0 and up.	*/
/*  stage3 route <net>	Route net named <net> only.	*/
/*							*/
/*  stage3 force	Force a terminal to be routable	*/
/*  stage3 effort	Level of effort (default 100)	*/
/*------------------------------------------------------*/

static int
qrouter_stage3(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    u_int effort;
    u_char dodebug;
    u_char dostep;
    u_char saveForce, saveOverhead;
    int i, idx, idx2, val, result, failcount = 0;
    NET net = NULL;

    static char *subCmds[] = {
	"debug", "mask", "route", "force", "step", "effort", NULL
    };
    enum SubIdx {
	DebugIdx, MaskIdx, RouteIdx, ForceIdx, StepIdx, EffortIdx
    };
   
    static char *maskSubCmds[] = {
	"none", "auto", "bbox", NULL
    };
    enum maskSubIdx {
	NoneIdx, AutoIdx, BboxIdx
    };

    // Command defaults

    dodebug = FALSE;
    dostep = FALSE;
    maskMode = MASK_AUTO;	// Mask mode is auto unless specified
    effort = 100;		// Moderate to high effort

    // Save these global defaults in case they are locally changed
    saveForce = forceRoutable;

    if (objc >= 2) {
	for (i = 1; i < objc; i++) {

	    if ((result = Tcl_GetIndexFromObj(interp, objv[i],
			(CONST84 char **)subCmds, "option", 0, &idx))
			!= TCL_OK)
		return result;

	    switch (idx) {
		case DebugIdx:
		    dodebug = TRUE;
		    break;

		case StepIdx:
		    dostep = TRUE;
		    break;

		case ForceIdx:
		    forceRoutable = TRUE;
		    break;

		case EffortIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "effort ?num?");
			return TCL_ERROR;
		    }
		    i++;
		    result = Tcl_GetIntFromObj(interp, objv[i], &val);
		    if (result != TCL_OK) return result;
		    effort = (u_int)val;
		    break;

		case RouteIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "route ?net?");
			return TCL_ERROR;
		    }
		    i++;
		    net = DefFindNet(Tcl_GetString(objv[i]));
		    if (net == NULL) {
			Tcl_SetResult(interp, "No such net", NULL);
			return TCL_ERROR;
		    }
		    break;

		case MaskIdx:
		    if (i >= objc - 1) {
			Tcl_WrongNumArgs(interp, 0, objv, "mask ?type?");
			return TCL_ERROR;
		    }
		    i++;
		    if ((result = Tcl_GetIndexFromObj(interp, objv[i],
				(CONST84 char **)maskSubCmds, "type", 0,
				&idx2)) != TCL_OK) {
			Tcl_ResetResult(interp);
			result = Tcl_GetIntFromObj(interp, objv[i], &val);
			if (result != TCL_OK) return result;
			else if (val < 0 || val > 200) {
			    Tcl_SetResult(interp, "Bad mask value", NULL);
			    return TCL_ERROR;
			}
			maskMode = (u_char)val;
		    }
		    else {
			switch(idx2) {
			    case NoneIdx:
				maskMode = MASK_NONE;
				break;
			    case AutoIdx:
				maskMode = MASK_AUTO;
				break;
			    case BboxIdx:
				maskMode = MASK_BBOX;
				break;
			}
		    }
		    break;
	    }
	}
    }

    if (dostep == FALSE) stepnet = -1;
    else stepnet++;

    if (net == NULL)
	failcount = dothirdstage(dodebug, stepnet, effort);
    else {
        /* To do:  Duplicate behavior of dothirdstage(), which	*/
	/* is to retain the original route solution and restore	*/
	/* it in case the routing fails.			*/

	if ((net != NULL) && (net->netnodes != NULL)) {
	    result = doroute(net, (u_char)0, dodebug);
	    failcount = (result == 0) ? 0 : 1;

	    /* Remove from FailedNets list if routing	*/
	    /* was successful				*/

	    if (result == 0 && FailedNets != NULL) {
		NETLIST fnet, lnet = NULL;
		for (fnet = FailedNets; fnet != NULL; fnet = fnet->next) {
		    if (fnet->net == net) {
			if (lnet == NULL)
			    FailedNets = fnet->next;
			else
			    lnet->next = fnet->next;
			free(fnet);
			break;
		    }
		    lnet = fnet;
		}
	    }
	}
    }
    Tcl_SetObjResult(interp, Tcl_NewIntObj(failcount));

    if (stepnet >= (Numnets - 1)) stepnet = -1;

    // Restore global defaults in case they were locally changed
    forceRoutable = saveForce;

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "cleanup"					*/
/*							*/
/*   Clean up the nets by removing adjacent vias where	*/
/*   such adjacent vias would cause a DRC violation.	*/
/*   Note that this must be done between the last	*/
/*   routing stage but before finding antenna		*/
/*   violations, output, and delay writing, as all of	*/
/*   these can be dependent on topology changes caused	*/
/*   by the cleanup.					*/
/*							*/
/* Options:						*/
/*							*/
/*   cleanup all	Clean up all nets in the design	*/
/*   cleanup net <name> [...]				*/
/* 			Clean up the named net(s)	*/
/*------------------------------------------------------*/

static int
qrouter_cleanup(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    int result, idx, i;
    NET net;

    static char *subCmds[] = {
	"all", "net", NULL
    };
    enum SubIdx {
	AllIdx, NetIdx
    };

    if (objc < 2) {
	Tcl_WrongNumArgs(interp, 0, objv, "?option?");
	return TCL_ERROR;
    }
    else {
	if ((result = Tcl_GetIndexFromObj(interp, objv[1],
		(CONST84 char **)subCmds, "option", 0, &idx))
		!= TCL_OK)
	    return result;

	// Quick check to see if cleanup_net can be avoided completely.
	for (i = 0; i < Num_layers; i++)
	    if (needblock[i] & (VIABLOCKX | VIABLOCKY))
		break;
	if (i == Num_layers) return TCL_OK;        /* No cleanup needed */

	switch (idx) {
	    case AllIdx:
		for (i = 0; i < Numnets; i++) {
		   net = Nlnets[i];
		   cleanup_net(net);
		}
		break;

	    case NetIdx:
		for (i = 2; i < objc; i++) {
		    net = DefFindNet(Tcl_GetString(objv[i]));
		    if (net != NULL)
			cleanup_net(net);
		}
		break;
	}
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "remove"					*/
/*							*/
/*   Remove a net or nets, or all nets, from the	*/
/*   design.						*/
/*							*/
/* Options:						*/
/*							*/
/*   remove all		Remove all nets from the design	*/
/*   remove net <name> [...]				*/
/*			Remove the named net(s) from	*/
/*			the design.			*/
/*------------------------------------------------------*/

static int
qrouter_remove(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    int result, idx, i;
    NET net;

    static char *subCmds[] = {
	"all", "net", NULL
    };
    enum SubIdx {
	AllIdx, NetIdx
    };

    if (objc < 2) {
	Tcl_WrongNumArgs(interp, 0, objv, "?option?");
	return TCL_ERROR;
    }
    else {
	if ((result = Tcl_GetIndexFromObj(interp, objv[1],
		(CONST84 char **)subCmds, "option", 0, &idx))
		!= TCL_OK)
	    return result;

	switch (idx) {
	    case AllIdx:
		for (i = 0; i < Numnets; i++) {
		   net = Nlnets[i];
		   ripup_net(net, (u_char)1, (u_char)1, (u_char)0);
		}
		draw_layout();
		break;
	    case NetIdx:
		for (i = 2; i < objc; i++) {
		    net = DefFindNet(Tcl_GetString(objv[i]));
		    if (net != NULL)
			ripup_net(net, (u_char)1, (u_char)1, (u_char)0);
		}
		draw_layout();
		break;
	}
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "failing"					*/
/*							*/
/*   List the nets that have failed routing.		*/
/*							*/
/* Options:						*/
/*							*/
/*   failing all	Move all nets to FailedNets	*/
/*			ordered by the standard metric	*/
/*   failing unordered	Move all nets to FailedNets,	*/
/*			as originally ordered		*/
/*   failing summary	List of failed and total nets	*/
/*------------------------------------------------------*/

static int
qrouter_failing(ClientData clientData, Tcl_Interp *interp,
                int objc, Tcl_Obj *CONST objv[])
{
    Tcl_Obj *lobj;
    NETLIST nl, nlast;
    NET net;
    int i, failcount;

    if (objc == 2) {
	if (!strncmp(Tcl_GetString(objv[1]), "unorder", 7)) {
	    // Free up FailedNets list and then move all
	    // nets to FailedNets

	    while (FailedNets != NULL) {
		nl = FailedNets->next;
		FailedNets = FailedNets->next;
		free(nl);
	    }
	    nlast = NULL;
	    for (i = 0; i < Numnets; i++) {
		net = Nlnets[i];
		nl = (NETLIST)malloc(sizeof(struct netlist_));
		nl->net = net;
		nl->next = NULL;
		if (nlast == NULL)
		    FailedNets = nl;
		else
		    nlast->next = nl;
		nlast = nl;
	    }
	}
	else if (!strncmp(Tcl_GetString(objv[1]), "all", 3)) {
	    while (FailedNets != NULL) {
		nl = FailedNets->next;
		FailedNets = FailedNets->next;
		free(nl);
	    }
	    create_netorder(0);
	    nlast = NULL;
	    for (i = 0; i < Numnets; i++) {
		net = Nlnets[i];
		nl = (NETLIST)malloc(sizeof(struct netlist_));
		nl->net = net;
		nl->next = NULL;
		if (nlast == NULL)
		    FailedNets = nl;
		else
		    nlast->next = nl;
		nlast = nl;
	    }
	}
	else if (!strncmp(Tcl_GetString(objv[1]), "summary", 7)) {
	    failcount = countlist(FailedNets);
	    lobj = Tcl_NewListObj(0, NULL);
	    Tcl_ListObjAppendElement(interp, lobj, Tcl_NewIntObj(failcount));
	    Tcl_ListObjAppendElement(interp, lobj, Tcl_NewIntObj(Numnets));
	    Tcl_SetObjResult(interp, lobj);
	}
	else {
	    Tcl_WrongNumArgs(interp, 0, objv, "all or unordered");
	    return TCL_ERROR;
	}
    }
    else {

	lobj = Tcl_NewListObj(0, NULL);

	for (nl = FailedNets; nl; nl = nl->next) {
	    Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewStringObj(nl->net->netname, -1));
	}
	Tcl_SetObjResult(interp, lobj);
    }

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "read_lef"					*/
/*------------------------------------------------------*/

static int
qrouter_readlef(ClientData clientData, Tcl_Interp *interp,
                int objc, Tcl_Obj *CONST objv[])
{
    char *LEFfile;
    int mscale;
    int i;

    if (objc != 2) {
	Tcl_SetResult(interp, "No LEF filename specified!", NULL);
	return TCL_ERROR;
    }
    LEFfile = Tcl_GetString(objv[1]);

    mscale = LefRead(LEFfile);
    if (Scales.mscale < mscale) Scales.mscale = mscale;
 
    for (i = 0; i < Num_layers; i++) {

       /* Set Vert from route info since this gets called a lot	*/
       /* (e.g., from eval_pt() and is more convenient to pull	*/
       /* from an array than calling a subroutine every time.	*/

       Vert[i] = (1 - LefGetRouteOrientation(i));
    }

    /* Resolve the base horizontal and vertical pitches */
    post_config(FALSE);

    /* Set DRC blockage behavior based on via and route widths */
    apply_drc_blocks(-1, 0.0, 0.0);

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "read_def"					*/
/*------------------------------------------------------*/

static int
qrouter_readdef(ClientData clientData, Tcl_Interp *interp,
                int objc, Tcl_Obj *CONST objv[])
{
    char *argv;
    u_char abort_on_error = FALSE;
    int result;

    /* Parse out options */

    while (objc > 0) {
	argv = Tcl_GetString(objv[objc - 1]);
	if (*argv == '-') {
	    if (!strncmp(argv + 1, "abort", 5))
		abort_on_error = TRUE;
	    objc--;
	}
	else break;
    }

    if ((DEFfilename == NULL) && (objc != 2)) {
	Tcl_SetResult(interp, "No DEF filename specified!", NULL);
	return TCL_ERROR;
    }

    if (objc == 2)
	result = read_def(Tcl_GetString(objv[1]));
    else
	result = read_def(NULL);

    if ((result != (u_char)0) && (abort_on_error == TRUE)) {
	Tcl_SetResult(interp, "Errors in input DEF file;  aborting.", NULL);
	return TCL_ERROR;
    }

    // Redisplay
    draw_layout();

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "write_def"					*/
/*------------------------------------------------------*/

static int
qrouter_writedef(ClientData clientData, Tcl_Interp *interp,
                 int objc, Tcl_Obj *CONST objv[])
{
    char *DEFoutfile = NULL;

    if (objc == 2)
	DEFoutfile = Tcl_GetString(objv[1]);
    else if (DEFfilename == NULL) {
	Tcl_SetResult(interp, "No DEF filename specified!", NULL);
	return TCL_ERROR;
    }
    else DEFoutfile = DEFfilename;

    write_def(DEFoutfile);
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "antenna"					*/
/* Use:							*/
/*	antenna init <cellname>				*/
/*	antenna check					*/
/*	antenna fix					*/
/*							*/
/* Calculate and handle antenna violations.  Option	*/
/* "init" declares the cellname that is an antenna	*/
/* anchoring cell.  This must be declared before	*/
/* routing.  "antenna check" can be called at any time	*/
/* and reports the number of antenna violations at each	*/
/* metal layer.  "antenna fix" attempts to fix all 	*/
/* antenna violations by anchoring each antenna to an	*/
/* available antenna cell tap.				*/
/*------------------------------------------------------*/

static int
qrouter_antenna(ClientData clientData, Tcl_Interp *interp,
                 int objc, Tcl_Obj *CONST objv[])
{
    char *option;
    u_char do_fix = (u_char)0;

    if (objc >= 2) {
	option = Tcl_GetString(objv[1]);
	if (objc == 3) antenna_cell = strdup(Tcl_GetString(objv[2]));

	if (!strcmp(option, "init")) {
	    if (objc != 3) {
		if (antenna_cell != NULL) {
		    Tcl_SetObjResult(interp, Tcl_NewStringObj(antenna_cell, -1));
		}
		else {
		    Tcl_SetResult(interp, "No antenna cell name specified.", NULL);
		    return TCL_ERROR;
		}
	    }
	}
	else if (!strcmp(option, "check")) {
	    resolve_antenna(antenna_cell, (u_char)0);
	}
	else if (!strcmp(option, "fix")) {
	    resolve_antenna(antenna_cell, (u_char)1);
	}
	else {
	    antenna_cell = Tcl_GetString(objv[1]);
	}
    }
    else {
	Tcl_SetResult(interp, "Usage: antenna init|check|fix [cellname]", NULL);
	return TCL_ERROR;
    }

    if (antenna_cell == NULL) {
	Tcl_SetResult(interp, "No antenna cell specified!", NULL);
	return TCL_ERROR;
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "write_failed"				*/
/*------------------------------------------------------*/

static int
qrouter_writefailed(ClientData clientData, Tcl_Interp *interp,
                 int objc, Tcl_Obj *CONST objv[])
{
    char *outfile = NULL;

    if (objc == 2)
	outfile = Tcl_GetString(objv[1]);
    else if (outfile == NULL) {
	Tcl_SetResult(interp, "No output filename specified!", NULL);
	return TCL_ERROR;
    }

    write_failed(outfile);
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "write_delays"				*/
/*------------------------------------------------------*/

static int
qrouter_writedelays(ClientData clientData, Tcl_Interp *interp,
                 int objc, Tcl_Obj *CONST objv[])
{
    char *delayoutfile = NULL;

    if (objc == 2)
	delayoutfile = Tcl_GetString(objv[1]);
    else if (delayfilename == NULL) {
	Tcl_SetResult(interp, "No delay filename specified!", NULL);
	return TCL_ERROR;
    }
    else delayoutfile = delayfilename;

    write_delays(delayoutfile);
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "read_config"				*/
/*------------------------------------------------------*/

static int
qrouter_readconfig(ClientData clientData, Tcl_Interp *interp,
                   int objc, Tcl_Obj *CONST objv[])
{
    FILE *configFILE;
    char *configname = NULL;

    if (objc == 2)
	configname = Tcl_GetString(objv[1]);
    else {
	Tcl_SetResult(interp, "No configuration filename specified!",
		NULL);
	return TCL_ERROR;
    }
    configFILE = fopen(configname, "r");
    if (configFILE == NULL) {
	Tcl_SetResult(interp, "Failed to open configuration file.",
		NULL);
	return TCL_ERROR;
    }
    read_config(configFILE, FALSE);
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "obstruction"				*/
/*							*/
/* Create an obstruction manually by specifying a	*/
/* rectangular bounding box and layer of the obstructed	*/
/* area.  Without options, returns a list of the	*/
/* defined user obstruction areas.			*/
/*							*/
/* Options:						*/
/*							*/
/*	obstruction					*/
/*	obstruction <xmin> <ymin> <xmax> <ymax> <layer>	*/
/*------------------------------------------------------*/

static int
qrouter_obs(ClientData clientData, Tcl_Interp *interp,
            int objc, Tcl_Obj *CONST objv[])
{
    Tcl_Obj *lobj;
    Tcl_Obj *oobj;
    LefList lefl;
    char *layername;
    DSEG obs;
    int layer, result;
    double x1, x2, y1, y2;

    if (objc == 1) {
	lobj = Tcl_NewListObj(0, NULL);
	for (obs = UserObs; obs; obs = obs->next) {
	    lefl = LefFindLayerByNum(obs->layer);
	    if (lefl == NULL) continue;
	    oobj = Tcl_NewListObj(0, NULL);
	    Tcl_ListObjAppendElement(interp, oobj, Tcl_NewDoubleObj(obs->x1));
	    Tcl_ListObjAppendElement(interp, oobj, Tcl_NewDoubleObj(obs->x2));
	    Tcl_ListObjAppendElement(interp, oobj, Tcl_NewDoubleObj(obs->y1));
	    Tcl_ListObjAppendElement(interp, oobj, Tcl_NewDoubleObj(obs->y2));
	    Tcl_ListObjAppendElement(interp, oobj,
			Tcl_NewStringObj(lefl->lefName, -1));
	    Tcl_ListObjAppendElement(interp, lobj, oobj);
	}
	Tcl_SetObjResult(interp, lobj);
    }
    else if (objc != 6) {
	Tcl_WrongNumArgs(interp, 1, objv, "x1 x2 y1 y2 layer");
	return TCL_ERROR;
    }
    else {
	layername = Tcl_GetString(objv[5]);
	layer = LefFindLayerNum(layername);
	if (layer < 0) {
	    Tcl_SetResult(interp, "No such layer name", NULL);
	    return TCL_ERROR;
	}
	else {
	    result = Tcl_GetDoubleFromObj(interp, objv[1], &x1);
	    if (result != TCL_OK) return result;
	    result = Tcl_GetDoubleFromObj(interp, objv[2], &y1);
	    if (result != TCL_OK) return result;
	    result = Tcl_GetDoubleFromObj(interp, objv[3], &x2);
	    if (result != TCL_OK) return result;
	    result = Tcl_GetDoubleFromObj(interp, objv[4], &y2);
	    if (result != TCL_OK) return result;

	    obs = (DSEG)malloc(sizeof(struct dseg_));
	    obs->x1 = x1;
	    obs->x2 = x2;
	    obs->y1 = y1;
	    obs->y2 = y2;
	    obs->layer = layer;
	    obs->next = UserObs;
	    UserObs = obs;
	}
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "ignore"					*/
/*							*/
/* Specify one or more net names to be ignored by the	*/
/* router.  With no options, returns a list of nets	*/
/* being ignored by the router.				*/
/*							*/
/* Options:						*/
/*							*/
/*	ignore [<net> ...]				*/
/*------------------------------------------------------*/

static int
qrouter_ignore(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    int i;
    NET net;
    Tcl_Obj *lobj;

    if (objc == 1) {
	lobj = Tcl_NewListObj(0, NULL);
	for (i = 0; i < Numnets; i++) {
	    net = Nlnets[i];
	    if (net->flags & NET_IGNORED) {
		Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewStringObj(net->netname, -1));
	    }
	}
	Tcl_SetObjResult(interp, lobj);
    }
    else {
	for (i = 1; i < objc; i++) {
	    net = DefFindNet(Tcl_GetString(objv[i]));
	    if (net == NULL) {
		Tcl_SetResult(interp, "No such net", NULL);
		return TCL_ERROR;
	    }
	    net->flags |= NET_IGNORED;
	}
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "priority"					*/
/*							*/
/* Specify one or more net names to be routed first by	*/
/* the router. 	With no options, returns a list of	*/
/* nets prioritized by the router.			*/
/*							*/
/* Options:						*/
/*							*/
/*	priority [<net> ...]				*/
/*------------------------------------------------------*/

static int
qrouter_priority(ClientData clientData, Tcl_Interp *interp,
                 int objc, Tcl_Obj *CONST objv[])
{
    int i, j;
    char *netname;
    NET net;
    STRING cn, ctest;
    Tcl_Obj *lobj;

    if (objc == 1) {
	lobj = Tcl_NewListObj(0, NULL);
	for (i = 0; i < Numnets; i++) {
	    net = Nlnets[i];
	    if (net->flags & NET_CRITICAL) {
		Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewStringObj(net->netname, -1));
	    }
	}
	Tcl_SetObjResult(interp, lobj);
    }
    else if (Nlnets == NULL) {
	Tcl_SetResult(interp, "Must read nets from DEF file before setting priority.",
		NULL);
	return TCL_ERROR;
    }
    else {

	/* Find the highest-numbered existing critical net so that	*/
	/* repeated calls to "priority" will work.			*/

	j = -1;
	for (i = 0; i < Numnets; i++) {
	    net = Nlnets[i];
	    if (net->flags & NET_CRITICAL)
		if (net->netorder > j)
		    j = net->netorder;
	}
	j++;

	for (i = 1; i < objc; i++) {
	    netname = Tcl_GetString(objv[i]);
	    net = DefFindNet(netname);
	    if (net == NULL) {
		Tcl_SetResult(interp, "No such net", NULL);
	    }
	    else if (!(net->flags & NET_CRITICAL)) {
		net->flags |= NET_CRITICAL;
		net->netorder = j++;

		/* NOTE:  CriticalNet is left over from the old config	*/
		/* file format.  Normally it will remain NULL.  If the	*/
		/* old config file format is used, then remove items	*/
		/* from it that match nets in the Tcl priority command.	*/

		for (cn = CriticalNet; cn && cn->next; cn = cn->next) {
		    if (!cn->next) break;
		    if (!strcmp(cn->next->name, netname)) {
			ctest = cn->next;
			cn->next = cn->next->next;
			ctest->next = CriticalNet;
			CriticalNet = ctest;
		    }
		}
	    }
	}
	create_netorder(0);
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "layer_info"					*/
/*							*/
/* Provide information gleaned from the LEF technology	*/
/* file or files					*/
/*							*/
/* Options:						*/
/*							*/
/*	layer_info [all]				*/
/*	layer_info <layername>|<layernum>		*/
/*	layer_info <layername>|<layernum> width		*/
/*	layer_info <layername>|<layernum> pitch		*/
/*	layer_info <layername>|<layernum> orientation	*/
/*	layer_info <layername>|<layernum> offset	*/
/*	layer_info <layername>|<layernum> spacing	*/
/*	layer_info maxlayer				*/
/*							*/
/* all: generate a list summary of route information	*/
/*	in the format of the info file generated with 	*/
/*	the "-i" command-line switch.			*/
/* <layer>: generate a list summary of information for	*/
/*	the specified route layer <layer>, format the	*/
/*	same as for option "all".			*/
/* <layer> pitch:  Return the layer pitch value.	*/
/* <layer> orientation:  Return the layer orientation.	*/
/* <layer> offset:  Return the layer offset value.	*/
/* <layer> spacing:  Return the layer minimum spacing	*/
/*	value.						*/
/* maxlayer:  Return the maximum number of route layers	*/
/* 	defined by the technology.			*/
/*							*/
/* No option is the same as option "layer_info all"	*/
/*------------------------------------------------------*/

static int
qrouter_layerinfo(ClientData clientData, Tcl_Interp *interp,
                  int objc, Tcl_Obj *CONST objv[])
{
    Tcl_Obj *lobj, *oobj;
    int i, idx, idx2, val, result, layer = -1;
    char *layername;

    static char *subCmds[] = {
	"all", "maxlayer", NULL
    };
    enum SubIdx {
	AllIdx, MaxLayerIdx
    };
   
    static char *layerSubCmds[] = {
	"width", "pitch", "orientation", "offset", "spacing", NULL
    };
    enum layerSubIdx {
	WidthIdx, PitchIdx, OrientIdx, OffsetIdx, SpacingIdx
    };

    idx = idx2 = -1;

    if (objc < 2) {
	idx = AllIdx;
    }
    else {
	layername = Tcl_GetString(objv[1]);
	layer = LefFindLayerNum(layername);
	if (layer == -1) {
	    if ((result = Tcl_GetIntFromObj(interp, objv[1], &val)) == TCL_OK) {
		layer = val;
	    }
	    else {
		Tcl_ResetResult(interp);
	
		if ((result = Tcl_GetIndexFromObj(interp, objv[1],
			(CONST84 char **)subCmds, "option", 0, &idx))
			!= TCL_OK) {
		    return result;
		}
	    }
	}
	else if (objc >= 3) { 
	    if ((result = Tcl_GetIndexFromObj(interp, objv[2],
			(CONST84 char **)layerSubCmds, "option", 0, &idx2))
			!= TCL_OK) {
		return result;
	    }
	    layer = LefFindLayerNum(layername);
	}
	else {
	    layer = LefFindLayerNum(layername);
	}
    }

    if (idx == -1 && layer == -1) {
	Tcl_SetResult(interp, "Bad layer", NULL);
	return TCL_ERROR;
    }
    if (layer < 0 || layer >= Num_layers) {
	Tcl_SetResult(interp, "Bad layer", NULL);
	return TCL_ERROR;
    }

    switch (idx) {
	case AllIdx:
	    oobj = Tcl_NewListObj(0, NULL);
	    for (i = 0; i < Num_layers; i++) {
		lobj = Tcl_NewListObj(0, NULL);
		Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewStringObj(LefGetRouteName(i), -1));
		Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewDoubleObj(LefGetRoutePitch(i)));
		Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewDoubleObj(LefGetRouteWidth(i)));
		if (LefGetRouteOrientation(i) == 1)
		    Tcl_ListObjAppendElement(interp, lobj,
				Tcl_NewStringObj("horizontal", -1));
		else
		    Tcl_ListObjAppendElement(interp, lobj,
				Tcl_NewStringObj("vertical", -1));
		Tcl_ListObjAppendElement(interp, oobj, lobj);
	    }
	    Tcl_SetObjResult(interp, oobj);
	    break;
	case MaxLayerIdx:
	    Tcl_SetObjResult(interp, Tcl_NewIntObj(Num_layers));
	    break;
    }

    switch (idx2) {
	case WidthIdx:
	    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(LefGetRouteWidth(layer)));
	    break;
	case PitchIdx:
	    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(LefGetRoutePitch(layer)));
	    break;
	case OrientIdx:
	    if (LefGetRouteOrientation(layer) == (u_char)0)
		Tcl_SetObjResult(interp, Tcl_NewStringObj("vertical", -1));
	    else
		Tcl_SetObjResult(interp, Tcl_NewStringObj("horizontal", -1));
	    break;
	case OffsetIdx:
	    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(LefGetRouteOffset(layer)));
	    break;
	case SpacingIdx:
	    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(LefGetRouteSpacing(layer)));
	    break;
	default:
	    if (idx != -1) break;
	    lobj = Tcl_NewListObj(0, NULL);
	    Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewStringObj(LefGetRouteName(layer), -1));
	    Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewDoubleObj(LefGetRoutePitch(layer)));
	    Tcl_ListObjAppendElement(interp, lobj,
			Tcl_NewDoubleObj(LefGetRouteWidth(layer)));
	    if (LefGetRouteOrientation(layer) == 1)
		Tcl_ListObjAppendElement(interp, lobj,
				Tcl_NewStringObj("horizontal", -1));
	    else
		Tcl_ListObjAppendElement(interp, lobj,
				Tcl_NewStringObj("vertical", -1));
	    Tcl_SetObjResult(interp, lobj);
	    break;
    }
    return TCL_OK;
}

/*------------------------------------------------------*/
/* Command "via"					*/
/*							*/
/* Various via configuration options for qrouter.	*/
/*							*/
/* stack: Value is the maximum number of vias that may	*/
/*	  be stacked directly on top of each other at	*/
/*	  a single point.  Value "none", "0", and "1"	*/
/*	  all mean the same thing.			*/
/*							*/
/* via_pattern:  (deprecated)				*/
/*							*/
/* use: List of names of vias to use.  If any via not	*/
/*	in this list is found when reading a .lef file	*/
/*	it will be ignored.				*/
/*							*/
/* Options:						*/
/*							*/
/*	via stack [none|all|<value>]			*/
/*	via pattern [normal|inverted]			*/
/*	via use <via_name> [<via_name> ...]		*/
/*------------------------------------------------------*/

static int
qrouter_via(ClientData clientData, Tcl_Interp *interp,
            int objc, Tcl_Obj *CONST objv[])
{
    int idx, idx2, result, value, i;
    char *vname;
    Tcl_Obj *lobj;
    LinkedStringPtr viaName, newVia;

    static char *subCmds[] = {
	"stack", "pattern", "use", NULL
    };
    enum SubIdx {
	StackIdx, PatternIdx, UseIdx
    };
   
    static char *stackSubCmds[] = {
	"none", "all", NULL
    };
    enum stackSubIdx {
	NoneIdx, AllIdx
    };

    static char *patternSubCmds[] = {
	"none", "normal", "inverted", NULL
    };
    enum patternSubIdx {
	PatNoneIdx, PatNormalIdx, PatInvertIdx
    };

    if (objc >= 2) {
	if ((result = Tcl_GetIndexFromObj(interp, objv[1],
		(CONST84 char **)subCmds, "option", 0, &idx))
		!= TCL_OK)
	    return result;

	if (objc == 2) {
	    switch (idx) {
		case StackIdx:
		    Tcl_SetObjResult(interp, Tcl_NewIntObj(StackedContacts));
		    break;
		case PatternIdx:
		    Tcl_SetObjResult(interp,
				Tcl_NewStringObj("deprecated", -1));
		    break;
		case UseIdx:
		    /* Return list of vias to use */
		    lobj = Tcl_NewListObj(0, NULL);
		    for (viaName = AllowedVias; viaName; viaName = viaName->next) {
			Tcl_ListObjAppendElement(interp, lobj,
				Tcl_NewStringObj(viaName->name, -1));
		    }
		    Tcl_SetObjResult(interp, lobj);
		    break;
	    }
	}
	else {
	    switch (idx) {
		case StackIdx:
		    result = Tcl_GetIntFromObj(interp, objv[2], &value);
		    if (result == TCL_OK) {
			if (value <= 0) value = 1;
			else if (value >= Num_layers) value = Num_layers - 1;
			StackedContacts = value;
			break;
		    }
		    Tcl_ResetResult(interp);
		    if ((result = Tcl_GetIndexFromObj(interp, objv[2],
				(CONST84 char **)stackSubCmds, "option",
				0, &idx2)) != TCL_OK)
			return result;
		    switch (idx2) {
			case NoneIdx:
			    StackedContacts = 1;
			    break;
			case AllIdx:
			    StackedContacts = Num_layers - 1;
			    break;
		    }
		    break;
		case PatternIdx:
		    if ((result = Tcl_GetIndexFromObj(interp, objv[2],
				(CONST84 char **)patternSubCmds, "option",
				0, &idx2)) != TCL_OK)
			return result;
		    break;
		case UseIdx:
		    /* Create list of vias to use */
		    for (i = 2; i < objc; i++) {
			vname = Tcl_GetString(objv[i]);
			/* First check if name is in list already */
			for (viaName = AllowedVias; viaName; viaName = viaName->next) {
			    if (!strcmp(vname, viaName->name))
				break;
			}	
			if (viaName != NULL) continue;
			newVia = (LinkedStringPtr)malloc(sizeof(LinkedString));
			newVia->name = strdup(vname);
			newVia->next = AllowedVias;
			AllowedVias = newVia;
		    }
		    /* Regenerate the ViaXX, etc., lists */
		    LefAssignLayerVias();
		    break;
	    }
	}
    }
    else {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "verbose"					*/
/*							*/
/* Set the level of how much output to print to the	*/
/* console.  0 is minimal output, 4 is maximum output	*/
/* With no argument, return the level of verbose. 	*/
/*							*/
/* Options:						*/
/*							*/
/*	resolution [<value>]				*/
/*------------------------------------------------------*/

static int
qrouter_verbose(ClientData clientData, Tcl_Interp *interp,
                int objc, Tcl_Obj *CONST objv[])
{
    int result, value;

    if (objc == 1) {
	Tcl_SetObjResult(interp, Tcl_NewIntObj(Verbose));
    }
    else if (objc == 2) {

	result = Tcl_GetIntFromObj(interp, objv[1], &value);
	if (result != TCL_OK) return result;
	if (value < 0 || value > 255) {
	    Tcl_SetResult(interp, "Verbose level out of range", NULL);
	    return TCL_ERROR;
	}
	Verbose = (u_char)value;
    }
    else {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }
    return QrouterTagCallback(interp, objc, objv);
}


/*------------------------------------------------------*/
/* Command "resolution"					*/
/*							*/
/* Set the level of resolution of the output relative	*/
/* to the base units of centimicrons.  The default	*/
/* value of 1 rounds all output values to the nearest	*/
/* centimicron;  value 10 is to the nearest nanometer,	*/
/* and so forth.					*/
/* With no argument, return the value of the output	*/
/* resolution.						*/
/*							*/
/* Options:						*/
/*							*/
/*	resolution [<value>]				*/
/*------------------------------------------------------*/

static int
qrouter_resolution(ClientData clientData, Tcl_Interp *interp,
                   int objc, Tcl_Obj *CONST objv[])
{
    int result, value;

    if (objc == 1) {
	Tcl_SetObjResult(interp, Tcl_NewIntObj(Scales.iscale));
    }
    else if (objc == 2) {

	result = Tcl_GetIntFromObj(interp, objv[1], &value);
	if (result != TCL_OK) return result;
	if (value < 1) {
	    Tcl_SetResult(interp, "Resolution out of range", NULL);
	    return TCL_ERROR;
	}
	Scales.iscale = value;
    }
    else {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "drc"					*/
/*							*/
/* Set qrouter options related to handling of DRC	*/
/* violations.						*/
/*							*/
/* Options:						*/
/*							*/
/*	drc <layer>|all <dist> <dist>			*/
/*							*/
/* Allow exceptions to DRC handling.  Normally qrouter	*/
/* enforces DRC distance between a via and route or	*/
/* between two vias on adjacent tracks, forcing a 	*/
/* keep-out area around a placed route if needed.	*/
/* "layer" is the name of a route layer.  <dist> (value	*/
/* in microns) will limit the enforcement if the DRC	*/
/* violation is less than the indicated distance.	*/
/* The first value is for via-to-via distance, and the	*/
/* second value is for route-to-via distance.  A value	*/
/* of zero means that the behavior is unchanged from	*/
/* what is automatically calculated from defined	*/
/* route and via width and spacing values.  A positive	*/
/* distance value loosens the DRC rule, while a		*/
/* negative distance value tightens it.			*/
/*							*/
/* Ignoring DRC errors is generally discouraged but	*/
/* may be necessary in some cases if the pitch is tight	*/
/* and assumes routes may be offset to clear vias,	*/
/* which is something qrouter does not know how to do.	*/
/* Only use this if routability is being impacted by	*/
/* DRC enforcement.					*/
/*------------------------------------------------------*/

static int
qrouter_drc(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    char *layername;
    int result, layer;
    double routedist, viadist;

    if (objc != 4) {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }

    layername = Tcl_GetString(objv[1]);
    if (!strcasecmp(layername, "all")) {
	layer = -1;
    }
    else {
	layer = LefFindLayerNum(layername);
	if (layer < 0) {
	    result = Tcl_GetIntFromObj(interp, objv[1], &layer);
	    if (result != TCL_OK) {
		Tcl_SetResult(interp, "No such layer name.\n", NULL);
		return result;
	    }
	}

	if ((layer < -1) || (layer >= LefGetMaxRouteLayer())) {
	    Tcl_SetResult(interp, "Layer number out of range.\n", NULL);
	    return TCL_ERROR;
	}
    }

    result = Tcl_GetDoubleFromObj(interp, objv[2], &viadist);
    if (result != TCL_OK) return result;

    result = Tcl_GetDoubleFromObj(interp, objv[3], &routedist);
    if (result != TCL_OK) return result;

    apply_drc_blocks(layer, viadist, routedist);

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "layers"					*/
/*							*/
/* Set the number of layers used for routing		*/
/* independently of the number defined in the		*/
/* technology LEF file.	 Without an argument, return	*/
/* the number of layers being used (this is not the 	*/
/* same as "layer_info maxlayer")			*/
/*							*/
/* Options:						*/
/*							*/
/*	layers [<number>]				*/
/*------------------------------------------------------*/

static int
qrouter_layers(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    int result, value;

    if (objc == 1) {
	Tcl_SetObjResult(interp, Tcl_NewIntObj(Num_layers));
    }
    else if (objc == 2) {
	result = Tcl_GetIntFromObj(interp, objv[1], &value);
	if (result != TCL_OK) return result;
	if (value <= 0 || value > LefGetMaxRouteLayer()) {
	    Tcl_SetResult(interp, "Number of layers out of range,"
			" setting to max.", NULL);
	    Num_layers = LefGetMaxRouteLayer();
	    return TCL_ERROR;
	}
	Num_layers = value;
    }
    else {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "passes"					*/
/*							*/
/* Set the maximum number of attempted passes of the	*/
/* route search algorithm, where the routing		*/
/* constraints (maximum cost and route area mask) are	*/
/* relaxed on each pass.				*/
/* With no argument, return the value for the maximum	*/
/* number of passes.					*/
/* The default number of passes is 10 and normally	*/
/* there is no reason for the user to change it.	*/
/*							*/
/* Options:						*/
/*							*/
/*	passes [<number>]				*/
/*------------------------------------------------------*/

static int
qrouter_passes(ClientData clientData, Tcl_Interp *interp,
               int objc, Tcl_Obj *CONST objv[])
{
    int result, value;

    if (objc == 1) {
	Tcl_SetObjResult(interp, Tcl_NewIntObj(Numpasses));
    }
    else if (objc == 2) {
	result = Tcl_GetIntFromObj(interp, objv[1], &value);
	if (result != TCL_OK) return result;
	if (value <= 0) {
	    Tcl_SetResult(interp, "Number of passes out of range", NULL);
	    return TCL_ERROR;
	}
	Numpasses = value;
    }
    else {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "vdd"					*/
/*							*/
/* Set the name of the net used for VDD power routing	*/
/* With no argument, return the name of the power	*/
/* net.							*/
/*							*/
/* Options:						*/
/*							*/
/*	vdd [<name>]					*/
/*------------------------------------------------------*/

static int
qrouter_vdd(ClientData clientData, Tcl_Interp *interp,
            int objc, Tcl_Obj *CONST objv[])
{
    if (objc == 1) {
	if (vddnet == NULL)
	    Tcl_SetObjResult(interp, Tcl_NewStringObj("(none)", -1));
	else
	    Tcl_SetObjResult(interp, Tcl_NewStringObj(vddnet, -1));
    }
    else if (objc == 2) {
	if (vddnet != NULL) free(vddnet);
	vddnet = strdup(Tcl_GetString(objv[1]));
    }
    else {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "gnd"					*/
/*							*/
/* Set the name of the net used for ground routing.	*/
/* With no argument, return the name of the ground	*/
/* net.							*/
/*							*/
/* Options:						*/
/*							*/
/*	gnd [<name>]					*/
/*------------------------------------------------------*/

static int
qrouter_gnd(ClientData clientData, Tcl_Interp *interp,
            int objc, Tcl_Obj *CONST objv[])
{
    if (objc == 1) {
	if (gndnet == NULL)
	    Tcl_SetObjResult(interp, Tcl_NewStringObj("(none)", -1));
	else
	    Tcl_SetObjResult(interp, Tcl_NewStringObj(gndnet, -1));
    }
    else if (objc == 2) {
	if (gndnet != NULL) free(gndnet);
	gndnet = strdup(Tcl_GetString(objv[1]));
    }
    else {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }
    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "cost"					*/
/*							*/
/* Query or set the value of a cost function		*/
/*							*/
/* Options:						*/
/*							*/
/*	cost segment					*/
/*	cost via					*/
/*	cost jog					*/
/*	cost crossover					*/
/*	cost block					*/
/*	cost offset					*/
/*	cost conflict					*/
/*------------------------------------------------------*/

static int
qrouter_cost(ClientData clientData, Tcl_Interp *interp,
             int objc, Tcl_Obj *CONST objv[])
{
    int idx, result, value;

    static char *subCmds[] = {
	"segment", "via", "jog", "crossover",
	"block", "offset", "conflict", NULL
    };
    enum SubIdx {
	SegIdx, ViaIdx, JogIdx, XOverIdx, BlockIdx, OffsetIdx, ConflictIdx
    };
   
    value = 0;
    if (objc == 3) {
	result = Tcl_GetIntFromObj(interp, objv[2], &value);
	if (result != TCL_OK) return result;

	// Disallow negative costs.

	if (value < 0) {
	    Tcl_SetResult(interp, "Bad cost value", NULL);
	    return TCL_ERROR;
	}
    }
    else if (objc != 2) {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }

    if ((result = Tcl_GetIndexFromObj(interp, objv[1],
		(CONST84 char **)subCmds, "option", 0, &idx)) != TCL_OK)
	return result;

    switch (idx) {
	case SegIdx:
	case ViaIdx:
	case ConflictIdx:

	// Segment, via, and conflict costs must not be zero or
	// bad things happen.

	    if (value <= 0) {
		Tcl_SetResult(interp, "Bad cost value", NULL);
		return TCL_ERROR;
	    }
    }

    switch (idx) {
	case SegIdx:
	    if (objc == 2)
		Tcl_SetObjResult(interp, Tcl_NewIntObj(SegCost));
	    else
		SegCost = value;
	    break;

	case ViaIdx:
	    if (objc == 2)
		Tcl_SetObjResult(interp, Tcl_NewIntObj(ViaCost));
	    else
		ViaCost = value;
	    break;

	case JogIdx:
	    if (objc == 2)
		Tcl_SetObjResult(interp, Tcl_NewIntObj(JogCost));
	    else
		JogCost = value;
	    break;

	case XOverIdx:
	    if (objc == 2)
		Tcl_SetObjResult(interp, Tcl_NewIntObj(XverCost));
	    else
		XverCost = value;
	    break;

	case OffsetIdx:
	    if (objc == 2)
		Tcl_SetObjResult(interp, Tcl_NewIntObj(OffsetCost));
	    else
		OffsetCost = value;
	    break;

	case BlockIdx:
	    if (objc == 2)
		Tcl_SetObjResult(interp, Tcl_NewIntObj(BlockCost));
	    else
		BlockCost = value;
	    break;

	case ConflictIdx:
	    if (objc == 2)
		Tcl_SetObjResult(interp, Tcl_NewIntObj(ConflictCost));
	    else
		ConflictCost = value;
	    break;
    }

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/

typedef struct clist_ *CLIST;

typedef struct clist_ {
   GATE gate;
   double congestion;
} Clist;

/*------------------------------------------------------*/
/* Compare function for qsort()				*/
/*------------------------------------------------------*/

int compcong(CLIST *a, CLIST *b)
{
   CLIST p = *a;
   CLIST q = *b;

   if (p->congestion < q->congestion)
      return (1);
   else if (p->congestion > q->congestion)
      return (-1);
   else
      return (0);
}

/*------------------------------------------------------*/
/* Command "congested"					*/
/*							*/
/* Return a list of instances, ranked by congestion.	*/
/* This list can be passed back to a placement tool to	*/
/* direct it to add padding around these cells to ease	*/
/* congestion and make the layout more routable.	*/
/*							*/
/* Options:						*/
/*							*/
/* congested <n>	List the top <n> congested	*/
/*			gates in the design.		*/
/*------------------------------------------------------*/

static int
qrouter_congested(ClientData clientData, Tcl_Interp *interp,
                  int objc, Tcl_Obj *CONST objv[])
{
    NET net;
    int i, x, y, nwidth, nheight, area, length;
    int entries, numgates, result;
    float density, *Congestion;
    CLIST *cgates, csrch;
    GATE gsrch;
    struct seg_ bbox;
    double dx, dy, cavg;
    Tcl_Obj *lobj, *dobj;

    if (objc == 2) {
	result = Tcl_GetIntFromObj(interp, objv[1], &entries);
	if (result != TCL_OK) return result;

	if (entries <= 0) {
	    Tcl_SetResult(interp, "List size must be > 0", NULL);
	    return TCL_ERROR;
	}
    }
    else
	entries = 0;

    Congestion = (float *)calloc(NumChannelsX * NumChannelsY,
			sizeof(float));

    // Use net bounding boxes to estimate congestion

    for (i = 0; i < Numnets; i++) {
	net = Nlnets[i];
	nwidth = (net->xmax - net->xmin + 1);
	nheight = (net->ymax - net->ymin + 1);
	area = nwidth * nheight;
	if (nwidth > nheight) {
	    length = nwidth + (nheight >> 1) * net->numnodes;
	}
	else {
	    length = nheight + (nwidth >> 1) * net->numnodes;
	}
	density = (float)length / (float)area;

	for (x = net->xmin; x < net->xmax; x++)
	    for (y = net->ymin; y < net->ymax; y++)
		if (x >= 0 && x < NumChannelsX &&
			y >= 0 && y < NumChannelsY)
		    CONGEST(x, y) += density;
    }

    // Use instance bounding boxes to estimate average congestion
    // in the area of an instance.

    numgates = 0;
    for (gsrch = Nlgates; gsrch; gsrch = gsrch->next) numgates++;

    cgates = (CLIST *)malloc(numgates * sizeof(CLIST));
    i = 0;

    for (gsrch = Nlgates; gsrch; gsrch = gsrch->next) {

	// Ignore pins, as the point of congestion planning
	// is to add padding to the core instances;  including
	// pins just makes it harder for the placement script
	// to process the information.

	if (gsrch->gatetype == PinMacro) continue;

	cgates[i] = (CLIST)malloc(sizeof(Clist));
	dx = gsrch->placedX;
	dy = gsrch->placedY;
	bbox.x1 = (int)((dx - Xlowerbound) / PitchX) - 1;
	bbox.y1 = (int)((dy - Ylowerbound) / PitchY) - 1;
	dx = gsrch->placedX + gsrch->width;
	dy = gsrch->placedY + gsrch->height;
	bbox.x2 = (int)((dx - Xlowerbound) / PitchX) - 1;
	bbox.y2 = (int)((dy - Ylowerbound) / PitchY) - 1;

	cavg = 0.0;
	for (x = bbox.x1; x <= bbox.x2; x++) {
	    for (y = bbox.y1; y <= bbox.y2; y++) {
		cavg += CONGEST(x, y);
	    }
	}
	cavg /= (bbox.x2 - bbox.x1 + 1);
	cavg /= (bbox.y2 - bbox.y1 + 1);

	cgates[i]->gate = gsrch;
	cgates[i++]->congestion = cavg / Num_layers;
    }

    // Re-set "numgates", because we have rejected pins from the
    // original count.
    numgates = i;

    // Order Clist and copy contents back to the interpreter as a list

    qsort((char *)cgates, numgates, (int)sizeof(CLIST), (__compar_fn_t)compcong);

    lobj = Tcl_NewListObj(0, NULL);

    if (entries == 0) entries = numgates;
    else if (entries > numgates) entries = numgates;
    for (i = 0; i < entries; i++) {
	csrch = cgates[i];
	gsrch = csrch->gate;
	dobj = Tcl_NewListObj(0, NULL);
	Tcl_ListObjAppendElement(interp, dobj,
			Tcl_NewStringObj(gsrch->gatename, -1));
	Tcl_ListObjAppendElement(interp, dobj,
			Tcl_NewDoubleObj(csrch->congestion));
	Tcl_ListObjAppendElement(interp, lobj, dobj);
    }
    Tcl_SetObjResult(interp, lobj);

    // Cleanup
    free(Congestion);
    for (i = 0; i < numgates; i++) free(cgates[i]);
    free(cgates);

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
/* Command "print"					*/
/*							*/
/* print an object                          		*/
/*							*/
/* Options:						*/
/*							*/
/*	print net <netname> 				*/
/*	print netnr <netnumber>				*/
/*      print gate <gatename>				*/
/*------------------------------------------------------*/

static int
qrouter_print(ClientData clientData, Tcl_Interp *interp,
              int objc, Tcl_Obj *CONST objv[])
{
    int idx, result, value;
    NET net;
    GATE gate;

    static char *subCmds[] = {
	"net", "netnr", "gate", NULL
    };
    enum SubIdx {
	NetIdx, NetNrIdx, GateIdx
    };

    value = 0;
    if (objc != 3) {
	Tcl_WrongNumArgs(interp, 1, objv, "option ?arg?");
	return TCL_ERROR;
    }

    if ((result = Tcl_GetIndexFromObj(interp, objv[1],
		(CONST84 char **)subCmds, "option", 0, &idx)) != TCL_OK)
	return result;

    switch (idx) {
        case NetIdx:
            net = DefFindNet(Tcl_GetString(objv[2]));
            if (net == NULL) {
                Tcl_SetResult(interp, "Net not found", NULL);
                return TCL_ERROR;
            }
            print_net(net);
            break;

        case NetNrIdx:
            result = Tcl_GetIntFromObj(interp, objv[2], &value);
            if (result != TCL_OK) return result;
            net = LookupNetNr(value);
            if (net == NULL) {
                Tcl_SetResult(interp, "Net not found", NULL);
                return TCL_ERROR;
            }
            print_net(net);
            break;

        case GateIdx:
            gate = DefFindGate(Tcl_GetString(objv[2]));
            if (gate == NULL) {
                Tcl_SetResult(interp, "Gate not found", NULL);
                return TCL_ERROR;
            }
            print_gate(gate);
    }

    return QrouterTagCallback(interp, objc, objv);
}

/*------------------------------------------------------*/
