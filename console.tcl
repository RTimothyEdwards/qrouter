# Tcl commands to run in the console before qrouter is initialized
 
slave alias qrouter::consoledown wm withdraw .
slave alias qrouter::consoleup wm deiconify .
slave alias qrouter::consoleontop raise .

# NOTE:  This is not recommended for qrouter, where everything runs
# off of the console.  If the console is closed, then the program
# should exit.
#
# wm protocol . WM_DELETE_WINDOW {tkcon slave slave qrouter::lowerconsole}
