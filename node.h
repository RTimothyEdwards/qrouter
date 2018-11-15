/*--------------------------------------------------------------*/
/* node.h -- general purpose autorouter                      	*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, based on code by Steve Beccue, 2003  */
/*--------------------------------------------------------------*/

#ifndef NODE_H

#define GND_NET		 1
#define VDD_NET		 2
#define ANTENNA_NET	 3
#define MIN_NET_NUMBER   4

void find_bounding_box(NET net);
void defineRouteTree(NET);
void print_nodes(char *filename);
void print_nlnets(char *filename);
void count_reachable_taps();
void check_variable_pitch(int, int *, int *);
void create_obstructions_from_variable_pitch(void);
void count_pinlayers(void);
void create_obstructions_from_gates(void);
void expand_tap_geometry(void);
void create_obstructions_inside_nodes(void);
void create_obstructions_outside_nodes(void);
void tap_to_tap_interactions(void);
void make_routable(NODE node);
void adjust_stub_lengths(void);
void block_route(int x, int y, int lay, u_char dir);
void find_route_blocks();
void clip_gate_taps(void);

#define NODE_H
#endif 


/* end of node.h */

