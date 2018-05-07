/*--------------------------------------------------------------*/
/* maze.h -- details of maze router                            	*/
/*--------------------------------------------------------------*/
/* Written by Tim Edwards, June 2011, based on work of Steve	*/
/* Beccue.							*/
/*--------------------------------------------------------------*/

#ifndef MAZE_H

int	set_powerbus_to_net(int netnum);
int     set_node_to_net(NODE node, int newnet, POINT *pushlist,
		SEG bbox, u_char stage);
int	disable_node_nets(NODE node);
int     set_routes_to_net(NODE node, NET net, int newnet, POINT *pushlist,
		SEG bbox, u_char stage);
NODE    find_unrouted_node(NET net);
void	remove_routes(ROUTE netroutes, u_char flagged);
u_char  ripup_net(NET net, u_char restore, u_char topmost, u_char retain);
POINT   eval_pt(GRIDP *ept, u_char flags, u_char stage);
int     commit_proute(ROUTE rt, GRIDP *ept, u_char stage);
void	writeback_segment(SEG seg, int netnum);
int     writeback_route(ROUTE rt);
int     writeback_all_routes(NET net);
NETLIST find_colliding(NET net, int *ripnum);
void    clear_non_source_targets(NET net, POINT *pushlist);
void    clear_target_node(NODE node);
int     count_targets(NET net);
int	set_route_to_net(NET net, ROUTE rt, int newflags, POINT *pushlist,
		SEG bbox, u_char stage);
void    route_set_connections(NET net, ROUTE route);


#define MAZE_H
#endif 

/* end of maze.h */
