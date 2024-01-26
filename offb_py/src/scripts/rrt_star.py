#! /usr/bin/env python3

path_planning_node_x = [-13.4326, -9.8068, -7.8753, -7.8345, -3.5050, 0.3540, 0]
path_planning_node_y = [-2.4703, -1.5131, 1.6073, 6.5957, 8.0252, 5.4986, 0]

#path_planning_node_x = [-1, -2, -3]
#path_planning_node_y = [-2, -3, -1]

num_node = len(path_planning_node_x)

path_planning_node_x = list(map(lambda x: x - path_planning_node_x[0], path_planning_node_x))
path_planning_node_y = list(map(lambda y: y - path_planning_node_y[0], path_planning_node_y))
