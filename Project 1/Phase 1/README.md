# Phase 1

**Problem Statement**

For this assignment, a planner is written which can generate paths from a start position to an end position in a known 3D environment.

**Running the Code**

To get the output from the code, run the following lines

map = load_map('./sample_maps/map0.txt', 0.1, 1, 0.5)

[path, num_expanded] = dijkstra(map, [0 -5 0], [10 20 6], 0)

plot_path(map, path)
