function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

% CMSC 828T Proj 1 Phase 1

if nargin < 4
    astar = false;
end

%% START YOUR CODE HERE %%
if collide(map,start) || collide(map,goal)
    path = [];
    num_expanded = 0;
    return
end
map{8} = true;

tol = map{5}/1e4;
x = find(start(1)<=map{2},1);
if isempty(x)
    x = length(map{2});
elseif (start(1)+(map{5}/2))<map{2}(x) && x~=1
    x = x - 1;
end
y = find(start(2)<=map{3},1);
if isempty(y)
    y = length(map{3});
elseif (start(2)+(map{5}/2))<map{3}(y) && y~=1
    y = y - 1;
end
z = find(start(3)<=map{4},1);
if isempty(z)
    z = length(map{4});
elseif (start(3)+(map{6}/2))<map{4}(z) && z~=1
    z = z - 1;
end
start_pos = [map{2}(x), map{3}(y), map{4}(z)];

x = find(goal(1)<=map{2},1);
if isempty(x)
    x = length(map{2});
elseif (goal(1)+(map{5}/2))<map{2}(x) && x~=1
    x = x - 1;
end
y = find(goal(2)<=map{3},1);
if isempty(y)
    y = length(map{3});
elseif (goal(2)+(map{5}/2))<map{3}(y) && y~=1
    y = y - 1;
end
z = find(goal(3)<=map{4},1);
if isempty(z)
    z = length(map{4});
elseif (goal(3)+(map{6}/2))<map{4}(z) && z~=1
    z = z - 1;
end
goal_pos = [map{2}(x), map{3}(y), map{4}(z)];
clear x y z

init_node.pos = start_pos;
init_node.gcost = 0;
if astar
    init_node.hcost = norm(start_pos-goal_pos);
else
    init_node.hcost = 0;
end
init_node.fcost = init_node.gcost + init_node.hcost;
init_node.parent = struct;

OPEN = containers.Map(init_node.fcost,init_node);
k = init_node.fcost;

actions = [map{5},0,0; -map{5},0,0; 0,map{5},0; 0,-map{5},0; 0,0,map{6}; 0,0,-map{6};
           map{5},map{5},0; map{5},-map{5},0; -map{5},map{5},0; -map{5},-map{5},0;
           0,map{5},map{6}; 0,map{5},-map{6}; 0,-map{5},map{6}; 0,-map{5},-map{6};
           map{5},0,map{6}; map{5},0,-map{6}; -map{5},0,map{6}; -map{5},0,-map{6};
           map{5},map{5},map{6}; map{5},-map{5},map{6}; -map{5},map{5},map{6}; -map{5},-map{5},map{6};
           map{5},map{5},-map{6}; map{5},-map{5},-map{6}; -map{5},map{5},-map{6}; -map{5},-map{5},-map{6}];

CLOSED = [];
path = goal_pos;

num_expanded = 0;
% Search algorithm
while ~isempty(OPEN)
    if length(OPEN(min(k)))~=1
        OPEN_temp = OPEN(min(k));
        current_node = OPEN_temp(1);
        OPEN_temp(1:end-1) = OPEN_temp(2:end);
        OPEN_temp(end) = [];
        OPEN(min(k)) = OPEN_temp;
    else
        current_node = OPEN(min(k));
        remove(OPEN,min(k));
        k = k(abs(k-min(k))>tol);
    end
    
    if abs(current_node.pos - goal_pos)<tol
        parent_node = current_node.parent;
        while sum(abs(parent_node.pos - start_pos)>tol)
            path = [path;parent_node.pos];
            parent_node = parent_node.parent;
        end
        break;
    end
    
    if ~isempty(CLOSED)
        match_x = find(abs(CLOSED(:,1) - current_node.pos(1))<tol);
        if ~isempty(match_x)
            match_xy = find(abs(CLOSED(match_x,2) - current_node.pos(2))<tol);
            if ~isempty(match_xy)
                match_xyz = find(abs(CLOSED(match_x(match_xy),3) - current_node.pos(3))<tol,1);
                if ~isempty(match_xyz)
                    continue;
                end
            end
        end
    end 
    CLOSED = [CLOSED;current_node.pos];
    % expand
    num_expanded = num_expanded + 1;
    for i=1:length(actions)
        neighbor_node.pos = current_node.pos + actions(i,:);
        % check if intersects with an obstacle (also checks if it lie
        % inside the boundary of the map)
        if collide(map,neighbor_node.pos)
            continue;
        end
        % check if closed already
        match_x = find(abs(CLOSED(:,1) - neighbor_node.pos(1))<tol);
        if ~isempty(match_x)
            match_xy = find(abs(CLOSED(match_x,2) - neighbor_node.pos(2))<tol);
            if ~isempty(match_xy)
                match_xyz = find(abs(CLOSED(match_x(match_xy),3) - neighbor_node.pos(3))<tol,1);
                if ~isempty(match_xyz)
                    continue;
                end
            end
        end
        neighbor_node.gcost = current_node.gcost + norm(neighbor_node.pos-current_node.pos);
        if astar
            neighbor_node.hcost = norm(neighbor_node.pos - goal_pos);
        else
            neighbor_node.hcost = 0;
        end
        neighbor_node.fcost = neighbor_node.gcost + neighbor_node.hcost;
        neighbor_node.parent = current_node;
        if isKey(OPEN,neighbor_node.fcost)
            OPEN(neighbor_node.fcost) = [OPEN(neighbor_node.fcost);neighbor_node];
        else
            OPEN(neighbor_node.fcost) = neighbor_node;
            k(end+1) = neighbor_node.fcost;
        end
    end
end
path = [path;start_pos];
path = flipud(path);

if ~isempty(find(collide(map,path) == 1,1))
    path = [];
    return
end
map{8} = false;
%% END YOUR CODE HERE %%

end