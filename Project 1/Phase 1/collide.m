function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% CMSC 828T Proj 1 Phase 1

% If the map is empty, output empty vector else, compute
if isempty(map)
    C = [];
else
    %% START YOUR CODE HERE %%
    tol = map{5}/1e4;
    C = ones(size(points,1),1);
    if map{8}
        for i=1:size(points,1)
            x_grid = find(abs(map{2}-points(i,1))<tol,1);
            if isempty(x_grid)
                C(i,1) = 0;
                continue;
            end
            y_grid = find(abs(map{3}-points(i,2))<tol,1);
            if isempty(y_grid)
                C(i,1) = 0;
                continue;
            end
            z_grid = find(abs(map{4}-points(i,3))<tol,1);
            if isempty(z_grid)
                C(i,1) = 0;
                continue;
            end
            C(i,1) = map{1}(x_grid,y_grid,z_grid);
        end
    else
        for i=1:size(points,1)
            x_grid = find(points(i,1)<=map{2},1);
            if isempty(x_grid)
                if points(i,1) > map{2}(end)+(map{5}/2)
                    C(i,1) = 0;
                    continue;
                else
                    x_grid = length(map{2});
                end
            elseif points(i,1)+(map{5}/2) < map{2}(x_grid)
                if x_grid~=1
                    x_grid = x_grid - 1;
                else
                    C(i,1) = 0;
                    continue;                    
                end
            end
            y_grid = find(points(i,2)<=map{3},1);
            if isempty(y_grid)
                if points(i,2) > map{3}(end)+(map{5}/2)
                    C(i,1) = 0;
                    continue;
                else
                    y_grid = length(map{3});
                end
            elseif points(i,2)+(map{5}/2) < map{3}(y_grid)
                if y_grid~=1
                    y_grid = y_grid - 1;
                else
                    C(i,1) = 0;
                    continue;                    
                end
            end
            z_grid = find(points(i,3)<=map{4},1);
            if isempty(z_grid)
                if points(i,3) > map{4}(end)+(map{6}/2)
                    C(i,1) = 0;
                    continue;
                else
                    z_grid = length(map{4});
                end
            elseif points(i,3)+(map{6}/2) < map{4}(z_grid)
                if z_grid~=1
                    z_grid = z_grid - 1;
                else
                    C(i,1) = 0;
                    continue;                    
                end
            end
            C(i,1) = map{1}(x_grid,y_grid,z_grid);
        end
    end
    %% END YOUR CODE HERE %%
end
end