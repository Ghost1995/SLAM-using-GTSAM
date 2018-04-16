function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

% CMSC 828T Proj 1 Phase 1

%% START YOUR CODE HERE %%
if ~isempty(map)
    axis([map{2}(1)-(map{5}/2) map{2}(end)+(map{5}/2) map{3}(1)-(map{5}/2) map{3}(end)+(map{5}/2) map{4}(1)-(map{6}/2) map{4}(end)+(map{6}/2)]);
    grid on
    grid minor
    
    hold on
    for i=1:size(map{7},1)
        X = [map{7}(i,1), map{7}(i,1), map{7}(i,2), map{7}(i,2), map{7}(i,1), map{7}(i,1);...
             map{7}(i,1), map{7}(i,2), map{7}(i,2), map{7}(i,1), map{7}(i,1), map{7}(i,1);...
             map{7}(i,1), map{7}(i,2), map{7}(i,2), map{7}(i,1), map{7}(i,2), map{7}(i,2);...
             map{7}(i,1), map{7}(i,1), map{7}(i,2), map{7}(i,2), map{7}(i,2), map{7}(i,2)];
        
        Y = [map{7}(i,3), map{7}(i,3), map{7}(i,3), map{7}(i,3), map{7}(i,3), map{7}(i,4);...
             map{7}(i,3), map{7}(i,3), map{7}(i,3), map{7}(i,3), map{7}(i,3), map{7}(i,4);...
             map{7}(i,4), map{7}(i,4), map{7}(i,4), map{7}(i,4), map{7}(i,3), map{7}(i,4);...
             map{7}(i,4), map{7}(i,4), map{7}(i,4), map{7}(i,4), map{7}(i,3), map{7}(i,4)];
        
        Z = [map{7}(i,5), map{7}(i,6), map{7}(i,6), map{7}(i,5), map{7}(i,5), map{7}(i,5);...
             map{7}(i,6), map{7}(i,6), map{7}(i,5), map{7}(i,5), map{7}(i,6), map{7}(i,6);...
             map{7}(i,6), map{7}(i,6), map{7}(i,5), map{7}(i,5), map{7}(i,6), map{7}(i,6);...
             map{7}(i,5), map{7}(i,6), map{7}(i,6), map{7}(i,5), map{7}(i,5), map{7}(i,5)];
        
        patch(map{2}(X),map{3}(Y),map{4}(Z),[map{7}(i,7)/255 map{7}(i,8)/255 map{7}(i,9)/255]);
    end
    
    if ~isempty(path)
        plot3(path(:,1),path(:,2),path(:,3));
    end
    hold off
end
%% END YOUR CODE HERE %%

end