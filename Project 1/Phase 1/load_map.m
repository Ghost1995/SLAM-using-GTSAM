function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  an obstacle.

% CMSC 828T Proj 1 Phase 1

% Output structure: 
    % Map is a cell array containing the following:
    %   --> map{1} contains a 3D logical occupancy grid
    %   --> map{2}, Map{3} and Map{4} store the discretized axes values
    %       corresponding to the X, Y and Z axes respectively
    %   --> map{5} and map{6} store the xy_res and z_res respectively

% Open file, read data, close file. Comments in file marked by #
fileID = fopen(filename);
fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);

%% START YOUR CODE HERE %%
map = cell(1,8);
boundary_index = find(strcmp(fileDat{1},'boundary'));
xy_res = (fileDat{5}(boundary_index)-fileDat{2}(boundary_index))/ceil((fileDat{5}(boundary_index)-fileDat{2}(boundary_index))/xy_res);
z_res = (fileDat{7}(boundary_index)-fileDat{4}(boundary_index))/ceil((fileDat{7}(boundary_index)-fileDat{4}(boundary_index))/z_res);
map{1} = zeros((fileDat{5}(boundary_index)-fileDat{2}(boundary_index))/xy_res,(fileDat{6}(boundary_index)-fileDat{3}(boundary_index))/xy_res,(fileDat{7}(boundary_index)-fileDat{4}(boundary_index))/z_res);
map{2} = fileDat{2}(boundary_index)+(xy_res/2):xy_res:fileDat{5}(boundary_index);
map{3} = fileDat{3}(boundary_index)+(xy_res/2):xy_res:fileDat{6}(boundary_index);
map{4} = fileDat{4}(boundary_index)+(z_res/2):z_res:fileDat{7}(boundary_index);
map{5} = xy_res;
map{6} = z_res;
map{7} = [];
map{8} = false;

if ~isempty(find(strcmp(fileDat{1},'block'),1))
    for i = find(strcmp(fileDat{1},'block'))'
        x_min = find((fileDat{2}(i)-margin)<=map{2},1);
        if (fileDat{2}(i)-margin+(xy_res/2))<map{2}(x_min) && x_min~=1
            x_min = x_min - 1;
        end
        x_max = find((fileDat{5}(i)+margin)>=map{2},1,'last');
        if (fileDat{5}(i)+margin-(xy_res/2))>map{2}(x_max) && x_max~=length(map{2})
            x_max = x_max + 1;
        end

        y_min = find((fileDat{3}(i)-margin)<=map{3},1);
        if (fileDat{3}(i)-margin+(xy_res/2))<map{3}(y_min) && y_min~=1
            y_min = y_min - 1;
        end
        y_max = find((fileDat{6}(i)+margin)>=map{3},1,'last');
        if (fileDat{6}(i)+margin-(xy_res/2))>map{3}(y_max) && y_max~=length(map{3})
            y_max = y_max + 1;
        end

        z_min = find((fileDat{4}(i)-margin)<=map{4},1);
        if (fileDat{4}(i)-margin+(z_res/2))<map{4}(z_min) && z_min~=1
            z_min = z_min - 1;
        end
        z_max = find((fileDat{7}(i)+margin)>=map{4},1,'last');
        if (fileDat{7}(i)+margin-(z_res/2))>map{4}(z_max) && z_max~=length(map{4})
            z_max = z_max + 1;
        end

        map{1}(x_min:x_max,y_min:y_max,z_min:z_max) = 1;
    
        map{7} = [map{7};[x_min x_max y_min y_max z_min z_max fileDat{8}(i) fileDat{9}(i) fileDat{10}(i)]];
    end
end
%% END YOUR CODE HERE %%

end