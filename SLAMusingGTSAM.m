afunction [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(Odom, ObservedLandMarks, StartingPose)
% INPUTS:
% Odom: Each column has the values of odometry between two time steps,
% eg.,  Odom(:,1) has the odometry between t=0 (starting position) and t=1
% ObservedLandMarks is a cell array of structures where
% each cell has a structure.
% Here each structure has Locations array of size
% (NumberOfObservedLocations x 2) and a vector Idx of size (1 x
% NumberOfObservedLocations)
% Each row of ObservedLandMarks{count}.Locations is x and y observed
% co-ordinates
% StartingPose is a 3 x 1 array of starting pose in [x; y; theta]
% OUTPUTS:
% AllPosesComputed: Array of size 3 x NumberOfSteps+1, the first column
% will be the pose at t=0
% LandMarksComputed: Each row has [ID, LocX, LocY], Note that LandMarkIDs
% have to be sorted in ascending order

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

%% Create keys for variables
% count = 1;
% for i=1:(size(Odom,2)+1)
%     x(i) = symbol('x',i);
%     landmarkID = ObservedLandMarks{i}.Idx;
%     for j=1:length(landmarkID)
%         if i==1 && j==1
%             l(count) = symbol('l',landmarkID(j));
%             count = count + 1;
%         elseif isempty(find(l==symbol('l',landmarkID(j))))
%             l(count) = symbol('l',landmarkID(j));
%             count = count + 1;
%         end
%     end
% end

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior
priorMean = Pose2(StartingPose(1),StartingPose(2),StartingPose(3));
priorNoise = noiseModel.Diagonal.Sigmas([0; 0; 0]);
graph.add(PriorFactorPose2(symbol('x',1), priorMean, priorNoise));

%% Add odometry
odometryNoise = noiseModel.Diagonal.Sigmas([0.3*max(abs(Odom(1,:))); 0.3*max(abs(Odom(2,:))); 0.1*max(abs(Odom(3,:)))]);
for i=1:size(Odom,2)
    odometry = Pose2(Odom(1,i),Odom(2,i),Odom(3,i));
    graph.add(BetweenFactorPose2(symbol('x',i), symbol('x',i+1), odometry, odometryNoise));
end

%% Add bearing/range measurement factors
brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1]);
for i=1:(size(Odom,2)+1)
    landmarkID = ObservedLandMarks{i}.Idx;
    landmarkLocation = ObservedLandMarks{i}.Locations;
    for j=1:length(landmarkID)
        if i==1
            bearingAngle = atan2(landmarkLocation(j,2), landmarkLocation(j,1)) - StartingPose(3);
        else
            bearingAngle = atan2(landmarkLocation(j,2), landmarkLocation(j,1)) - StartingPose(3) - sum(Odom(3,1:i-1));
        end
        graph.add(BearingRangeFactor2D(symbol('x',i), symbol('l',landmarkID(j)), Rot2(bearingAngle), norm(landmarkLocation(j,:)), brNoise));
    end
end

% print
% graph.print(sprintf('\nFull graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
for i=1:(size(Odom,2)+1)
    if i==1
        initialEstimate.insert(symbol('x',1), Pose2(StartingPose(1),StartingPose(2),StartingPose(3)));
        landmarkID = ObservedLandMarks{i}.Idx;
        landmarkLocation = ObservedLandMarks{i}.Locations;
        for j=1:length(landmarkID)
            loc = StartingPose(1:2)+landmarkLocation(j,:)';
            initialEstimate.insert(symbol('l',landmarkID(j)), Point2(loc(1),loc(2)));
        end
    else
        initialEstimate.insert(symbol('x',i), Pose2(StartingPose(1)+sum(Odom(1,1:i-1)),StartingPose(2)+sum(Odom(2,1:i-1)),StartingPose(3)+sum(Odom(3,1:i-1))));
        landmarkID = ObservedLandMarks{i}.Idx;
        landmarkLocation = ObservedLandMarks{i}.Locations;
        for j=1:length(landmarkID)
            if ~initialEstimate.exists(symbol('l',landmarkID(j)))
                loc = StartingPose(1:2)+sum(Odom(1:2,1:i-1),2)+landmarkLocation(j,:)';
                initialEstimate.insert(symbol('l',landmarkID(j)), Point2(loc(1),loc(2)));
            end
        end
    end
end

% initialEstimate.print(sprintf('\nInitial estimate:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
% result.print(sprintf('\nFinal result:\n'));

AllPosesComputed = zeros(3,size(Odom,2)+1);
LandMarksComputed = zeros(size(result)-size(AllPosesComputed,2),3);
k = 1;
for i=1:(size(Odom,2)+1)
    AllPosesComputed(:,i) = [result.at(symbol('x',i)).x; result.at(symbol('x',i)).y; result.at(symbol('x',i)).theta];
    landmarkID = ObservedLandMarks{i}.Idx;
    for j=1:length(landmarkID)
        if isempty(find(LandMarksComputed(:,1)==landmarkID(j),1))
            LandMarksComputed(k,:) = [landmarkID(j), result.at(symbol('l',landmarkID(j))).x, result.at(symbol('l',landmarkID(j))).y];
            k = k + 1;
        end
    end
end
[~,I] = sort(LandMarksComputed(:,1));
LandMarksComputed = LandMarksComputed(I,:);

%% Plot Covariance Ellipses
% cla;hold on
% 
% marginals = Marginals(graph, result);
% plot2DTrajectory(result, [], marginals);
% plot2DPoints(result, 'b', marginals);
% 
% for i=1:(size(Odom,2)+1)
%     landmarkID = ObservedLandMarks{i}.Idx;
%     for j=1:length(landmarkID)
%         plot([result.at(symbol('x',i)).x; result.at(symbol('l',landmarkID(j))).x],[result.at(symbol('x',i)).y; result.at(symbol('l',landmarkID(j))).y], 'c-');
%     end
% end
% axis([-0.6 4.8 -1 1])
% axis equal
% view(2)

end