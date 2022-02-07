% ======
% ROB521_assignment1.m
% ======
%
% This assignment will introduce you to the idea of motion planning for  
% holonomic robots that can move in any direction and change direction of 
% motion instantaneously.  Although unrealistic, it can work quite well for
% complex large scale planning.  You will generate mazes to plan through 
% and employ the PRM algorithm presented in lecture as well as any 
% variations you can invent in the later sections.
% 
% There are three questions to complete (5 marks each):
%
%    Question 1: implement the PRM algorithm to construct a graph
%    connecting start to finish nodes.
%    Question 2: find the shortest path over the graph by implementing the
%    Dijkstra's or A* algorithm.
%    Question 3: identify sampling, connection or collision checking 
%    strategies that can reduce runtime for mazes.
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plots, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file.
%
% requires: basic Matlab, 
%
% S L Waslander, January 2022
%
clear; close all; clc;

% set random seed for repeatability if desired
% rng(1);

% ==========================
% Maze Generation
% ==========================
%
% The maze function returns a map object with all of the edges in the maze.
% Each row of the map structure draws a single line of the maze.  The
% function returns the lines with coordinates [x1 y1 x2 y2].
% Bottom left corner of maze is [0.5 0.5], 
% Top right corner is [col+0.5 row+0.5]
%

%Maze Parameters
row = 7; % Maze rows
col = 7; % Maze columns
map = maze(row,col); % Creates the maze
start = [0.5, 1.0]; % Start at the bottom left
finish = [col+0.5, row]; % Finish at the top right

h = figure(1);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;


% ======================================================
% Question 1: construct a PRM connecting start and finish
% ======================================================
%
% Using 500 samples, construct a PRM graph whose milestones stay at least 
% 0.1 units away from all walls, using the MinDist2Edges function provided for 
% collision detection.  Use a nearest neighbour connection strategy and the 
% CheckCollision function provided for collision checking, and find an 
% appropriate number of connections to ensure a connection from  start to 
% finish with high probability.


% variables to store PRM components
nS = 500;  % number of samples to try for milestone creation
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]
edge_v = [];

disp("Time to create PRM graph")
tic;
% ------insert your PRM generation code here-------
%iterate over number of samples
for i = 1:nS
    candidate_ms = rand(1,2)* (col+0.5); %generate a random candidate milestone
    min_d = MinDist2Edges(candidate_ms,map);
    if min_d > 0.1  %ensure the point is not within 0.1 units of an edge
        milestones(end+1,:) = candidate_ms; %add to milestones list
    end
end
adjacency_matrix=zeros(length(milestones)); %initialize adjacency matrix for edges
for q=1:length(milestones)  %test for edges between milestones
    pt1 = milestones(q,:);
    mindist = 10000000;
    temp_nlist =[];
    for v = 1:length(milestones) %for each milestone, test with all others
        pt2 = milestones(v,:);
        delta = pt2-pt1;
        edge_dist = sqrt((delta(:,1)^2+delta(:,2)^2));
        if edge_dist < mindist %1.5 % only proceed if the node is nearer than any other nodes 
            mindist=edge_dist;
            if q~=v
                [inCollision, ~]=CheckCollision(pt1,pt2,map); % check for edge collisions
                if inCollision ~= 1
                    edge_v(end+1) = edge_dist;
                    edges(end+1,:) = [pt1 pt2];
                    adjacency_matrix(q,v)=1*edge_dist;
                    adjacency_matrix(v,q)=1;
                end
            end
        end
    end 
end

% ------end of your PRM generation code -------
toc;

figure(1);
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
end
str = sprintf('Q1 - %d X %d Maze PRM', row, col);
title(str);
drawnow;

print -dpng assignment1_q1.png

%%
% =================================================================
% Question 2: Find the shortest path over the PRM graph
% =================================================================
%
% Using an optimal graph search method (Dijkstra's or A*) , find the 
% shortest path across the graph generated.  Please code your own 
% implementation instead of using any built in functions.

disp('Time to find shortest path');
tic;

% Variable to store shortest path
spath = []; % shortest path, stored as a milestone row index sequence

% I will be implementing the A* algorithm
% ------insert your shortest path finding algorithm here-------
f=ones(length(milestones),1).*1000000; % initialize f to infinity for all milestones
g=ones(length(milestones),1).*1000000; %cost to come
h=ones(length(milestones),1).*1000000; %cost to go
tentative_f=ones(length(milestones),1).*1000000; % initialize f to infinity for all milestones
tentative_g=ones(length(milestones),1).*1000000; %cost to come
tentative_h=ones(length(milestones),1).*1000000; %cost to go
open_list = [1]; %nodes that have not been visited
closed_list = []; %nodes that have already been visited
came_from = zeros(length(milestones),1); %came_from[n] is the node preceding node n

g(1) =0; %cost to come at start is 0
h(1) = compute_heuristic(start,finish); %cost to go is the straight line distance to finish
%note that a helper function "compute_heuristic" is created and included in
%this directory, it simply calculates the distance between two points
f(1) =g(1)+h(1); %f+g+h
%refer to nodes by their milestone number, start is ms=1
%finish is ms=2
goal_reached = 0;
while ~isempty(open_list) %search while the open list of nodes is not empty
    %Here, choose node in open_list with lowest fscore
    min_fscore = 10000000;
    lowestscore_ms = 0;
    for i=1:length(open_list)
        milestone = open_list(i);
        if f(milestone) < min_fscore
            min_fscore = f(milestone);
            lowestscore_ms = milestone;
        end
    end
    
    open_list = open_list(open_list~=lowestscore_ms); %pop q (the node with lowest fscore) off the open list
    length(open_list);
    q = lowestscore_ms; %q is the current node being expanded

    neighbors_q = []; %generate successors of q, initialize to null

    for i=1:length(adjacency_matrix(:,1)) %iterate over adjacency matrix to find the neighbors
       
       if adjacency_matrix(i,q)~=0
            neighbors_q(:,end+1)=i; %store milestone# in list
        end
    end

    %for successor in q (for each neigbor of current node q)
    for i=1:length(neighbors_q)
        successor = neighbors_q(:,i);
        if successor == 2 %if successor is the goal node (milestone 2)
            goal_reached=1; %flag for search completed
            came_from(successor) = q; %set q as parent of successor node
            disp("reached goal") 
        end
        tentative_g(successor)= g(q) + compute_heuristic(milestones(q,:),milestones(successor,:));
        tentative_h(successor)= compute_heuristic(milestones(successor,:),finish);
        tentative_f(successor)=tentative_g(successor)+2*tentative_h(successor);
        if tentative_g(successor) < g(successor)%This path to neighbor is better than any previous one  
                came_from(successor) = q;
                g(successor)=tentative_g(successor);
                f(successor)=tentative_g(successor) + compute_heuristic(milestones(successor,:),finish);
                if ~ismember(successor,open_list) %if neighbor not already in open list of nodes to expand
                    open_list(end+1)=successor; %add to open list
                end           
        end
    end 

end
    
  
% ------end of shortest path finding algorithm------- 
%Now construct the path 
toc;    
spath=[];
currnode = 2;
spath = [[],2];
%
while currnode ~= 1
    %create the path starting from ms=2 (finish) to ms=1 (start) from
    %came_from list of parent nodes
    currnode = came_from(currnode);
    spath(end+1)=currnode;
    if length(spath)>length(milestones)
        disp ("circular path failure")
        return
    end

end
spath = flip(spath,2);

% plot the shortest path

figure(1);
for i=1:length(spath)-1
    plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
end
str = sprintf('Q2 - %d X %d Maze Shortest Path', row, col);
title(str);
drawnow;

print -dpng assingment1_q2.png

%%
% ================================================================
% Question 3: find a faster way
% ================================================================
%
% Modify your milestone generation, edge connection, collision detection 
% and/or shortest path methods to reduce runtime.  What is the largest maze 
% for which you can find a shortest path from start to goal in under 20 
% seconds on your computer? (Anything larger than 40x40 will suffice for 
% full marks)



%I will be adding deterministic sampling, with evenly spaced nodes, as well
%as a lazy collision checking algorithm, where edges are only checked for
%collision as they are added to the A* search

%%params (larger maze)
row = 61;
col = 61;

map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

h = figure(2);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

fprintf("Attempting large %d X %d maze... \n", row, col);
tic;        
% ------insert your optimized algorithm here------

%%%%%%%%%%
%PRM Generation Portion (deterministic sampling)
%%%%%%%%%%
%create 2 arrays the size of the maze, with evenly spaced entries ~1 unit
%apart
ms_x = linspace(1,row,row)'; 
ms_y = linspace(1,col,col)';
%use these 2 arrays to generate the milestones
for i = 1:length(ms_y)
    for j=1:length(ms_x)
        milestones(end+1,:) = [ms_x(i) ms_y(j)];
    end
end

prm1=toc
%generate adjacency matrix again
adjacency_matrix=zeros(length(milestones));
for q=1:length(milestones)
    pt1 = milestones(q,:);
    mindist = 10000000;
    temp_nlist =[];
    for v = 1:length(milestones)
        pt2 = milestones(v,:);
        if edge_dist < 1.5 %this time, we assume nodes nearby are collision-free, we check for edge collisions later
            if q~=v
                    adjacency_matrix(q,v)=1;
                    adjacency_matrix(v,q)=1; %add to adjacency matrix
            end
        end
    end % end of for
    
end

prm2 = toc
%%%%%%%%%%%%%%%
%Modified A* search with Lazy collision checking
%%%%%%%%%%%%%%%
spath = []; % shortest path, stored as a milestone row index sequence

% ------insert your shortest path finding algorithm here-------
f=ones(length(milestones),1).*1000000; % initialize f to infinity for all milestones
g=ones(length(milestones),1).*1000000; %cost to come
h=ones(length(milestones),1).*1000000; %cost to go
open_list = [1]; %nodes that have not been visited
closed_list = []; %nodes that have already been visited
came_from = zeros(length(milestones),1); %came_from[n] is the node preceding node n

g(1) =0;
h(1) = compute_heuristic(start,finish);
f(1) =g(1)+h(1); %f=g+h
%refer to nodes by their milestone number, start is ms=1
%finish is ms=2
goal_reached = 0;
while ~isempty(open_list)
    %Here, choose node in open_list with lowest fscore
    min_fscore = 10000000;
    lowestscore_ms = 0;
    for i=1:length(open_list)
        milestone = open_list(i);
        
        if f(milestone) < min_fscore
            min_fscore = f(milestone);
            lowestscore_ms = milestone;
        end
    end
    %pop q off the open list
    open_list = open_list(open_list~=lowestscore_ms);
    length(open_list);
    q = lowestscore_ms; %q is the current node being expanded
    %generate successors of q

    neighbors_q = [];
    for i=1:length(adjacency_matrix(:,1)) %milestones)
       
       if adjacency_matrix(i,q)~=0
            neighbors_q(:,end+1)=i; %store milestone# in list

        end
    end

    %for successor in q % for each neigbor of current node q
    for i=1:length(neighbors_q)
        successor = neighbors_q(:,i);
        if successor == 2 %if successor is the goal node (milestone 2)
            %before creating this path make sure there is no collision
           [inCollision, ~]=CheckCollision(milestones(q,:),milestones(successor,:),map);
                if inCollision ~= 1
                    goal_reached=1;
                    came_from(successor) = q;
                    disp("reached goal")
                    break %stop search once goal node is reached
                end
        end
        tentative_g= g(q) + compute_heuristic(milestones(q,:),milestones(successor,:));

        if tentative_g < g(successor)
                %// This path to neighbor is better than any previous one

                %A* is as usual up until this point, this is where the lazy checking is added
                % before creating a path make sure there is no collision
                % between parent node and successor node

                [inCollision, ~]=CheckCollision(milestones(q,:),milestones(successor,:),map);
                if inCollision ~= 1 %if no collision, proceed as usual
                    came_from(successor) = q;
                    g(successor)=tentative_g;
                    f(successor)=tentative_g + compute_heuristic(milestones(successor,:),finish);
                    if ~ismember(successor,open_list)
                        open_list(end+1)=successor;
                    end
                end
        end
    end 
    if successor ==2 && inCollision ~= 1 %stop when search is finished
        break
    end
end
    
%Now construct the path   
% ------end of shortest path finding algorithm-------    
%
if goal_reached
    %create and plot the shortest path
    spath=[];
    currnode = 2;
    spath = [[],2];
    while currnode ~= 1
        
        currnode = came_from(currnode);
        spath(end+1)=currnode;
        if length(spath)>length(milestones)
            disp ("circular path failure")
            return
        end
    
    end
    spath = flip(spath,2);

    % ------end of your optimized algorithm-------
    dt = toc;
    
    figure(2); hold on;
    plot(milestones(:,1),milestones(:,2),'m.');

    if (~isempty(spath))
        for i=1:length(spath)-1
            plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
        end
    end
    str = sprintf('Q3 - %d X %d Maze solved in %f seconds', row, col, dt);
    title(str);
    
    print -dpng assignment1_q3.png
else
    %debugging pruposes only, see how far the path explored before failing
    dt=toc
    plot(milestones(:,1),milestones(:,2),'m.');

    disp("no path exists, sample more points")
    %find forward path
    current =1;
    spath =[1];
    for i = 1:length(came_from)
        if came_from(i)==current
            current = i;
            spath(end+1)=current;
        end

    end
    if (~isempty(spath))
        for i=1:length(spath)-1
            plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
        end
    end
end

%% Compute Heuristic Helper Function

%function [h]=compute_heuristic(currnode,endnode)
    %straight line distance to finish
 %   delta = endnode-currnode;
 %   edge_dist = sqrt((delta(:,1)^2+delta(:,2)^2));
 %   h = edge_dist;
%end