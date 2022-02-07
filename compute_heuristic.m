function [h]=compute_heuristic(currnode,endnode)
    %straight line distance to finish
    delta = endnode-currnode;
    edge_dist = sqrt((delta(:,1)^2+delta(:,2)^2));
    h = edge_dist;
end