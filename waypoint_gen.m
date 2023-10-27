function waypoint_list = waypoint_gen(waypoints)
    sz = size(waypoints);
    waypoints = [zeros(1,sz(2));waypoints];
    
    max_dist = 3;
    
    waypoint_list = [];
    for i = 1:sz(1)
        divisions = 1;
        difference  = waypoints(i+1,:) - waypoints(i,:);
        distance = norm(difference(1:3));
        if distance > max_dist
            divisions = ceil(distance/max_dist);
        end
        for j = 1:divisions
            
            next_point = [waypoints(i,1:3)+difference(1:3).*(j/divisions),waypoints(i,4:sz(2))+difference(4:sz(2)) ];
            waypoint_list = [waypoint_list; next_point];
        end
        
    end
    
end