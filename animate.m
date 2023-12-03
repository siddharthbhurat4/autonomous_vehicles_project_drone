function animate(positions,angles,trajectory,des_trajectory)
    [m,g,Ix,Iy,Iz,l,k,b]=get_model();
    axle_x = [-l/2 0 0;
               l/2 0 0];
    axle_y = [0 -l/2 0;
              0  l/2 0];

    xmax = max(max(positions(:,1)),l);
    ymax = max(max(positions(:,2)),l);
    zmax = max(max(positions(:,3)),l);

    xmin = min(min(positions(:,1)),l);
    ymin = min(min(positions(:,2)),l);
    zmin = min(min(positions(:,3)),l);
    
    r = 0.1*l; %radius of propellers
    ang = linspace(0,2*pi);
    x_circle = r*cos(ang);
    y_circle = r*sin(ang);
    z_circle = zeros(1,length(ang));
    propeller = [x_circle',y_circle',z_circle'];
    
    
    [p1,q1] = size(propeller);
    [p2,q2] = size(axle_x);
    [mm,nn] = size(angles);
    traj_size = size(trajectory,1);
    traj_step = floorDiv(mm,traj_size);
    for ii=1:mm
        x = positions(ii,1);
        y = positions(ii,2);
        z = positions(ii,3);
        phi = angles(ii,1); 
        theta = angles(ii,2);
        psi = angles(ii,3);
        R = get_rotation_zyx(phi,theta,psi);
        
        for i=1:p2
            r_body = axle_x(i,:)';
            r_world = R*r_body;
            new_axle_x(i,:) = r_world';
        end
        new_axle_x = [x y z] +new_axle_x;
        
        for i=1:p2
            r_body = axle_y(i,:)';
            r_world = R*r_body;
            new_axle_y(i,:) = r_world';
        end
        new_axle_y = [x y z] +new_axle_y;
        
        for i=1:p1
            r_body = propeller(i,:)';
            r_world = R*r_body;
            new_propeller(i,:) = r_world'; 
        end
        new_propeller1 = new_axle_x(1,:) + new_propeller;
        new_propeller3 = new_axle_x(2,:) + new_propeller;
        new_propeller2 = new_axle_y(1,:) + new_propeller;
        new_propeller4 = new_axle_y(2,:) + new_propeller;
        ii_2 = floorDiv(ii,traj_step);
        if ii_2 >0
            plot3(trajectory(1:ii_2,1),trajectory(1:ii_2,2),trajectory(1:ii_2,3), ...
                    '-o','Color','r','MarkerSize',2);
        end
        hold on
        plot3(des_trajectory(:,1),des_trajectory(:,2),des_trajectory(:,3),'b');
        hold off
        line(new_axle_x(:,1),new_axle_x(:,2),new_axle_x(:,3),'Linewidth',2); hold on;
        line(new_axle_y(:,1),new_axle_y(:,2),new_axle_y(:,3),'Linewidth',2);
        patch(new_propeller1(:,1),new_propeller1(:,2),new_propeller1(:,3),'r');
        patch(new_propeller2(:,1),new_propeller2(:,2),new_propeller2(:,3),'g');
        patch(new_propeller3(:,1),new_propeller3(:,2),new_propeller3(:,3),'b');
        patch(new_propeller4(:,1),new_propeller4(:,2),new_propeller4(:,3),'c');
        %axis(1.05*[xmin xmax ymin ymax zmin zmax]);
        axis(1.5*[xmin xmax ymin ymax zmin zmax]);
        
        xlabel('x'); ylabel('y'); zlabel('z');
        %view(0,90)
        view(3)
        pause(0.0001)
        if (ii~=mm)
            clf
        end
end