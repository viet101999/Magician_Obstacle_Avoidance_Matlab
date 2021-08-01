function collision =  checkCollision(q, pObstacle)
        % this should compare the swept out region of the robot and
        %ensure that region does not include the obstacle
        %computes the point on the link that is closest to any workspace obstacle
        %at position pObstacle, if it is at the same location as the
        %obstacle, the two are in collision
        %q: configuration of the arm
        %pObstacle: xy position of the point obstacle
        %eps_min: distance from a point obstacle that the robot must be to
        %keep out of collision
        eps_min = 0.5;  %TODO:  this could be v*LinkLength
        o = fkmagician(q(1),q(2),q(3)); % o is a vector of the origins for DH frames of a planar robot arm.
        collision = false;
        
        for i = 1:numel(q) % check whether the floating points are on the obstacles
            %find the nearest point on the link to the obstacle
            if i == 1
                p = findClosestPointOnLine([0 0 0],o(i,:),pObstacle);
            else
                p = findClosestPointOnLine(o(i-1,:),o(i,:),pObstacle);
            end
            rho = dist(p,pObstacle);
            if rho <= eps_min
                % if the point is within eps_min of the obstacle, it is
                % in collision
                collision = true;
                return;
            end
        end
    end