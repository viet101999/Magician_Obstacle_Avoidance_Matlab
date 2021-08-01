function [Fvec, p,rho] =  frepPtFloatingPoint(q, pObstacle, eta, rhoNot) % 5.2.2 The Repulsive field
        % Task 2  (Graduate students 5pts, Undergrads, 5pts E.C.):
        %computes the forces that repel a point on the link that is closest to any workspace obstacle
        %at positon pObstacle, given by equation 5.6 & 5.7 in RD&C
        %q: configuration of the arm
        %pObstacle: xy position of the point obstacle
        %eta: vector parameter that scales the forces for each degree-of-freedom
        %rhoNot: defines the distance of influence of the obstacle
        Fvec = zeros(numel(q),3); %Force vector to repulse each link from the obstacle
        o = fkmagician(q(1),q(2),q(3)); % o is a vector of the origins for DH frames of a planar robot arm.
        p = zeros(numel(q),3);
        rho = zeros(numel(q),3);
        
        for i = 1:numel(q) % compute repulsive force for each link
            %find the nearest point on the link to the obstacle
            if i == 1
                p(i,:) = findClosestPointOnLine([0 0 0],o(i,:),pObstacle);
            else
                p(i,:) = findClosestPointOnLine(o(i-1,:),o(i,:),pObstacle);
            end
            %calculate the distance from the floating point to the obstacle
            rho(i) = dist(p(i,:),pObstacle);
            %if it is inside rho0, calculate the force
            if rho(i) <= rhoNot
                delRho = (p(i,:) -pObstacle)/rho(i);
                Fvec(i,:) = eta(i)*(1/rho(i) - 1/rhoNot)*(1/rho(i)^2)*delRho; % 5.2.2 The Repulsive field
            end
        end
    end