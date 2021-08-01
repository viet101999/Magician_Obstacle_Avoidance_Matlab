function Fvec = frepPt(q, pObstacle, eta, rhoNot) %#ok<DEFNU>
        % Task 1  (5pts) repulsion from point obstacle
        %frepPt computes the forces that repel each DH frame origin from a point
        %at positon pObstacle, given by equation 5.6 & 5.7 in RD&C
        %q: configuration of the arm
        %pObstacle: xy position of the point obstacle
        %eta: vector parameter that scales the forces for each degree-of-freedom
        %rhoNot: defines the distance of influence of the obstacle
        Fvec = zeros(numel(q),2); %Force vector to repulse each origin from the obstacle
        o = computeOrigins(q); % o is a vector of the origins for DH frames of a planar robot arm.
        
        for i = 1:numel(q) % compute repulsive force for each origin
            rho = dist(o(i,:),pObstacle);
            if rho < rhoNot
                delRho = (o(i,:)-pObstacle)/rho;
                Fvec(i,:)= eta(i)*(1/rho - 1/rhoNot)*(1/rho^2)*delRho;
            end
        end
    end