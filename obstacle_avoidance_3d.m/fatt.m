    function Fvec =  fatt(q, oGoal,zeta,d) % 5.2.1 The Attractive field
        %fatt computes the forces that attract each DH frame origin to their goal
        %configurations, given by equation 5.4 in RD&C
        %q: configuration of the arm
        %oGoal: goal position of each DH frame origin
        %zeta: vector parameter that scales the forces for each degree-of-freedom
        %d: the distance that defines the transition from conic toparabolic
        Fvec = zeros(numel(q),3); %Force vector to attract each origin to the goal
        o = fkmagician(q(1),q(2),q(3)); % o is a vector of the origins for DH frames of a planar robot arm.
        
        for i = 1:numel(q) % compute attractive force for each origin
            err = dist(o(i,:),oGoal(i,:));
            if err <= d
                Fvec(i,:)= -zeta(i)*( o(i,:)-oGoal(i,:)  );
            else
                Fvec(i,:)= -d*zeta(i)*( o(i,:)-oGoal(i,:)  )/err;
            end
        end
    end