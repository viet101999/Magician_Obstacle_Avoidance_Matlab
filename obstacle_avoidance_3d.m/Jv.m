function J = Jv(q,ic) %page 177
        o = fkmagician(q(1),q(2),q(3));
        Augo = [0,0,0;o];  %add frame 0  (augmented origin)
        % J = [z0 x (o_c - o_0),  z1 x (o_c - o_2), ...., z_(n-1) x (o_c - o_(n-1))
        J = zeros(3,numel(q));
        for c = 1:ic
            oDiff=  Augo(ic+1,:)-Augo(c,:);
            J(1,c) = -oDiff(3);
            J(2,c) = oDiff(2);
            J(3,c) = oDiff(1);
        end
    end
