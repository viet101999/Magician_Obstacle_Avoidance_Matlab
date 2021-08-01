    function d = distance(q1,q2)% norm 2 distance between two vectors
        a = fkmagician(q1(1),q1(2),q1(3));
        p1 = a(5,:);
        b = fkmagician(q2(1),q2(2),q2(3));
        p2 = b(5,:);
        d=sum((p1-p2).^2).^.5;
    end