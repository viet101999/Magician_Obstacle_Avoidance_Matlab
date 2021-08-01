    function d = dist(a,b)% norm 2 distance between two vectors
        d=sum((a-b).^2).^.5;
    end