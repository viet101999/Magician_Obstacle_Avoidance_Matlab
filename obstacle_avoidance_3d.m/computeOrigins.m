    function o = computeOrigins(q)
    % input theta -> output x,y
        %Computes o, the (x,y) coordinate of the DH frame for each link in q
        linkLen = [.5;.5;.5;.5;.5];
        qSum = cumsum(q);
        oDelta = [linkLen,linkLen].*[cos(qSum),sin(qSum)];
        o = cumsum(oDelta);
    end