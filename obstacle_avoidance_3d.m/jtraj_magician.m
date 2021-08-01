function [qt,qdt,qddt] = jtraj_magician(q0,q1,tv,qd0,qd1)
    if length(tv) > 1
        tscal = max(tv);
        t = tv(:)/tscal;
    else
        tscal = 1;
        t = (0:(tv-1))'/(tv-1); % normalized time from 0 -> 1
    end

    q0 = q0(:);
    q1 = q1(:);

    if nargin == 3 % The general input parameters are 3, qd0, qd1 is set to 
        qd0 = zeros(size(q0));
        qd1 = qd0;
    elseif nargin == 5
        qd0 = qd0(:);
        qd1 = qd1(:);
    else
        error('incorrect number of arguments')
    end

    % The most basic case, acceleration, speed is 0, the following polynomial coefficient is solved, the acceleration is set to 0, and the speed is retained.
    % intermediate movement, speed can not be 0, tscal is generally 1
    
    % compute the polynomial coefficients
    A = 6*(q1 - q0) - 3*(qd1+qd0)*tscal;
    B = -15*(q1 - q0) + (8*qd0 + 7*qd1)*tscal;
    C = 10*(q1 - q0) - (6*qd0 + 4*qd1)*tscal;
        % There is no D here, D is actually 1/2 of the initial angular acceleration. The initial acceleration is 0, so D is 0.
    E = qd0*tscal; % as the t vector has been normalized
    F = q0;
    
    % This is calculated by matrix operation, and the angles of each time are calculated.
    tt = [t.^5 t.^4 t.^3 t.^2 t ones(size(t))];
    c = [A B C zeros(size(A)) E F]'; % coefficient matrix, need to be transposed
    
    qt = tt*c; % to obtain the angle
    
    % compute optional velocity
    if nargout >= 2
                 c = [ zeros(size(A)) 5*A 4*B 3*C zeros(size(A)) E ]'; % The velocity coefficient matrix has changed
                 qdt = tt*c/tscal; % takes a guide to the trajectory function
    end
 
    % compute optional acceleration
    if nargout == 3
                 c = [ zeros(size(A)) zeros(size(A)) 20*A 12*B 6*C zeros(size(A))]'; % for acceleration, for secondary guidance
        qddt = tt*c/tscal^2;
    end
end

