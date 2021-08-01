posi0 = [250;150;0];
posi1 = [-300;50;0];

q0_temp = ikmagician(posi0(1),posi0(2),posi0(3));
q1_temp = ikmagician(posi1(1),posi1(2),posi1(3));

q0 = q0_temp(1:4,1);
q1 = q1_temp(1:4,1);
tv = 0:.04:1;
    if length(tv) > 1
        tscal = max(tv);
        t = tv(:)/tscal;
    else
        tscal = 1;
        t = (0:(tv-1))'/(tv-1); % normalized time from 0 -> 1
    end

    q0 = q0(:);
    q1 = q1(:);

%     if nargin == 3
        qd0 = zeros(size(q0));
        qd1 = qd0;
%     elseif nargin == 5
%         qd0 = qd0(:);
%         qd1 = qd1(:);
%     else
%         error('incorrect number of arguments')
%     end

    % compute the polynomial coefficients
    A = 6*(q1 - q0) - 3*(qd1+qd0)*tscal;
    B = -15*(q1 - q0) + (8*qd0 + 7*qd1)*tscal;
    C = 10*(q1 - q0) - (6*qd0 + 4*qd1)*tscal;
    E = qd0*tscal; % as the t vector has been normalized
    F = q0;

    tt = [t.^5 t.^4 t.^3 t.^2 t ones(size(t))];
    c = [A B C zeros(size(A)) E F]';
    
    qt_test = tt*c;

%     % compute optional velocity
%     if nargout >= 2
%         c = [ zeros(size(A)) 5*A 4*B 3*C  zeros(size(A)) E ]';
%         qdt = tt*c/tscal;
%     end
% 
%     % compute optional acceleration
%     if nargout == 3
%         c = [ zeros(size(A))  zeros(size(A)) 20*A 12*B 6*C  zeros(size(A))]';
%         qddt = tt*c/tscal^2;
%     end
    qt_test
%     qdt
%     qddt