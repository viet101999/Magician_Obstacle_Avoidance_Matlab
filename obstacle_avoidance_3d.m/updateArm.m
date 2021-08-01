    function updateArm(o,hline,hpts) %redraws arm
        set(hline, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)]);
        set(hpts, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)]);
    end