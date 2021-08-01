    function updateArm_3d(o,hline,hpts) %redraws arm
        set(hline, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)], 'zdata',[0;o(:,3)]);
        set(hpts, 'xdata',[0;o(:,1)], 'ydata',[0;o(:,2)], 'zdata',[0;o(:,3)]);
    end