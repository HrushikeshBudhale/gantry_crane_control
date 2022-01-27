function plot_graph_LQG(t, X, Xh, y)
    %PLOT_GRAPH plot wrt time
    figure('Name','Observable states States','NumberTitle','off');
    subplot(3,1,1);
    plot(t,X(:,1),'r','LineWidth',1.0);
    hold on;
    plot(t,Xh(:,1),'k--','LineWidth',2.0);
    if size(y) > 1
        plot(t,y(:,1),'b');
    end
    hold off;
    legend('Actual','Estimate','Input');
    title('Cart position (m)');
    subplot(3,1,2);
    plot(t,X(:,3),'r','LineWidth',1.0);
    hold on;
    plot(t,Xh(:,3),'k--','LineWidth',2.0);
    if size(y) > 1
        plot(t,y(:,2),'b');
    end
    hold off;
    legend('Actual','Estimate','Input');
    title('Theta1 (raddian)');
    subplot(3,1,3);
    plot(t,X(:,5),'r','LineWidth',1.0);
    hold on;
    plot(t,Xh(:,5),'k--','LineWidth',2.0);
    if size(y) > 1
        plot(t,y(:,3),'b');
    end
    hold off;
    legend('Actual','Estimate','Input');
    title('Theta2 (raddian)');

    % We can see that estimate about non observable states also converges to
    % actual values
    figure('Name','Unobservable States','NumberTitle','off');
    subplot(3,1,1);
    plot(t,X(:,2),'r','LineWidth',1.0);
    hold on;
    plot(t,Xh(:,2),'k--','LineWidth',2.0);
    hold off;
    legend('Actual','Estimate');
    title('Cart velocity (m/s)');
    subplot(3,1,2);
    plot(t,X(:,4),'r','LineWidth',1.0);
    hold on;
    plot(t,Xh(:,4),'k--','LineWidth',2.0);
    hold off;
    legend('Actual','Estimate');
    title('Angular velocity1 (raddian/s)');
    subplot(3,1,3);
    plot(t,X(:,6),'r','LineWidth',1.0);
    hold on;
    plot(t,Xh(:,6),'k--','LineWidth',2.0);
    hold off;
    legend('Actual','Estimate');
    title('Angular velocity2  (raddian/s)');
end

