function animate_LQG(t, X, y, Xh, len1, len2, fig_name, save_animation)
    %ANIMATE_SCEN Summary of this function goes here
    %   Detailed explanation goes here
    crane_y = 25;
    f = figure('Name',fig_name,'NumberTitle','off');
    rect = [10, 10, 530, 410];
    
    for k = 1:length(t)
        % Wipe the slate clean
        clf;
        axis([-10 20 0 crane_y+5])

        % Plot the scene
        hold on;
        plot([-10 20], [crane_y crane_y],'k-', 'LineWidth', 2);
        
        % real system
        [xp,yp] = get_pend_pos(X(k,1), crane_y, len1, X(k,3));
        plot([X(k,1) xp], [crane_y yp],'k-', 'LineWidth', 2);
        plot(xp,yp,'ko','MarkerSize',15,'MarkerFaceColor',[1 0 0]);
        [xp,yp] = get_pend_pos(X(k,1), crane_y, len2, X(k,5));
        plot([X(k,1) xp], [crane_y yp],'k-', 'LineWidth', 2);
        plot(xp,yp,'ko','MarkerSize',15,'MarkerFaceColor',[1 0 0]);
        plot(X(k,1),crane_y,'ks','MarkerSize',30,'MarkerFaceColor',[0 0 1]);
        
        % noisy system
        if length(y) > 1
            [xp,yp] = get_pend_pos(y(k,1), crane_y, len1, y(k,2));
            plot([y(k,1) xp], [crane_y yp],'k--', 'LineWidth', 1);
            plot(xp,yp,'ko','MarkerSize',15);
            [xp,yp] = get_pend_pos(y(k,1), crane_y, len2, y(k,3));
            plot([y(k,1) xp], [crane_y yp],'k--', 'LineWidth', 1);
            plot(xp,yp,'ko','MarkerSize',15);
            plot(y(k,1),crane_y,'ks','MarkerSize',30);
        end

        % estimated system
        [xp,yp] = get_pend_pos(Xh(k,1), crane_y, len1, Xh(k,3));
        plot([Xh(k,1) xp], [crane_y yp],'g-', 'LineWidth', 2);
        plot(xp,yp,'go','MarkerSize',15,'LineWidth', 2);
        [xp,yp] = get_pend_pos(Xh(k,1), crane_y, len2, Xh(k,5));
        plot([Xh(k,1) xp], [crane_y yp],'g-', 'LineWidth', 2);
        plot(xp,yp,'go','MarkerSize',15,'LineWidth', 2);
        plot(Xh(k,1),crane_y,'gs','MarkerSize',30);

        % Decorate the plot
        title_str = "time t= " + num2str(t(k)) + " sec";
        title(title_str);
        grid on;

        % Force matlab to draw the image at this point
        if save_animation == 0
            pause(0.01);
        else
%             movieVector(k) = getframe(f, [2, 2, 520, 520]);
            movieVector(k) = getframe(f,rect);
        end
    end
    
    % Save the movie
    if save_animation == 1
        myWriter = VideoWriter(fig_name, 'MPEG-4');
        myWriter.FrameRate = 30;
        
        open(myWriter);
        writeVideo(myWriter, movieVector);
        close(myWriter);
    end

