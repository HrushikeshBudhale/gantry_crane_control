function [xp,yp] = get_pend_pos(x,y, length, theta)
    %GET_PEND_POS Returns x,y coordinates of the pendulum
    xp = x + length*sin(theta);
    yp = y - length*cos(theta);
end

