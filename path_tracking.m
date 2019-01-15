clc; clear all; close all;

path_x = 0 : 0.07 : pi;
path_y = sin(path_x);

path = [path_x', path_y'];

vel = [0;0];
pos = path(1,:)';
f = [0;0];
ang = 0;

wp_size = size(path,1);
wp_indx = 1;

h = 0.01;
t_end = 100;
t_current = 0;
indx = 1;

while t_current < t_end
    WPcur = path(wp_indx,:);
    WPnxt = path(wp_indx+1,:);
    dn = sqrt((WPnxt(1) - WPcur(1))^2 + ((WPnxt(2) - WPcur(2))^2));
    u_wp = ((pos(1) - WPcur(1))*(WPnxt(1) - WPcur(1)) + (pos(2) - WPcur(2))*(WPnxt(2) - WPcur(2)))/dn^2;
    yaw_wp = atan2((WPnxt(2) - WPcur(2)), (WPnxt(1) - WPcur(1)));
    while u_wp >= 1
        wp_indx = wp_indx + 1;
        if wp_indx >= wp_size
            break;
        end
        WPcur = path(wp_indx,:);
        WPnxt = path(wp_indx+1,:);
        dn = sqrt((WPnxt(1) - WPcur(1))^2 + ((WPnxt(2) - WPcur(2))^2));
        u_wp = ((pos(1) - WPcur(1))*(WPnxt(1) - WPcur(1)) + (pos(2) - WPcur(2))*(WPnxt(2) - WPcur(2)))/dn^2;
        yaw_wp = atan2((WPnxt(2) - WPcur(2)), (WPnxt(1) - WPcur(1)));
    end
    Xd = WPcur(1) + u_wp*(WPnxt(1) - WPcur(1));
    Yd = WPcur(2) + u_wp*(WPnxt(2) - WPcur(2));
    Lpv = 0.5;
    Xpv = Xd + Lpv*cos(yaw_wp);
    Ypv = Yd + Lpv*sin(yaw_wp);
    yaw_d = atan2(Ypv - pos(2), Xpv - pos(1));

    v_x = 0.1;
    A = [cos(yaw_d) sin(yaw_d);
        -sin(yaw_d) cos(yaw_d)];
    vel = A'*[v_x;0];
    
    result(indx,:) = [t_current, pos', vel'];
    
    pos = pos + vel*h;
    t_current = t_current + h;
    
    indx = indx + 1;
    
    if (wp_indx >= wp_size); break; end
end

figure
set(gcf,'Color',[1,1,1])
plot(path(:,1), path(:,2), 'LineWidth',2)
hold on
plot(result(:,2), result(:,3), '--', 'Linewidth', 2)
grid on