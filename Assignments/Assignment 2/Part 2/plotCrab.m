function plotCrab(t,v,u, vr, Ur)
crab = atan(v(:,1)./u(:,1))*180/pi;
sideslip = asin(vr(:,1)./Ur(:,1))*180/pi;
figure(4)
figure(gcf)

subplot(211)
plot(t,crab)
title('\beta_c [deg]'); xlabel('time (s)');
grid on
subplot(212)
plot(t, sideslip)
title('\beta [deg]'); xlabel('time (s)');
grid on
end