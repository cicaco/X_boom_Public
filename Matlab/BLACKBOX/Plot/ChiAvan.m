function ChiAvan(BoomInfo,YOUT,TOUT)
chi= YOUT(:,6).*norm(BoomInfo.Aero.P_Finish_Dx)./(vecnorm(YOUT(:,7:9)'))';
V=(vecnorm(YOUT(:,7:9)'))';
figure(20)
plot(TOUT(:),chi);
title('time vs $\chi$','Interpreter','latex');
xlabel('t [s]');
ylabel('$\chi$','Interpreter','latex');
figure(21)
plot(TOUT(:),V);
title('Velocità');
end

