%% plot_comparison.m
% Called automatically after watertank_compare.slx finishes (model StopFcn).
% Plots e = Desired water level - Feedback for PI, TD3-RL and LSTM with the
% transient region highlighted.

names = {'Conventional PI','TD3-RL','LSTM'};
E = {errLog_PI, errLog_RL, errLog_LSTM};
col = [0 0.447 0.741; 0.85 0.325 0.098; 0.466 0.674 0.188];
sty = {'-','--','-.'};
hl  = [0 20];

fig = figure('Color','w','Name','Error comparison','Position',[80 80 1000 650]);
try, theme(fig,'light'); catch, end
tiledlayout(2,1,'TileSpacing','compact');

nexttile; hold on; grid on; box on
patch([hl(1) hl(2) hl(2) hl(1)],[-1 -1 10 10],[1 0.93 0.6],'EdgeColor','none', ...
      'FaceAlpha',0.6,'DisplayName','Highlighted area');
for c = 1:3
    stairs(E{c}.Time,squeeze(E{c}.Data),sty{c},'Color',col(c,:),'LineWidth',1.6,'DisplayName',names{c});
end
yline(0,'k:','HandleVisibility','off'); ylim([-1 10]);
xlabel('Time (s)'); ylabel('Error (m)'); legend('Location','northeast');
title('Error  e(t) = Desired water level - Feedback');

nexttile; hold on; grid on; box on
for c = 1:3
    stairs(E{c}.Time,squeeze(E{c}.Data),sty{c},'Color',col(c,:),'LineWidth',1.8,'DisplayName',names{c});
end
yline(0,'k:','HandleVisibility','off'); xlim(hl); ylim([-0.5 1]);
xlabel('Time (s)'); ylabel('Error (m)'); legend('Location','northeast');
title('Highlighted area (0-20 s), zoomed');

fprintf('\n%-16s %8s %8s %12s\n','Controller','IAE','ISE','Mean|e| 60-100s');
for c = 1:3
    e = squeeze(E{c}.Data); t = E{c}.Time;
    fprintf('%-16s %8.2f %8.2f %12.4f\n',names{c},sum(abs(e)),sum(e.^2),mean(abs(e(t>=60))));
end
