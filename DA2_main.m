%% DA-2 : Water-tank level control  -  PI vs TD3-RL vs LSTM
% Plant / PI / TD3 parts follow the MathWorks example
% "Tune PI Controller Using Reinforcement Learning" (TD3).
% The LSTM controller replaces the PI controller in the same plant model
% and uses the previous 4 memory samples of [error; integral of error].
%
% Run order: build_models.m (once)  ->  DA2_main.m

clear; clc; close all;
rng(0,"twister");
Ts = 1;  Tf = 100;                 % sample time / sim time (from example)
resDir = fullfile(pwd,'results'); if ~isfolder(resDir), mkdir(resDir); end

if ~isfile('watertankPI_cmp.slx') || ~isfile('watertankLSTM.slx')
    build_models;
end
piMdl = 'watertankPI_cmp';  lstmMdl = 'watertankLSTM';
load_system(piMdl); load_system(lstmMdl);

%% 1. Controller gains
% Conventional PI (tuned with Control System Tuner, values from the example)
Kp_PI = 3.43726595858363;
Ki_PI = 0.0357784909005128;

% TD3-RL : pretrained TD3 agent, its actor weights are the PI gains
load("WaterTankPITuningTD3AgentUseCase1.mat","agent1")
p = getLearnableParameters(getActor(agent1));
Ki_RL = double(p{1}(1));
Kp_RL = double(p{1}(2));
fprintf('Conventional PI : Kp = %.4f  Ki = %.4f\n',Kp_PI,Ki_PI);
fprintf('TD3-RL PI       : Kp = %.4f  Ki = %.4f\n',Kp_RL,Ki_RL);

%% 2. Generate training data for the LSTM (imitation of the PI controller)
% Closed-loop episodes are run with randomised reference, initial level,
% noise seed and perturbed gains (to visit a wide range of states).
% The target for every sample is the conventional PI law
%       u*(k) = Kp*e(k) + Ki*Ts*sum_{j<k} e(j)
nEp = 300;
in(1:nEp) = Simulink.SimulationInput(piMdl);
for i = 1:nEp
    in(i) = baseInput(in(i),piMdl,Ts,Tf, 8+4*rand, 2*rand, randi(1e4));
    in(i) = setGains(in(i),piMdl,Kp_PI*(0.4+1.6*rand),Ki_PI*(0.3+3*rand));
end
fprintf('Simulating %d training episodes...\n',nEp);
out = sim(in,'ShowProgress','off');

W = 4;                               % memory window (previous 4 samples)
X = {}; Y = [];
for i = 1:nEp
    e  = squeeze(out(i).errLog.Data);  e = e(:)';
    ie = [0 Ts*cumsum(e(1:end-1))];    % forward-Euler integral (state)
    z  = [zeros(2,W-1) [e; ie]];       % zero-padded like the controller
    for k = 1:numel(e)
        X{end+1,1} = z(:,k:k+W-1);                 %#ok<SAGROW>
        Y(end+1,1) = Kp_PI*e(k) + Ki_PI*ie(k);     %#ok<SAGROW>
    end
end
allX = cat(2,X{:});
scale.x = max(abs(allX),[],2);       % per-channel normalisation
scale.u = max(abs(Y));
Xn = cellfun(@(x) (x./scale.x)', X,'UniformOutput',false);   % time x channel
Yn = Y/scale.u;
fprintf('Training samples: %d\n',numel(Yn));

%% 3. LSTM network and training
layers = [
    sequenceInputLayer(2,Name='in')          % [e ; integral e]
    lstmLayer(32,OutputMode='last',Name='lstm')
    fullyConnectedLayer(16)
    tanhLayer
    fullyConnectedLayer(1,Name='u')];

idx  = randperm(numel(Yn)); nVal = round(0.15*numel(Yn));
iVal = idx(1:nVal); iTr = idx(nVal+1:end);
opts = trainingOptions("adam", ...
    MaxEpochs=60, MiniBatchSize=256, InitialLearnRate=3e-3, ...
    LearnRateSchedule="piecewise", LearnRateDropPeriod=20, LearnRateDropFactor=0.3, ...
    Shuffle="every-epoch", ValidationData={Xn(iVal),Yn(iVal)}, ...
    ValidationFrequency=200, Plots="none", Verbose=true, VerboseFrequency=500);
[net,info] = trainnet(Xn(iTr),Yn(iTr),layers,"mse",opts);
save('lstmController.mat','net','scale','W');

fig = figure('Color','w','Position',[100 100 700 350]); theme(fig,'light');
semilogy(info.TrainingHistory.Iteration,info.TrainingHistory.Loss,'Color',[.6 .6 .6]); hold on
semilogy(info.ValidationHistory.Iteration,info.ValidationHistory.Loss,'o-','LineWidth',1.5);
grid on; xlabel('Iteration'); ylabel('MSE loss (normalised)');
legend('Training','Validation'); title('LSTM controller training');
exportgraphics(fig,fullfile(resDir,'lstm_training_loss.png'),Resolution=150);

%% 4. Closed-loop test: same scenario as the MathWorks example
% reference = 10, initial level = 1, noise seed = 1
scen = struct('name',{'Test 1 (ref=10, H0=1)','Test 2 (ref=12, H0=0.5)','Test 3 (ref=8, H0=2)'}, ...
              'ref',{10,12,8},'h0',{1,0.5,2},'seed',{1,7,23});
names = {'Conventional PI','TD3-RL','LSTM'};
R = struct();
for s = 1:numel(scen)
    sc = scen(s);
    inPI = baseInput(Simulink.SimulationInput(piMdl),piMdl,Ts,Tf,sc.ref,sc.h0,sc.seed);
    r{1} = sim(setGains(inPI,piMdl,Kp_PI,Ki_PI));
    r{2} = sim(setGains(inPI,piMdl,Kp_RL,Ki_RL));
    r{3} = sim(baseInput(Simulink.SimulationInput(lstmMdl),lstmMdl,Ts,Tf,sc.ref,sc.h0,sc.seed));
    for c = 1:3
        R(s,c).t   = r{c}.errLog.Time;
        R(s,c).e   = squeeze(r{c}.errLog.Data);
        R(s,c).u   = squeeze(r{c}.uLog.Data);
        R(s,c).h   = squeeze(r{c}.hLog.Data);
        R(s,c).lqg = sum(r{c}.cost.Data);
        R(s,c).ref = sc.ref;
    end
end

%% 5. Metrics table (Test 1) and robustness table (all tests)
T = metricsTable(R(1,:),names,Ts);
disp(T);
fprintf('RMS difference LSTM vs PI control signal (Test 1): %.4f\n', rms(R(1,3).u-R(1,1).u));
writetable(T,fullfile(resDir,'metrics_test1.csv'),'WriteRowNames',true);

rob = zeros(numel(scen),3);
for s = 1:numel(scen), for c = 1:3, rob(s,c) = sum(abs(R(s,c).e))*Ts; end, end
Trob = array2table(rob,'VariableNames',matlab.lang.makeValidName(names), ...
                   'RowNames',{scen.name});
disp('IAE for all test scenarios:'); disp(Trob);
writetable(Trob,fullfile(resDir,'iae_all_tests.csv'),'WriteRowNames',true);

%% 6. Plots - error signal comparison with highlighted area
col = [0 0.447 0.741; 0.85 0.325 0.098; 0.466 0.674 0.188];
sty = {'-','--','-.'};
hl  = [0 20];                        % highlighted transient window (s)

fig = figure('Color','w','Position',[50 50 1100 750]); theme(fig,'light');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
title(tl,'Error signal  e(t) = Desired water level - Feedback   (Test 1)','FontWeight','bold');

ax1 = nexttile(tl,[1 2]); hold on; grid on; box on
yl = [min(arrayfun(@(x) min(x.e),R(1,:)))-0.5, max(arrayfun(@(x) max(x.e),R(1,:)))+0.5];
patch([hl(1) hl(2) hl(2) hl(1)],[yl(1) yl(1) yl(2) yl(2)],[1 0.93 0.6], ...
      'EdgeColor',[0.9 0.6 0],'LineStyle','--','FaceAlpha',0.5,'DisplayName','Highlighted area');
for c = 1:3
    stairs(R(1,c).t,R(1,c).e,sty{c},'Color',col(c,:),'LineWidth',1.6,'DisplayName',names{c});
end
yline(0,'k:','HandleVisibility','off'); ylim(yl); xlim([0 Tf]);
xlabel('Time (s)'); ylabel('Error (m)'); legend('Location','northeast');
title('Full simulation');

ax2 = nexttile(tl); hold on; grid on; box on
set(ax2,'Color',[1 0.97 0.85]);
for c = 1:3
    stairs(R(1,c).t,R(1,c).e,sty{c},'Color',col(c,:),'LineWidth',1.8,'DisplayName',names{c});
end
yline(0,'k:','HandleVisibility','off'); xlim(hl);
xlabel('Time (s)'); ylabel('Error (m)'); title('Highlighted area: transient (0-20 s)');
legend('Location','northeast');

ax3 = nexttile(tl); hold on; grid on; box on
for c = 1:3
    stairs(R(1,c).t,R(1,c).e,sty{c},'Color',col(c,:),'LineWidth',1.4,'DisplayName',names{c});
end
yline(0,'k:','HandleVisibility','off'); xlim([20 Tf]); ylim([-1 1]);
xlabel('Time (s)'); ylabel('Error (m)'); title('Steady state (20-100 s), zoomed');
legend('Location','northeast');
exportgraphics(fig,fullfile(resDir,'error_comparison.png'),Resolution=150);

% water level and control signal
fig = figure('Color','w','Position',[80 80 1100 450]); theme(fig,'light');
tl = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; hold on; grid on; box on
for c = 1:3, plot(R(1,c).t,R(1,c).h,sty{c},'Color',col(c,:),'LineWidth',1.6); end
yline(R(1,1).ref,'k:','Reference'); xlabel('Time (s)'); ylabel('Water level H (m)');
legend(names,'Location','southeast'); title('Water level');
nexttile; hold on; grid on; box on
for c = 1:3, stairs(R(1,c).t,R(1,c).u,sty{c},'Color',col(c,:),'LineWidth',1.4); end
xlabel('Time (s)'); ylabel('Controller output u'); legend(names); title('Control effort');
exportgraphics(fig,fullfile(resDir,'level_and_control.png'),Resolution=150);

% error in all three test scenarios
fig = figure('Color','w','Position',[80 80 1200 380]); theme(fig,'light');
tl = tiledlayout(1,numel(scen),'TileSpacing','compact','Padding','compact');
title(tl,'Error signal for different references / initial levels','FontWeight','bold');
for s = 1:numel(scen)
    nexttile; hold on; grid on; box on
    patch([hl(1) hl(2) hl(2) hl(1)],[-20 -20 20 20],[1 0.93 0.6],'EdgeColor','none','FaceAlpha',0.5);
    for c = 1:3, stairs(R(s,c).t,R(s,c).e,sty{c},'Color',col(c,:),'LineWidth',1.4); end
    ylim([min(arrayfun(@(x) min(x.e),R(s,:)))-0.5, max(arrayfun(@(x) max(x.e),R(s,:)))+0.5]);
    xlabel('Time (s)'); ylabel('Error (m)'); title(scen(s).name);
    if s==1, legend(['Highlighted area' names]); end
end
exportgraphics(fig,fullfile(resDir,'error_all_tests.png'),Resolution=150);

save(fullfile(resDir,'DA2_results.mat'),'R','T','Trob','Kp_PI','Ki_PI','Kp_RL','Ki_RL','scen','names');
disp('Done. Figures and tables saved in ./results');

%% ------------------------------------------------------------------------
function in = baseInput(in,mdl,Ts,Tf,ref,h0,seed)
    in = in.setVariable('Ts',Ts).setVariable('Tf',Tf);
    in = in.setBlockParameter(sprintf([mdl '/Desired \nWater Level']),'Value',num2str(ref));
    in = in.setBlockParameter([mdl '/Water-Tank System/H'],'InitialCondition',num2str(h0));
    in = in.setBlockParameter(sprintf([mdl '/Band-Limited\nWhite Noise']),'Seed',num2str(seed));
end

function in = setGains(in,mdl,Kp,Ki)
    blk = [mdl '/PID Controller'];
    in = in.setBlockParameter(blk,'P',num2str(Kp,10));
    in = in.setBlockParameter(blk,'I',num2str(Ki,10));
end

function T = metricsTable(Rrow,names,Ts)
    n = numel(Rrow);
    [IAE,ISE,ITAE,SSE,Rise,Settle,OS,maxU,LQG] = deal(zeros(n,1));
    for c = 1:n
        t = Rrow(c).t; e = Rrow(c).e; h = Rrow(c).h;
        IAE(c)  = sum(abs(e))*Ts;
        ISE(c)  = sum(e.^2)*Ts;
        ITAE(c) = sum(t.*abs(e))*Ts;
        SSE(c)  = mean(abs(e(t>=60)));
        si = stepinfo(h,t,Rrow(c).ref,h(1));
        Rise(c) = si.RiseTime; Settle(c) = si.SettlingTime; OS(c) = si.Overshoot;
        maxU(c) = max(abs(Rrow(c).u));
        LQG(c)  = Rrow(c).lqg;
    end
    T = table(IAE,ISE,ITAE,SSE,Rise,Settle,OS,maxU,LQG, ...
        'VariableNames',{'IAE','ISE','ITAE','MeanAbsErr_60_100s','RiseTime_s', ...
                         'SettlingTime_s','Overshoot_pct','MaxControl','LQG_Cost'}, ...
        'RowNames',names);
end
