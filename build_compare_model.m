%% build_compare_model.m
% Builds watertank_compare.slx: three identical water-tank loops in one model
%   1) Conventional PI   (Control System Tuner gains)
%   2) TD3-RL            (PI gains learned by the TD3 agent)
%   3) LSTM              (PI block replaced by LSTM, 4-sample memory)
% All loops share the same reference (10 m), initial level (1 m) and noise
% seed, so their error signals e = Desired level - Feedback can be compared
% directly on one scope.

Ts = 1; Tf = 100; %#ok<NASGU>
mdl = 'watertank_compare';
if bdIsLoaded(mdl), close_system(mdl,0); end
if isfile([mdl '.slx']), delete([mdl '.slx']); end
if ~isfile('watertankPI_cmp.slx') || ~isfile('watertankLSTM.slx'), build_models; end

Kp_PI = 3.43726595858363;  Ki_PI = 0.0357784909005128;
S = load('WaterTankPITuningTD3AgentUseCase1.mat','agent1');
p = getLearnableParameters(getActor(S.agent1));
Ki_RL = double(p{1}(1));   Kp_RL = double(p{1}(2));

new_system(mdl); open_system(mdl);
set_param(mdl,'StopTime','Tf', ...
    'InitFcn','Ts = 1; Tf = 100;', 'PreLoadFcn','Ts = 1; Tf = 100;', ...
    'StopFcn','plot_comparison', 'ReturnWorkspaceOutputs','off');

loops = {'Conventional PI','watertankPI_cmp',Kp_PI,Ki_PI,'PI'
         'TD3-RL',         'watertankPI_cmp',Kp_RL,Ki_RL,'RL'
         'LSTM',           'watertankLSTM',  [],   [],   'LSTM'};

for i = 1:3
    sub = [mdl '/' loops{i,1}];
    add_block('simulink/Ports & Subsystems/Subsystem',sub, ...
        'Position',[100 60+(i-1)*130 260 140+(i-1)*130]);
    Simulink.SubSystem.deleteContents(sub);
    load_system(loops{i,2});
    Simulink.BlockDiagram.copyContentsToSubSystem(loops{i,2},sub);

    if ~isempty(loops{i,3})
        set_param([sub '/PID Controller'],'P',num2str(loops{i,3},10),'I',num2str(loops{i,4},10));
    end
    % same noise realisation as the report (seed = 1)
    set_param(sprintf([sub '/Band-Limited\nWhite Noise']),'seed','1');
    % unique workspace variable names per loop
    tw = find_system(sub,'BlockType','ToWorkspace');
    for k = 1:numel(tw)
        set_param(tw{k},'VariableName',[get_param(tw{k},'VariableName') '_' loops{i,5}]);
    end
    % outports: error and water level
    s1 = get_param([sub '/Sum1'],'PortHandles');
    add_block('simulink/Sinks/Out1',[sub '/error'],'Position',[700 300 730 314]);
    add_line(sub,s1.Outport(1),get_param([sub '/error'],'PortHandles').Inport(1),'autorouting','smart');
    w = get_param([sub '/Water-Tank System'],'PortHandles');
    add_block('simulink/Sinks/Out1',[sub '/level'],'Position',[700 360 730 374]);
    add_line(sub,w.Outport(1),get_param([sub '/level'],'PortHandles').Inport(1),'autorouting','smart');
end

% error scope + level scope
add_block('simulink/Signal Routing/Mux',[mdl '/MuxE'],'Inputs','3','Position',[380 60 385 300]);
add_block('simulink/Signal Routing/Mux',[mdl '/MuxH'],'Inputs','3','Position',[380 330 385 400]);
add_block('simulink/Sinks/Scope',[mdl '/Error Comparison'],'Position',[460 160 500 200]);
add_block('simulink/Sinks/Scope',[mdl '/Water Level Comparison'],'Position',[460 345 500 385]);
add_line(mdl,'MuxE/1','Error Comparison/1');
add_line(mdl,'MuxH/1','Water Level Comparison/1');
for i = 1:3
    le = add_line(mdl,[loops{i,1} '/1'],sprintf('MuxE/%d',i),'autorouting','smart');
    set_param(le,'Name',['e_' loops{i,5}]);
    lh = add_line(mdl,[loops{i,1} '/2'],sprintf('MuxH/%d',i),'autorouting','smart');
    set_param(lh,'Name',['H_' loops{i,5}]);
end
scLim = {[-1 10],[0 12]}; scNm = {'Error Comparison','Water Level Comparison'};
for k = 1:2
    cfg = get_param([mdl '/' scNm{k}],'ScopeConfiguration');
    cfg.ShowLegend = true; cfg.ShowGrid = true; cfg.Title = scNm{k};
    cfg.YLimits = scLim{k};
end

add_block('built-in/Note',[mdl '/info'],'Position',[100 470]);
set_param([mdl '/info'],'Text', sprintf(['Same plant, reference (10 m), initial level (1 m) and noise in all three loops.\n' ...
    'e = Desired water level - Feedback.  Run the model: scopes show e and H;\n' ...
    'after the run, plot_comparison draws the error plot with the highlighted area.']));

save_system(mdl);
close_system('watertankPI_cmp',0); close_system('watertankLSTM',0);
fprintf('Built %s.slx  (TD3 gains: Kp=%.4f Ki=%.4f)\n',mdl,Kp_RL,Ki_RL);
