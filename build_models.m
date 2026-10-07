%% build_models.m
% Creates the two comparison models from the MathWorks plant 'watertankLQG':
%   watertankPI_cmp.slx   - original plant + PI controller, with loggers
%   watertankLSTM.slx     - same plant, PI controller replaced by LSTM block
% Logged signals (To Workspace, sampled every Ts):
%   errLog = Desired water level - feedback (Sum1 output)
%   uLog   = controller output
%   hLog   = water level H

srcMdl = 'watertankLQG';
load_system(srcMdl);

%% 1) PI model with loggers
piMdl = 'watertankPI_cmp';
if bdIsLoaded(piMdl), close_system(piMdl,0); end
if isfile([piMdl '.slx']), delete([piMdl '.slx']); end
save_system(srcMdl,piMdl);           % copy (leaves srcMdl unchanged on disk)
close_system(piMdl,0); load_system(piMdl);
addLoggers(piMdl,[piMdl '/PID Controller']);
save_system(piMdl);

%% 2) LSTM model: replace the PID Controller block
lstmMdl = 'watertankLSTM';
if bdIsLoaded(lstmMdl), close_system(lstmMdl,0); end
if isfile([lstmMdl '.slx']), delete([lstmMdl '.slx']); end
save_system(piMdl,lstmMdl);
close_system(lstmMdl,0); load_system(lstmMdl);

pid = [lstmMdl '/PID Controller'];
pos = get_param(pid,'Position');
ph  = get_param(pid,'PortHandles');

% source feeding the PID (Sum1 output) and every destination of that net
inLine  = get_param(ph.Inport(1),'Line');
inSrc   = get_param(inLine,'SrcPortHandle');
inDsts  = get_param(inLine,'DstPortHandle');
inDsts  = inDsts(inDsts ~= ph.Inport(1));
% destinations of the PID output net
outLine = get_param(ph.Outport(1),'Line');
outDsts = get_param(outLine,'DstPortHandle');

delete_line(inLine); delete_line(outLine); delete_block(pid);

ctl = [lstmMdl '/LSTM Controller'];
add_block('simulink/User-Defined Functions/MATLAB System',ctl, ...
    'System','LSTMController','Position',pos);
set_param(ctl,'NetFile','lstmController.mat','Ts','Ts');
cph = get_param(ctl,'PortHandles');

for d = [inDsts(:)' cph.Inport(1)]
    add_line(lstmMdl,inSrc,d,'autorouting','smart');
end
for d = outDsts(:)'
    add_line(lstmMdl,cph.Outport(1),d,'autorouting','smart');
end
save_system(lstmMdl);
close_system(lstmMdl,0); close_system(piMdl,0); close_system(srcMdl,0);
disp('Models built: watertankPI_cmp.slx, watertankLSTM.slx');

%% ------------------------------------------------------------------------
function addLoggers(mdl,ctlBlk)
    % error (Sum1 output)
    sum1 = get_param([mdl '/Sum1'],'PortHandles');
    logTo(mdl,'errLog',sum1.Outport(1),[60 300]);
    % controller output
    c = get_param(ctlBlk,'PortHandles');
    logTo(mdl,'uLog',c.Outport(1),[60 360]);
    % water level
    w = get_param([mdl '/Water-Tank System'],'PortHandles');
    logTo(mdl,'hLog',w.Outport(1),[60 420]);
end

function logTo(mdl,name,srcPort,xy)
    blk = [mdl '/' name];
    add_block('simulink/Sinks/To Workspace',blk, ...
        'VariableName',name,'SaveFormat','Timeseries','SampleTime','Ts', ...
        'Position',[xy(1) xy(2) xy(1)+70 xy(2)+30]);
    p = get_param(blk,'PortHandles');
    add_line(mdl,srcPort,p.Inport(1),'autorouting','smart');
end
