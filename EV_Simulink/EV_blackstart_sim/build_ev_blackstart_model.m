function build_ev_blackstart_model()
% EV-based blackstart (phasor-level) Simulink model with robust wiring.
% It rebuilds from scratch and safely connects all lines.

mdl = 'EV_Blackstart_Sim';
% --- reset ---
bdclose('all'); pause(0.05);
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);

% --- helpers ---
    function blk = addblk(lib, name, pos, varargin)
        blk = [mdl '/' name];
        add_block(lib, blk, 'Position', pos, varargin{:});
    end
    function safe_add(src, dst)
        % src,dst are 'Block/port' strings
        try
            add_line(mdl, src, dst, 'autorouting','on');
        catch ME
            if contains(ME.message,'already has a line connection')
                % delete existing connection on dst inport then retry
                [dstBlk, dstPort] = strtok(dst,'/');
                dstPort = str2double(dstPort(2:end));
                ph = get_param([mdl '/' dstBlk], 'PortHandles');
                hL = get(ph.Inport(dstPort), 'Line');
                if hL~=-1, delete_line(hL); end
                add_line(mdl, src, dst, 'autorouting','on');
            else
                rethrow(ME);
            end
        end
    end

% --- constants/params ---
x=40; y=40; w=80; h=25;
addblk('simulink/Commonly Used Blocks/Constant','f_ref',[x y x+w y+h],'Value','50');
addblk('simulink/Commonly Used Blocks/Constant','M_const',[x y+50 x+w y+50+h],'Value','5000');
addblk('simulink/Commonly Used Blocks/Constant','D_const',[x y+100 x+w y+100+h],'Value','50000');
addblk('simulink/Commonly Used Blocks/Constant','Pform_max',[x y+150 x+w y+150+h],'Value','40000');

% --- frequency ODE path ---
addblk('simulink/Math Operations/Sum','Sum_f_minus_fref',[x+160 y x+200 y+h],'Inputs','+-');
addblk('simulink/Math Operations/Gain','Gain_D',[x+230 y x+290 y+h],'Gain','50000');
addblk('simulink/Math Operations/Sum','Sum_Pmismatch_minus_D',[x+320 y x+380 y+h],'Inputs','+-');
addblk('simulink/Math Operations/Gain','Gain_1_over_M',[x+410 y x+470 y+h],'Gain','1/5000');
addblk('simulink/Continuous/Integrator','Integrator_f',[x+500 y-10 x+540 y+h+10],'InitialCondition','50');

% wire f - f_ref to damping branch
safe_add('Integrator_f/1','Sum_f_minus_fref/1');
safe_add('f_ref/1','Sum_f_minus_fref/2');
safe_add('Sum_f_minus_fref/1','Gain_D/1');

% --- (f_ref - f) branch for droop signals ---
addblk('simulink/Math Operations/Sum','Sum_fref_minus_f',[x+160 y+60 x+200 y+60+h],'Inputs','+-');
safe_add('f_ref/1','Sum_fref_minus_f/1');
safe_add('Integrator_f/1','Sum_fref_minus_f/2');

% tap (f_ref - f) with Goto/From
addblk('simulink/Signal Routing/Goto','goto_fref_minus_f',[x+210 y+100 x+240 y+100+h],'GotoTag','fref_minus_f');
addblk('simulink/Signal Routing/From','from_fref_minus_f_former',[x+160 y+150 x+190 y+150+h],'GotoTag','fref_minus_f');
addblk('simulink/Signal Routing/From','from_fref_minus_f_follow',[x+160 y+190 x+190 y+190+h],'GotoTag','fref_minus_f');
safe_add('Sum_fref_minus_f/1','goto_fref_minus_f/1');

% --- EV1 grid-former: P_form = sat(K_form*(f_ref - f),0..Pform_max) ---
addblk('simulink/Math Operations/Gain','Gain_K_form',[x+230 y+150 x+290 y+150+h],'Gain','200000');
addblk('simulink/Discontinuities/Saturation','Saturation_Pform',[x+320 y+150 x+380 y+150+h],'UpperLimit','Pform_max','LowerLimit','0');
safe_add('from_fref_minus_f_former/1','Gain_K_form/1');
safe_add('Gain_K_form/1','Saturation_Pform/1');

% --- followers aggregate (placeholder droop): P_follow = sat(K_follow*(f_ref - f),0..200e3) ---
addblk('simulink/Math Operations/Gain','Gain_K_follow',[x+230 y+190 x+290 y+190+h],'Gain','6000');
addblk('simulink/Discontinuities/Saturation','Saturation_Pfollow',[x+320 y+190 x+380 y+190+h],'UpperLimit','200000','LowerLimit','0');
safe_add('from_fref_minus_f_follow/1','Gain_K_follow/1');
safe_add('Gain_K_follow/1','Saturation_Pfollow/1');

% --- Pgen = P_form + P_follow ---
addblk('simulink/Math Operations/Sum','Sum_Pgen',[x+410 y+150 x+470 y+150+h],'Inputs','++');
safe_add('Saturation_Pform/1','Sum_Pgen/1');
safe_add('Saturation_Pfollow/1','Sum_Pgen/2');

% --- Loads (staged) ---
addblk('simulink/Sources/Step','Load_step1',[x+160 y+250 x+200 y+250+h],'Time','30','Before','0','After','100000');
addblk('simulink/Sources/Step','Load_step2',[x+160 y+280 x+200 y+280+h],'Time','120','Before','0','After','200000');
addblk('simulink/Sources/Step','Load_step3',[x+160 y+310 x+200 y+310+h],'Time','240','Before','0','After','250000');
addblk('simulink/Math Operations/Sum','Sum_Pload',[x+230 y+275 x+290 y+275+h],'Inputs','+++');
safe_add('Load_step1/1','Sum_Pload/1');
safe_add('Load_step2/1','Sum_Pload/2');
safe_add('Load_step3/1','Sum_Pload/3');

% --- (Pgen - Pload) then minus D*(f - f_ref) ---
addblk('simulink/Math Operations/Sum','Sum_Pgen_minus_Pload',[x+320 y+200 x+380 y+200+h],'Inputs','+-');
safe_add('Sum_Pgen/1','Sum_Pgen_minus_Pload/1');
safe_add('Sum_Pload/1','Sum_Pgen_minus_Pload/2');

% feed into ODE sum
safe_add('Sum_Pgen_minus_Pload/1','Sum_Pmismatch_minus_D/1');
safe_add('Gain_D/1','Sum_Pmismatch_minus_D/2');
safe_add('Sum_Pmismatch_minus_D/1','Gain_1_over_M/1');
safe_add('Gain_1_over_M/1','Integrator_f/1');

% --- Scopes & logs ---
addblk('simulink/Sinks/Scope','Scope_Main',[x+560 y x+620 y+140]); set_param([mdl '/Scope_Main'],'NumInputPorts','3');
safe_add('Integrator_f/1','Scope_Main/1');
safe_add('Sum_Pload/1','Scope_Main/2');
safe_add('Sum_Pgen/1','Scope_Main/3');

addblk('simulink/Sinks/To Workspace','toWS_f',[x+560 y+150 x+620 y+170], ...
    'VariableName','f_Hz','SaveFormat','StructureWithTime');
addblk('simulink/Sinks/To Workspace','toWS_Pgen',[x+560 y+180 x+620 y+200], ...
    'VariableName','P_gen','SaveFormat','StructureWithTime');
addblk('simulink/Sinks/To Workspace','toWS_Pload',[x+560 y+210 x+620 y+230], ...
    'VariableName','P_load','SaveFormat','StructureWithTime');
addblk('simulink/Sinks/To Workspace','toWS_Pform',[x+560 y+240 x+620 y+260], ...
    'VariableName','P_form','SaveFormat','StructureWithTime');
addblk('simulink/Sinks/To Workspace','toWS_Pfollow',[x+560 y+270 x+620 y+290], ...
    'VariableName','P_follow','SaveFormat','StructureWithTime');

safe_add('Integrator_f/1','toWS_f/1');
safe_add('Sum_Pgen/1','toWS_Pgen/1');
safe_add('Sum_Pload/1','toWS_Pload/1');
safe_add('Saturation_Pform/1','toWS_Pform/1');
safe_add('Saturation_Pfollow/1','toWS_Pfollow/1');

% --- sim params ---
set_param(mdl,'StopTime','600','Solver','ode45');

save_system(mdl);
disp('✅ Built EV_Blackstart_Sim.slx with all connections.');
open_system(mdl);
end
