function blackstart_ev_microgrid
%% Adaptive Microgrid Blackstart using EVs as Mobile Generators


clearvars -except ans; clc; close all;

%% Nominal + simulation timing
f0   = 50;       % nominal frequency [Hz]
Tend = 600;      % total sim time [s]

% Virtual swing (frequency dynamics)
M = 5e3;         % "inertia-like" coefficient [W/Hz*s]
D = 5e4;         % damping [W/Hz]

%% Load restoration (critical -> essential -> general)
events.load_times    = [30 120 240 360];         % [s]
events.load_sizes_kW = [100 200 250 300];        % [kW]

%% EV fleet spec:  [id  t_in  t_out  Pmax(kW)  Ecap(kWh)  SOC0(%)  eta]
EVspec = [
   1    0     600     60        60        80     0.95;   % EV1 grid-former
   2   50     600     50        50        65     0.95;
   3   90     600     40        40        55     0.95;
   4  180     480     60        70        70     0.95;
   5  300     600     80        80        90     0.95;
];

gridFormerID   = 1;
SOCmin = 20; SOCmax = 95;      % SOC limits [%]
Pform_max_kW   = 40;           % grid-former power cap [kW]
K_base         = 6e3;          % fleet droop stiffness [W/Hz]
alpha_soc      = 0.7;          % SOC weight (0..1)

%% Distance-based prioritization (NEW)
% Set distance (km) of each EV to the site. EV1 can be 0 (already on site).
distance_km = [0 4 7 3 2];     % EV1..EV5
beta_dist   = 0.01;            % penalty per km (0.01 => ~1% less priority per km)

%% Pre-compute/pack parameters
load_steps = events.load_times(:);
load_sizes = events.load_sizes_kW(:)*1e3; % [W]

N     = size(EVspec,1);
t_in  = EVspec(:,2);
t_out = EVspec(:,3);
Pmax  = EVspec(:,4)*1e3;      % [W]
Ecap  = EVspec(:,5)*3.6e6;    % [J]
SOC0  = EVspec(:,6);
eta   = EVspec(:,7);

params.f0 = f0; params.M = M; params.D = D;
params.K_base = K_base; params.alpha_soc = alpha_soc;
params.Pmax = Pmax; params.Ecap = Ecap; params.SOCmin = SOCmin; params.SOCmax = SOCmax;
params.eta = eta; params.gridFormerID = gridFormerID;
params.Pform_max = Pform_max_kW*1e3; % [W]
params.t_in  = t_in; params.t_out = t_out;
params.load_steps = load_steps; params.load_sizes = load_sizes;

% Distance parameters into params (NEW)
params.distance_km = distance_km(:);
params.beta_dist   = beta_dist;

%% ODE state: x = [f; SOC(1..N)]
x0 = [f0; SOC0];

opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.2);
[t, x] = ode45(@(tt,xx) dyn(tt,xx,params), [0 Tend], x0, opts);

f   = x(:,1);
SOC = x(:,2:end);

% Signals for plots
Pload_series = arrayfun(@(tt) Pload_of_t(tt, load_steps, load_sizes), t);

% Rebuild EV powers across trajectory using same logic (for plotting)
Pev_series = zeros(numel(t), N);
for k = 1:numel(t)
    tt    = t(k);
    f_k   = f(k);
    SOC_k = SOC(k,:)';
    present       = (tt >= t_in) & (tt <= t_out);
    can_discharge = present & (SOC_k > SOCmin+0.1);

    % Grid-former power
    K_form = 2e5;  % [W/Hz]
    P_form = max(0, min(params.Pform_max, K_form*(f0 - f_k)));
    if ~present(gridFormerID), P_form = 0; end

    % Followers: SOC+distance weights
    headroom   = zeros(N,1);
    soc_weight = zeros(N,1);
    for i = 1:N
        if can_discharge(i)
            headroom(i)   = Pmax(i);
            soc_weight(i) = max(0, (SOC_k(i)-SOCmin)/(SOCmax-SOCmin));
        end
    end
    follower = true(N,1); follower(gridFormerID) = false;

    dist_fac = 1 + beta_dist * distance_km(:);  % NEW
    w = headroom .* ((1-alpha_soc) + alpha_soc*soc_weight);
    w = w ./ dist_fac;                          % NEW
    w(~follower) = 0;
    if sum(w) > 0, w = w / sum(w); end
    K_i = K_base * w;

    P_ev_k = zeros(N,1);
    for i = 1:N
        if follower(i) && can_discharge(i)
            P_cmd       = K_i(i) * (f0 - f_k);
            P_ev_k(i)   = max(0, min(Pmax(i), P_cmd));
        end
    end
    P_ev_k(gridFormerID) = P_form;
    Pev_series(k,:) = P_ev_k;
end

Pgen_series = sum(Pev_series,2);

%% Plots
% 1) Frequency (with blackout marker)
figure('Name','Frequency'); plot(t, f, 'LineWidth',1.6);
yline(f0,'--'); xlabel('Time [s]'); ylabel('Frequency [Hz]');
title('Frequency Restoration during Blackstart'); grid on;
xline(0,'r--','Blackout Detected','LabelOrientation','horizontal');  % NEW

% 2) Load vs Generation
figure('Name','Power Balance'); plot(t, Pload_series/1e3, 'LineWidth',1.6); hold on;
plot(t, Pgen_series/1e3, 'LineWidth',1.6);
xlabel('Time [s]'); ylabel('Power [kW]'); grid on;
legend('Load','Total EV Gen','Location','best');
title('Load Pickup vs EV Generation');

% 3) EV Power (stacked)
figure('Name','EV Power Outputs');
area(t, max(Pev_series,0)/1e3); xlabel('Time [s]'); ylabel('Power [kW]');
title('Individual EV Outputs (stacked)'); grid on;
legend(arrayfun(@(i) sprintf('EV%d (Dist = %dkm)', i, distance_km(i)), 1:N, 'UniformOutput', false), ...
       'Location','bestoutside');


% 4) SOC
figure('Name','SOC');
plot(t, SOC, 'LineWidth',1.4); xlabel('Time [s]'); ylabel('SOC [%]');
title('EV SOC Trajectories'); grid on; ylim([SOCmin-5 SOCmax+5]);
yline(SOCmin,'--'); yline(SOCmax,'--');
legend(arrayfun(@(i) sprintf('EV%d',i), 1:N, 'UniformOutput', false), 'Location','bestoutside');

% 5) Events timeline
figure('Name','Events');
stairs(t, Pload_series/1e3, 'LineWidth',1.6); hold on;
for i=1:N
    plot([t_in(i) t_out(i)], [-(10+i*5) -(10+i*5)], 'LineWidth',4);
end
xlabel('Time [s]'); ylabel('kW (load)'); grid on;
title('Staged Load Pickup & EV Availability (bars below)');
ylim([-10-5*N, max(Pload_series/1e3)*1.2]);

% Expose for re-plotting / KPI scripts
assignin('base','t',t);
assignin('base','f',f);
assignin('base','Pload_series',Pload_series);
assignin('base','Pev_series',Pev_series);
assignin('base','Pgen_series',Pgen_series);
assignin('base','SOC',SOC);
assignin('base','Pmax',Pmax);
assignin('base','distance_km',distance_km);
assignin('base','Ecap_kWh',EVspec(:,5));
assignin('base','eta',eta);


end % ===== end main function =====

% ---------- Local helpers (must be after main) ----------
function P = Pload_of_t(t, load_steps, load_sizes)
% Piecewise-constant load sum up to time t
P = sum(load_sizes(t >= load_steps));
end

function dx = dyn(t, x, params)
% ODE: x = [f; SOC(1..N)]
f   = x(1); 
SOC = x(2:end);
N   = numel(SOC);

% Load
Pload = Pload_of_t(t, params.load_steps, params.load_sizes);

% Availability/SOC
present       = (t >= params.t_in) & (t <= params.t_out);
can_discharge = present & (SOC(:) > params.SOCmin+0.1);

% Grid-former power with cap
K_form = 2e5;  % [W/Hz]
P_form = max(0, min(params.Pform_max, K_form*(params.f0 - f)));
if ~present(params.gridFormerID), P_form = 0; end

% Followers: weights with SOC + distance (NEW)
headroom   = zeros(N,1);
soc_weight = zeros(N,1);
for i = 1:N
    if can_discharge(i)
        headroom(i)   = params.Pmax(i);
        soc_weight(i) = max(0, (SOC(i)-params.SOCmin)/(params.SOCmax-params.SOCmin));
    end
end
follower = true(N,1); follower(params.gridFormerID) = false;

dist_fac = 1 + params.beta_dist * params.distance_km;  % NEW
w = headroom .* ((1-params.alpha_soc) + params.alpha_soc*soc_weight);
w = w ./ dist_fac;                                      % NEW
w(~follower) = 0;
if sum(w) > 0, w = w / sum(w); end

K_i = params.K_base * w;

% EV powers
P_ev = zeros(N,1);
for i = 1:N
    if follower(i) && can_discharge(i)
        P_cmd = K_i(i) * (params.f0 - f);
        P_ev(i) = max(0, min(params.Pmax(i), P_cmd));
    end
end
P_ev(params.gridFormerID) = P_form;

% Frequency dynamics
Pgen     = sum(P_ev);
Pmismatch= Pgen - Pload;
dfdt     = (Pmismatch - params.D*(f - params.f0)) / params.M;

% SOC dynamics
dSOC = zeros(N,1);
for i = 1:N
    if present(i)
        dSOC(i) = -(P_ev(i) / params.Ecap(i)) * 100 / params.eta(i);
    else
        dSOC(i) = 0;
    end
    if (SOC(i) <= params.SOCmin && dSOC(i) < 0), dSOC(i) = 0; end
    if (SOC(i) >= params.SOCmax && dSOC(i) > 0), dSOC(i) = 0; end
end

dx = [dfdt; dSOC];
end
