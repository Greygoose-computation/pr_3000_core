
load ('data.mat')

%g=config.env.g;

% symbolic declerations 

syms mp mt h_pivot h_cmload l_np1 l_np2 accel Np1 Np2 U_fgl g frcnp1 frcnp2
syms frc_lr frc_lf

% mp=config.m.pay;
% mt=config.m.trailer;
% h_pivot=config.h.pivot;
% h_cmload=config.h.com_load;
% l_np1=config.l.np1;
% l_np2=config.l.np2;

m_load=mp+mt;    % combined load

[v,h]=get_slope(config.env.slope);  % vertical and horiontal components 

n_total=m_load*g*v;

% simple friction model 

%--------------------------------------------------------------------------
% Load friction Dynamics 
%--------------------------------------------------------------------------


Np1 = ((U_fgl*h_pivot)+(m_load*g*v*l_np2)-(m_load*accel*h_cmload)-(m_load*g*h))/(l_np1+l_np2);
Np2 = (n_total)-(Np1);

%--------------------------------------------------------------------------
% Load motion dynamics 
%--------------------------------------------------------------------------
F_lfrc=(Np1*frc_lf+Np2*frc_lr)+(m_load*accel)+(m_load*g*h);
U_fgl = F_lfrc + m_load*accel + m_load*g*h;
%%U_fgl = vpa(U_fgl, 3);

%--------------------------------------------------------------------------
% simplify don't touch | Load dynamics 
%--------------------------------------------------------------------------

% --- isolate U_fgl cleanly (do not change earlier code) ---
Ueqn = sym('U_fgl') == U_fgl;     % make an equation: original symbol == your expression

% Bring to one side: E == 0
E = lhs(Ueqn) - rhs(Ueqn);

% Linear isolate (stable for long expressions)
a = simplify(diff(E, sym('U_fgl')));      % coefficient of U_fgl
b = simplify(subs(E, sym('U_fgl'), 0));   % constant term

U_fgl_sol = simplify(-b/a, 'Steps', 200); % explicit U_fgl = ...
%U_fgl_sol = vpa(U_fgl_sol, 3);            % keep your 3-digit style

%U_fgl_explicit = sym('U_fgl') == U_fgl_sol; % test var 

%--------------------------------------------------------------------------
% Robot dynamics 
%--------------------------------------------------------------------------
syms m_drv m_chbt
syms h_cmrbt l_nr1 l_nr2
syms Fp frc_drv frc_cstr N_drv N_cstr real 

m_robot=m_drv+m_chbt;    % combined robot mass 
nr_total=m_robot*g*v;

%--------------------------------------------------------------------------
% Robot friction Dynamics 
%--------------------------------------------------------------------------

N_drv=((m_robot*g*l_nr2*v)-(m_robot*g*h_cmrbt*h)-(U_fgl_sol*h_pivot))/(l_nr1+l_nr2);

N_cstr=(nr_total-N_drv);

%--------------------------------------------------------------------------
% Robot motion Dynamics 
%--------------------------------------------------------------------------


F_p = (N_cstr*frc_cstr)+(m_robot*accel)+(m_robot*g*h);

%F_p = vpa(F_p,3);   % mute now for accuracy 

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% --- isolate accel: || do not touch 

syms Fp_sym real            % a symbol for the LHS "F_p"

eqn0 = Fp_sym - F_p;        % bring to one side: eqn0 == 0

A = simplify(diff(eqn0, accel));      % coefficient of accel
B = simplify(subs(eqn0, accel, 0));   % constant term

accel_sol = simplify(-B/A, 'Steps', 200);  % accel = ...
%%accel_sol = vpa(accel_sol, 3);             % keep same numeric style

%%accel_eqn = accel == accel_sol;

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

%%
residual = simplify(subs(Fp_sym - F_p, accel, accel_sol), 'Steps', 200);


%% addidition of the static friction in the model (before this point subs and check the values)

