
%--------------------------------------------------------------------------

% basic wheel dynamics 

% Kt=0.01;  Torque current constant
% T_drive = Kt * i_a; Kt=Torque current constant , ia=current(amps)
% T_drive = F_m * r_drive  % torque : force * radius 

% Fm = (Kt * i_a)/(r_drive)

%--------------------------------------------------------------------------

%--------------------------------------------------------------------------

% friction modelling 

% N_drive : normal force on the drive wheel 
% l_b : length (COM to rear of the robot)
% l_f : length (COM to front of the robot)
% M_b : mass of battery 
% M_w : mass of the deadweight 
% frc_gl : force loss or gain due to acceleration / decceleration 
% h_mb : hieght of the battery mass from the groung 
% h_mw : mass of the deadweight 
% h_pivot : hieght of the pivot 

% calculating the frc_gl pivoting around drive wheel 

% theta : inclination of the machine wrt ground truth 

% N_drive =[(l_f*M_b*g)-(M_b*g*cos(theta)*l_f)-(M_b*g*sin(theta)*h_mb)-(M_w*g*sin(theta)*h_mw)+(frc_gl*h_pivot)]/(l_f+l_b)
% N_rcacc =[(l_f*M_b*cos(theta)*l_f)+(M_b*g*sin(theta)*h_mb)+(frc_gl*h_pivot)+(M_w*g*sin(theta)*h_mw)]/(l_f+l_b)

%--------------------------------------------------------------------------

%--------------------------------------------------------------------------

% traction modelling 

% traction available at the drive wheel
% f_drive = N_drive * COF_drv_dyn {0.3 -> 0.6 for COF_drv_dyn}  

% -------------------------------------------------------------------------

% -------------------------------------------------------------------------

% friction force modelling 
% F_rc = [N_drive*COF_drv_dyn + N_rcacc*COF_cstr_dyn]  {0.3 -> 0.6 for COF_drv/cstr_dyn}

% -------------------------------------------------------------------------

% -------------------------------------------------------------------------

% force modelling 
% F_r  = [F_rc+F_ac]

%--------------------------------------------------------------------------
% ----------------------controls law modelling-----------------------------
%--------------------------------------------------------------------------

















