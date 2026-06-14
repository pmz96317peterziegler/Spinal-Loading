% Project 1 - Using IK Solutions
% Names: Nick Linkowski, Ben Brown, Peter Ziegler, Aidan Jones, Brady
% Stein, and Bara Mbaye

%position global variables
close all
clear variables

H = 1.6; %height (m)
M = 60; %mass (kg)
mo = 5; %object mass (kg)
g = 9.81; %gravity (m/s/s)


x_target = 0.2;  %(m)
y_target = 0.1; % (m)
angles_matrix = Compute_IK_Solutions(H, x_target, y_target);

% Storage vectors
tpn = []; %storage for local position angles 
F_muscle_spine = []; %storage vector for spinal forces for each position
tg = []; %global knee angles
MFS = []; %storage for local stability forces for each position
MSHV = []; %storage for local shear forces for each position
ERR = []; %error storage

%Anthropometric data calculations 
lfoot = 0.152*H*0.8;
hfoot = 0.039*H;
mfoot = 0.0145*M*2;
cgfoot_L = 0.5*lfoot;
cgfoot_h = 0.5*hfoot;

lleg = (0.285-0.039)*H;
mleg = 0.0465*M*2;
cgleg = 0.567*lleg;

 lthigh = (0.48-0.285)*H;
 mthigh = 0.1*M*2;
 cgthigh = 0.567*lthigh;

 lthn = (0.818-0.48)*H;
 mthn = 0.578*M;
 cgthn = 0.66*lthn;

 lua = 0.186*H;
 mua = 0.028*M*2;
 cgua = 0.436*lua;

 lfah = (0.145+0.108)*H;
 mfah = (0.016+0.006)*M*2;
 cgfah = (0.43*0.145*H*0.016*M+(0.145*H+0.506*0.108*H)*0.006*M)/(mfah*0.5);

 L_center_hand_dist = (0.145+0.108/2)*H;
% Loop through all positions
for j = 1:size(angles_matrix,1)  % Loop through all rows (positions)
    % Extract angles for this position
    for i = 1:6
        tpn(i) = angles_matrix(j,i);
    end
    
    angle_1 = tpn(1);
    angle_2 = tpn(2);
    angle_3 = tpn(3);
    angle_4 = tpn(4);
    angle_5 = tpn(5);
    angle_6 = tpn(6);

    Rground = [cosd(angle_1) -sind(angle_1) 0 0;
        sind(angle_1) cosd(angle_1) 0 0;
        0 0 1 0;
        0 0 0 1]; 

    Rankle = [cosd(angle_2) -sind(angle_2) 0 0;
        sind(angle_2) cosd(angle_2) 0 0;
        0 0 1 0;
        0 0 0 1];

    Rknee = [cosd(angle_3) -sind(angle_3) 0 0;
        sind(angle_3) cosd(angle_3) 0 0;
        0 0 1 0;
        0 0 0 1];

    Rhip = [cosd(angle_4) -sind(angle_4) 0 0;
        sind(angle_4) cosd(angle_4) 0 0;
        0 0 1 0;
        0 0 0 1];

    Rshoulder = [cosd(angle_5) -sind(angle_5) 0 0;
        sind(angle_5) cosd(angle_5) 0 0;
        0 0 1 0;
        0 0 0 1];

    Relbow = [cosd(angle_6) -sind(angle_6) 0 0;
        sind(angle_6) cosd(angle_6) 0 0;
        0 0 1 0;
        0 0 0 1];

    Lfoot = [1 0 0 lfoot;
        0 1 0 -hfoot;
        0 0 1 0;
        0 0 0 1];

    Lfoot_cg = [1 0 0 cgfoot_L;
        0 1 0 -cgfoot_h;
        0 0 1 0;
        0 0 0 1];

    Lleg = [1 0 0 lleg;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Lleg_cg = [1 0 0 cgleg;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Lthigh = [1 0 0 lthigh;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Lthigh_cg = [1 0 0 cgthigh;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Ldistal = [1 0 0 lthn*0.75;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];
    %out of all the transformation matrices the above and below are 
    %are the largest assumptions/largest contributors.
    %the above communicates that the lower spinal muscle connects
    %up the spine at a 75% the length of the trunk head and neck
    %==============
    %the lower one communicates that the attachment point of the muscle
    %to the hips looks like this

    %the trunk head and neck
    %   |
    %   |
    %   |
    %   |
    %   |
    %   |           x
    %   |           |
    %   |     .     ----y
    %the above is the spine and the period is the point of contact for the
    %muscle and it is 5% the length of the trunk head and neck
    Lproximal = [1 0 0 0;
        0 1 0 0.05*lthn;
        0 0 1 0;
        0 0 0 1];

    Lthn = [1 0 0 lthn;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Lthn_cg = [1 0 0 cgthn;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Lua = [1 0 0 lua;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Lua_cg = [1 0 0 cgua;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Lfah = [1 0 0 lfah;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    Lfah_cg = [1 0 0 cgfah;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

    L_center_hand_T = [1 0 0 L_center_hand_dist;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];


    %these still result in the 4x4 transformation matrix
    Tankle = Rground*Lfoot;
    Tknee = Tankle*Rankle*Lleg;
    Thip = Tknee*Rknee*Lthigh;
    Tproximal = Thip*Rhip*Lproximal;
    Tdistal = Thip*Rhip*Ldistal;
    Tshoulder = Thip*Rhip*Lthn;
    Telbow = Tshoulder*Rshoulder*Lua;
    Tfingers = Telbow*Relbow*Lfah;
    Tobject = Telbow*Relbow*L_center_hand_T;

    Tfoot_cg = Rground*Lfoot_cg;
    Tleg_cg = Tankle*Rankle*Lleg_cg;
    Tthigh_cg = Tknee*Rknee*Lthigh_cg;
    Tthn_cg = Thip*Rhip*Lthn_cg;
    Tua_cg = Tshoulder*Rshoulder*Lua_cg;
    Tfah_cg = Telbow*Relbow*Lfah_cg;
    Tobject_cg = Tobject;

    Merr = (mfoot+mleg+mthigh+mthn+mua+mfah)-M;
    xcg = (Tfoot_cg(1,4)*mfoot+Tleg_cg(1,4)*mleg+Tthigh_cg(1,4)*mthigh+Tthn_cg(1,4)*mthn+Tua_cg(1,4)*mua+Tfah_cg(1,4)*mfah+Tobject_cg(1,4)*mo)/(M+mo);
    xcg_withoutobject = (Tfoot_cg(1,4)*mfoot+Tleg_cg(1,4)*mleg+Tthigh_cg(1,4)*mthigh+Tthn_cg(1,4)*mthn+Tua_cg(1,4)*mua+Tfah_cg(1,4)*mfah)/M;
    ycg = (Tfoot_cg(2,4)*mfoot+Tleg_cg(2,4)*mleg+Tthigh_cg(2,4)*mthigh+Tthn_cg(2,4)*mthn+Tua_cg(2,4)*mua+Tfah_cg(2,4)*mfah+Tobject_cg(2,4)*mo)/(M+mo);
    yfingers = Tfingers(2,4);
    
    %example:  T_center_hand = [1 0 0 L_center_hand;
    %    0 1 0 0;
    %    0 0 1 0;
    %    0 0 0 1];
    %if we wanted r_center_hand
    %we would pull the 1st row on the 4th column for the x value
    %and the 4th columns 2nd row for the y value.
    rFN = [xcg 0 0];
    rfoot = [Tfoot_cg(1,4) Tfoot_cg(2,4) 0];
    rleg = [Tleg_cg(1,4) Tleg_cg(2,4) 0];
    rthigh = [Tthigh_cg(1,4) Tthigh_cg(2,4) 0];
    rjoint = [Thip(1,4) Thip(2,4) 0];
    %the below variable is the global position of the attachment point for
    %the spinal muscle
    r_spine_muscle = [Tproximal(1,4) Tproximal(2,4) 0];%
    %this section sets up the unit vector to be pointing from the
    %attachment point at the hips up to the attachment point at the
    %shoulders
    unit_vec_muscle = [Tdistal(1,4)-Tproximal(1,4),Tdistal(2,4)-Tproximal(2,4),0];
    unit_vec_muscle = unit_vec_muscle/norm(unit_vec_muscle);
    
    % this calculates the global moment that the spinal muscle creates
    % around the origin. This works because all of the other moments
    % created from weight etc are calculated as global moments instead of
    % moments at certain joints.
    muscle_m = cross(r_spine_muscle,unit_vec_muscle);
    FN = [0 (M+mo)*g 0];
    Wfoot = [0 -mfoot*g 0];
    Wleg = [0 -mleg*g 0];
    Wthigh = [0 -mthigh*g 0];

    total_weight_m = cross(rfoot,Wfoot)+cross(rleg,Wleg)+cross(rthigh,Wthigh)+cross(rFN,FN);

    equilibrium_matrix = [1 0 unit_vec_muscle(1);
           0 1 unit_vec_muscle(2);
        -rjoint(2) rjoint(1) muscle_m(3)];

    Net_force_vector = [0; mfoot*g+mleg*g+mthigh*g-(M+mo)*g; -total_weight_m(3)];


    % original form of Ax=b -> x = A_inv*b

    Final_solution = inv(equilibrium_matrix)*Net_force_vector;

    F_hip = [Final_solution(1), Final_solution(2), 0];
    u_spine = [Tshoulder(1,4)-Thip(1,4), Tshoulder(2,4)-Thip(2,4), 0];
    u_spine = u_spine/norm(u_spine);
    %this calculates the compression force generated by the spinal muscle
    %(basically the part of the force of the hip that aligns with the x
    %component of the spines axis.
    magF_stab = dot(F_hip,u_spine);
    %these M..... variables store the individual values so that they can
    %then be plotted once the loop has been completed.
    MFS(j) = magF_stab;
    F_stab = magF_stab*u_spine;
    %the part of the hips force that aligns with the y component of the
    %spines axis.
    perp = [-u_spine(2), u_spine(1), 0];
    magF_shear = dot(F_hip, perp); 
    MSHV(j) = magF_shear;
    Ferr = norm(F_hip) - sqrt((magF_shear^2)+(magF_stab^2));
    ERR(j) = Ferr;
    F_muscle_spine(j) = Final_solution(3);
    tg(j) = angle_3;

    if (rFN(1) <= -lfoot/0.8)
        fprintf('Person in position %d is falling backwards \n',j)
    elseif (rFN(1) > 0)
        fprintf('Person in position %d is falling forwards \n',j)
    else
        fprintf('Person in position %d is stable \n',j)
    end
    
    if mod(j, 4) == 0 || j == 1
        figure(j)
        hold on
        %this section plots the links of every 4th calculated position
        title(sprintf('Position %d', j))
        plot([0 Tankle(1,4)],[0 Tankle(2,4)],'-b')
        plot([Tankle(1,4) Tknee(1,4)],[Tankle(2,4) Tknee(2,4)],'-b')
        plot([Tknee(1,4) Thip(1,4)],[Tknee(2,4) Thip(2,4)],'-b')
        plot([Thip(1,4) Tshoulder(1,4)],[Thip(2,4) Tshoulder(2,4)],'-b')
        plot([Tshoulder(1,4) Telbow(1,4)],[Tshoulder(2,4) Telbow(2,4)],'-b')
        plot([Telbow(1,4) Tfingers(1,4)],[Telbow(2,4) Tfingers(2,4)],'-b')
        plot([Tdistal(1,4) Tproximal(1,4)],[Tdistal(2,4) Tproximal(2,4)],'-r')
    
        %this section plots all of the centers of masses
        scatter(Tfoot_cg(1,4),Tfoot_cg(2,4),'*r')
        scatter(Tleg_cg(1,4),Tleg_cg(2,4),'*r')
        scatter(Tthigh_cg(1,4),Tthigh_cg(2,4),'*r')
        scatter(Tthn_cg(1,4),Tthn_cg(2,4),'*r')
        scatter(Tua_cg(1,4),Tua_cg(2,4),'*r')
        scatter(Tfah_cg(1,4),Tfah_cg(2,4),'*r')
        scatter(Tobject_cg(1,4),Tobject_cg(2,4),'*r')
        scatter(xcg,ycg,'*k')
        
        %no squishing or stretching
        %consistent viewing for the figures
        daspect([1 1 1])
        xlim([-0.6, 0.6])
    end
end  

% Plot summary graphs after all positions are processed
figure(j+1)
hold on
plot(tg, F_muscle_spine,'-g')
xlabel('Knee Angle (degrees)')
ylabel('Spinal Force (N)')
title('Spinal Forces vs Knee Angle')

figure(j+2)
hold on 
scatter(tg,MSHV,'*','b')
xlabel('Knee Angle (degrees)')
ylabel('Force (N)')
title('Shear Forces vs Knee Angle')

figure(j+3)
hold on 
scatter(tg,MFS,'*','r')
xlabel('Knee Angle (degrees)')
ylabel('Force (N)')
title('Stability Force vs. Knee Angle')
