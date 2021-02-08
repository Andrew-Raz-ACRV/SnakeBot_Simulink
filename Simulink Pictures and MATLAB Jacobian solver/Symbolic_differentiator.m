function Symbolic_differentiator()

clc
syms q1; syms q2; syms q3; syms q6; syms r;
r_ = 173.58;

%initial J at initial Q's
q1_in = 50; q2_in = 50; q3_in = 50; q6_in = 0;
display('Started at X, Y, Z and pitch of:');
X_in = (q3_in*sin(q2_in/r_) + r_*(1-cos(q2_in/r_)))*cos(q6_in)
Y_in = (q3_in*sin(q2_in/r_) + r_*(1-cos(q2_in/r_)))*sin(q6_in)
Z_in = q3_in*cos(q2_in/r_) + r_*sin(q2_in/r_) + q1_in
P_in = atan(sin(q6_in)*tan(q2_in/r_))

%INPUT FROM HAND:
display('rate of change goal: ');
dX_aim = [10; -15; 80; -0.1]

%TARGET DESTINATION:
X_aim = X_in + dX_aim(1);
Y_aim = Y_in + dX_aim(2);
Z_aim = Z_in + dX_aim(3);
P_aim = P_in + dX_aim(4);

%BUILD THE JACOBIAN:
Q = [q1; q2; q3; q6];
%X = f(Q);
F1 = f1(Q); F2 = f2(Q); F3 = f3(Q); F4 = f4(Q);
F = [F1; F2; F3; F4];
display('Jacobian symbolically is:');
J = jacobian(F,Q) %Display symbolic Jacobian

%initialise rate of change
dX = [0;0;0;0];
dQ = [0;0;0;0];
X_current = X_in; Y_current = Y_in; Z_current = Z_in; P_current = P_in; 
Q1_current = q1_in; Q2_current = q2_in; Q3_current = q3_in; Q6_current = q6_in; 
%interatively get to goal:
    for i=1:50
        %Get current Position / orientation:
        X_current = (Q3_current*sin(Q2_current/r_) + r_*(1-cos(Q2_current/r_)))*cos(Q6_current);
        Y_current = (Q3_current*sin(Q2_current/r_) + r_*(1-cos(Q2_current/r_)))*sin(Q6_current);
        Z_current = Q3_current*cos(Q2_current/r_) + r_*sin(Q2_current/r_) + Q1_current;
        P_current = atan(sin(Q6_current)*tan(Q2_current/r_)); 
            %Get Current numerical Jacobian:
        J = vpa(subs(J,[q1;q2;q3;q6;r],[Q1_current;Q2_current;Q3_current;Q6_current;r_]));
            %Solve current rate of change aim:
        dX(1) = X_aim - X_current; dX(2) = Y_aim - Y_current; dX(3) = Z_aim - Z_current; dX(4) = P_aim - P_current; 
            %Solve instantaneous motor values
        dQ = pinv(J)*dX;
            %Solve new current motor position
        Q1_current = dQ(1) + Q1_current;
        Q2_current = dQ(2) + Q2_current;
        Q3_current = dQ(3) + Q3_current;
        Q6_current = dQ(4) + Q6_current;
    end

%Grab final results
q1_final = Q1_current;
q2_final = Q2_current;
q3_final = Q3_current;
q6_final = Q6_current;

X_final = (q3_final*sin(q2_final/r_) + r_*(1-cos(q2_final/r_)))*cos(q6_final);
Y_final = (q3_final*sin(q2_final/r_) + r_*(1-cos(q2_final/r_)))*sin(q6_final);
Z_final = q3_final*cos(q2_final/r_) + r_*sin(q2_final/r_) + q1_final;
P_final = atan(sin(q6_final)*tan(q2_final/r_));

display('The resultant displacement is');
dX_actual = round(X_final - X_in) 
dY_actual = round(Y_final - Y_in) 
dZ_actual = round(Z_final - Z_in) 
dP_actual = P_final - P_in

end

function [X] = f(Q)
syms r;
X = zeros(4,1);

q1 = Q(1); q2 = Q(2); q3 = Q(3); q6 = Q(4);

X(1) = (q3*sin(q2/r) + r*(1-cos(q2/r)))*cos(q6);
X(2) = (q3*sin(q2/r) + r*(1-cos(q2/r)))*sin(q6);
X(3) = q3*cos(q2/r) + r*sin(q2/r) + q1;
X(4) = atan(sin(q6)*tan(q2/r));

end

function [X] = f1(Q)
syms r;
q1 = Q(1); q2 = Q(2); q3 = Q(3); q6 = Q(4);
X = (q3*sin(q2/r) + r*(1-cos(q2/r)))*cos(q6);

end

function [Y] = f2(Q)
syms r;
q1 = Q(1); q2 = Q(2); q3 = Q(3); q6 = Q(4);
Y = (q3*sin(q2/r) + r*(1-cos(q2/r)))*sin(q6);

end

function [Z] = f3(Q)
syms r;
q1 = Q(1); q2 = Q(2); q3 = Q(3); q6 = Q(4);
Z = q3*cos(q2/r) + r*sin(q2/r) + q1;

end

function [P] = f4(Q)
syms r;
q1 = Q(1); q2 = Q(2); q3 = Q(3); q6 = Q(4);
P = atan(sin(q6)*tan(q2/r));

end
