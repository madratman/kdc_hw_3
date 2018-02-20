clc
close all
clear all
syms t1 t2
disp('Leg R');
A = 0.5*[sin(t1)+sin(t1+t2); cos(t1) + cos(t1+t2)];
eqn = A == [ - 0.2;0.8];
answer = solve(eqn);
theta(1) = eval(answer.t1(2)); % select angle that matches the diagram
theta(2) = eval(answer.t2(2)); % select angle that matches the diagram
disp(eval(answer.t1));
disp(eval(answer.t2));

disp('Leg L');
A = 0.5*[sin(t1)+sin(t1+t2); cos(t1) + cos(t1+t2)];
eqn = A == [0.2;0.8];
answer = solve(eqn);
theta(3) = eval(answer.t1(2)); % select angle that matches the diagram
theta(4) = eval(answer.t2(2)); % select angle that matches the diagram
disp(eval(answer.t1));
disp(eval(answer.t2));