%% Initialize twolink model
% The ros connection is shutted down intially and then 
% started again

rosshutdown
clear all
close all
clc
rosinit

twolink(1).joint = 'joint_1';
twolink(1).control_input_topic = '/twolink/joint_1_effort_controller/command';
twolink(1).state_topic = '/twolink/joint_states';

twolink(2).joint = 'joint_2';
twolink(2).control_input_topic = '/twolink/joint_2_position_controller/command';

%% Set solver settings and save them to mat file

solversettings.t_start = 0.0;
solversettings.t_stop = 2.0;
solversettings.t_sample = 0.0005;
solversettings.rel_tol = 1e-7;
solversettings.abs_tol = 1e-7;

save('settings.mat','solversettings')

%%

load('settings.mat')
open('twolink_simulink_analysis')
sim('twolink_simulink_analysis')

%% Generate data
u=input.Data;
y=output.Data;

system=iddata(y,u,Ts);
system_normalized=detrend(system);

w = linspace(0.001,1000,2048);
sys_np=spa(system,[],w);

bode(sys_np)