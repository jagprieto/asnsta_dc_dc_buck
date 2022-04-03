clc;
clear all;
rng('default');
warning('off','all');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULATION CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global PARAMETERS; 
PARAMETERS.PROBLEM_TYPE = 0; % 0-> Con carga en arranque; 1 ->Sin carga en arranque
PARAMETERS.NOISE_ACTIVATION = 0; 
if PARAMETERS.PROBLEM_TYPE == 0
    PARAMETERS.TOTAL_TIME = 2.0e-1;
else
    PARAMETERS.TOTAL_TIME = 5.0e-1;    
end
PARAMETERS.V_NOISE = 0.01; % Nivel de ruido de sensor de voltaje
PARAMETERS.I_NOISE = 0.005; % Nivel de ruido de senor de corriente
if PARAMETERS.NOISE_ACTIVATION == 0
    PARAMETERS.NOISE_FILTER_GAIN = 2.0;
    PARAMETERS.V_NOISE = PARAMETERS.V_NOISE*1E-12; % Nivel de ruido de sensor de voltaje
    PARAMETERS.I_NOISE = PARAMETERS.I_NOISE*1E-12; % Nivel de ruido de senor de corriente
else
    PARAMETERS.NOISE_FILTER_GAIN = 4.0;
end
PARAMETERS.SAMPLING_TIME = 1.0e-4; % ZOH time (sampling time for the control system block!!)
PARAMETERS.SETTLING_TIME = 0.1; % Regula el tiempo (s) de establecimiento desde condiciones iniciales.
PARAMETERS.CURRENT_MAX = 2.5;
PARAMETERS.CUT_OFF_FREQUENCY_MAX = PARAMETERS.CURRENT_MAX*pi/(2*PARAMETERS.SETTLING_TIME);
PARAMETERS.CUT_OFF_FREQUENCY_MIN = 0.5*PARAMETERS.CUT_OFF_FREQUENCY_MAX;
PARAMETERS.CUT_OFF_FREQUENCY_DISTURBANCE_V = 1/(PARAMETERS.NOISE_FILTER_GAIN*PARAMETERS.SAMPLING_TIME);
PARAMETERS.CUT_OFF_FREQUENCY_DISTURBANCE_I = PARAMETERS.CUT_OFF_FREQUENCY_DISTURBANCE_V;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PARAMETERS.Vg = 80; % V (source voltage)
PARAMETERS.Vd = 48; % V (output desired voltage)
PARAMETERS.d_Vd = 0.0; % V/s (output desired voltage velocity)
PARAMETERS.dd_Vd = 0.0; % V/s^2 (output desired voltage acceleration)
PARAMETERS.C = 1e-3; % F
PARAMETERS.R = 50; % Ohm
PARAMETERS.L = 5e-3; % H
PARAMETERS.SWITCHING_FREQUENCY = 1.0e4; 
PARAMETERS.R2 = 70; % Ohm
PARAMETERS.R3 = 30; % Ohm
PARAMETERS.DETAIL_VIEW_SIZE = 200;
PARAMETERS


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RUN SIMULATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simulink_model = 'dc_buck_converter.slx';

% Run Gonzalez-Prieto control.
PARAMETERS.CONTROLLER_TYPE = 1; % 1-> Gonzalez/Doval<; 0->Zang/Li
PARAMETERS.VERSION = 1;
sim(simulink_model,PARAMETERS.TOTAL_TIME); 

times_glez = simulation_data.time;
v_glez = simulation_data.signals(1).values; 
i_glez = simulation_data.signals(2).values; 
u_glez = simulation_data.signals(3).values; 
cut_off_frequency = simulation_data.signals(12).values;

% Run Wang/Li control.
PARAMETERS.CONTROLLER_TYPE = 0; 
sim(simulink_model,PARAMETERS.TOTAL_TIME); 
times_li = simulation_data.time;
v_li = simulation_data.signals(1).values; 
i_li = simulation_data.signals(2).values; 
u_li = simulation_data.signals(3).values; 
switch_li = simulation_data.signals(4).values; 


% Plot the results
time_range = 70;
if (PARAMETERS.PROBLEM_TYPE  == 0)
    time_detail = 0.1;
else
    time_detail = 0.3;
end

legend_texts = {'Wang/Li', 'Gonzalez-Prieto'};
plot_font_size = 10;
legend_font_size = 12;

% Voltage with steady-state detailed view and current
figure(1);
clf(1);
subplot(3,1,1);
plot(times_li, v_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, v_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('v(t) [A]', 'FontSize', plot_font_size);
title('Voltage ','FontSize', plot_font_size);
legend(legend_texts,'FontSize', legend_font_size);

subplot(3,1,2);
plot(times_li, v_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, v_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('v(t) [A]', 'FontSize', plot_font_size);
title('Voltage detail at steady-state','FontSize', plot_font_size);
xlim([PARAMETERS.TOTAL_TIME - 20*PARAMETERS.SAMPLING_TIME, PARAMETERS.TOTAL_TIME]);

subplot(3,1,3);
plot(times_li, i_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, i_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('i(t) [A]', 'FontSize', plot_font_size);
title('Current ','FontSize', plot_font_size);

% Duty cycle and cut-off over frequency
figure(2);
clf(2);
subplot(2,1,1);
plot(times_li, u_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, u_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('u(t)', 'FontSize', plot_font_size);
title('Duty cycle ','FontSize', plot_font_size);
legend(legend_texts,'FontSize', legend_font_size);

subplot(2,1,2);
plot(times_glez, cut_off_frequency, '-r', 'LineWidth',1.5);
grid on;
hold on;
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('w_c(t)', 'FontSize', plot_font_size);
title('Adaptive cut-off frequency','FontSize', plot_font_size);
% xlim([PARAMETERS.TOTAL_TIME - 20*PARAMETERS.SAMPLING_TIME, PARAMETERS.TOTAL_TIME]);


% Details
figure(3);
clf(3);
subplot(3,2,1);
plot(times_li, v_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, v_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('v(t) [V]', 'FontSize', plot_font_size);
title('Voltage at startup','FontSize', plot_font_size);
xlim([0, time_range*PARAMETERS.SAMPLING_TIME]);

subplot(3,2,3);
plot(times_li, i_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, i_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('i(t) [A]', 'FontSize', plot_font_size);
title('Current at startup','FontSize', plot_font_size);
xlim([0, time_range*PARAMETERS.SAMPLING_TIME]);

subplot(3,2,5);
plot(times_li, u_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, u_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('u(t)', 'FontSize', plot_font_size);
title('Duty cycle at startup','FontSize', plot_font_size);
xlim([0, time_range*PARAMETERS.SAMPLING_TIME]);
legend(legend_texts,'FontSize', legend_font_size);

subplot(3,2,2);
plot(times_li, v_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, v_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('v(t) [V]', 'FontSize', plot_font_size);
title(strcat('Voltage at t=', num2str(time_detail), ' s'),'FontSize', plot_font_size);
xlim([time_detail - time_range*PARAMETERS.SAMPLING_TIME/2, time_detail + time_range*PARAMETERS.SAMPLING_TIME/2]);

subplot(3,2,4);
plot(times_li, i_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, i_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('i(t) [A]', 'FontSize', plot_font_size);
title(strcat('Current at t=', num2str(time_detail), ' s'),'FontSize', plot_font_size);
xlim([time_detail - time_range*PARAMETERS.SAMPLING_TIME/2, time_detail + time_range*PARAMETERS.SAMPLING_TIME/2]);

subplot(3,2,6);
plot(times_li, u_li, '-b', 'LineWidth',1.0);
grid on;
hold on;
plot(times_glez, u_glez, '-r', 'LineWidth',1.5);
xlabel('Time [s]', 'FontSize', plot_font_size);
ylabel('u(t)', 'FontSize', plot_font_size);
title(strcat('Duty cycle at t=', num2str(time_detail), ' s'),'FontSize', plot_font_size);
xlim([time_detail - time_range*PARAMETERS.SAMPLING_TIME/2, time_detail + time_range*PARAMETERS.SAMPLING_TIME/2]);





% graphics_path = "C:/Users/jagprieto/Desktop/PAPERS/ON PREPARATION/DC CONVERTERS/MANUSCRIPT International Journal of Dynamics and Control/GRAPHICS/";
% 
% figure(1);
% file_name = strcat('voltage_current_seq_', num2str(PARAMETERS.PROBLEM_TYPE), '_noise_', num2str(PARAMETERS.NOISE_ACTIVATION));
% file_name = strcat(graphics_path, file_name, '.pdf')
% export_fig(file_name, '-transparent', '-nocrop');
% 
% figure(2);
% file_name = strcat('duty_cycle_cut_off_frequency_seq_', num2str(PARAMETERS.PROBLEM_TYPE), '_noise_', num2str(PARAMETERS.NOISE_ACTIVATION));
% file_name = strcat(graphics_path, file_name, '.pdf')
% export_fig(file_name, '-transparent', '-nocrop');
% 
% 
% figure(3);
% file_name = strcat('voltage_current_duty_cycle_seq_', num2str(PARAMETERS.PROBLEM_TYPE), '_noise_', num2str(PARAMETERS.NOISE_ACTIVATION));
% file_name = strcat(graphics_path, file_name, '.pdf')
% export_fig(file_name, '-transparent', '-nocrop');

