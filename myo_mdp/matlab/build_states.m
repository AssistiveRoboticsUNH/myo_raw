%function build_states()

N_STATES = 4;
N_ACTIONS = 4;

[data, emg, imu] = preprocess('../../data/work/0/');

[idx_emg, C_emg] = kmeans(emg, N_STATES);
[idx_imu, C_imu] = kmeans(imu, N_ACTIONS);

csvwrite('../data/emg.dat', [emg, idx_emg]);
csvwrite('../data/imu.dat', [imu, idx_imu]);

csvwrite('../data/emg_labels.dat', idx_emg);
csvwrite('../data/imu_labels.dat', idx_imu);

%emgKnnmdl = fitcknn(emg,idx_emg);
%imuKnnmdl = fitcknn(imu,idx_imu);

state_centers = knnsearch(imu, C_imu);
state_centers_emg = emg(state_centers,:);
state_emg_amp = sqrt(sum(state_centers_emg'.^2));
csvwrite('../data/state_power.dat',[[1:N_STATES]', state_emg_amp']);

