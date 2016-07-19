function demo_gmm()

% a row is a vector of coordinates

UPPER = csvread('ort_data_u');
LOWER = csvread('ort_data_l');

[Priors2_u, Mu2_u, Sigma2_u, expData_u, expSigma_u] = GMM_myo(UPPER');
[Priors2_l, Mu2_l, Sigma2_l, expData_l, expSigma_l] = GMM_myo(LOWER');

csvwrite('../data/demo_u.dat', (expData_u(2:end, :))');
csvwrite('../data/demo_l.dat', (expData_l(2:end, :))');

UPPER = csvread('emg_data_u');
LOWER = csvread('emg_data_l');

[Priors2_u, Mu2_u, Sigma2_u, expData_u, expSigma_u] = GMM_myo(UPPER');
[Priors2_l, Mu2_l, Sigma2_l, expData_l, expSigma_l] = GMM_myo(LOWER');

csvwrite('../data/emg_u.dat', (expData_u(2:end, :))');
csvwrite('../data/emg_l.dat', (expData_l(2:end, :))');

