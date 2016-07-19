function demo(n)

cd('../matlab');
path = '../../data/work/';
UPPER = [];
LOWER = [];
for i=0:n-1
    upper = csvread(strcat(path,int2str(i),'/orientation_u.mat'));
    lower = csvread(strcat(path,int2str(i),'/orientation_l.mat'));
    
    % downsample
    upper = upper(6:5:end-5, :);
    lower = lower(6:5:end-5, :);
    
    N_u = length(upper);
    N_l = length(lower);
    if N_l < N_u
        N = N_l;
        upper = upper(1:N, :);
    elseif N_l > N_u
        N = N_u;
        lower = lower(1:N, :);
    else
        N = N_l;
    end
    
    upper = [ (1:N)', upper];
    UPPER = [UPPER; upper];

    
    lower = [ (1:N)', lower];
    LOWER = [LOWER; lower];  
end

[Priors2_u, Mu2_u, Sigma2_u, expData_u, expSigma_u] = GMM_myo(UPPER');
[Priors2_l, Mu2_l, Sigma2_l, expData_l, expSigma_l] = GMM_myo(LOWER');

csvwrite('../data/demo_u.dat', (expData_u(2:end, :))');
csvwrite('../data/demo_l.dat', (expData_l(2:end, :))');



