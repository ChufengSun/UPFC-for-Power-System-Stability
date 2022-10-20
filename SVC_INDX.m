function f = svc_indx
% syntax: f = svc_indx

% determines the relationship between svc and nc loads
% checks for svc
% determines number of SVCs
% f is a dummy variable
f = 0;
global svc_con load_con  n_svc  svc_idx svcll_idx
n_svc = 0;
svc_idx = [];
if ~isempty(svc_con)
    [n_svc npar] = size(svc_con);
    svc_idx = zeros(n_svc,1);
    % set defaults for lead lag
    if npar<9
       svc_con(:,8:9) = zeros(n_svc,2);
    end
    svcll_idx = find(svc_con(:,9)~=0);
    for j = 1:n_svc
       index = find(svc_con(j,2)==load_con(:,1));
       if ~isempty(index)
          svc_idx(j) = index;
       else
          error('you must have the svc bus declared as a non-conforming load')
       end
    end
end
       
    