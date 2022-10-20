% s_simu.m  

% An m.file to simulate power system transients  
% using the Matlab Power System Toolbox
% This m-file takes the dynamic and load flow data and
% calculates the response of the power system to a fault
% which is specified in a switching file
% see one of the supplied data files (data2a.m) for the 
% switching file format


%
clear
clear global
close % close graphics windows
tic % set timer
plot_now=0;
jay = sqrt(-1);
pst_var % set up global variables 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cont_on_off(1:4)=[1   0    0    0];
  coef_cont(1:4)=[.05 .01  0.1  .1];   
  time_cont(1:4)=[10   10    10   0];     
      cont_(1:4)=[0   0    0    0];
non_linear=1;

% The UPFC PI-controllers gains
KI_Ddc1=6.487179 ;
KP_Ddc1=50.000000; 

KI_Dac1=11.902358;
KP_Dac1=9.411120; 

KI_Dp1= 13.753582; 
KP_Dp1=7.253064;    

KI_Dq1=0.000000;   
KP_Dq1=18.707602;

UPFC1_line_indx= 23;     
%UPFCs  
xE_1=.1;    
xB_1=0.003;
Vc1_(1,1)=1.;
Cdc_1=1;


% input data file
dfile='SdataNE1.m';
lfile =length(dfile);
% strip off .m and convert to lower case
dfile = lower(dfile(1:lfile-2));
eval(dfile);
sys_freq = 50;
basrad = 2*pi*sys_freq; % default system frequency is 60 Hz
basmva =100;
syn_ref = 0 ;     % synchronous reference frame
ibus_con = []; % ignore infinite buses in transient simulation


line(UPFC1_line_indx,5)=0;
Z_1 = line(UPFC1_line_indx,3)+jay*line(UPFC1_line_indx,4);
line(UPFC1_line_indx,4)=line(UPFC1_line_indx,4)+xB_1;
fromBus1=line(UPFC1_line_indx,1);
toBus2=line(UPFC1_line_indx,2);


% solve for loadflow - loadflow parameter

      [bus_sol,line,line_flw] = ...
         loadflow(bus,line,1e-9,30,1.0,'n',2);
     
      bus = bus_sol;  % solved loadflow solution needed for
      % initialization
    
nline=length(line(:,1));

%set machine, exciter, governor and pss indexes
% note: dc index set in dc load flow
f=mac_indx;
f=exc_indx;
f = svc_indx;

[n_mot,dummy] = size(ind_con);
[n_ig,dummy] = size(igen_con);
if isempty(n_mot); n_mot = 0;end
if isempty(n_ig); n_ig = 0; end
ntot = n_mac+n_mot+n_ig;
ngm = n_mac + n_mot;
n_pm = n_mac;
disp(' ')
disp('Performing simulation.')
%
% construct simulation switching sequence as defined in sw_con
tswitch(1) = sw_con(1,1);
k = 1;
n_switch = length(sw_con(:,1));
k_inc = zeros(n_switch-1,1);
t_switch = zeros(n_switch,1);
for sw_count = 1:n_switch-1
   h(sw_count) = sw_con(sw_count,7);
   if h(sw_count)==0, h(sw_count) = 0.01;end
   k_inc(sw_count) = fix((sw_con(sw_count+1,1)-sw_con(sw_count,1))/h(sw_count));
   if k_inc(sw_count)==0;k_inc(sw_count)=1;end
   h(sw_count) = (sw_con(sw_count+1,1)-sw_con(sw_count,1))/k_inc(sw_count);
   t_switch(sw_count+1) =t_switch(sw_count) +  k_inc(sw_count)*h(sw_count);
   t(k:k-1+k_inc(sw_count)) = t_switch(sw_count):h(sw_count):t_switch(sw_count+1)-h(sw_count);
   k=k+k_inc(sw_count);
end
%
k = sum(k_inc)+1; % k is the total number of time steps in the simulation
t(k) = sw_con(n_switch,1);
[n dummy]=size(mac_con) ;
n_bus = length(bus(:,1));
%
% create zero matrices for variables to make algorithm more efficient?
z = zeros(n,k);
z1 = zeros(1,k);
zm = zeros(1,k);if n_mot>1;zm = zeros(n_mot,k);end
zig = zeros(1,k);if n_ig>1;zig = zeros(n_ig,k);end
zdc = zeros(2,k);if n_conv>2; zdc = zeros(n_conv,k);end
zdcl = zeros(1,k);if n_dcl>1;zdcl=zeros(n_dcl,k);end
% set dc parameters  
Vdc = zdc;
i_dc = zdc;  
P_dc = zdc; dc_sig = zdc; cur_ord = zdc;
alpha = zdcl; 
gamma = zdcl;  
dc_sig = zdc;
i_dcr = zdcl; i_dci = zdcl; v_dcc = zdcl;
di_dcr = zdcl; di_dci = zdcl; dv_dcc = zdcl;
v_conr = zdcl; v_coni = zdcl; dv_conr = zdcl; dv_coni = zdcl;

v_p = z1;
theta = zeros(n_bus+1,k);bus_v = zeros(n_bus+1,k);
mac_ang = z; mac_spd = z; dmac_ang = z; dmac_spd = z;
pmech = z; pelect = z; mac_ref = z1;  sys_ref = z1; 
edprime = z; eqprime = z; dedprime = z; deqprime = z;
psikd = z; psikq = z; dpsikd = z; dpsikq = z;
pm_sig = z;
z_tg = zeros(1,k);if n_tg~=0;z_tg = zeros(n_tg,k);end
tg1 = z_tg; tg2 = z_tg; tg3 = z_tg; 
dtg1 = z_tg; dtg2 = z_tg; dtg3 = z_tg;
z_pss = zeros(1,k);if n_pss~=0;z_pss = zeros(n_pss,k);end
pss1 = z_pss; pss2 = z_pss; pss3 = z_pss;
dpss1 = z_pss; dpss2 = z_pss; dpss3 = z_pss;
curd = z; curq = z; curdg = z; curqg = z; fldcur = z;
ed = z; eq = z; eterm = z; qelect = z;
vex = z; cur_re = z; cur_im = z; psi_re = z; psi_im = z;
ze = zeros(1,k);if n_exc~=0; ze = zeros(n_exc,k);end
V_B = ze;exc_sig = ze;
V_TR = ze; V_R = ze; V_A = ze; V_As = ze; Efd = ze; R_f = ze;
dV_TR = ze; dV_R = ze; dV_As = ze; dEfd = ze; dR_f = ze;
pss_out = ze;
vdp = zm; vqp = zm; slip = zm; 
dvdp = zm; dvqp = zm; dslip = zm;
s_mot = zm; p_mot = zm; q_mot = zm;
vdpig = zig; vqpig = zig; slig = zig; 
dvdpig = zig; dvqpig = zig; dslig = zig;
s_igen = zig; pig = zig; qig = zig; tmig = zig;
if n_svc~=0
   B_cv = zeros(n_svc,k); dB_cv = zeros(n_svc,k);svc_sig = zeros(n_svc,k);
   B_con = zeros(n_svc,k); dB_con=zeros(n_svc,k);
   %%%%%%%%%%%%%%%%%%%%%%%%
   V_svc_ref = zeros(n_svc,k);
   global V_svc
   V_svc =zeros(n_svc,k);
else
   B_cv = zeros(1,k);dB_cv = zeros(1,k); svc_sig = zeros(1,k);
   B_con = zeros(1,k);dB_con=zeros(1,k);
end
if n_lmod ~= 0
   lmod_st = zeros(n_lmod,k); dlmod_st = lmod_st; lmod_sig = lmod_st;
else
   lmod_st = zeros(1,k); dlmod_st = lmod_st; lmod_sig = lmod_st;
end
if n_rlmod ~= 0
   rlmod_st = zeros(n_rlmod,k); drlmod_st = rlmod_st; rlmod_sig = rlmod_st;
else
   rlmod_st = zeros(1,k); drlmod_st = rlmod_st; rlmod_sig = rlmod_st;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UPFC1 state variables 
Vc1_=z1;dVc1_=z1;Vc1_ref=z1;D_dc1=z1;dD_dc1=z1;H_dc1=z1; 
VE1_=z1;dVE1_=z1;DeltaE1_=z1;dDeltaE1_=z1;
VB1_=z1;dVB1_=z1;DeltaB1_=z1;dDeltaB1_=z1;
% UPFC1 variables , set-points, and PI-controller outputs
VEt1_=z1; VEt1_ref=z1; D_ac1=z1;dD_ac1=z1;H_ac1=z1; VEt1_abs=z1;
VBt1_=z1; VBB1_=z1;
%
mE1_=z1;
DeltaEp1_=z1;
DeltaBp1_=z1;
mB1_=z1;

p1_=z1;p1_ref=z1;D_p1=z1;dD_p1=z1;H_p1=z1;
q1_=z1;q1_ref=z1;D_q1=z1;dD_q1=z1;H_q1=z1;
VE1_=zeros(1,k);VB1_=zeros(1,k);VBt1_=zeros(1,k);
I1E2_=zeros(2,k);IE_1=zeros(1,k);PEin1_ =z1;
mE1_=z1;mB1_=z1;DeltaE1_=z1;DeltaB1_=z1;PBout1_=z1;



sys_freq = ones(1,k);
disp('constructing reduced y matrices')
% step 1: construct reduced Y matrices 

disp('initializing motor,induction generator, svc and dc control models')       
bus = svc(0,1,bus,0);%initialize svc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if zou ==1 | zou ==2
V_svc_ref(:,1) = svc_pot(:,4);
V_svc(:,1) = svc_pot(:,4);
end

%=======|
aux_file
%=======|

% initialization alters the bus matrix and dc parameters are required

y_switch % calculates the reduced y matrices for the different switching conditions
disp('initializing other models')
% step 2: initialization
theta(1:n_bus,1) = bus(:,3)*pi/180;
bus_v(1:n_bus,1) = bus(:,2).*exp(jay*theta(1:n_bus,1));

flag = 0;
bus_int = bus_intprf;% pre-fault system
disp('generators')
f = mac_tra(0,1,bus,flag);
disp('generator controls')
f = smpexc(0,1,bus,flag);
% initialize load modulation control
if ~isempty(lmod_con)
   disp('load modulation')
   f = lmod(0,1,bus,flag);
end
if ~isempty(rlmod_con)
   disp('reactive load modulation')
   f = rlmod(0,1,bus,flag);
end

% initialize non-linear loads
if ~isempty(load_con)
   disp('non-linear loads')
   vnc = nc_load(bus,flag,Y_ncprf,Y_ncgprf);
else
   nload = 0;
end
if ~isempty(dcsp_con)
   bus_sim = bus;
   bus_int = bus_intprf;
   Y1 = Y_gprf;
   Y2 = Y_gncprf;
   Y3 = Y_ncgprf;
   Y4 = Y_ncprf;
   Vr1 = V_rgprf; 
   Vr2 = V_rncprf;
   bo = boprf;
   h_sol = i_simu(1,1,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo);
   % reinitialize dc controls
   f = dc_cont(0,1,bus,flag);
   % initialize dc line
   f = dc_line(0,1,bus,flag);  
end
H_sum = sum(mac_con(:,16)./mac_pot(:,1));
% step 3: perform a predictor-corrector integration 
kt = 0;
ks = 1;
k_tot = sum(k_inc);
lswitch = length(k_inc);
ktmax = k_tot-k_inc(lswitch);
bus_sim = bus;
plot_now = 0;
while (kt<=ktmax)
   k_start = kt+1;
   if kt==ktmax
      k_end = kt + k_inc(ks);
   else
      k_end = kt + k_inc(ks) + 1;
   end
   for k = k_start:k_end
      % step 3a: network solution
      % mach_ref(k) = mac_ang(syn_ref,k);
      mach_ref(k) = 0;
      pmech(:,k+1) = pmech(:,k);
      tmig(:,k+1) = tmig(:,k);
      
      if n_conv~=0;cur_ord(:,k+1) = cur_ord(:,k);end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      if k==time_cont(1)/h(1) & cont_(1)==1
            Vc1_ref(1,k)=Vc1_ref(1,k)+coef_cont(1);

      elseif k==time_cont(2)/h(1) & cont_(2)==1  
         VEt1_ref(1,k)=VEt1_ref(1,k)+coef_cont(2);

      elseif k==time_cont(3)/h(1) & cont_(3)==1  
            p1_ref(1,k)=p1_ref(1,k)+coef_cont(3);

       elseif k==time_cont(4)/h(1) & cont_(4)==1  
             q1_ref(1,k)=q1_ref(1,k)+coef_cont(4);
      end 
      if zou==1 | zou==2
          V_svc_ref(:,k+1)=V_svc_ref(:,k);    
      end
      Vc1_ref(1,k+1)=Vc1_ref(1,k);
      VEt1_ref(1,k+1)=VEt1_ref(1,k);         
      p1_ref(1,k+1)=p1_ref(1,k);
      q1_ref(1,k+1)=q1_ref(1,k);

      
      
      
      flag = 1;
      timestep = int2str(k);
      % network-machine interface    
      f = mac_tra(0,k,bus_sim,flag);
      % Calculate current injections and bus voltages and angles
      if k >= sum(k_inc(1:3))+1
         % fault cleared
         bus_sim = bus_pf2;
         bus_int = bus_intpf2;
         Y1 = Y_gpf2;
         Y2 = Y_gncpf2;
         Y3 = Y_ncgpf2;
         Y4 = Y_ncpf2;
         Vr1 = V_rgpf2; 
         Vr2 = V_rncpf2;
         bo = bopf2;
         Y_upfc = Y_upfcpf2;
         V_upfc = V_upfcpf2;
         h_sol = i_simu(k,ks,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo,Y_upfc,V_upfc);     
      elseif k >=sum(k_inc(1:2))+1
         % near bus cleared
         bus_sim = bus_pf1;
         bus_int = bus_intpf1;
         Y1 = Y_gpf1;
         Y2 = Y_gncpf1;
         Y3 = Y_ncgpf1;
         Y4 = Y_ncpf1;
         Vr1 = V_rgpf1; 
         Vr2 = V_rncpf1;
         bo = bopf1;
         Y_upfc = Y_upfcpf1;
         V_upfc = V_upfcpf1;
         h_sol = i_simu(k,ks,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo,Y_upfc,V_upfc);   
      elseif k>=k_inc(1)+1
         % fault applied
         bus_sim = bus_f;
         bus_int = bus_intf;
         Y1 = Y_gf;
         Y2 = Y_gncf;
         Y3 = Y_ncgf;
         Y4 = Y_ncf;
         Vr1 = V_rgf; 
         Vr2 = V_rncf;
         bo = bof;
         Y_upfc = Y_upfcf;
         V_upfc = V_upfcf;
         h_sol = i_simu(k,ks,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo,Y_upfc,V_upfc);     
      elseif k<k_inc(1)+1
         % pre fault
         bus_sim = bus;
         bus_int = bus_intprf;
         Y1 = Y_gprf;
         Y2 = Y_gncprf;
         Y3 = Y_ncgprf;
         Y4 = Y_ncprf;
         Vr1 = V_rgprf; 
         Vr2 = V_rncprf;
         bo = boprf;
         Y_upfc = Y_upfcprf;
         V_upfc = V_upfcprf;
         h_sol = i_simu(k,ks,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo,Y_upfc,V_upfc);  
      end
      % network interface for control models     
      f = smpexc(0,k,bus_sim,flag);    
      i_plot=k-plot_now;
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      if i_plot == 10
          plot_now = k;
          % plot plot plot plot
          if cont_(1)==1
             plot(t(1:k),Vc1_(1,1:k),'r')
             title('Vc1');
          elseif cont_(2)==1
             plot(t(1:k),abs(VEt1_(1,1:k)),'r')
             title('VEt1');
          elseif cont_(3)==1
             plot(t(1:k),p1_(1,1:k),'r')
             title('P1');
          elseif cont_(4)==1
             plot(t(1:k),q1_(1,1:k),'r')
             title('Q1');
          end
          
          v_p(1:k)=abs(bus_v(bus_idx(1),1:k));
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          % plot the voltage of the faulted bus
         %plot(t(1:k),v_p(1:k),'r')
         %title('Voltage Magnitude at Fault Bus');
         %xlabel('time (s)');

          %xlabel('time (s)');
          %drawnow
          plot(t(1:k), mac_ang(2,1:k) - mac_ang(10,1:k), 'r');
      end
      
      
      % step 3b: compute dynamics and integrate
      flag = 2;
      sys_freq(k) = 1.0;	   
      f = mac_tra(0,k,bus_sim,flag);
      f = smpexc(0,k,bus_sim,flag);     
      if n_svc~=0
          v_svc = abs(bus_v(bus_int(svc_con(:,2)),k));
          f = msvc_sig(t(k),k);
          f = svc(0,k,bus_sim,flag,v_svc);
      end
          
      j = k+1;
      % following statements are predictor steps
      mac_ang(:,j) = mac_ang(:,k) + h_sol*dmac_ang(:,k); 
      mac_spd(:,j) = mac_spd(:,k) + h_sol*dmac_spd(:,k);
      edprime(:,j) = edprime(:,k) + h_sol*dedprime(:,k);
      eqprime(:,j) = eqprime(:,k) + h_sol*deqprime(:,k);
      psikd(:,j) = psikd(:,k) + h_sol*dpsikd(:,k);
      psikq(:,j) = psikq(:,k) + h_sol*dpsikq(:,k);
      Efd(:,j) = Efd(:,k) + h_sol*dEfd(:,k);
      V_R(:,j) = V_R(:,k) + h_sol*dV_R(:,k);
      V_As(:,j) = V_As(:,k) + h_sol*dV_As(:,k);
      R_f(:,j) = R_f(:,k) + h_sol*dR_f(:,k);
      V_TR(:,j) = V_TR(:,k) + h_sol*dV_TR(:,k);
      pss1(:,j) = pss1(:,k) + h_sol*dpss1(:,k);
      pss2(:,j) = pss2(:,k) + h_sol*dpss2(:,k);
      pss3(:,j) = pss3(:,k) + h_sol*dpss3(:,k);
      tg1(:,j) = tg1(:,k) + h_sol*dtg1(:,k);
      tg2(:,j) = tg2(:,k) + h_sol*dtg2(:,k);
      tg3(:,j) = tg3(:,k) + h_sol*dtg3(:,k);
      vdp(:,j) = vdp(:,k) + h_sol*dvdp(:,k);
      vqp(:,j) = vqp(:,k) + h_sol*dvqp(:,k);
      slip(:,j) = slip(:,k) + h_sol*dslip(:,k);
      vdpig(:,j) = vdpig(:,k) + h_sol*dvdpig(:,k);
      vqpig(:,j) = vqpig(:,k) + h_sol*dvqpig(:,k);
      slig(:,j) = slig(:,k) + h_sol*dslig(:,k);
      B_cv(:,j) = B_cv(:,k) + h_sol*dB_cv(:,k);
      B_con(:,j) = B_con(:,k) + h_sol*dB_con(:,k);
      lmod_st(:,j) = lmod_st(:,k) + h_sol*dlmod_st(:,k);
      rlmod_st(:,j) = rlmod_st(:,k)+h_sol*drlmod_st(:,k);
      v_conr(:,j) = v_conr(:,k) + h_sol*dv_conr(:,k);
      v_coni(:,j) = v_coni(:,k) + h_sol*dv_coni(:,k);
      i_dcr(:,j) = i_dcr(:,k) + h_sol*di_dcr(:,k);
      i_dci(:,j) = i_dci(:,k) + h_sol*di_dci(:,k);
      v_dcc(:,j) = v_dcc(:,k) + h_sol*dv_dcc(:,k);
      
     %UPFC1
    Vc1_(1,j) = Vc1_(1,k)+h_sol*dVc1_(1,k);    
    D_dc1(1,j) = D_dc1(1,k)+h_sol*dD_dc1(1,k);
    D_ac1(1,j) = D_ac1(1,k)+h_sol*dD_ac1(1,k);
    D_p1(1,j) = D_p1(1,k)+h_sol*dD_p1(1,k);
    D_q1(1,j) = D_q1(1,k)+h_sol*dD_q1(1,k);

     
     mE1_(1,j) = mE1_(1,k);
     DeltaEp1_(1,j) = DeltaEp1_(1,k);
     DeltaBp1_(1,j) = DeltaBp1_(1,k);
     mB1_(1,j) = mB1_(1,k);
     
     H_dc1(1,j)=H_dc1(1,k);
     H_ac1(1,j)=H_ac1(1,k);
     H_p1(1,j)=H_p1(1,k);
     H_q1(1,j)=H_q1(1,k);
      
      flag = 1;
      % mach_ref(j) = mac_ang(syn_ref,j);
      mach_ref(j) = 0;
      % perform network interface calculations again with predicted states
	  f = mac_tra(0,j,bus_sim,flag);
           
      % Calculate current injections and bus voltages and angles
      if j >= sum(k_inc(1:3))+1
         % fault cleared
         bus_sim = bus_pf2;
         bus_int = bus_intpf2;
         Y1 = Y_gpf2;
         Y2 = Y_gncpf2;
         Y3 = Y_ncgpf2;
         Y4 = Y_ncpf2;
         Vr1 = V_rgpf2; 
         Vr2 = V_rncpf2;
         bo = bopf2;
         Y_upfc = Y_upfcpf2;
         V_upfc = V_upfcpf2;
         h_sol = i_simu(j,ks,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo,Y_upfc,V_upfc);     
      elseif j >=sum(k_inc(1:2))+1
         % near bus cleared
         bus_sim = bus_pf1;
         bus_int = bus_intpf1;
         Y1 = Y_gpf1;
         Y2 = Y_gncpf1;
         Y3 = Y_ncgpf1;
         Y4 = Y_ncpf1;
         Vr1 = V_rgpf1; 
         Vr2 = V_rncpf1;
         bo = bopf1;
         Y_upfc = Y_upfcpf1;
         V_upfc = V_upfcpf1;
         h_sol = i_simu(j,ks,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo,Y_upfc,V_upfc);   
      elseif j>=k_inc(1)+1
         % fault applied
         bus_sim = bus_f;
         bus_int = bus_intf;
         Y1 = Y_gf;
         Y2 = Y_gncf;
         Y3 = Y_ncgf;
         Y4 = Y_ncf;
         Vr1 = V_rgf; 
         Vr2 = V_rncf;
         bo = bof;
         Y_upfc = Y_upfcf;
         V_upfc = V_upfcf;
         h_sol = i_simu(j,ks,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo,Y_upfc,V_upfc);     
      elseif k<k_inc(1)+1
         % pre fault
         bus_sim = bus;
         bus_int = bus_intprf;
         Y1 = Y_gprf;
         Y2 = Y_gncprf;
         Y3 = Y_ncgprf;
         Y4 = Y_ncprf;
         Vr1 = V_rgprf; 
         Vr2 = V_rncprf;
         bo = boprf;
         Y_upfc = Y_upfcprf;
         V_upfc = V_upfcprf;
         h_sol = i_simu(j,ks,k_inc,h,bus_sim,Y1,Y2,Y3,Y4,Vr1,Vr2,bo,Y_upfc,V_upfc);     
      end
      vex(:,j)=vex(:,k);
      cur_ord(:,j) = cur_ord(:,k);
      f = smpexc(0,j,bus_sim,flag);
         
      
      flag = 2;
      f = mac_tra(0,j,bus_sim,flag);
      f = smpexc(0,j,bus_sim,flag);
      if n_svc~=0
         f = msvc_sig(t(j),j);
         v_svc = abs(bus_v(bus_int(svc_con(:,2)),j));
         bus_sim = svc(0,j,bus_sim,flag,v_svc);
      end
      
     
      % following statements are corrector steps
      mac_ang(:,j) = mac_ang(:,k) +...
         h_sol*(dmac_ang(:,k)+dmac_ang(:,j))/2.;
      mac_spd(:,j) = mac_spd(:,k) +...
         h_sol*(dmac_spd(:,k)+dmac_spd(:,j))/2.;
      edprime(:,j) = edprime(:,k) +...
         h_sol*(dedprime(:,k)+dedprime(:,j))/2.;
      eqprime(:,j) = eqprime(:,k) +...
         h_sol*(deqprime(:,k)+deqprime(:,j))/2.;
      psikd(:,j) = psikd(:,k) +...
         h_sol*(dpsikd(:,k)+dpsikd(:,j))/2.;
      psikq(:,j) = psikq(:,k) +...
         h_sol*(dpsikq(:,k)+dpsikq(:,j))/2.;
      Efd(:,j) = Efd(:,k) +...
         h_sol*(dEfd(:,k)+dEfd(:,j))/2.;
      V_R(:,j) = V_R(:,k) +...
         h_sol*(dV_R(:,k)+dV_R(:,j))/2.;
      V_As(:,j) = V_As(:,k) +...
         h_sol*(dV_As(:,k)+dV_As(:,j))/2.;
      R_f(:,j) = R_f(:,k) +...
         h_sol*(dR_f(:,k)+dR_f(:,j))/2.;
      V_TR(:,j) = V_TR(:,k) +...
         h_sol*(dV_TR(:,k)+dV_TR(:,j))/2.;
      pss1(:,j) = pss1(:,k) +h_sol*(dpss1(:,k)+dpss1(:,j))/2.;
      pss2(:,j) = pss2(:,k) +h_sol*(dpss2(:,k)+dpss2(:,j))/2.;
      pss3(:,j) = pss3(:,k) +h_sol*(dpss3(:,k)+dpss3(:,j))/2.;
      tg1(:,j) = tg1(:,k) + h_sol*(dtg1(:,k) + dtg1(:,j))/2.;
      tg2(:,j) = tg2(:,k) + h_sol*(dtg2(:,k) + dtg2(:,j))/2.;
      tg3(:,j) = tg3(:,k) + h_sol*(dtg3(:,k) + dtg3(:,j))/2.;
      vdp(:,j) = vdp(:,k) + h_sol*(dvdp(:,j) + dvdp(:,k))/2.;
      vqp(:,j) = vqp(:,k) + h_sol*(dvqp(:,j) + dvqp(:,k))/2.;
      slip(:,j) = slip(:,k) + h_sol*(dslip(:,j) + dslip(:,k))/2.;
      vdpig(:,j) = vdpig(:,k) + h_sol*(dvdpig(:,j) + dvdpig(:,k))/2.;
      vqpig(:,j) = vqpig(:,k) + h_sol*(dvqpig(:,j) + dvqpig(:,k))/2.;
      slig(:,j) = slig(:,k) + h_sol*(dslig(:,j) + dslig(:,k))/2.;
      B_cv(:,j) = B_cv(:,k) + h_sol*(dB_cv(:,j) + dB_cv(:,k))/2.;
      B_con(:,j) = B_con(:,k) + h_sol*(dB_con(:,j) + dB_con(:,k))/2.;
      lmod_st(:,j) = lmod_st(:,k) + h_sol*(dlmod_st(:,j) + dlmod_st(:,k))/2.;
      rlmod_st(:,j) = rlmod_st(:,k) + h_sol*(drlmod_st(:,j) + drlmod_st(:,k))/2.;
      v_conr(:,j) = v_conr(:,k) + h_sol*(dv_conr(:,j) + dv_conr(:,k))/2.;
      v_coni(:,j) = v_coni(:,k) + h_sol*(dv_coni(:,j) + dv_coni(:,k))/2.;
      i_dcr(:,j) = i_dcr(:,k) + h_sol*(di_dcr(:,j) + di_dcr(:,k))/2.;
      i_dci(:,j) = i_dci(:,k) + h_sol*(di_dci(:,j) + di_dci(:,k))/2.;
      v_dcc(:,j) = v_dcc(:,k) + h_sol*(dv_dcc(:,j) + dv_dcc(:,k))/2.;
      
      %UPFC1
    Vc1_(1,j)=Vc1_(1,k)+h_sol*(dVc1_(1,k)+dVc1_(1,j))/2.;    
    D_dc1(1,j)=D_dc1(1,k)+h_sol*(dD_dc1(1,k)+dD_dc1(1,j))/2.;
    D_ac1(1,j)=D_ac1(1,k)+h_sol*(dD_ac1(1,k)+dD_ac1(1,j))/2.;
    D_p1(1,j)=D_p1(1,k)+h_sol*(dD_p1(1,k)+dD_p1(1,j))/2.;
    D_q1(1,j)=D_q1(1,k)+h_sol*(dD_q1(1,k)+dD_q1(1,j))/2.;
    
    
    
    H_dc1(1,k+1)=KP_Ddc1*(Vc1_ref(1,k)-Vc1_(1,k));
    VEt1=abs(VEt1_(1,k));
    H_ac1(1,k+1)=KP_Dac1*(VEt1_ref(1,k)-VEt1);
    H_p1(1,k+1)=KP_Dp1*(p1_ref(1,k)-p1_(1,k));
    H_q1(1,k+1)=KP_Dq1*(q1_ref(1,k)-q1_(1,k));
      % UPFC1 
        %-- PI-Vdc1 controller
        DeltaEp1_(1,k+1)=DeltaEp1_0;
        if cont_on_off(1)==1
           DeltaEp1_(1,k+1)=DeltaEp1_0 + H_dc1(1,k+1) + D_dc1(1,k+1);
        end
        %-- PI-VEt1 controller
        mE1_(1,k+1) = mE1_0;
        if cont_on_off(2)==1
           mE1_(1,k+1) = mE1_0 + H_ac1(1,k+1) + D_ac1(1,k+1);
        end  
        %-- PI-q1 controller
        DeltaBp1_(1,k+1) = DeltaBp1_0;
        if cont_on_off(3)==1
            DeltaBp1_(1,k+1) = DeltaBp1_0 + H_p1(1,k+1) + D_p1(1,k+1);%%%%%%%%%%%%%%%%% 感性状态下： P controller 肯定是＋号； 
        end                                                          %%%%%%%%%%%%%%%%% 容性工作时：P controller 肯定是－号较好.
        %-- PI-p1 controller
        mB1_(1,k+1) = mB1_0;
        if cont_on_off(4)==1
            mB1_(1,k+1) = mB1_0 - H_q1(1,k+1) - D_q1(1,k+1);
        end
        
        
    end
    kt = kt + k_inc(ks);
    ks = ks+1;
end 


