function h_sol = i_simu(k,ks,k_inc,h,bus_sim,Y_g,Y_gnc,Y_ncg,Y_nc,rec_V1,rec_V2,bo,Y_upfc,V_upfc)

%Purpose forms the network interface variables
% Inputs: k - the current time step
%         ks - indicates the switching times
%         k_inc - the number of time seps between switching points
%         h vector of time steps
%         bus_sim value of bus matrix at this switching time
%         Y_g - reduced Y matrix for generators
%         Y_gnc - mutual reduced Y generators-nc loads
%         Y_ncg - mutual reduced Y nc loads generators
%         Y_nc - reduced Y matrix nc loads
%         rec_V1 - voltage recovery matrix generators
%         rec_V2 - voltage recovery matrix nc loads
%         bo bus order for this switching time
% Output: h_sol - the time step at this value of ks


global bus_v  theta bus_int

global psi_re psi_im  n_mac 
global cur_re  cur_im 

global load_con nload

global vdp vqp n_mot idmot iqmot p_mot q_mot ind_con

global vdpig vqpig n_ig idig iqig  pig qig igen_con 

global i_dc Vdc alpha gamma dcc_pot i_dcr  i_dci Pdc
global r_idx i_idx ac_bus rec_ac_bus inv_ac_bus n_conv

global controllers_on_off non_linear
% UPFC1 
global fromBus1 toBus2 UPFC1_line_indx Tve_1 Tde_1 Tdb_1 Tvb_1 
global I1E2_ IE1_ z11_ z12_ xB_1 xE_1 Cdc_1 zt_1 PEin1_ PBout1_
% state variables
global Vc1_ dVc1_ Vc1_ref D_dc1 dD_dc1 H_dc1 KI_Ddc1
global VE1_ dVE1_ DeltaE1_ dDeltaE1_ 
global VB1_ dVB1_ DeltaB1_ dDeltaB1_
% functions
global VEt1_ VEt1_ref D_ac1 dD_ac1 H_ac1 KI_Dac1
global p1_     p1_ref D_p1  dD_p1  H_p1  KI_Dp1
global q1_     q1_ref D_q1  dD_q1  H_q1  KI_Dq1
% control signals
global mE1_ DeltaEp1_ mB1_ DeltaBp1_ up_on VEt1_abs

jay = sqrt(-1);  
vE1=0.5*Vc1_(1,k)*mE1_(1,k)*exp(jay*DeltaEp1_(1,k));
vB1=0.5*Vc1_(1,k)*mB1_(1,k)*exp(jay*DeltaBp1_(1,k));

VEB=[vE1
     vB1];

flag =1;
 if isempty(n_mot); n_mot = 0;end
 if isempty(n_ig); n_ig =0;end
 jay = sqrt(-1);  
 psi = psi_re(:,k) + jay*psi_im(:,k);  
 vmp = vdp(:,k) + jay*vqp(:,k);
 vmpig = vdpig(:,k) + jay*vqpig(:,k);
 if (n_mot~=0&n_ig==0)
    ntot = n_mac + n_mot;
    ngm = n_mac + n_mot;
    int_volt=[psi; vmp]; % internal voltages of generators and motors 
 elseif (n_mot==0 & n_ig~=0)
    ntot = n_mac + n_ig;
    ngm = n_mac;
    int_volt=[psi; vmpig]; % internal voltages of generators and ind. generators 
 elseif  (n_mot~=0&n_ig~=0)
    ntot = n_mac + n_mot + n_ig;
    ngm = n_mac + n_mot;
    int_volt=[psi; vmp; vmpig]; % internal voltages of generators, motors & ind. generators  
 else
    int_volt = psi;
 end
 h_sol = h(ks);   
 %%%%%%
 h_sol = 0.01;
 %%%%%%
 nbus = length(bus_sim(:,1));
 cur = Y_g*int_volt ; % network solution currents into generators       
 b_v(bo(nload+1:nbus),1) = rec_V1*int_volt ;   % bus voltage reconstruction

 if nload~=0
   if k~=1; kk = k-1;else; kk=k;end;
   vnc = bus_v(bus_int(load_con(:,1)),kk);% initial value
   vnc = nc_load(bus_sim,flag,Y_nc,Y_ncg,int_volt,vnc,1e-9,k);

   % set nc load voltages
   b_v(bo(1:nload),1)=vnc;
   b_v(bo(nload+1:nbus),1) = b_v(bo(nload+1:nbus),1)+rec_V2*vnc;
   cur = cur + Y_gnc*vnc;% modify generator currents for nc loads
 end 
 cur = cur +  Y_upfc*VEB;
 b_v(bo(1:nbus),1) = b_v(bo(1:nbus),1) +  V_upfc*VEB;
 % note: the dc bus voltages are the equivalent HT bus voltages
 %       and not the LT bus voltages
 bus_v(bus_int(bus_sim(:,1)),k) = b_v;
 theta(bus_int(bus_sim(:,1)),k) = angle(b_v);     
 cur_re(:,k) = real(cur(1:n_mac));
 cur_im(:,k) = imag(cur(1:n_mac)); % generator currents
 if n_mot~=0
    idmot(:,k) = -real(cur(n_mac+1:ngm));%induction motor currents
    iqmot(:,k) = -imag(cur(n_mac+1:ngm));%current out of network
    s_mot(:,k) = bus_v(bus_int(ind_con(:,2)),k).*(idmot(:,k)-jay*iqmot(:,k));
    p_mot(:,k) = real(s_mot(:,k));
    q_mot(:,k) = imag(s_mot(:,k));
 end 
 if n_ig~=0
    idig(:,k) = -real(cur(ngm+1:ntot));%induction generator currents
    iqig(:,k) = -imag(cur(ngm+1:ntot));%current out of network
    s_igen(:,k) = bus_v(bus_int(igen_con(:,2)),k).*(idig(:,k)-jay*iqig(:,k));
    pig(:,k) = real(s_igen(:,k));
    qig(:,k) = imag(s_igen(:,k));
 end 

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%  UPFC1     
I1E2_(:,k)=[-(z12_+jay*xE_1)/zt_1     jay*xE_1/zt_1
               -jay*xE_1/zt_1      (z11_+jay*xE_1)/zt_1]*[  bus_v(fromBus1,k)-vE1
                                                           bus_v(toBus2,k)-vE1-vB1 ];
 IE1_(1,k)=I1E2_(1,k)-I1E2_(2,k);   

dVc1_(1,k)=-((3*mE1_(1,k))/(4*Cdc_1))*[cos(DeltaEp1_(1,k))  sin(DeltaEp1_(1,k))]*[real(IE1_(1,k));imag(IE1_(1,k))] ...
           +((3*mB1_(1,k))/(4*Cdc_1))*[cos(DeltaBp1_(1,k))  sin(DeltaBp1_(1,k))]*[real(I1E2_(2,k));imag(I1E2_(2,k))]; 
     
     
VEt1_(1,k) = bus_v(fromBus1,k)-z11_*I1E2_(1,k); 
VBt1_(1,k) =-VEt1_(1,k)+(z12_-jay*xB_1)*I1E2_(2,k) + bus_v(toBus2,k);
             
                                                
    
VBB1_(1,k)=VEt1_(1,k)+vB1;   

ss1 = VBB1_(1,k)*conj(I1E2_(2,k));
       p1_(1,k) = real(ss1);                                                        
       q1_(1,k) = imag(ss1);  

 

if non_linear==1
  dD_dc1(1,k) = KI_Ddc1*(Vc1_ref(1,k)-Vc1_(1,k));                                   
  dD_ac1(1,k) = KI_Dac1*(VEt1_ref(1,k)-abs(VEt1_(1,k)));                              
  dD_p1(1,k)=KI_Dp1*(p1_ref(1,k)-p1_(1,k));             
  dD_q1(1,k)=KI_Dq1*(q1_ref(1,k)-q1_(1,k));             
end

PEin1_(1,k)=real((vE1)*conj(IE1_(1,k)));
PBout1_(1,k)=real((vB1)*conj(I1E2_(2,k)));
