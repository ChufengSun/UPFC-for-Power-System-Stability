% bus and line are known

if line_flw(UPFC1_line_indx,4) < 0
          f = 0;
          f = fromBus1;
   fromBus1 = toBus2;
     toBus2 = f;
end


loc_upfc1=.2;        % location of UPFC1 in the Line
z11_=loc_upfc1*Z_1;
z12_=(1-loc_upfc1)*Z_1+jay*xB_1;
 nbus = length(bus(:,1));

theta(1:nbus,1)=bus(:,3)*pi/180;
bus_volt(1:nbus,1)=bus(1:nbus,2).*exp(jay*theta(1:nbus,1));
%
v1=bus_volt(fromBus1,1);
v2=bus_volt(toBus2,1);
I_12=(v1-v2)/(z11_+z12_);
VEt1_norm=v1-z11_*I_12;
sEt1_norm=VEt1_norm*conj(I_12);
pEt1_norm=real(sEt1_norm);
qEt1_norm=imag(sEt1_norm);
VEtMag=abs(VEt1_norm);
CoNtoRL=1;%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if CoNtoRL==1
    VEt1_norm = VEt1_norm - 0.00;
    pEt1_norm = pEt1_norm - 0.10;
    qEt1_norm = qEt1_norm - 0.0;
end    

%bus PV specifications ( bus number: 40)
Bus_40=[(nbus+1) abs(VEt1_norm) 0  -pEt1_norm  0       0            0        0   0  2  99  -99  1.0  0  0];  
%bus PQ specifications ( bus number: 41)
Bus_41=[(nbus+2)      1         0      0       0   -pEt1_norm   -qEt1_norm   0   0  3  99  -99  1.0  0  0];
% modify bus data to include two extra buses
bus_mod=[bus;Bus_40;Bus_41];

Line_split1=[fromBus1 (nbus+1) real(z11_) imag(z11_)  0  0  0  0  0  0] ;

Line_split2=[(nbus+2)  toBus2  real(z12_) imag(z12_)  0  0  0  0  0  0];
% modify Line specificatios to include one extra trans. line
line_mo=line;

line_mo(UPFC1_line_indx,:)=Line_split1;
line_mod=[line_mo;Line_split2];

%  loadflow re-calculation to obtain the system conditions 
%  with all UPFC1 functions in operation
[bus_sol_mod,line_mod,line_flw_mod] = ...
                   loadflow(bus_mod,line_mod,1e-9,30,1.0,'n',1);

bus_volt_mod(1:nbus+2,1)=bus_sol_mod(1:nbus+2,2).*exp(jay*bus_sol_mod(1:nbus+2,3)*pi/180);
v1=bus_volt_mod(fromBus1,1);
v2=bus_volt_mod(toBus2,1);
vPVG=bus_volt_mod(nbus+1,1);
vPQL=bus_volt_mod(nbus+2,1);
I12=(v1-vPVG)/z11_; % is equal to :  
%I23=(vPQL-v2)/z12_;
%1111111111111111111111111111111111111111111111111111111111111111111111111111111
FROM1=find(line_flw_mod(:,2)==fromBus1 & line_flw_mod(:,3)== (nbus+1));
ps_=line_flw_mod(FROM1,4);
qs_=line_flw_mod(FROM1,5);     % power flows from sending end bus
nline_mod = length(line_mod(:,1));
TO2=find(line_flw_mod(:,2)==toBus2 & line_flw_mod(:,3)== (nbus+2));
pr_=-line_flw_mod(TO2,4);
qr_=-line_flw_mod(TO2,5);     % power flows to reciving end bus
Vs_mag=bus_sol_mod(fromBus1,2);
Vs_ang=bus_sol_mod(fromBus1,3)*pi/180;
Vr_mag=bus_sol_mod(toBus2,2);
Vr_ang=bus_sol_mod(toBus2,3)*pi/180;

 zt_1 = -z11_*z12_ - jay*xE_1*z11_ - jay*xE_1*z12_ ;
 
 au=-(z12_+jay*xE_1)/zt_1;      a_mag=abs(au);   a_ang=angle(au);
 bu=z12_/zt_1;                  b_mag=abs(bu);   b_ang=angle(bu); 
 cu=jay*xE_1/zt_1;              c_mag=abs(cu);   c_ang=angle(cu);
 du=-jay*xE_1/zt_1;             d_mag=abs(du);   d_ang=angle(du);
 
 ku=-jay*xE_1/zt_1;             k_mag=abs(ku);   k_ang=angle(ku); 
 fu=-z11_/zt_1;                 f_mag=abs(fu);   f_ang=angle(fu);
 gu=(z11_+jay*xE_1)/zt_1;       g_mag=abs(gu);   g_ang=angle(gu);
 hu=-(z11_+jay*xE_1)/zt_1;      h_mag=abs(hu);   h_ang=angle(hu);

 x0=[abs(VEt1_norm) angle(VEt1_norm) 0 0];
 TolX=eps;
 x=zeros(4,1);
 x=fsolve('my_fun_mm',x0,TolX);
 VE1=x(1);
 AngVE1=x(2);  
 vE1=VE1*exp(jay*AngVE1);
 VB1=x(4);
 AngVB1=x(3); 
 vB1=VB1*exp(jay*AngVB1);


 
% ---- just check the results ----------
 
I1E2=[-(z12_+jay*xE_1)/zt_1     jay*xE_1/zt_1
         -jay*xE_1/zt_1       (z11_+jay*xE_1)/zt_1]*[ v1-vE1
                                                     v2-vE1-vB1];                                                        
IE1=I1E2(1)-I1E2(2);
 s1s = v1*conj(I1E2(1));
 s2r = v2*conj(I1E2(2));
se=vE1*conj(IE1);
sb=vB1*conj(I1E2(2));
  VET=v1-z11_*I1E2(1);
  VBB=VET+vB1;
  SBB=VBB*conj(I1E2(2));
 
  
  % 
VE1_(1,1)=x(1);       % state variable
mE1_(1,1)=2*VE1_(1,1);% control signal
mE1_0=mE1_(1,1);

DeltaE1_(1,1)=x(2);   % state variable
DeltaEp1_(1,1)=x(2);  % control signal
DeltaEp1_0=x(2);

DeltaB1_(1,1)=x(3);   % state variable
DeltaBp1_(1,1)=x(3);  % control signal
DeltaBp1_0=x(3);

VB1_(1,1)=x(4);       % state variable
mB1_(1,1)=2*VB1_(1,1);% control signal
mB1_0=mB1_(1,1);

DeltaE1_0=x(2);
DeltaB1_0=x(3);



  Vc1_(1,1)=1;
Vc1_ref(1,1)=1;
 
  VEt1_(1,1) = v1-z11_*I1E2(1);
  VEt1_abs(1,1) = abs(VEt1_(1,1));
  
  VEt1_ref(1,1)=abs(VE1_(1,1));
VBt1 = -VEt1_(1,1)+(z12_-jay*xB_1)*I1E2(2) + v2;              
                                                
VBB1 = VEt1_(1,1)+vB1;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
ss1 = VBB1*conj(I1E2(2));
p1_(1,1) = real(ss1);                                                         
q1_(1,1) = imag(ss1);   
p1_ref(1,1)=real(ss1);
q1_ref(1,1)=imag(ss1);
 %power flows across DC link capacitor
PEin1_(1,1) = real(se);
PBout1_(1,1) = real(sb);

 % bus : up to date 
 bus = bus_sol_mod(1:nbus,:);
 line_mod_temp = line_mod(1:nline,:);
 line_mod_temp(UPFC1_line_indx,:)=line(UPFC1_line_indx,:);
 line = line_mod_temp(1:nline,:);
 