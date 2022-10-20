function [delP,delQ,P,Q,conv_flag] = ...
                 calc(nbus,V,ang,Y,Pg,Qg,Pl,Ql,sw_bno,g_bno,tol)
% Syntax:  [delP,delQ,P,Q,conv_flag] = 
%                calc(nbus,V,ang,Y,Pg,Qg,Pl,Ql,sw_bno,g_bno,tol)
%
% Purpose: calculates power mismatch and checks convergence
%          also determines the values of P and Q based on the 
%          supplied values of voltage magnitude and angle
%returns the active and reactive power mismatches at the buses 
%and the estimated active and reactive powers P and Q.
%The flag conv_flag is set to 1 if the mismatch is less than 
%the specified tolerance tol, and is set to 0 otherwise.
% Version: 2.0 eliminates do loop
% Input:   nbus      - total number of buses
%          bus_type  - load_bus(3), gen_bus(2), swing_bus(1)
%          V         - magnitude of bus voltage
%          ang       - angle(rad) of bus voltage
%          Y         - admittance matrix
%          Pg        - real power of generation
%          Qg        - reactive power  of generation
%          Pl        - real power of load
%          Ql        - reactive power of load
%	 sw_bno - a vector having zeros at all  swing_bus locations ones otherwise
%	 g_bno  - a vector having zeros at all  generator bus locations ones otherwise
%   sw_bno - a vector , formed in loadflow, of length nbus. All entries 
%are 1, except for that corresponding to the swing buses which are 0 
%   g_bno   - a vector , formed in loadflow, of length nbus. All entries
%are 1, except for that corresponding to the generator buses which are 0
%      tol       - a tolerance of computational error
%
% Output:  delP      - real power mismatch
%          delQ      - reactive power mismatch
%          P         - calculated real power
%          Q         - calculated reactive power
%          conv_flag - 0, converged
%                      1, not yet converged
%Algorithm:
%The current injected into each bus is obtained by pre multiplying the 
%complex bus voltage vector by the Y matrix. From the current and the 
%voltage, the active(P) and reactive(Q) powers are calculated.  
%The resultant power mismatches are 
%             delP=Pg-Pl-P      delQ=Qg-Ql-Q

% ************************************************************
jay = sqrt(-1);
swing_bus = 1;
gen_bus = 2;
load_bus = 3;
% voltage in rectangular coordinate
V_rect = V.*exp(jay*ang);  
% bus current injection
cur_inj = Y*V_rect;
% power output based on voltages 
S = V_rect.*conj(cur_inj);
P = real(S); Q = imag(S);
delP = Pg - Pl - P;
delQ = Qg - Ql - Q;
% zero out mismatches on swing bus and generation bus
delP=delP.*sw_bno;
delQ=delQ.*sw_bno;
delQ=delQ.*g_bno;
%  total mismatch
[pmis,ip]=max(abs(delP));
[qmis,iq]=max(abs(delQ));
mism = pmis+qmis;
if mism > tol,
    conv_flag = 1;
  else
    conv_flag = 0;
end
return