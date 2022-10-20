function [Jac11,Jac12,Jac21,Jac22]=form_jac(V,ang,Y,ang_red,volt_red)
% Syntax:  [Jac] = form_jac(V,ang,Y,ang_red,volt_red)
%          [Jac11,Jac12,Jac21,Jac22] = form_jac(V,ang,Y,...
%                                      ang_red,volt_red)
%
% Purpose: form the Jacobian matrix using sparse matrix techniques
%
% Input:   V        - magnitude of bus voltage
%          ang      - angle(rad) of bus voltage
%          Y        - admittance matrix
%          ang_red  - matrix to eliminate swing bus  voltage magnitude and angle 
%                     entries
%          volt_red - matrix to eliminate generator bus voltage magnitude
%                     entries
% Output:  Jac      - jacobian matrix
%          Jac11,Jac12,Jac21,Jac22 - submatrices of 
%                                      jacobian matrix  
% See also:   
%All outputs are in sparse matrix form.
%Jac = form_jac(V,ang,Y,ang_red,volt_red) returns the sparse jacobian matrix using the admittance matrix Y in sparse matrix form. The jacobian consists of the partial derivatives of active and reactive powers P and Q with respect to the bus voltage magnitudes (p.u.) and angles (rad)
%Inputs:
%V - 		vector of bus voltage magnitudes in p.u.
%ang -	 	vector of bus voltage angles in radians
%ang_red - 	a transformation matrix, generated in loadflow, which eliminates the swing bus voltage magnitude and angle from the Jacobian
%volt_red - 	a transformation matrix, generated in loadflow, which eliminates the generator bus voltage magnitudes from the Jacobian
%Outputs:
%Jac11 -    partial derivatives of P to angle, swing bus angle eliminated
%Jac12 -    partial derivatives of P to voltage, generator and swing bus voltages eliminated
%Jac21 -     partial derivatives of Q to angle, swing bus angle eliminated
%Jac22 -    partial derivatives of Q to voltage, generator and swing bus voltages eliminated
%The dimensions of the individual Jacobian matrices are  n by n, n by m, m by n  and m by m respectively. Where n is the  number of non-swing buses,  and m is the number of  buses which are neither swing nor generator buses.
%Algorithm:
%The elements of the jacobian are obtained by taking partial derivatives of active and 
%reactive powers of all buses in polar coordinates. Row and column reductions are then applied to eliminate those entries corresponding to specified components of bus angle and voltage.

% ***********************************************************
jay = sqrt(-1);
exp_ang = exp(jay*ang);
% Voltage rectangular coordinates
V_rect = V.*exp_ang;
CV_rect=conj(V_rect);
Y_con = conj(Y);
%vector of conjugate currents
i_c=Y_con*CV_rect;
% complex power vector
S=V_rect.*i_c;
S=sparse(diag(S));
Vdia=sparse(diag(V_rect));
CVdia=conj(Vdia);
Vmag=sparse(diag(abs(V)));
S1=Vdia*Y_con*CVdia;
t1=((S+S1)/Vmag)*volt_red';
t2=(S-S1)*ang_red';
J11=-ang_red*imag(t2);
J12=ang_red*real(t1);
J21=volt_red*real(t2);
J22=volt_red*imag(t1);
if nargout > 3
   Jac11 = J11; clear J11
   Jac12 = J12; clear J12
   Jac21 = J21; clear J21
   Jac22 = J22; clear J22
else
   Jac11 = [J11 J12;
	       J21 J22];
end

