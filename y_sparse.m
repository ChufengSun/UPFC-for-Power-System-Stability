function     [Y,nSW,nPV,nPQ,SB] = y_sparse(bus,line)
% Syntax:    [Y,nSW,nPV,nPQ,SB] = y_sparse(bus,line) 
%
% Purpose:   build sparse admittance matrix Y from the line data
%uses the bus data and the line data in matrices bus and line to return 
%the admittance matrix Y in sparse matrix form. 
%Forms the global index matrix bus_int.
% Input:     bus  - bus data
%            line - line data
%
% Output:    Y    - sparse admittance matrix
%            nSW  - total number of swing buses
%            nPV  - total number generator buses
%            nPQ  - total number of load buses
%            SB   - internal bus numbers of swing bus
%Global Variable
%The index matrix bus_int is a vector of length equal to the highest bus
%number specified in the first column of bus.It is zero everywhere 
%except for the locations corresponding to the specified bus numbers. At
%these locations, the entry is the corresponding row number of bus. Thus
%bus_int(bus(:,1)) gives the vector  [1 2 3 4 ..number of buses]'.
% See also:  
%
% Calls:    
%

% ************************************************************
global bus_int

jay = sqrt(-1);
swing_bus = 1;
gen_bus = 2;
load_bus = 3;

nline = length(line(:,1));     % number of lines
nbus = length(bus(:,1));     % number of buses
r=zeros(nline,1);
rx=zeros(nline,1);
chrg=zeros(nline,1);
z=zeros(nline,1);
y=zeros(nline,1);

Y = sparse(1,1,0,nbus,nbus);

% set up internal bus numbers for second indexing of buses
busmax = max(round(bus(:,1)));
bus_int = zeros(busmax,1);
ibus = [1:1:nbus]';
for i = 1:nbus
  bus_int(round(bus(i,1))) = i;
end
bus_int'
% process line data and build admittance matrix Y
  r = line(:,3);
  rx = line(:,4);
  chrg =jay*sparse(diag( 0.5*line(:,5)));
  z = r + jay*rx;     % line impedance
  y = sparse(diag(ones(nline,1)./z));

% determine connection matrices including tap changers and phase shifters
  from_bus = round(line(:,1));
  from_int = bus_int(from_bus);
  to_bus = round(line(:,2));
  to_int = bus_int(to_bus);
  tap_index = find(abs(line(:,6))>0);
  tap=ones(nline,1);
  tap(tap_index)=1. ./line(tap_index,6);
  phase_shift = line(:,7);
  tap = tap.*exp(-jay*phase_shift*pi/180);

  % sparse matrix formulation
  iline = [1:1:nline]';
  C_from = sparse(from_int,iline,tap,nbus,nline,nline);
  C_to = sparse(to_int,iline,ones(nline,1),nbus,nline,nline);
  C_line = C_from - C_to;

% form Y matrix from primative line ys and connection matrices
  Y=C_from*chrg*C_from' + C_to*chrg*C_to' ;
  Y = Y + C_line*y*C_line';
  Gb = bus(:,8);     % bus conductance
  Bb = bus(:,9);     % bus susceptance

% add diagonal shunt admittances
  Y = Y + sparse(ibus,ibus,Gb+jay*Bb,nbus,nbus);

if nargout > 1
  % count buses of different types
  nSW = 0;
  nPV = 0;
  nPQ = 0;
  bus_type=round(bus(:,10));
  load_index=find(bus_type==3);
  gen_index=find(bus_type==2);
  SB=find(bus_type==1);
  nSW=length(SB);
  nPV=length(gen_index);
  nPQ=length(load_index);
end

return

