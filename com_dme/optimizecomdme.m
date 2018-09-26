function [trajKnots, xOpt] = optimizecomdme()

global robot m N g simTime

IC = [robot.homeConfiguration',[0,0,0], robot.centerOfMass',zeros(1,6),[0 m*g 0], simTime/(N-1)]';
FC = [[60,0,30]*pi/180,[0,0,0],[0.6,1.0892,0],zeros(1,6),[0 m*g 0],0]';
xInit = guess_solution(IC,FC); 
numDecVar = length(xInit);
numState  = length(IC); 
[xLow, xUpp] = setStateBoundaries(numDecVar,IC,FC);


usrfun = 'conscomdmev2';
conseqno=5; %how many constraint equations are there? check from cons file
constraint_no= (N-1)*(3*conseqno)+1;

Flow = [ 0;            %lowB bound of the cost (final time)
         zeros(constraint_no,1)]; %lowB of the dynamics of x y and v (each size N)
% Flow(7:18:end)=-1;%Change to fit DME constraint
% Flow(8:18:end)=-1;
% Flow(9:18:end)=-1;

LossUpperBound = 10000;
Fupp = [ LossUpperBound;
         zeros(constraint_no,1) ];
% Fupp(7:18:end)=1;%Change to fit DME constraint
% Fupp(8:18:end)=1;
% Fupp(9:18:end)=1;


%This little 4-line block of setting variables for Snopt are not used very
%often; we set them to the defaults; ObjRow for example defines which row the
%objective function is located within the constraints col-vec ('F' in the
%userFun)
numConstraints  = length(Flow);
xMul   = zeros(numDecVar,1); xState = zeros(numDecVar,1);
Fmul   = zeros(numConstraints,1); Fstate = zeros(numConstraints,1);
ObjAdd = 0; ObjRow = 1;

%Setting the linear and nonlinear sparsity patterns--this version does not
%supply any of the sparsity patterns; it tells Snopt that every entry of the
%Jacobian needs to be calculated
A  = [];  iAfun = [];  jAvar = [];
[iGfun,jGvar] = find(ones(numConstraints, numDecVar));

%Set the Optimal Parameters for SNOPT. See chapter 7, pg 62 'Optimal Parameters'
%Note we first set 'Defaults' to start SNOPT on a clean-slate; very important!
snset('Defaults');              %You NEED this to flush snopt clean before a run!
snseti('Derivative option', 0); %Telling snopt we know nothing about the jacobian
snseti('Verify level', 3);      %Slows performance but very useful

%Sumary and Solution files; see chapter 8 of SNOPT guide (section 8.8, 8.9)
snprint('resultGradySNOPT.txt');
snsummary('resultGradySNOPT.sum');

%Call snopt
solveopt = 1;       %Still have no idea what this flag tells snopt (other than 'optimize')
tic                 %We are going to time snopt
[xOpt,F,xMul,Fmul,INFO] = snoptcmex( solveopt, ...
				                     xInit,xLow,xUpp,xMul,xState, ...
				                     Flow,Fupp,Fmul,Fstate,        ...
				                     ObjAdd,ObjRow,A,iAfun,jAvar,  ...
				                     iGfun,jGvar,usrfun );
runTime=toc        %we are timing snopt

for i=1:N
    trajKnots{i} = xOpt((i-1)*numState+1:i*numState);
end
fprintf('Trajectory Built, Info: %d \n',INFO)
end