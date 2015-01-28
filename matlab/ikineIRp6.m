%	Q = IKINEIRp6(ROBOT, T, CONFIG)
%
% Solve the inverse kinematics of the IRp-6 ROBOT (with added sixth DOF) 
% whose end-effector pose is given by T.
%
% The optional third argument specifies the configuration of the arm in
% the form of a string containing one or more of the configuration codes:
%
%
% REFERENCE:
%
% Inverse kinematics for a IRp-6 robot based on the equations by Wojciech Szynkiewicz
%
%
% AUTHOR:
% Wojciech Szynkiewicz
% IAiIS
% 15/11/2004

% MOD HISTORY
%  ver. 1.1

function theta = ikineIRp6(T, configuration)
% Parametry kinematyczne robota IRp-6 (dane dla dwóch robotów)
   d1 = 0.70; % wysokoœæ kolumny -- w tej funkcji nie jest u¿ywane
   a2 = 0.45; % d³ugoœæ ramienia dolnego
   a3 = 0.67; % d³ugoœæ ramienia górnego
   d5 = 0.15; % odsuniêcie 6-tego stopnia swobody
   d7 = 0.20; % d³ugoœæ chwytka

%	if robot.n ~= 6,%
%		error('Solution only applicable for 6DOF manipulator');
%	end

%	if robot.mdh ~= 0,
%		error('Solution only applicable for standard DH conventions');
%	end
    EPS = 1e-12; 
    % Lower and upper joint limits for IRp-6 robot manipulator
    qmin = [-170 -130 -25 -90 -180 -180]' * pi / 180;
    qmax = [170 -50 40 90 180 180]' * pi / 180;
   
%	L = robot.links;
%	a2 = L{3}.A;
%	a3 = L{4}.A;
%	d5 = L{5}.D;

	if ~ishomog(T),
		error('T is not a homogeneous xform');
	end

	% undo base transformation
%	T = inv(robot.base) * T;

	% The following parameters are extracted from the Homogeneous 
	% Transformation Matrix
	Nx = T(1,1);
	Ny = T(2,1);
	Nz = T(3,1);
    
	Ox = T(1,2);
	Oy = T(2,2);
	Oz = T(3,2);

	Ax = T(1,3);
	Ay = T(2,3);
	Az = T(3,3);

	Px = T(1,4);
	Py = T(2,4);
	Pz = T(3,4);

	% The configuration parameter determines what n2,n5 values are used
	% and how many solutions are determined which have values of -1 or +1.

	if nargin < 3,
		configuration = '';
	else
		configuration = lower(configuration);
	end

	% default configuration

	n2 =  1;	% theta(2) > 0
	n5 =  1;	% theta(5) > 0
	if ~isempty(findstr(configuration, 'm')),
		n5 = -1;
	end
	if ~isempty(findstr(configuration, 'p')),
		n5 = 1;
	end

	if ~isempty(findstr(configuration, 'f')),
		n2 = 1;
	end
	if ~isempty(findstr(configuration, 'b')),
		n2 = -1;
	end

	
	% Solve for theta(1)
	% 
	theta(1)= atan2(Py,Px);
    
	% Solve for theta(5)
    S1 = sin(theta(1));
    C1 = cos(theta(1));
    
    C5 = -Ay*C1 + Ax*S1;
    S5 = n5*sqrt(1-C5*C5);
    theta(5)= atan2(S5, C5);

	%
	% Solve for theta(4)
	%
	if abs(S5) > EPS,
        theta(4) = atan2(Ax*C1+Ay*S1, Az);

    % Solve for theta(6)
        theta(6) = atan2(Oy*C1-Ox*S1, Nx*S1-Ny*C1); 
    else
	% Singularity at S5=0
        if abs(theta(5)) < EPS
            disp('Singularity at theta(5) = 0');
            theta(4) = atan2(Oy,Nx);
            theta(6) = 0;
        end
        if abs(pi-theta(5)) < EPS
            disp('Singularity at theta(5) = pi');
            theta(4) = atan2(Oy,-Nx);
            theta(6) = 0;
        end
    end    
	% 
    % Solve for theta(2)
    A = C1*Px + S1*Py - d5*cos(theta(4));
    B = -Pz - d5*sin(theta(4));
    C = 2*a2*B;
    D = 2*a2*A;
    E = a2*a2 + A*A + B*B - a3*a3;
    R = C*C + D*D;
    theta(2) = atan2(E/(sqrt(R)), n2*sqrt(1 - (E*E)/R))-atan2(D,C);
   
    % Solve for theta(3)
    
    theta(3) = atan2(B-a2*sin(theta(2)), A-a2*cos(theta(2)));
    
    for i=1:6,
        if theta(i) < qmin(i)
            sprintf('Lower limit for theta-%d',i);
            disp('LL');
        elseif theta(i) > qmax(i)
            sprintf('Upper limit for theta-%d',i);
            disp('UL');
        end
    end