function [com, comJac,cmm] = cominfo(obj, varargin)
            %centerOfMass Compute the center of mass position and Jacobian
            %   COM = centerOfMass(ROBOT) computes the center of mass
            %   position of ROBOT at its home configuration relative to the
            %   base frame.
            %
            %   COM = centerOfMass(ROBOT, Q) computes the center of mass 
            %   position of ROBOT at the specified joint configuration Q 
            %   relative to the base frame.
            %
            %   [COM, COMJAC] = centerOfMass(ROBOT, ...) also returns the
            %   center of mass Jacobian COMJAC as the second output argument.
            %   COMJAC relates center of mass velocity to joint
            %   velocities. 
            %
            %   COM is a 3-by-1 vector, and COMJAC is a 3-by-vNum matrix,
            %   where vNum is the velocity number of ROBOT (i.e. the
            %   degrees of freedom).
            %
            %   Examples:
            %       % Load example robot 
            %       load exampleRobots.mat
            %
            %       % Set lbr robot dynamics input data format to 'row'
            %       lbr.DataFormat = 'row';
            %
            %       % Compute the center of mass position and Jacobian at
            %       % home configuration
            %       [com, comJac] = centerOfMass(lbr);
            
            narginchk(1,2);
            q = validateDynamicsFunctionInputs(obj.TreeInternal, false, varargin{:});
            [com, totalmass, cmm] = robotics.manip.internal.RigidBodyTreeDynamics.centroidalMomentumMatrix(obj.TreeInternal, q);
            comJac = cmm(4:6,:)/totalmass;
            com = resultPostProcess(obj.TreeInternal, com);
        end