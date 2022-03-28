function prepare_optimization_matrix(obj, num_in_contact_frames, num_vertices )

% PREPARE_OPTIMIZATION_MATRIX Fills the matrix relating to the unilateral
%                             and friction cone constraints used by the
%                             optimization problem solver.
% 
% PROCEDURE: Assuming that Fc_i = [fx,fy,fz] is the reaction force applied 
%            to the vertex i, the unilateral constraints is
% 
%                 fz >= 0,
% 
%            The simplified friction cone constraint is as
% 
%                 -mu fz <= fx,fy <= mu fz,
% 
%            where mu is the friction coefficient. The unilateral and the
%            simplified friction cone constriant can be written as
% 
%                 A_i Fc_i <= 0,
% 
%            The constraints for all the vertices can be written together as
% 
%                 A Fc <= 0.
%   
%            For ...
%             - quadprog: Ax <= b (should include -x <= 0).
%             - OSQP    : l <= Ax <= u (should include -Inf <= -x <= 0).
%             - APOASES : l <= Ax <= u (should include -Inf <= -x <= 0).
%            So in all the cases we define: Ax_Lb <= Ax <= Ax_Ub where Ax_Lb = -Inf.

total_num_vertices = num_vertices * num_in_contact_frames; % number of vertices per foot * number of feet
num_variables = 3 * total_num_vertices; % number of unknowns (3 force components per vertex)
num_constr = 5 * total_num_vertices;    % number of constraint: simplified friction cone + non negativity of vertical force

% fill the optimization matrix
obj.A = zeros(num_constr, num_variables);
obj.Ax_Ub = zeros(num_constr, 1);
obj.Ax_Lb = -1e20 + zeros(num_constr, 1);

% Constraint "Fz=0 if no contact" formulated as Aeq x = 0.
% Aeq shall be concatenated with A in the case of OSQP.
obj.Aeq = zeros(total_num_vertices, num_variables);
obj.beq = zeros(total_num_vertices, 1);
obj.ulb = zeros(num_variables, 1);

constr_matrix = [...
    1, 0, -obj.mu; ...% first 4 rows: simplified friction cone
    0, 1, -obj.mu; ...
    -1, 0, -obj.mu; ...
    0, -1, -obj.mu; ...
    0, 0, -1]; ...% non negativity of vertical force
    
% fill a block diagonal matrix with all the constraints
Ac = repmat({constr_matrix}, 1, total_num_vertices); % Repeat Matrix for every vertex as a cell array
obj.A(:,1:num_variables) = blkdiag(Ac{1:total_num_vertices});

% Create an OSQP problem object
if obj.useOSQP
    obj.osqpProb = osqp;
end
end