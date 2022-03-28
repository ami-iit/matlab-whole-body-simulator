function prepare_foot_print (obj, num_in_contact_frames, foot_print)

    %     PREPARE_FOOT_PRINT : This function writes the local coordinates of the foot vertices in a cell array format
    % 
    %     PROCEDURE: There are three possibilities:
    %             1) All the feet have the same foot prints and the foot_print is a matrix representing the coordinates of the foot print,
    %             2) All the feet have the same foot print and the foot_print is a cell with only one element which is matrix representing the coordinates of the foot print,
    %             3) The foot_print is a cell array with m elements where each element is a matrix representing the coordinates of the foot print for the corresponding foot.
    % 
    %     **FORMAT**: Contact_object.prepare_foot_print (num_in_contact_frames, num_vertices, foot_print)
    % 
    %     **INPUT:**
    %             - num_in_contact_frames:  [SCALAR]                    The number of the feet (i.e. the links interacting with the ground
    %             - foot_print:             [(3m) x k] or [CELL ARRAY]  The coordinates of the foot prints of the feet represented in the corresponding foot frame
    % 
    %     **OUTPUT:**
    % 
    %     **AUTHORS:**  Venus Pasandi
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

    
    if iscell(foot_print) && (length(foot_print) == num_in_contact_frames)
        
        obj.foot_print = foot_print;
        
    elseif iscell(foot_print)
        
        obj.foot_print = repmat(foot_print(1),num_in_contact_frames,1);
        
    else
        
        obj.foot_print = repmat({foot_print(1:3,1:obj.num_vertices)},num_in_contact_frames,1);
        
    end
end