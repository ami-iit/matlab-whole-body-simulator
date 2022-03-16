function links_in_contact = get_feet_contact_state(obj, num_in_contact_frames)

    %     LINKS_IN_CONTACT : This function determines if each foot at the current time instance is in contact with the ground or not.
    % 
    %     PROCEDURE: A foot is in contact with the ground if one of its vertices is in contact with the ground.
    % 
    %     **FORMAT**: links_in_contact = Contact_obj.get_feet_contact_state(num_in_contact_frames)
    % 
    %     **INPUT:**
    %                 - num_in_contact_frames: [SCALAR] The number of the feet
    % 
    %     **OUTPUT:**
    %                 - links_in_contact: [m x 1] The contact status of the feet
    % 
    %     **AUTHORS:** Venus Pasandi, Nuno Guedelha
    % 
    %     all authors are with the Italian Istitute of Technology (IIT)
    %     email: name.surname@iit.it
    % 
    %     PLACE AND DATE: <Genoa, March 2022>

% ------------- INITIALIZATION ----------------------------
links_in_contact = false(1, num_in_contact_frames);

% ------------- MAIN COMPUTATION --------------------------
for counter = 1 : num_in_contact_frames
    index_vertices = obj.num_vertices*(counter-1)+1 : obj.num_vertices*counter;
    links_in_contact(counter) = any(obj.is_in_contact(index_vertices));
end

end