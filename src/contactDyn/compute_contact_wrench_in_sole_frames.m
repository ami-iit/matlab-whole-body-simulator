 function [wrench_left_foot, wrench_right_foot] = compute_contact_wrench_in_sole_frames(contact_forces, H_LFOOT, H_RFOOT, vertex)
% trasforms the pure forces on foot vertices in wrench in sole frames
R_LFOOT = H_LFOOT(1:3,1:3);
R_RFOOT = H_RFOOT(1:3,1:3);

wrench_left_foot = zeros(6,1);
wrench_right_foot = zeros(6,1);

contact_forces_left = contact_forces(1:12);
contact_forces_right = contact_forces(13:24);

for i=1:4
    j = (i-1)*3 + 1;
    wrench_left_foot(1:3)  = wrench_left_foot(1:3) + R_LFOOT'*contact_forces_left(j:j+2);
    wrench_left_foot(4:6)  = wrench_left_foot(4:6) - wbc.skew(vertex(:,i))*(R_LFOOT'*contact_forces_left(j:j+2));
    wrench_right_foot(1:3) = wrench_right_foot(1:3) + R_RFOOT'*contact_forces_right(j:j+2);
    wrench_right_foot(4:6) = wrench_right_foot(4:6) - wbc.skew(vertex(:,i))*(R_RFOOT'*contact_forces_left(j:j+2));
end
end