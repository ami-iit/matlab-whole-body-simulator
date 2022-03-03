function links_in_contact = getFeetContactState(obj, num_inContact_frames)

links_in_contact = false(1,num_inContact_frames);
for counter = 1 : num_inContact_frames
    links_in_contact(counter) = any(obj.is_in_contact(1+4*(counter-1):4*counter));
end

end