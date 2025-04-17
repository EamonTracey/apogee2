%% ACS_CFD_Interpolation.m

clear all
clc

%% Read in data
og_data = table2array(readtable("CDR_CFD_2.csv"));

flap_angle = og_data(:,1);
mach_number = og_data(:,2);
drag_force = og_data(:,3);

 mach_number0 = zeros; 
    drag_force0 = zeros; 
    mach_number45 = zeros; 
    drag_force45 = zeros; 

for ix = 1:flap_angle(:) 
    if flap_angle(ix) == 0  
        mach_number0(ix) = mach_number(ix); 
        drag_force0(ix) = drag_force(ix);
    else
        mach_number45(ix) = mach_number(ix); 
        drag_force45(ix) = drag_force(ix);
    end 
end 



for ix = 1:flap_angle(:)
    while flap_angle == 0
    


    end 
end 