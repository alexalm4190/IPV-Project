%%

clear;
close all;

%%

base_data_dir='C:\Users\Alexandre\Documents\Alexandre\IST\PIV\Datasets\lab2\';
d1=dir([base_data_dir 'depth1*']);
d2=dir([base_data_dir 'depth2*']);
r1=dir([base_data_dir 'rgb_image1*']);
r2=dir([base_data_dir 'rgb_image2*']);

minlen = length(d1(1).name);

%%

for i=1:length(d1)
    if( length(d1(i).name) == minlen + 1 )
        garbage = '_00';
    elseif( length(d1(i).name) == minlen + 2 )
        garbage = '_0';
    else
        garbage = '_000';
    end    
        
    name = strsplit(d1(i).name, '_');
    new_name = char(strcat(name(1), garbage, name(2)));
    dos(sprintf('rename "%s" "%s"', strcat(base_data_dir, d1(i).name), new_name));
    
    name = strsplit(d2(i).name, '_');
    new_name = char(strcat(name(1), garbage, name(2)));
    dos(sprintf('rename "%s" "%s"', strcat(base_data_dir, d2(i).name), new_name));
    
    name = strsplit(r1(i).name, '_');
    new_name = char(strcat(name(1), '_', name(2), garbage, name(3)));
    dos(sprintf('rename "%s" "%s"', strcat(base_data_dir, r1(i).name), new_name));
   
    name = strsplit(r2(i).name, '_');
    new_name = char(strcat(name(1), '_', name(2), garbage, name(3)));
    dos(sprintf('rename "%s" "%s"', strcat(base_data_dir, r2(i).name), new_name));
end    