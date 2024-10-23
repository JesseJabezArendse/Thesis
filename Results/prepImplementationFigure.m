% Assuming All_Vs is 1x500x38, we'll treat the second dimension (500) as Voltage, and the third (38) as Buffer Index

figure
% Loop over the Z dimension to plot each slice as a line along the Z-axis
for z = 1:size(All_Vs, 3)
    % Plot the 500 values along the Y axis (Voltage) and stack along Z axis (Buffer Index)
    plot3(repmat(z, 1, size(All_Vs, 2)), 1:size(All_Vs, 2), (squeeze(All_Vs(1, :, z)) - min(squeeze(All_Vs(1, :, z)))), 'LineWidth', 2);
    hold on;
end
azimuth = 45;     % Azimuth angle in degrees
elevation = 30;   % Elevation angle in degrees

% Set the view to the specified azimuth and elevation
view(azimuth, elevation);

xlabel('Frequency Index');
ylabel('Time [sample index / frequency]');
zlabel('Voltage');
title('Stacked Voltage Plots');
hold off;


R_cable = 0.16;
C_Cable = 3.66e-9 ;
L_Cable = 23e-6 + 0.75e-3;

R_test = (1.2)   ;
C_test = (5*4700e-6) ;
L_test = (0)     ;

R_test2 = (2.2)   ;
C_test2 = (200e-6) ;
L_test2 = (0)     ;


index = 0;
frequencies = linspace(0.1,5000,1000000);
for w = frequencies
    index = index + 1;
    
    impedance_cable = R_cable + 1i*w*L_Cable;

    impedance_UT1 = R_test / (1 + 1i*w*R_test*C_test);
    impedance_UT2 = R_test2 / (1 + 1i*w*R_test2*C_test2);

    impedance_batt = Batt.R_0 + 1i*w*Batt.L_R +  (Batt.R_1 / (1 + 1i*w*Batt.R_1*Batt.C_1)) + (Batt.R_2 / (1 + 1i*w*Batt.R_2*Batt.C_2)); 
    
    impedance =  impedance_UT1 + impedance_UT2 + impedance_cable;
    All_Expected_Re(index) = real(impedance);
    All_Expected_Im(index) = -imag(impedance);
end

figure
% Create scatter plot
scatter(All_Expected_Re, All_Expected_Im, 'LineWidth', 0.8, 'DisplayName', 'Expected');
hold on


Real_Z = monitored.Re_Z.Data(:,:,end);
Imag_Z = monitored.Im_Z.Data(:,:,end);



% Create a color array based on the indices
colors = 1:length(Real_Z);  % Use indices for color mapping

% Create scatter plot
scatter(Real_Z, Imag_Z, 10, 'LineWidth', 1, 'DisplayName', 'Implementation');
% Add labels and title
xlabel('Real-axis');
ylabel('Imaginary-axis');
title('Nyquist Plot');
hold on

legend