function [fitresult, gof] = createFittedNyquist(Real_Z, Imag_Z)

%% Fit: 'nyquist'.
[xData, yData] = prepareCurveData( Real_Z, Imag_Z );

% Set up fittype and options.
ft = fittype( 'poly6' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );
opts.Normalize = 'on';
opts.Robust = 'Bisquare';

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'nyquist' );
h = plot( fitresult, xData, yData, 'predobs', 0.9 );
legend( h, 'Imag_Z vs. Real_Z', 'nyquist', 'Lower bounds (nyquist)', 'Upper bounds (nyquist)', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'Real_Z', 'Interpreter', 'none' );
ylabel( 'Imag_Z', 'Interpreter', 'none' );
grid on


