s.x = s.A*s.x + s.B*s.u;
s.P = s.A * s.P * s.A' + s.Q;
% Compute Kalman gain factor:
K = s.P*s.H'*inv(s.H*s.P*s.H'+s.R);
% Correction based on observation:
s.x = s.x + K*(s.z-s.H*s.x);
s.P = s.P - K*s.H*s.P;