function p = fk3001(theta1, theta2, theta3)
    % DH Transformation Matrices
    T12 = [cosd(theta1), 0, -sind(theta1), 0; sind(theta1), 0, cosd(theta1), 0; 0, -1, 0, 95; 0, 0, 0 1];
    T23 = [cosd(theta2-90), -sind(theta2-90), 0, (100*cosd(theta2-90)); sind(theta2-90), cosd(theta2-90), 0, (100*sind(theta2-90)); 0, 0, 1, 0; 0, 0, 0 1];
    T34 = [cosd(theta3+90), -sind(theta3+90), 0, (100*cosd(theta3+90)); sind(theta3+90), cosd(theta3+90), 0, (100*sind(theta3+90)); 0, 0, 1, 0; 0, 0, 0 1];

    T13 = T12*T23;
    T14 = T13*T34;
    
    % 3x3 matrix of joint positions
    p = [T12(1:3,4), T13(1:3,4), T14(1:3,4)];
    
    % 1x3 matrix representing end effector position
    % p = T14(1:3,4);
end