function j = ik3001(p)
    dist1 = sqrt(p(1)^2 + p(2)^2);
    
    % checking for underground point
    if p(3) < 0 || p(3) > 295
        error('Error: value out of range');
    end
    
    % checking for out of range point
    if sqrt(dist1^2 + (p(3) - 95)^2) > 200
        error('Error: value out of range');
    end
    
    % J1 calcs
    C1 = p(2) / dist1;
    D1 = p(1) / dist1;
    theta1 = atan2d(C1, D1);
    if theta1 < -90 || theta1 > 90
        error('Error: value out of range');
    end

    % J3 calcs
    C3 = 1 - (p(1)^2 + p(2)^2 + (p(3)-95)^2)/(2*100^2);
    D3 = sqrt(1 - C3^2);
    theta3 = atan2d(C3, D3);
    if theta3 < -90 || theta3 > 90
        theta3 = 90 - atan2d(C3, -D3);
    end

    % J2 calcs
    Db = sqrt(.5-.5*sind(theta3));
    Cb = sqrt(.5+.5*sind(theta3));
    b = atan2d(Cb,Db);
    a = atan2d(p(3) - 95, dist1);
    
    theta2 = 90 - (a + b);
    if theta2 < -90 || theta1 > 90
        error('Error: value out of range');
    end
    
    j = [theta1;theta2;theta3];
end