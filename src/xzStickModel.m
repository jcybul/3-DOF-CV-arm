function ee = xzstickModel(q)
    % adding the base joint to the position matrix
    p0 = zeros(3,1);
    p = [p0 fk3001(q(1), q(2), q(3))];
    
    % outputting the end-effector position
    ee = p(:,4);
    
    % plotting the links
    plot3(p(1,:),p(2,:),p(3,:), '-o', 'LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);grid on;hold on;
    
    % plotting the joints
    text(p(1,4),p(2,4),p(3,4),['  (', num2str(p(1,4),3), ', ', num2str(p(2,4),3),', ', num2str(p(3,4),3), ')']);
    
    %plotting end effector velocity
    
    % graph axis setup
    xlabel('X Axis');
    ylabel('Y Axis');
    zlabel('Z Axis');
    axis([0, 200, -200, 200, 0, 200]);
    %view(0,0);
    hold off;
end

