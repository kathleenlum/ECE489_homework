function pts = plotCRS(q)
    % plotCRS Calculates and plots the forward kinematics of CRS robot arm.
    % Input: 
    %   q - 3x1 column vector of joint angles [theta1; theta2; theta3] in radians
    % Output: 
    %   pts - 4x4 matrix containing the [X; Y; Z; 1] coordinates of each joint

    % Extract joint angles from the input vector
    theta1 = q(1, 1);
    theta2 = q(2, 1);
    theta3 = q(3, 1);

    % --- Denavit-Hartenberg (DH) Parameters ---
    % Physical dimensions of the CRS robot (in meters)
    d1 = 0.254;  a1 = 0;      alpha1 = -pi/2;
    d2 = 0;      a2 = 0.254;  alpha2 = 0;
    d3 = 0;      a3 = 0.254;  alpha3 = 0;

    % --- Link 1: Frame 0 to Frame 1 ---
    R_z1 = [cos(theta1), -sin(theta1), 0, 0;
            sin(theta1),  cos(theta1), 0, 0;
            0,            0,           1, 0;
            0,            0,           0, 1];
             
    T_z1 = [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, d1;
            0, 0, 0, 1];
             
    T_x1 = [1, 0, 0, a1;
            0, 1, 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 1];
             
    R_x1 = [1, 0,           0,            0;
            0, cos(alpha1), -sin(alpha1), 0;
            0, sin(alpha1),  cos(alpha1), 0;
            0, 0,           0,            1];
                 
    H0_1 = R_z1 * T_z1 * T_x1 * R_x1;

    % --- Link 2: Frame 1 to Frame 2 ---

    R_z2 = [cos(theta2), -sin(theta2), 0, 0;
            sin(theta2),  cos(theta2), 0, 0;
            0,            0,           1, 0;
            0,            0,           0, 1];
             
    T_z2 = [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, d2;
            0, 0, 0, 1];
             
    T_x2 = [1, 0, 0, a2;
            0, 1, 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 1];
             
    R_x2 = [1, 0,           0,            0;
            0, cos(alpha2), -sin(alpha2), 0;
            0, sin(alpha2),  cos(alpha2), 0;
            0, 0,           0,            1];
                 
    H1_2 = R_z2 * T_z2 * T_x2 * R_x2;

    % --- Link 3: Frame 2 to Frame 3 (End-Effector) ---
    R_z3 = [cos(theta3), -sin(theta3), 0, 0;
            sin(theta3),  cos(theta3), 0, 0;
            0,            0,           1, 0;
            0,            0,           0, 1];
             
    T_z3 = [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, d3;
            0, 0, 0, 1];
             
    T_x3 = [1, 0, 0, a3;
            0, 1, 0, 0;
            0, 0, 1, 0;
            0, 0, 0, 1];
             
    R_x3 = [1, 0,           0,            0;
            0, cos(alpha3), -sin(alpha3), 0;
            0, sin(alpha3),  cos(alpha3), 0;
            0, 0,           0,            1];
                 
    H2_3 = R_z3 * T_z3 * T_x3 * R_x3;

    % Compute the transformations relative to Frame 0
    H0_2 = H0_1 * H1_2;          % Base to Link 2
    H0_3 = H0_1 * H1_2 * H2_3;   % Base to End-Effector

    % --- Extracting Joint Coordinates ---
    % Multiply the global transformation matrices by the origin vector [0;0;0;1] 
    % to find the absolute 3D Cartesian coordinates of each joint.
    O0_0 = [0; 0; 0; 1];           % Base origin (fixed at 0,0,0)
    O0_1 = H0_1 * [0; 0; 0; 1];    % Frame 1 origin w.r.t world frame
    O0_2 = H0_2 * [0; 0; 0; 1];    % Frame 2 origin w.r.t world frame
    P0   = H0_3 * [0; 0; 0; 1];    % End-effector position w.r.t world frame

    % Combine points into an array (columns are joints, rows are X, Y, Z, 1)
    pts = [O0_0, O0_1, O0_2, P0];

    % =========================================================================
    % --- Plotting the Robot Arm ---
    % =========================================================================
    % Extract the X, Y, and Z rows from the 'pts' matrix
    X = pts(1, :);
    Y = pts(2, :);
    Z = pts(3, :);

    % Create the 3D plot
    figure;
    plot3(X, Y, Z, '-ko', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    grid on;
    
    % Formatting the plot for better visualization
    xlabel('X-axis (m)');
    ylabel('Y-axis (m)');
    zlabel('Z-axis (m)');
    title('CRS Robot Arm Forward Kinematics');
    axis equal; % Ensures the lengths aren't visually distorted
    
    % Set axis limits based on max reach (approx 0.8m)
    xlim([-0.8 0.8]); ylim([-0.8 0.8]); zlim([0 0.8]); 
end