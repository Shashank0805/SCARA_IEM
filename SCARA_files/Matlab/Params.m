
A_mat=[0 0 1 0;
       0 0 0 1;
       0 0 0 0;
       0 0 0 0];

B_mat=[0 0;
      0 0;
      1 0;
      0 1];

% theta2' and theta1' represents derivatives of theta2 and theta1
M_matrix=[8.42179+0.009*cos(theta2), 5.6086 + 0.0045*cos(theta2), 0;
             (1/4)*(5.1064 + 0.018*cos(theta2)), 1.2766, 0;
             0 , 0, 0.0025
    ];

G_matrix=[0;
          0;
          0.04905];

C_matrix=[1.0-0.0045*sin(theta2*theta1'), (-0.0045*sin(theta2*theta1'))-(0.0045*sin(theta2*theta2')), 0;
          0.0045 *sin(theta2*theta1'), 1, 0;
          0, 0, 1
    ];


