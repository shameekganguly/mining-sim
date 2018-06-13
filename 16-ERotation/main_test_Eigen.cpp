
#include <iostream>
#include <Eigen/Dense>

int main()
{
    Eigen::MatrixXd m(2,2), m2(2,2);
    m(0,0) = 3;
    m(1,0) = 2;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);

    m << 3, -1,
         2,  1;

    std::cout << m << std::endl;
    m = -m;
    std::cout << m << std::endl;

    Eigen::VectorXd n(2), x(2), n2(4);
    n(0) = 3;
    n(1) = 2;
    std::cout << n << std::endl;
    x = m*n;
    std::cout << x << std::endl;


    double x1 = x(0);
    std::cout << x1 << std::endl;

    m2 << n.transpose(), x.transpose();
    std::cout << m2 << std::endl;

    n2 << n, x;
    std::cout << n2.transpose() << std::endl;

    std::cout << 2*n2 << std::endl;

    Eigen::MatrixXd Ep(3,9);
    Eigen::VectorXd DC(9), angularVel(3);
    // Ep << 1, 1, 1, 1, 1, 1, 1, 1, 1,
    //       1, 1, 1, 1, 1, 1, 1, 1, 1,
    //       1, 1, 1, 1, 1, 1, 1, 1, 1;

    Ep << -1.27921e-19, -0.0385037, 0.100685, -4.97302e-19, -0.170987, 0.457323, -1.12687e-17, -0.468274, -0.175267,
          0.0385037,  8.26566e-19,-0.488242,0.170987, 3.67061e-18, 0.107794, 0.468274, 1.00525e-17, 0.000785398,
          -0.100685, 0.488242, -6.98645e-19, -0.457323, -0.107794, -3.17331e-18, 0.175267, -0.000785398, 1.21616e-18;
    //DC << 1, 1, 1, 1, 1, 1, 1, 1, 1;
    DC <<   0.976483, 0.201371, 0.0770074, -0.215587,  0.914645,   0.341974, -0.0015708,  -0.350534,   0.936549;

    angularVel = Ep * DC;

    std::cout << angularVel << std::endl;


}