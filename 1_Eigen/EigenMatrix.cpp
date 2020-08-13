#include <iostream>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

int main(int argc, char**argv)
{
    //Fixed size matrix
    Eigen::Matrix<float,2,3> matrix_23; //2 by 3
    Eigen::Vector3d v_3d; // 3 by 1
    Eigen::Matrix<float,3,1> vd_3d; // 3 by 1

    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); // 3 by 3

    //Dynamic matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    Eigen::MatrixXd matrix_x;

    //Initialize
    matrix_23 << 1,2,3,4,5,6;

    //Print the matrix
    std::cout << "matrix33 : \n" << matrix_33 << std::endl;

    // Print by index
    std::cout << "print value by index:\n" ;
    for (int i = 0; i < matrix_23.cols(); i++)
    {
        for (int j = 0; j < matrix_33.rows(); j++)
        {
            std::cout << matrix_33(i,j) << " ";
        } 
        std::cout << std::endl;
    }

    //Multiplication
    v_3d << 3,2,1;
    vd_3d << 4,5,6;

    //Eigen::Matrix<double,2,1> result = matrix_23 * v_3d;
    Eigen::Matrix<double,2,1> result = matrix_23.cast<double>() * v_3d;

    std::cout << "[1,2,3;4,5,6]*[4,5,6]:" << result.transpose() << std::endl;

    Eigen::Matrix<float,2,1> result2 = matrix_23 * vd_3d;
    std::cout << "[1,2,3;4,5,6]*[4,5,6]:" << result2.transpose() << std::endl;


    //Eigen::Matrix<float,2,3> result_wrong_dimension = matrix_23 * vd_3d;



    matrix_33 = Eigen::Matrix3d::Random();
    std::cout << "random matrix: \n" << matrix_33 << std::endl;
    std::cout << "transpose: \n" << matrix_33.transpose() << std::endl;
    std::cout << "sum: " << matrix_33.sum() << std::endl;
    std::cout << "trace: " << matrix_33.trace() << std::endl;
    std::cout << "10 times: \n" << 10 * matrix_33 << std::endl;
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;
    std::cout << "det: " << matrix_33.determinant() << std::endl;

    //Eigen solver
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    std::cout << "Eigen value: " << eigen_solver.eigenvalues().transpose() << std::endl;
    std::cout << "Eigen vector: \n" << eigen_solver.eigenvectors() << std::endl;

    // Nx = v
    Eigen::Matrix<double, 50, 50> matrix_NN = Eigen::MatrixXd::Random(50,50);
    matrix_NN = matrix_NN.transpose() * matrix_NN;
    Eigen::Matrix<double,50,1> v_Nd = Eigen::MatrixXd::Random(50,1);
    
    //Normal inverse
    clock_t time_st = clock();
    Eigen::Matrix<double,50,1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "Time of inverse : " << 1000*(clock() - time_st)/(double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    //QR
    time_st = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    std::cout << "Time of QR: " << 1000*(clock() - time_st)/(double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    //cholesky. positive definite matrix
    time_st = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    std::cout << "Time of cholesky: " << 1000*(clock() - time_st)/(double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    return 0;

}