#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include<opencv2/opencv.hpp>

// function y = exp(ax^2 + bx + c) + w
int main(int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0; // Real parameter
    double ae = 2.0, be = 0.0, ce = -2.0; // Estimated parameter, initialize

    int data_size = 100;

    //---------Noise parameter-----------//
    double w_sigma = 1.0;
    double inv_sigma = 1.0/w_sigma; // ?
    //---------Noise parameter-----------//

    cv::RNG rng; //random number

    // Generate data
    std::vector<double> x_data, y_data;
    for (int i = 0; i < data_size; i++)
    {
        double x = i / data_size;
        x_data.push_back(x);
        y_data.push_back(std::exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma*w_sigma));
    }
    // Generate data

    //--------------iteration cbegin--------------//
    int iterations = 100;
    double cost, last_cost;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    for (int iter = 0; iter < iterations; iter++)
    {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();

        cost = 0;

        for (int i = 0; i < data_size; i++)
        {
            double xi = x_data[i];
            double yi = y_data[i];

            double error = yi - std::exp(ae * xi * xi + be*xi + ce);

            // Calculate Jacob
            Eigen::Vector3d J;
            J[0] = xi * xi * std::exp(ae * xi * xi + be*xi + ce);// x^2e^(ax^2 + bx + c)
            J[1] = // xe^(ax^2 + bx + c)
            J[2] = // e^(ax^2 + bx + c)
        }
    }
}   