#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if(estimations.at(0).size()!=rmse.size()) {
        cout << "Size Mismatch - Error ! \n";
        return rmse;
    }

    VectorXd temp(4), sum(4), mean(4), res(4);
    temp << 0,0,0,0;
    sum << 0,0,0,0;

    for(int i=0; i<estimations.size(); i++)
    {
        res = estimations[i]-ground_truth[i];
        temp = res.array()*res.array();
        sum += temp;
    }

    mean = sum/estimations.size();
    rmse = mean.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3, 4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float t1 = px*px + py*py;   //px^2 + py^2

    if(t1==0) {
        cout << "CalculateJacobian() -- Divide by zero - Error ! \n";
        return Hj;
    }

    float t2 = pow(t1, 0.5);    //sqrt(px^2 + py^2)
    float t3 = pow(t2, 3);      //(sqrt(px^2 + py^2))^3
    float t4 = vx*py - vy*px;
    float t4_2 = vy*px - vx*py;
    float t5 = t4/t3;
    float t6 = t4_2/t3;

    Hj <<   px/t2, py/t2, 0, 0,
            -py/t1, px/t1, 0, 0,
            py*t5, px*t6, px/t2, py/t2;


    return Hj;
}
