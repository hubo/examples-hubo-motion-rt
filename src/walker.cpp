#include <Hubo_Tech.h>


int main(int argc, char **argv)
{
    Hubo_Tech hubo;

    Vector6d q, qvel;
    q.setZero();
    q(3) = 0.3;
    q(2) = -0.15;
    q(4) = -0.15;

    Eigen::Vector3d vel;
    vel << 1, 0, 0;

    hubo.hipVelocityIK( qvel, vel, RIGHT, q );

    std::cout << qvel.transpose() << std::endl;




    q(3) = 0.25;
    q(2) = -0.10;
    q(4) = -0.15;


    hubo.hipVelocityIK( qvel, vel, RIGHT, q );

    std::cout << qvel.transpose() << std::endl;


    q(3) = 0.16;
    q(2) = -0.01;
    q(4) = -0.15;


    hubo.hipVelocityIK( qvel, vel, RIGHT, q );

    std::cout << qvel.transpose() << std::endl;




    q(3) = 0.3;
    q(2) = -0.45;
    q(4) = 0.15;


    hubo.hipVelocityIK( qvel, vel, RIGHT, q );

    std::cout << qvel.transpose() << std::endl;



}
