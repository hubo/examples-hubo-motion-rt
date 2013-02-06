#include <Hubo_Tech.h>

int main( int argc, char **argv )
{
    Hubo_Tech hubo("tech-handshake");

    Vector6d ql, ql0, qr, qr0;
    double time, dt;
    double springGain = 0.8;
    double compGain   = 0.7;

    int i=0, imax = 30;

    ql0 << -20.0/180.0*M_PI, 0, 0, -M_PI/2.0+20.0/180.0*M_PI, 0.0, 0.0;
    qr0 = ql0;
    ql = ql0;
    qr = qr0;

    hubo.setLeftArmAngles( ql0, true );
    hubo.setRightArmAngles( qr0, true );

    hubo.update();
    time = hubo.getTime();

    while(!daemon_sig_quit)
    {
        hubo.update();
        dt = hubo.getTime() - time;
        time = hubo.getTime();
        
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            if(i == imax)
                std::cout << hubo.getRightHandMx() << "\t" << hubo.getRightHandMy() << std::endl;
            
            hubo.setJointVelocity( REB, compGain*hubo.getRightHandMx()
                                        + springGain*(qr0(3)-hubo.getJointAngle(REB)) );

            hubo.setJointVelocity( RSY, -compGain*hubo.getRightHandMy()
                                        + springGain*(qr0(2)-hubo.getJointAngle(RSY)) );

            
            hubo.setJointVelocity( LEB, -compGain*hubo.getLeftHandMx()
                                        + springGain*(ql0(3)-hubo.getJointAngle(LEB)) );

            hubo.setJointVelocity( LSY, compGain*hubo.getLeftHandMy()
                                        + springGain*(ql0(2)-hubo.getJointAngle(LSY)) );
            
            hubo.sendControls();
        }
        
        
    }



}
