#include "Hubo_Tech.h"
#include <Fastrak.h>
    
int main( int argc, char **argv )
{
    Hubo_Tech hubo;
    
    Fastrak fastrak;
    fastrak.initFastrak();

    Vector6d q; q.setZero();
    
    
    Eigen::Vector3d pS, pE, pW, pWP, pVW, pVWP, pAX, iHat, jHat, kHat;
    iHat << 1.0, 0.0, 0.0;
    jHat << 0.0, 1.0, 0.0;
    kHat << 0.0, 0.0, 1.0;

    Eigen::Quaterniond quat;
    Eigen::Isometry3d Tax;
    double ptime, dt;
    int i=0, imax=2000;
    
    hubo.update();
    
    while(true)
    {
        hubo.update();
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();
        
//        if( dt > 0 )
        {
            i++; if(i>imax) i=0;
            
            fastrak.getPose( pS, quat, 1, true );
            fastrak.getPose( pE, quat, 2, false );
            fastrak.getPose( pW, quat, 3, false );
            pW -= pE;
            pE -= pS;

            q(0) = atan2( (-pE.dot(kHat)), (pE.dot(iHat)) );
            q(1) = atan2( ( pE.dot(jHat)), (pE.dot(iHat)) );

            q(3) = asin( ( (pE.cross(pW)).norm() )/(pE.norm()*pW.norm()) );


            pWP = pW - (pW.dot(pE)/pE.norm())*pE;
            pAX = pE.cross(iHat);
            Tax = Eigen::AngleAxisd(q(3),pAX);
            pVW = Tax*pE;
            pVWP = pVW - (pVW.dot(pE)/pE.norm())*pE ;
            q(2) = asin( ((pVWP.cross(pWP)).norm() )/(pVWP.norm()*pWP.norm()) );
           
            if( i == imax )
                std::cout << q.transpose() << std::endl;
//                std::cout << "pS:" << pS.transpose() <<  "\tpE:" << pE.transpose() << "\tpW:" << pW.transpose() << std::endl;
        }

    }
}
