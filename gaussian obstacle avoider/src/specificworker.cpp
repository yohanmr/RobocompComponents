/*
 *    Copyright (C) 2017 by Yohan M R
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include "math.h"
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}
float mincost=1;
float minangle;

/**
* \brief Default destructor
*/

SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);
	

	return true;
}
float SpecificWorker::getcost(float x, float y, float angle, float fvar, float svar, float rvar) //Gaussian function to calculate cost
{
    float alpha= atan2(y,x) - angle + 1.57;
    alpha= atan2(sin(alpha), cos(alpha));
    float var= ((alpha<=0) ? rvar:fvar);
    float a=(cos(angle)*cos(angle)/(2*var*var))+(sin(angle)*sin(angle)/(2*svar*svar));
    float b=(sin(2*angle)/(4*var*var))+(sin(2*angle)/(4*svar*svar));
    float c=(sin(angle)*sin(angle)/(2*var*var))+(cos(angle)*cos(angle)/(2*svar*svar));
    return exp(-((a*x*x)+(2*b*x*y)+(c*y*y)));

}
int ci=1;
float x,y,angle,fvar,svar,rvar;

void SpecificWorker::compute( )
{



    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;


        float dist=ldata.front().dist;
        if(ci<=8) //Checking the cost in 8 separate directions at an angle of 45degrees
        {
        switch(ci)
        {
            //X and Y are calculated wrt to the bot at the origin and (x,y) being the co-ordinates of the obstacle.
            case 1:{
                    ::x=0; ::y=dist;
                    break;
                    }
            case 2: {
                    ::x=sqrt(dist/2);
                    ::y=sqrt(dist/2);
                    break;
                    }
           case 3: {
                    ::x=dist; ::y=0;
                    break;
                    }
           case 4: {
                    ::x=sqrt(dist/2);
                    ::y=-sqrt(dist/2);
                    break;
                    }
           case 5: {
                    ::x=0; ::y=-(dist);
                    break;
                    }
           case 6: {
                    ::x=-sqrt(dist/2);
                    ::y=-sqrt(dist/2);
                    break;
                    }
           case 7: {
                    ::x=-(dist); ::y=0;
                    break;
                    }
           case 8:{
                   ::x=-sqrt(dist/2);
                   ::y=sqrt(dist/2);
                   }
        default: break;
        }
        if(mincost>SpecificWorker::getcost(::x,::y,0,0.2,(0.2/6),(0.2/6)))
        {
            mincost=SpecificWorker::getcost(::x,::y,0,0.2,(0.2/6),(0.2/6));
            minangle=1.57*(0.25*(ci-1)); // Calculating angle at which it should move.
        }
        differentialrobot_proxy->setSpeedBase(0, 3.14/4*100);
        usleep(10000);
        ci++;
        }
        else //Moving in the direction of minimum cost
        {
            ci=1;
            mincost=1;
            differentialrobot_proxy->setSpeedBase(0, minangle*10);
            usleep(100000);
            differentialrobot_proxy->setSpeedBase(200, 0);
            usleep (500000);
        }







    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }

}





