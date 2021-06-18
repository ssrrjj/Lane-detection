/*
 * test_readpar.cpp
 *
 *  Created on: Jun 1, 2021
 *      Author: aina
 */

#include "lanepar.h"

int main()
{
    LanePar par("lanedet.par");
    std::cout<<"plane_dist_threshold: "<<par.plane_dist_threshold<<std::endl;
}


