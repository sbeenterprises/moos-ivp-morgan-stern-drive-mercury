/************************************************************/
/*    NAME: Eduardo Eiras                                            */
/*    ORGN: Rio de Janeiro                             */
/*    FILE: PIDParameterReader.h                                      */
/*    DATE: September, 2023                           */
/************************************************************/


#ifndef PIDPARAMETERREADER_H
#define PIDPARAMETERREADER_H

#include "lanchaPID.h" 

std::vector<double> readPIDParameters(const std::string& filename);

#endif
