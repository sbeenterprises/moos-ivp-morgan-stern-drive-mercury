/************************************************************/
/*    NAME: Eduardo Eiras                                            */
/*    ORGN: Rio de Janeiro                             */
/*    FILE: PIDParameterReader.cpp                                        */
/*    DATE: September, 2023                           */
/************************************************************/

#include "lanchaPID.h"
#include "PIDParameterReader.h"
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

std::vector<double> readPIDParameters(const std::string& filename) {
    std::vector<double> parameters; // Vector to store KP, KI, KD
    double KP = 0.0, KI = 0.0, KD = 0.0, LOWER_BOUND = 0.0, UPPER_BOUND = 0.0;

    // Open the file for reading
    std::ifstream file(filename);

    if (file.is_open()) {
        std::string line;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string paramName;
            char equalSign;
            double paramValue;

            // Attempt to parse each line as "paramName = paramValue"
            if (iss >> paramName >> equalSign >> paramValue) {
                if (equalSign == '=' && !iss.fail()) {
                    // Successfully parsed a parameter
                    if (paramName == "KP") {
                        KP = paramValue;
                    } else if (paramName == "KI") {
                        KI = paramValue;
                    } else if (paramName == "KD") {
                        KD = paramValue;
                    } else if (paramName == "UPPER_BOUND") {
                        UPPER_BOUND = paramValue;
                    } else if (paramName == "LOWER_BOUND") {
                        LOWER_BOUND = paramValue;
                    }
                }
            }
        }

        // Close the file
        file.close();

        // Store the parameters in the vector
        parameters.push_back(KP);
        parameters.push_back(KI);
        parameters.push_back(KD);
        parameters.push_back(UPPER_BOUND);
        parameters.push_back(LOWER_BOUND);
    } else {
        std::cerr << "PIDParameterReader Error: Unable to open the file " << filename << std::endl;
    }

    return parameters;
}