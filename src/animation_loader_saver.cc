#include "cloth_geometry.h"
#include "json.hpp"
#include <iostream>
#include <fstream>
using namespace std;
/*
 * Placeholder functions for Milestone 2
 */
using json = nlohmann::json;
void Mesh::saveAnimationTo(const std::string& fn)
{   printf("fightjusodf %d\n", keyframes.size());
    
    if(keyframes.size() > 0){
         

        json overall;
        
        for(int x = 0; x < keyframes.size(); x++){
            json keyson;
            for(glm::fquat rotor : keyframes.at(x).rel_rot){
                json quatson;
                quatson.push_back(rotor.x);
                quatson.push_back(rotor.y);
                quatson.push_back(rotor.z);
                quatson.push_back(rotor.w);
                keyson.push_back(quatson);
            }
            overall.push_back(keyson);

        }
        ofstream myfile;
        myfile.open (fn);
    myfile << overall;
    myfile.close();



     
       
    
   
}


}

void Mesh::loadAnimationFrom(const std::string& fn)
{
    printf("fightFDFSDFSDFSDFSDFSDFjusodf %d\n", keyframes.size());
    std::ifstream ifs(fn);
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
                (std::istreambuf_iterator<char>()    ) );
    json framerinos = json::parse(content);
    for(int q =0; q < framerinos.size(); q++){
        KeyFrame keygan;
        for(int r = 0; r < framerinos[q].size(); r++){
            glm::fquat quatimoto = glm::fquat(framerinos[q][r][3], framerinos[q][r][0], framerinos[q][r][1], framerinos[q][r][2]);
            keygan.rel_rot.push_back(quatimoto);
        }
        keyframes.push_back(keygan);
    }
    printf("test the sizes of first level and second level %d %d\n", framerinos.size(), framerinos[0].size());
}
