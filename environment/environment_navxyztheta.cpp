//
// Created by lrm on 22-7-28.
//

#include <cstring>
#include "environment_navxyztheta.h"
#include "sbpl/discrete_space_information/environment.h"
#include <sbpl/utils/2Dgridsearch.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>
#include <cmath>


Environment_NavXYZTheta::Environment_NavXYZTheta() {
    HashTableSize = 0;
    Coord2StateIDHashTable = NULL;
    Coord2StateIDHashTable_lookup = NULL;
}
Environment_NavXYZTheta::~Environment_NavXYZTheta()
{
    SBPL_PRINTF("destroying XYTHETALAT\n");

    // delete the states themselves first
    for (int i = 0; i < (int)StateID2CoordTable.size(); i++) {
        delete StateID2CoordTable.at(i);
        StateID2CoordTable.at(i) = NULL;
    }
    StateID2CoordTable.clear();

    // delete hashtable
    if (Coord2StateIDHashTable != NULL) {
        delete[] Coord2StateIDHashTable;
        Coord2StateIDHashTable = NULL;
    }
    if (Coord2StateIDHashTable_lookup != NULL) {
        delete[] Coord2StateIDHashTable_lookup;
        Coord2StateIDHashTable_lookup = NULL;
    }
}

void Environment_NavXYZTheta::ReadConfiguration(FILE* fCfg)
{
    // read in the configuration of environment and initialize
    // EnvNAVXYZTHETALATCfg structure
    char sTemp[1024], sTemp1[1024];
    int dTemp;
    int x, y;

    // discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    strcpy(sTemp1, "discretization(cells):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format (discretization)" <<
           " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    EnvNAVXYZTHETALATCfg.EnvWidth_c = atoi(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    EnvNAVXYZTHETALATCfg.EnvHeight_c = atoi(sTemp);

    // Scan for optional NumThetaDirs parameter. Check for following obsthresh.
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "NumThetaDirs:");
    if (strcmp(sTemp1, sTemp) != 0) {
        // optional NumThetaDirs not available; default is NAVXYTHETALAT_THETADIRS (16)
        strcpy(sTemp1, "obsthresh:");
        if (strcmp(sTemp1, sTemp) != 0) {
            std::stringstream ss;
            ss << "ERROR: configuration file has incorrect format" <<
               " Expected " << sTemp1 << " got " << sTemp;
            throw SBPL_Exception(ss.str());
        }
        else {
            EnvNAVXYZTHETALATCfg.NumThetaDirs = NAVXYTHETALAT_THETADIRS;
        }
    }
    else {
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            throw SBPL_Exception("ERROR: ran out of env file early (NumThetaDirs)");
        }
        EnvNAVXYZTHETALATCfg.NumThetaDirs = atoi(sTemp);

        //obsthresh:
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            throw SBPL_Exception("ERROR: ran out of env file early (obsthresh)");
        }
        strcpy(sTemp1, "obsthresh:");
        if (strcmp(sTemp1, sTemp) != 0) {
            std::stringstream ss;
            ss << "ERROR: configuration file has incorrect format" <<
               " Expected " << sTemp1 << " got " << sTemp <<
               " see existing examples of env files for the right format of heading";
            throw SBPL_Exception(ss.str());
        }
    }

    // obsthresh
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.obsthresh = atoi(sTemp);
    SBPL_PRINTF("obsthresh = %d\n", EnvNAVXYZTHETALATCfg.obsthresh);

    //cost_inscribed_thresh:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "cost_inscribed_thresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format" <<
           " Expected " << sTemp1 << " got " << sTemp <<
           " see existing examples of env files for the right format of heading";
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.cost_inscribed_thresh = atoi(sTemp);
    SBPL_PRINTF("cost_inscribed_thresh = %d\n", EnvNAVXYZTHETALATCfg.cost_inscribed_thresh);

    //cost_possibly_circumscribed_thresh:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format" <<
           " Expected " << sTemp1 << " got " << sTemp <<
           " see existing examples of env files for the right format of heading";
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
    SBPL_PRINTF("cost_possibly_circumscribed_thresh = %d\n", EnvNAVXYZTHETALATCfg.cost_possibly_circumscribed_thresh);

    //cellsize
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "cellsize(meters):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format" <<
           " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.cellsize_m = atof(sTemp);

    //speeds
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "nominalvel(mpersecs):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format" <<
           " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.nominalvel_mpersecs = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "timetoturn45degsinplace(secs):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format" <<
           " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.timetoturn45degsinplace_secs = atof(sTemp);

    // start(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {  // 读取的是start(meters,rads)
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {   // 读取的是x
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.StartX_c = CONTXY2DISC(atof(sTemp), EnvNAVXYZTHETALATCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.StartY_c = CONTXY2DISC(atof(sTemp), EnvNAVXYZTHETALATCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }

    EnvNAVXYZTHETALATCfg.StartZ_c = atof(sTemp); // z不分精度了
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }

    EnvNAVXYZTHETALATCfg.StartTheta_rad = atof(sTemp);

    if (EnvNAVXYZTHETALATCfg.StartX_c < 0 ||
        EnvNAVXYZTHETALATCfg.StartX_c >= EnvNAVXYZTHETALATCfg.EnvWidth_c)
    {
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }
    if (EnvNAVXYZTHETALATCfg.StartY_c < 0 ||
        EnvNAVXYZTHETALATCfg.StartY_c >= EnvNAVXYZTHETALATCfg.EnvHeight_c)
    {
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }

    // end(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.EndX_c = CONTXY2DISC(atof(sTemp), EnvNAVXYZTHETALATCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.EndY_c = CONTXY2DISC(atof(sTemp), EnvNAVXYZTHETALATCfg.cellsize_m);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.EndZ_c = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvNAVXYZTHETALATCfg.EndTheta_rad = atof(sTemp);

    if (EnvNAVXYZTHETALATCfg.EndX_c < 0 ||
        EnvNAVXYZTHETALATCfg.EndX_c >= EnvNAVXYZTHETALATCfg.EnvWidth_c)
    {
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }
    if (EnvNAVXYZTHETALATCfg.EndY_c < 0 ||
        EnvNAVXYZTHETALATCfg.EndY_c >= EnvNAVXYZTHETALATCfg.EnvHeight_c)
    {
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }
    // z 先不判断了，因为高度没有栅格限制
    // unallocate the 2d environment
    if (EnvNAVXYZTHETALATCfg.Grid2D != NULL) {
        for (x = 0; x < EnvNAVXYZTHETALATCfg.EnvWidth_c; x++) {
            delete[] EnvNAVXYZTHETALATCfg.Grid2D[x];
        }
        delete[] EnvNAVXYZTHETALATCfg.Grid2D;
        EnvNAVXYZTHETALATCfg.Grid2D = NULL;
    }

    // allocate the 2D environment
    EnvNAVXYZTHETALATCfg.Grid2D = new unsigned char*[EnvNAVXYZTHETALATCfg.EnvWidth_c];
    for (x = 0; x < EnvNAVXYZTHETALATCfg.EnvWidth_c; x++) {
        EnvNAVXYZTHETALATCfg.Grid2D[x] = new unsigned char[EnvNAVXYZTHETALATCfg.EnvHeight_c];
    }

    // environment:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    for (y = 0; y < EnvNAVXYZTHETALATCfg.EnvHeight_c; y++) {
        for (x = 0; x < EnvNAVXYZTHETALATCfg.EnvWidth_c; x++) {
            if (fscanf(fCfg, "%d", &dTemp) != 1) {
                throw SBPL_Exception("ERROR: incorrect format of config file");
            }
            EnvNAVXYZTHETALATCfg.Grid2D[x][y] = dTemp;
        }
    }
}


bool Environment_NavXYZTheta::InitializeEnv(const char* sEnvFile, const std::vector<sbpl_2Dpt_t>& perimeterptsV,
                   const char* sMotPrimFile){
    EnvNAVXYZTHETALATCfg.FootprintPolygon = perimeterptsV;  // 机器人尺寸二维就够了

    SBPL_INFO("InitializeEnv start: sEnvFile=%s sMotPrimFile=%s\n", sEnvFile, sMotPrimFile);
    fflush(stdout);

    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        std::stringstream ss;
        ss << "ERROR: unable to open " << sEnvFile;
        throw SBPL_Exception(ss.str());
    }

    ReadConfiguration(fCfg);
    fclose(fCfg);

    if (sMotPrimFile != NULL) {
        FILE* fMotPrim = fopen(sMotPrimFile, "r");
        if (fMotPrim == NULL) {
            std::stringstream ss;
            ss << "ERROR: unable to open " << sMotPrimFile;
            throw SBPL_Exception(ss.str());
        }
        if (ReadMotionPrimitives(fMotPrim) == false) {
            throw SBPL_Exception("ERROR: failed to read in motion primitive file");
        }

        EnvNAVXYZTHETALATCfg.StartTheta = ContTheta2DiscNew(EnvNAVXYZTHETALATCfg.StartTheta_rad);
        if (EnvNAVXYZTHETALATCfg.StartTheta < 0 ||
            EnvNAVXYZTHETALATCfg.StartTheta >= EnvNAVXYZTHETALATCfg.NumThetaDirs)
        {
            throw new SBPL_Exception("ERROR: illegal start coordinates for theta");
        }
        EnvNAVXYZTHETALATCfg.EndTheta = ContTheta2DiscNew(EnvNAVXYZTHETALATCfg.EndTheta_rad);
        if (EnvNAVXYZTHETALATCfg.EndTheta < 0 ||
            EnvNAVXYZTHETALATCfg.EndTheta >= EnvNAVXYZTHETALATCfg.NumThetaDirs)
        {
            throw new SBPL_Exception("ERROR: illegal goal coordinates for theta");
        }

        InitGeneral(&EnvNAVXYZTHETALATCfg.mprimV);
        fclose(fMotPrim);
    }
    else {
        InitGeneral( NULL);
    }

    SBPL_PRINTF("size of env: %d by %d\n", EnvNAVXYZTHETALATCfg.EnvWidth_c, EnvNAVXYZTHETALATCfg.EnvHeight_c);

    return true;
}

bool Environment_NavXYZTheta::InitGeneral(std::vector<SBPL_xyztheta_mprimitive>* motionprimitiveV){
    // Initialize other parameters of the environment
    InitializeEnvConfig(motionprimitiveV);

    // initialize Environment
    InitializeEnvironment();

    // pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

void Environment_NavXYZTheta::InitializeEnvConfig(std::vector<SBPL_xyztheta_mprimitive>* motionprimitiveV)
{
    // additional to configuration file initialization of EnvNAVXYZTHETALATCfg if
    // necessary

    // dXY dirs
    EnvNAVXYZTHETALATCfg.dXY[0][0] = -1;
    EnvNAVXYZTHETALATCfg.dXY[0][1] = -1;
    EnvNAVXYZTHETALATCfg.dXY[1][0] = -1;
    EnvNAVXYZTHETALATCfg.dXY[1][1] = 0;
    EnvNAVXYZTHETALATCfg.dXY[2][0] = -1;
    EnvNAVXYZTHETALATCfg.dXY[2][1] = 1;
    EnvNAVXYZTHETALATCfg.dXY[3][0] = 0;
    EnvNAVXYZTHETALATCfg.dXY[3][1] = -1;
    EnvNAVXYZTHETALATCfg.dXY[4][0] = 0;
    EnvNAVXYZTHETALATCfg.dXY[4][1] = 1;
    EnvNAVXYZTHETALATCfg.dXY[5][0] = 1;
    EnvNAVXYZTHETALATCfg.dXY[5][1] = -1;
    EnvNAVXYZTHETALATCfg.dXY[6][0] = 1;
    EnvNAVXYZTHETALATCfg.dXY[6][1] = 0;
    EnvNAVXYZTHETALATCfg.dXY[7][0] = 1;
    EnvNAVXYZTHETALATCfg.dXY[7][1] = 1;

    sbpl_xy_theta_pt_t temppose;
    temppose.x = 0.0;
    temppose.y = 0.0;
//    temppose.z = 0.0;
    temppose.theta = 0.0;
    std::vector<sbpl_2Dcell_t> footprint;
    get_2d_footprint_cells(
            EnvNAVXYZTHETALATCfg.FootprintPolygon,
            &footprint,
            temppose,
            EnvNAVXYZTHETALATCfg.cellsize_m);
    SBPL_PRINTF("number of cells in footprint of the robot = %d\n", (unsigned int)footprint.size());

    for (std::vector<sbpl_2Dcell_t>::iterator it = footprint.begin(); it != footprint.end(); ++it) {
        SBPL_PRINTF("Footprint cell at (%d, %d)\n", it->x, it->y);
    }

#if DEBUG
    SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", (int)footprint.size());
    for(int i = 0; i < (int) footprint.size(); i++)
    {
        SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x, footprint.at(i).y,
                     DISCXY2CONT(footprint.at(i).x, EnvNAVXYZTHETALATCfg.cellsize_m),
                     DISCXY2CONT(footprint.at(i).y, EnvNAVXYZTHETALATCfg.cellsize_m));
    }
#endif

    if (motionprimitiveV == NULL) {
        DeprecatedPrecomputeActions();
    }
    else {
        PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);
    }
}

bool Environment_NavXYZTheta::ReadMotionPrimitives(FILE* fMotPrims){
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    SBPL_INFO("Reading in motion primitives...");
    fflush(stdout);

    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        fflush(stdout);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) {
        return false;
    }
    if (fabs(fTemp - EnvNAVXYZTHETALATCfg.cellsize_m) > ERR_EPS) {
        SBPL_ERROR("ERROR: invalid resolution %f (instead of %f) in the dynamics file\n", fTemp, EnvNAVXYZTHETALATCfg.cellsize_m);
        fflush(stdout);
        return false;
    }
    SBPL_INFO("resolution_m: %f\n", fTemp);

    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    SBPL_INFO("sTemp: %s\n", sTemp);
    if (strncmp(sTemp, "min_turning_radius_m:", 21) == 0) {
        bUseNonUniformAngles = true;
    }
    SBPL_INFO("bUseNonUniformAngles = %d", bUseNonUniformAngles);

    if (bUseNonUniformAngles) {
        float min_turn_rad;
        strcpy(sExpected, "min_turning_radius_m:");
        if (strcmp(sTemp, sExpected) != 0) {
            SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
            fflush(stdout);
            return false;
        }
        if (fscanf(fMotPrims, "%f", &min_turn_rad) == 0) {
            return false;
        }
        SBPL_PRINTF("min_turn_rad: %f\n", min_turn_rad);
        fflush(stdout);
        if (fscanf(fMotPrims, "%s", sTemp) == 0) {
            return false;
        }
    }

    // read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0) {
        return false;
    }
    if (dTemp != EnvNAVXYZTHETALATCfg.NumThetaDirs) {
        SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n", dTemp, EnvNAVXYZTHETALATCfg.NumThetaDirs);
        return false;
    }
    SBPL_PRINTF("numberofangles: %d\n", dTemp);
    EnvNAVXYZTHETALATCfg.NumThetaDirs = dTemp;

    if (bUseNonUniformAngles) {
        // read in angles
        EnvNAVXYZTHETALATCfg.ThetaDirs.clear();
        for (int i = 0; i < EnvNAVXYZTHETALATCfg.NumThetaDirs; i++)
        {
            std::ostringstream string_angle_index;
            string_angle_index << i;
            std::string angle_string = "angle:" + string_angle_index.str();

            float angle;
            strcpy(sExpected, angle_string.c_str());
            if (fscanf(fMotPrims, "%s", sTemp) == 0) {
                return false;
            }
            if (strcmp(sTemp, sExpected) != 0) {
                SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
                return false;
            }
            if (fscanf(fMotPrims, "%f", &angle) == 0) {
                return false;
            }
            SBPL_PRINTF("%s %f\n", angle_string.c_str(), angle);
            EnvNAVXYZTHETALATCfg.ThetaDirs.emplace_back(angle);
        }
        EnvNAVXYZTHETALATCfg.ThetaDirs.emplace_back(2.0 * M_PI); // Add 2 PI at end for overlap
    }

    // read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
        return false;
    }
    SBPL_PRINTF("totalnumberofprimitives: %d\n", totalNumofActions);

    // Read in motion primitive for each action
    for (int i = 0; i < totalNumofActions; i++) {
        SBPL_xyztheta_mprimitive motprim;

        if (!Environment_NavXYZTheta::ReadinMotionPrimitive(&motprim, fMotPrims)) {
            return false;
        }

        EnvNAVXYZTHETALATCfg.mprimV.emplace_back(motprim);
    }
    SBPL_PRINTF("done");
    SBPL_FFLUSH(stdout);
    return true;
}

bool Environment_NavXYZTheta::ReadinMotionPrimitive(SBPL_xyztheta_mprimitive* pMotPrim, FILE* fIn){
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;
    float fTemp;

    // read in actionID
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        fflush(stdout);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1) {
        return false;
    }

    // read in start angle
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0) {
        SBPL_ERROR("ERROR reading startangle\n");
        return false;
    }
    pMotPrim->starttheta_c = dTemp;

    // read in end pose
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }

    if (ReadinCell(&pMotPrim->endcell, fIn) == false) { // 末端节点是int
        SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
        return false;
    }

    // read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1) {
        return false;
    }
    pMotPrim->additionalactioncostmult = dTemp;

    if (bUseNonUniformAngles) {
        // read in action turning radius
        strcpy(sExpected, "turning_radius:");
        if (fscanf(fIn, "%s", sTemp) == 0) {
            return false;
        }
        if (strcmp(sTemp, sExpected) != 0) {
            SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
            return false;
        }
        if (fscanf(fIn, "%f", &fTemp) != 1) {
            return false;
        }
        pMotPrim->turning_radius = fTemp;
    }

    // read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1) {
        return false;
    }
    // all intermposes should be with respect to 0,0 as starting pose since it
    // will be added later and should be done after the action is rotated by
    // initial orientation
    for (int i = 0; i < numofIntermPoses; i++) {
        sbpl_xyz_theta_pt_t intermpose;
        if (ReadinPose(&intermpose, fIn) == false) {
            SBPL_ERROR("ERROR: failed to read in intermediate poses\n");
            return false;
        }
        pMotPrim->intermptV.emplace_back(intermpose);
    }

    // Check that the last pose of the motion matches (within lattice
    // resolution) the designated end pose of the primitive
    sbpl_xyz_theta_pt_t sourcepose;
    sourcepose.x = DISCXY2CONT(0, EnvNAVXYZTHETALATCfg.cellsize_m);
    sourcepose.y = DISCXY2CONT(0, EnvNAVXYZTHETALATCfg.cellsize_m);  // 转到栅格中心点

    sourcepose.theta = DiscTheta2ContNew(pMotPrim->starttheta_c);   // 转到浮点数
    double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;

    int endtheta_c;
    int endx_c = CONTXY2DISC(mp_endx_m, EnvNAVXYZTHETALATCfg.cellsize_m);
    int endy_c = CONTXY2DISC(mp_endy_m, EnvNAVXYZTHETALATCfg.cellsize_m);
//    int ednz_c =
//    float endz_c =
    endtheta_c = ContTheta2DiscNew(mp_endtheta_rad);
    if (endx_c != pMotPrim->endcell.x ||
        endy_c != pMotPrim->endcell.y ||
        endtheta_c != pMotPrim->endcell.theta)
    {
        SBPL_ERROR( "ERROR: incorrect primitive %d with startangle=%d "
                    "last interm point %f %f %f does not match end pose %d %d %d\n",
                    pMotPrim->motprimID, pMotPrim->starttheta_c,
                    pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x,
                    pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y,
                    pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta,
                    pMotPrim->endcell.x, pMotPrim->endcell.y,
                    pMotPrim->endcell.theta);
        SBPL_FFLUSH(stdout);
        return false;
    }
    return true;
}

bool Environment_NavXYZTheta::ReadinCell(
        sbpl_xyz_theta_cell_t* cell,
        FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->x = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->y = atoi(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->z = atoi(sTemp);  // 末端节点是用cell来记录的
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->theta = atoi(sTemp);

    // normalize the angle
    cell->theta = normalizeDiscAngle(cell->theta);
    // cell->theta = NORMALIZEDISCTHETA(cell->theta, EnvNAVXYZTHETALATCfg.NumThetaDirs);

    return true;
}

bool Environment_NavXYZTheta::ReadinPose(sbpl_xyz_theta_pt_t* pose, FILE* fIn){
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->x = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->y = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->z = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->theta = atof(sTemp);

    pose->theta = normalizeAngle(pose->theta);  // 限定范围在[0,2*pi);

    return true;
}

void Environment_NavXYZTheta::PrecomputeActionswithCompleteMotionPrimitive(          //
        std::vector<SBPL_xyztheta_mprimitive>* motionprimitiveV){
    SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
    EnvNAVXYZTHETALATCfg.ActionsV = new EnvNAVXYZTHETALATAction_t*[EnvNAVXYZTHETALATCfg.NumThetaDirs];
    EnvNAVXYZTHETALATCfg.PredActionsV = new std::vector<EnvNAVXYZTHETALATAction_t*>[EnvNAVXYZTHETALATCfg.NumThetaDirs];
    std::vector<sbpl_2Dcell_t> footprint;

    if (motionprimitiveV->size() % EnvNAVXYZTHETALATCfg.NumThetaDirs != 0) {
        throw SBPL_Exception("ERROR: motionprimitives should be uniform across actions");
    }

    EnvNAVXYZTHETALATCfg.actionwidth = ((int)motionprimitiveV->size()) / EnvNAVXYZTHETALATCfg.NumThetaDirs;

    // iterate over source angles
    int maxnumofactions = 0;
    for (int tind = 0; tind < EnvNAVXYZTHETALATCfg.NumThetaDirs; tind++) {
        SBPL_PRINTF("pre-computing for angle %d out of %d angles\n", tind, EnvNAVXYZTHETALATCfg.NumThetaDirs);

        EnvNAVXYZTHETALATCfg.ActionsV[tind] = new EnvNAVXYZTHETALATAction_t[EnvNAVXYZTHETALATCfg.actionwidth];

        // compute sourcepose
        sbpl_xyz_theta_pt_t sourcepose;
        sourcepose.x = DISCXY2CONT(0, EnvNAVXYZTHETALATCfg.cellsize_m);
        sourcepose.y = DISCXY2CONT(0, EnvNAVXYZTHETALATCfg.cellsize_m);
        sourcepose.z = 0;   // z 没有离散
        sourcepose.theta = DiscTheta2ContNew(tind);

        // iterate over motion primitives
        int numofactions = 0;
        int aind = -1;
        for (int mind = 0; mind < (int)motionprimitiveV->size(); mind++) {
            //find a motion primitive for this angle
            if (motionprimitiveV->at(mind).starttheta_c != tind) {
                continue;
            }

            aind++;
            numofactions++;

            // action index
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].aind = aind;

            // start angle
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].starttheta = tind;

            // compute dislocation
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;    // 末端状态是离散的数字
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].dZ = motionprimitiveV->at(mind).endcell.z;

            // compute and store interm points as well as intersecting cells
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV.clear();
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.clear();

            sbpl_xyz_theta_cell_t previnterm3Dcell;
            previnterm3Dcell.x = 0;
            previnterm3Dcell.y = 0;
            previnterm3Dcell.z = 0;

            // Compute all the intersected cells for this action (intermptV and interm3DcellsV)
            for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++) {
                sbpl_xyz_theta_pt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
                EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV.emplace_back(intermpt);

                // also compute the intermediate discrete cells if not there already
                sbpl_xyz_theta_pt_t pose;
                pose.x = intermpt.x + sourcepose.x;
                pose.y = intermpt.y + sourcepose.y;
                pose.z = intermpt.z + sourcepose.z;
                pose.theta = intermpt.theta;

                sbpl_xyz_theta_cell_t intermediate2dCell;
                intermediate2dCell.x = CONTXY2DISC(pose.x, EnvNAVXYZTHETALATCfg.cellsize_m);
                intermediate2dCell.y = CONTXY2DISC(pose.y, EnvNAVXYZTHETALATCfg.cellsize_m);

                // add unique cells to the list
                if (EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size() == 0 ||
                    intermediate2dCell.x != previnterm3Dcell.x ||
                    intermediate2dCell.y != previnterm3Dcell.y)
                {
                    EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.push_back(intermediate2dCell);
                }

                previnterm3Dcell = intermediate2dCell;
            }

            // compute linear and angular time
            double linear_distance = 0;
            for (unsigned int i = 1; i < EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV.size(); i++) {
                double x0 = EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV[i - 1].x;
                double y0 = EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV[i - 1].y;
                double x1 = EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV[i].x;
                double y1 = EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV[i].y;
                double dx = x1 - x0;
                double dy = y1 - y0;
                linear_distance += sqrt(dx * dx + dy * dy);
            }
            double linear_time = linear_distance / EnvNAVXYZTHETALATCfg.nominalvel_mpersecs;
            double angular_distance;
            angular_distance = fabs(computeMinUnsignedAngleDiff(
                    DiscTheta2ContNew(EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].endtheta),
                    DiscTheta2ContNew(EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].starttheta)));

            double angular_time = angular_distance / ((PI_CONST / 4.0) /
                                                      EnvNAVXYZTHETALATCfg.timetoturn45degsinplace_secs);
            // make the cost the max of the two times
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].cost =
                    (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM * std::max(linear_time, angular_time)));
            // use any additional cost multiplier
            EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;

            // now compute the intersecting cells for this motion (including ignoring the source footprint)
//            get_2d_motion_cells(
//                    EnvNAVXYZTHETALATCfg.FootprintPolygon,
//                    motionprimitiveV->at(mind).intermptV,
//                    &EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intersectingcellsV,
//                    EnvNAVXYZTHETALATCfg.cellsize_m);

#if DEBUG
            SBPL_FPRINTF(fDeb,
                         "action tind=%2d aind=%2d: dX=%3d dY=%3d endtheta=%3d (%6.2f degs -> %6.2f degs) "
                         "cost=%4d (mprimID %3d: %3d %3d %3d) numofintermcells = %d numofintercells=%d\n",
                         tind,
                         aind,
                         EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].dX,
                         EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].dY,
                         EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].endtheta,
                         EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV[0].theta * 180 / PI_CONST,
                         EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intermptV.size() - 1].theta * 180 / PI_CONST, EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].cost,
                         motionprimitiveV->at(mind).motprimID, motionprimitiveV->at(mind).endcell.x,
                         motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.theta,
                         (int)EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].interm3DcellsV.size(),
                         (int)EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.size());
#endif

            // add to the list of backward actions
            int targettheta = EnvNAVXYZTHETALATCfg.ActionsV[tind][aind].endtheta;
            if (targettheta < 0) {
                targettheta = targettheta + EnvNAVXYZTHETALATCfg.NumThetaDirs;
            }
            EnvNAVXYZTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYZTHETALATCfg.ActionsV[tind][aind]));
        }

        if (maxnumofactions < numofactions) {
            maxnumofactions = numofactions;
        }
    }

    // at this point we don't allow nonuniform number of actions
    if (motionprimitiveV->size() != (size_t)(EnvNAVXYZTHETALATCfg.NumThetaDirs * maxnumofactions)) {
        std::stringstream ss;
        ss << "ERROR: nonuniform number of actions is not supported" <<
           " (maxnumofactions=" << maxnumofactions << " while motprims=" <<
           motionprimitiveV->size() << " thetas=" <<
           EnvNAVXYZTHETALATCfg.NumThetaDirs;
        throw SBPL_Exception(ss.str());
    }

    // now compute replanning data
    ComputeReplanningData();

    SBPL_PRINTF("done pre-computing action data based on motion primitives\n");
}

void Environment_NavXYZTheta::ComputeReplanningData(){
    // iterate over all actions
    // orientations
    for (int tind = 0; tind < EnvNAVXYZTHETALATCfg.NumThetaDirs; tind++) {
        // actions
        for (int aind = 0; aind < EnvNAVXYZTHETALATCfg.actionwidth; aind++) {
            // compute replanning data for this action
            ComputeReplanningDataforAction(&EnvNAVXYZTHETALATCfg.ActionsV[tind][aind]);
        }
    }
}

void Environment_NavXYZTheta::ComputeReplanningDataforAction(
        EnvNAVXYZTHETALATAction_t* action)
{
    int j;

    // iterate over all the cells involved in the action
    sbpl_xyz_theta_cell_t startcell3d, endcell3d;
    for (int i = 0; i < (int)action->intersectingcellsV.size(); i++) {
        // compute the translated affected search Pose - what state has an
        // outgoing action whose intersecting cell is at 0,0
        startcell3d.theta = action->starttheta;
        startcell3d.x = -action->intersectingcellsV.at(i).x;
        startcell3d.y = -action->intersectingcellsV.at(i).y;
        startcell3d.z = -action->intersectingcellsV.at(i).z;

        // compute the translated affected search Pose - what state has an
        // incoming action whose intersecting cell is at 0,0
        if (bUseNonUniformAngles) {
            endcell3d.theta = normalizeDiscAngle(action->endtheta);
        }
        else {
            endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
        }
        endcell3d.x = startcell3d.x + action->dX;
        endcell3d.y = startcell3d.y + action->dY;
        endcell3d.z = startcell3d.z + action->dZ;

        //store the cells if not already there
        for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
            if (affectedsuccstatesV.at(j) == endcell3d) {
                break;
            }
        }
        if (j == (int)affectedsuccstatesV.size()) {
            affectedsuccstatesV.push_back(endcell3d);
        }

        for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
            if (affectedpredstatesV.at(j) == startcell3d) {
                break;
            }
        }
        if (j == (int)affectedpredstatesV.size()) {
            affectedpredstatesV.push_back(startcell3d);
        }
    } // over intersecting cells

    // add the centers since with h2d we are using these in cost computations
    // ---intersecting cell = origin
    // compute the translated affected search Pose - what state has an outgoing
    // action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -0;
    startcell3d.y = -0;

    // compute the translated affected search Pose - what state has an incoming
    // action whose intersecting cell is at 0,0
    if (bUseNonUniformAngles) {
        endcell3d.theta = normalizeDiscAngle(action->endtheta);
    }
    else {
        endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
    }
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    //store the cells if not already there
    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) {
            break;
        }
    }
    if (j == (int)affectedsuccstatesV.size()) {
        affectedsuccstatesV.push_back(endcell3d);
    }

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) {
            break;
        }
    }
    if (j == (int)affectedpredstatesV.size()) {
        affectedpredstatesV.push_back(startcell3d);
    }

    //---intersecting cell = outcome state
    // compute the translated affected search Pose - what state has an outgoing
    // action whose intersecting cell is at 0,0
    startcell3d.theta = action->starttheta;
    startcell3d.x = -action->dX;
    startcell3d.y = -action->dY;

    // compute the translated affected search Pose - what state has an incoming
    // action whose intersecting cell is at 0,0
    if (bUseNonUniformAngles) {
        endcell3d.theta = normalizeDiscAngle(action->endtheta);
    }
    else {
        endcell3d.theta = NORMALIZEDISCTHETA(action->endtheta, EnvNAVXYTHETALATCfg.NumThetaDirs);
    }
    endcell3d.x = startcell3d.x + action->dX;
    endcell3d.y = startcell3d.y + action->dY;

    for (j = 0; j < (int)affectedsuccstatesV.size(); j++) {
        if (affectedsuccstatesV.at(j) == endcell3d) {
            break;
        }
    }
    if (j == (int)affectedsuccstatesV.size()) {
        affectedsuccstatesV.push_back(endcell3d);
    }

    for (j = 0; j < (int)affectedpredstatesV.size(); j++) {
        if (affectedpredstatesV.at(j) == startcell3d) {
            break;
        }
    }
    if (j == (int)affectedpredstatesV.size()) {
        affectedpredstatesV.push_back(startcell3d);
    }
}

int Environment_NavXYZTheta::SetStart(double x, double y, double z, double theta) {

}

int Environment_NavXYZTheta::SetGoal(double x, double y,double z,  double theta){

}

void Environment_NavXYZTheta::SetGoalTolerance(double tol_x, double tol_y, double tol_z, double tol_theta) {

}

void Environment_NavXYZTheta::GetCoordFromState(int stateID, int& x, int& y, int& z,int& theta) const {

}


int Environment_NavXYZTheta::GetStateFromCoord(int x, int y, int z, int theta){

}

void Environment_NavXYZTheta::GetActionsFromStateIDPath(std::vector<int>* stateIDPath,
                               std::vector<EnvNAVXYTHETALATAction_t>* action_list){

}

void Environment_NavXYZTheta::ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath,
                                       std::vector<sbpl_xy_theta_pt_t>* xythetaPath){

}

void Environment_NavXYZTheta::PrintState(int stateID, bool bVerbose, FILE* fOut){

}

void Environment_NavXYZTheta::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){

}

void GetLazyPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){}
void Environment_NavXYZTheta::GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){}
void Environment_NavXYZTheta::GetLazyPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost){}

/**
 * \brief returns all successors states, costs of corresponding actions
 *        and pointers to corresponding actions, each of which is a motion
 *        primitive
 *        if actionindV is NULL, then pointers to actions are not returned
 */
void Environment_NavXYZTheta::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                      std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ){}

void Environment_NavXYZTheta::GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV){}
void Environment_NavXYZTheta::GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV){}
void Environment_NavXYZTheta::GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ){}
int Environment_NavXYZTheta::GetTrueCost(int parentID, int childID){}
 bool isGoal(int id){}


/** \brief this function fill in Predecessor/Successor states of edges
 *         whose costs changed
 *         It takes in an array of cells whose traversability changed, and returns
 *         (in vector preds_of_changededgesIDV) the IDs of all states that have
 *         outgoing edges that go through the changed cells
 */
void Environment_NavXYZTheta::GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                    std::vector<int> *preds_of_changededgesIDV){}

/**
 * \brief same as GetPredsofChangedEdges, but returns successor states.
 *        Both functions need to be present for incremental search
 */
void Environment_NavXYZTheta::GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                    std::vector<int> *succs_of_changededgesIDV){}

/**
 * \brief see comments on the same function in the parent class
 */
void Environment_NavXYZTheta::SetAllActionsandAllOutcomes(CMDPSTATE* state){}

/**
 * \brief see comments on the same function in the parent class
 */
int Environment_NavXYZTheta::GetFromToHeuristic(int FromStateID,int ToStateID){}

/**
 * \brief see comments on the same function in the parent class
 */
int Environment_NavXYZTheta::GetGoalHeuristic(int stateID){}

/**
 * \brief see comments on the same function in the parent class
 */
int Environment_NavXYZTheta::GetStartHeuristic(int stateID){}

/**
 * \brief see comments on the same function in the parent class
 */
int Environment_NavXYZTheta::SizeofCreatedEnv(){}

/**
 * \brief see comments on the same function in the parent class
 */
void Environment_NavXYZTheta::PrintVars() { }

const EnvNAVXYTHETALATHashEntry_t* GetStateEntry(int state_id){

}



unsigned int Environment_NavXYZTheta::GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Z, unsigned int Theta){}

EnvNAVXYTHETALATHashEntry_t* GetHashEntry_hash(int X, int Y, int Z, int Theta){}
EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Z, int Theta){}
EnvNAVXYTHETALATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Z, int Theta){}
EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Z, int Theta){}


void Environment_NavXYZTheta::InitializeEnvironment(){}

void Environment_NavXYZTheta::PrintHashTableHist(FILE* fOut){}