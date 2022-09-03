//
// Created by lrm on 22-7-28.
//

#ifndef INC_3DPLAN_ENVIRONMENT_NAVXYZTHETA_H
#define INC_3DPLAN_ENVIRONMENT_NAVXYZTHETA_H

#include <vector>
#include <iostream>
#include <sbpl/utils/utils.h>
#include "sbpl/discrete_space_information/environment_navxythetalat.h"
#include "common.h"

struct EnvNAVXYZTHETALATAction_t
{
    unsigned char aind; //index of the action (unique for given starttheta)
    char starttheta;
    // 末端状态 离散表示
    char dX;
    char dY;
    char dZ;
    char endtheta;
    unsigned int cost;
    std::vector<sbpl_3Dcell_t> intersectingcellsV;
    //start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
    std::vector<sbpl_xyz_theta_pt_t> intermptV;
    //start at 0,0,starttheta and end at endcell in discrete domain
    std::vector<sbpl_xyz_theta_cell_t> interm3DcellsV;

    int motprimID;
    double turning_radius;

};

// 三维
struct EnvNavXYZTHETALATHashEntry_t
{
    int stateID;
    int X;
    int Y;
    int Z;
    char Theta;
    int iteration;
};

struct SBPL_xyztheta_mprimitive
{
    int motprimID;
    unsigned char starttheta_c;
    int additionalactioncostmult;
    sbpl_xyz_theta_cell_t endcell;
    double turning_radius;
    //intermptV start at 0,0,starttheta and end at endcell in continuous
    //domain with half-bin less to account for 0,0 start
    std::vector<sbpl_xyz_theta_pt_t> intermptV;
};

//configuration parameters
struct EnvNavXYZTHETALATConfig_t
{
    int EnvWidth_c;
    int EnvHeight_c;
    int NumThetaDirs;
    int StartX_c;
    int StartY_c;
    int StartZ_c;
    int StartTheta;
    int EndX_c;
    int EndY_c;
    int EndZ_c;
    int EndTheta;
    unsigned char** Grid2D; // 先用int测试先

    std::vector<double> ThetaDirs;
    double StartTheta_rad;
    double EndTheta_rad;
    double min_turning_radius_m;

    // the value at which and above which cells are obstacles in the maps sent from outside
    // the default is defined above
    unsigned char obsthresh;

    // the value at which and above which until obsthresh (not including it)
    // cells have the nearest obstacle at distance smaller than or equal to
    // the inner circle of the robot. In other words, the robot is definitely
    // colliding with the obstacle, independently of its orientation
    // if no such cost is known, then it should be set to obsthresh (if center
    // of the robot collides with obstacle, then the whole robot collides with
    // it independently of its rotation)
    unsigned char cost_inscribed_thresh;

    // the value at which and above which until cost_inscribed_thresh (not including it) cells
    // **may** have a nearest osbtacle within the distance that is in between
    // the robot inner circle and the robot outer circle
    // any cost below this value means that the robot will NOT collide with any
    // obstacle, independently of its orientation
    // if no such cost is known, then it should be set to 0 or -1 (then no cell
    // cost will be lower than it, and therefore the robot's footprint will
    // always be checked)
    int cost_possibly_circumscribed_thresh; // it has to be integer, because -1 means that it is not provided.

    double nominalvel_mpersecs;

    //double nominalangvel_radpersecs;

    double timetoturn45degsinplace_secs;

    double cellsize_m;

    int dXY[NAVXYTHETALAT_DXYWIDTH][2];

    //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
    EnvNAVXYZTHETALATAction_t** ActionsV;   // 保存所有的运动基元
    //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i
    std::vector<EnvNAVXYZTHETALATAction_t*>* PredActionsV;

    int actionwidth; //number of motion primitives
    std::vector<SBPL_xyztheta_mprimitive> mprimV;

    std::vector<sbpl_2Dpt_t> FootprintPolygon;  // 机器人尺寸
};

class Environment_NavXYZTheta: public EnvironmentNAVXYTHETALAT{
public:
    Environment_NavXYZTheta();
    ~Environment_NavXYZTheta();

    void ReadConfiguration(FILE* fCfg); // 环境模型不一样，需要修改
    bool InitGeneral(std::vector<SBPL_xyztheta_mprimitive>* motionprimitiveV);
    void InitializeEnvConfig(std::vector<SBPL_xyztheta_mprimitive>* motionprimitiveV);
    bool InitializeEnv(const char* sEnvFile, const std::vector<sbpl_2Dpt_t>& perimeterptsV, // 机器人尺寸
                       const char* sMotPrimFile);
    bool ReadMotionPrimitives(FILE* fMotPrims);
    bool ReadinMotionPrimitive(SBPL_xyztheta_mprimitive* pMotPrim, FILE* fIn);
    bool ReadinCell(sbpl_xyz_theta_cell_t* cell,  FILE* fIn);    // 读取每条运动基元的终点状态
    bool ReadinPose(sbpl_xyz_theta_pt_t* pose, FILE* fIn);       // 读取每条运动基元
    void PrecomputeActionswithCompleteMotionPrimitive(          //
            std::vector<SBPL_xyztheta_mprimitive>* motionprimitiveV);

    void ComputeReplanningData();
    /**
     * \brief sets start in meters/radians
     */
    virtual int SetStart(double x, double y, double z, double theta);

    /**
     * \brief sets goal in meters/radians
     */
    virtual int SetGoal(double x, double y, double z, double theta);

    /**
     * \brief sets goal tolerance. (Note goal tolerance is ignored currently)
     */
    virtual void SetGoalTolerance(double tol_x, double tol_y, double tol_z, double tol_theta);

    /**
     * \brief returns state coordinates of state with ID=stateID
     */
    virtual void GetCoordFromState(int stateID, int& x, int& y, int& z, int& theta) const;

    /**
     * \brief returns stateID for a state with coords x,y,theta
     */
    virtual int GetStateFromCoord(int x, int y, int z, int theta);

    /**
     * \brief returns the actions / motion primitives of the passed path.
     */
    virtual void GetActionsFromStateIDPath(std::vector<int>* stateIDPath,
                                           std::vector<EnvNAVXYTHETALATAction_t>* action_list);

    /** \brief converts a path given by stateIDs into a sequence of
     *         coordinates. Note that since motion primitives are short actions
     *         represented as a sequence of points,
     *         the path returned by this function contains much more points than the
     *         number of points in the input path. The returned coordinates are in
     *         meters,meters,radians
     */
    virtual void ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath,
                                                   std::vector<sbpl_xy_theta_pt_t>* xythetaPath);

    /**
     * \brief prints state info (coordinates) into file
     */
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief returns all predecessors states and corresponding costs of actions
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
    virtual void GetLazyPreds(int TargetStateID, std::vector<int>* PredIDV,
                              std::vector<int>* CostV, std::vector<bool>* isTrueCost);
    virtual void GetPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
    virtual void GetLazyPredsWithUniqueIds(int TargetStateID, std::vector<int>* PredIDV,
                                           std::vector<int>* CostV, std::vector<bool>* isTrueCost);

    /**
     * \brief returns all successors states, costs of corresponding actions
     *        and pointers to corresponding actions, each of which is a motion
     *        primitive
     *        if actionindV is NULL, then pointers to actions are not returned
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV,
                          std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);

    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV,
                              std::vector<int>* CostV, std::vector<bool>* isTrueCost,
                              std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV,
                                       std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV,
                                           std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV = NULL);
    virtual int GetTrueCost(int parentID, int childID);
    virtual bool isGoal(int id);


    /** \brief this function fill in Predecessor/Successor states of edges
     *         whose costs changed
     *         It takes in an array of cells whose traversability changed, and returns
     *         (in vector preds_of_changededgesIDV) the IDs of all states that have
     *         outgoing edges that go through the changed cells
     */
    virtual void GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *preds_of_changededgesIDV);

    /**
     * \brief same as GetPredsofChangedEdges, but returns successor states.
     *        Both functions need to be present for incremental search
     */
    virtual void GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV,
                                        std::vector<int> *succs_of_changededgesIDV);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetGoalHeuristic(int stateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int GetStartHeuristic(int stateID);

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual int SizeofCreatedEnv();

    /**
     * \brief see comments on the same function in the parent class
     */
    virtual void PrintVars();

    const EnvNAVXYTHETALATHashEntry_t* GetStateEntry(int state_id) const;

protected:

    EnvNavXYZTHETALATConfig_t EnvNAVXYZTHETALATCfg; // 三维 x y z theta
    EnvironmentNAVXYTHETALAT_t EnvNAVXYTHETALAT;    // 记录的是id
    std::vector<sbpl_xyz_theta_cell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
    std::vector<sbpl_xyz_theta_cell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvNAVXYTHETALATHashEntry_t*>* Coord2StateIDHashTable;
    //vector that maps from stateID to coords
    std::vector<EnvNAVXYTHETALATHashEntry_t*> StateID2CoordTable;

    EnvNAVXYTHETALATHashEntry_t** Coord2StateIDHashTable_lookup;

    virtual unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Z, unsigned int Theta);

    virtual EnvNAVXYTHETALATHashEntry_t* GetHashEntry_hash(int X, int Y, int Z, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Z, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Z, int Theta);
    virtual EnvNAVXYTHETALATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Z, int Theta);

    //pointers to functions
    EnvNAVXYTHETALATHashEntry_t* (EnvironmentNAVXYTHETALAT::*GetHashEntry)(int X, int Y, int Z, int Theta);
    EnvNAVXYTHETALATHashEntry_t* (EnvironmentNAVXYTHETALAT::*CreateNewHashEntry)(int X, int Y, int Z, int Theta);

    virtual void InitializeEnvironment();

    virtual void PrintHashTableHist(FILE* fOut);

    virtual void ComputeReplanningDataforAction(EnvNAVXYZTHETALATAction_t* action);
};


#endif //INC_3DPLAN_ENVIRONMENT_NAVXYZTHETA_H
