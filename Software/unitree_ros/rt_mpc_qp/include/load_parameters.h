/*************************************************************
 * Copyright: Walker Team, UBTech
 * Auther: chunyu.chen
 * Build Date: 30/07/2018
 * Modify Date:
 *************************************************************/

/**
 * @file:
 * @brief:Load the motion parameters which are used in walking, controller and keeping balance.
 */

#ifndef LOAD_PARAMETERS_H
#define LOAD_PARAMETERS_H

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <fstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


/**
 * @namespace gait
 */

namespace gait{

    //controller parameters
    typedef enum {
        BODY_COMPLIANCE_CTRL_PARA = 0,
        ANKLE_COMPLIANCE_CTRL_PARA,
        ATTITUDE_CTRL_PARA,
        FOOT_PLACEMENT_PARA
    }ControllerParaGroup;

    //body compliance controller parameters
    typedef struct {
        double _kp;
        double _kv;
        double _force;
        std::string _index;
    }BodyComplianceCtrlParaInfo;

    typedef struct {
        double x[3];
        double y[3];
        double z[3];
    }BodyComplianceCtrlPara;

    //ankle compliance controller parameters
    typedef struct {
        double _kp;
        double _kv;
        double _zmp_kv;
        std::string _index;
    }AnkleComplianceCtrlParaInfo;

    typedef struct {
        double x_left[3];
        double x_right[3];
        double y_left[3];
        double y_right[3];
    }AnkleComplianceCtrlPara;

   //attitude controller parameters
    typedef struct {
        double _kp;
        double _kv;
        std::string _index;
    }AttitudeCtrlParaInfo;

    typedef struct {
        double roll[2];
        double pitch[2];
    }AttitudeCtrlPara;

    //foot placement controller parameters
    typedef struct {
        double _kp;
        double _kv;
        std::string _index;
    }FootPlacementParaInfo;

    typedef struct {
        double x[2];
        double y[2];
    }FootPlacementPara;

    typedef std::map<std::string, BodyComplianceCtrlParaInfo> BodyComplianceCtrlParaMap;
    typedef std::map<std::string, AnkleComplianceCtrlParaInfo> AnkleComplianceCtrlParaMap;
    typedef std::map<std::string, AttitudeCtrlParaInfo> AttitudeCtrlParaMap;
    typedef std::map<std::string, FootPlacementParaInfo> FootPlacementParaMap;
    /**
     * @class LoadPara
     * @brief Load the motion parameters
     */

    class LoadPara{
    public:
        LoadPara();
        /**
         * @brief constructor
         */

        ~LoadPara();
        /**
         * @brief destructor
         */

        bool LoadParaConf(const std::string &fileCtrl, const std::string &filePlan, BodyComplianceCtrlPara *body_com,
                          AnkleComplianceCtrlPara *ankle_com, AttitudeCtrlPara *att, FootPlacementPara *foot);
        /**
         * @brief
         * @input
         * @output
         */

    private:

        bool BodyComplianceCtrlConf(const std::string &file,
                                    ControllerParaGroup group,
                                    BodyComplianceCtrlParaMap *map);

        bool AnkleComplianceCtrlConf(const std::string &file,
                                     ControllerParaGroup group,
                                     AnkleComplianceCtrlParaMap *map);

        bool AttitudeCtrlConf(const std::string &file,
                              ControllerParaGroup group,
                              AttitudeCtrlParaMap *map);

        bool FootPlacementParaConf(const std::string &file,
                                   ControllerParaGroup group,
                                   FootPlacementParaMap *map);


    };
}//namespace gait




#endif //LOAD_PARAMETERS_H