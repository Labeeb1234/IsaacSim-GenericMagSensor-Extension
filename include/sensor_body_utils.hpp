#ifndef SENSOR_BODY_UTILS_HPP
#define SENSOR_BODY_UTILS_HPP

#include "pxr/usd/usd/stage.h"
#include <pxr/usd/usdUtils/stageCache.h>
#include <omni/fabric/usd/PathConversion.h>

#include <usdrt/scenegraph/usd/usd/stage.h>
#include <usdrt/scenegraph/usd/usd/prim.h>
#include <usdrt/scenegraph/base/gf/vec3f.h>
#include <usdrt/scenegraph/base/gf/quatf.h>
#include <usdrt/scenegraph/usd/usd/tokens.h>
#include <usdrt/scenegraph/usd/sdf/path.h>
#include <usdrt/scenegraph/usd/usd/attribute.h>
#include <usdrt/gf/vec.h>
#include <usdrt/scenegraph/base/gf/matrix4d.h>
#include <usdrt/scenegraph/usd/usdPhysics/rigidBodyAPI.h>
#include <usdrt/scenegraph/usd/usdGeom/xformable.h>
#include <usdrt/scenegraph/usd/rt/xformable.h>

#include <algorithm>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <execution>
#include <regex>
#include <array>
#include <random>
#include <iostream>
#include <iomanip>

#include "math_utils.hpp"

namespace isaac::generic::mag_sensor{

static usdrt::UsdStageRefPtr getActiveStage(){
    const std::vector<PXR_NS::UsdStageRefPtr> stages = PXR_NS::UsdUtilsStageCache::Get().GetAllStages();
    if (stages.size() != 1){
        return nullptr;
    }
    auto stage_id = PXR_NS::UsdUtilsStageCache::Get().GetId(stages[0]).ToLongInt();
    return usdrt::UsdStage::Attach(omni::fabric::UsdStageId(stage_id));
}

class SensorBodyUtils{
public:
    SensorBodyUtils(){}
    ~SensorBodyUtils(){}
    struct SensorBodyParamters{
        float mass = 0.0;
        usdrt::GfVec3f diagonal_inertias{0, 0, 0}; // Diagonal inertias of the sensor body link (Ixx, Iyy, Izz) in kg*m^2.
        usdrt::GfVec3f center_of_mass{0, 0, 0};    // Center of mass of the sensor body link (x, y, z) in meters.
        usdrt::GfVec3f body_drag_coef_{0.0, 0.0, 0.0}; // optional (the body is quite small)
    };
    struct SensorBodyMotion{
        usdrt::GfVec3d translate{0.0, 0.0, 0.0}; // sensor body pos in [m]
        usdrt::GfQuatd orient{1.0, 0.0, 0.0, 0.0}; // sensor body orientation in quaternions
    };

    const void reset(){
        sensor_rtxform_.ClearWorldXform();
        sensor_rtxform_ = usdrt::RtXformable{};
        sensor_body_link_ = usdrt::UsdPrim{nullptr};
    }

    bool loadSensorBody(const omni::fabric::PathC &target){
        auto stage = getActiveStage();
        if(!stage){
            return false;
        }
        const std::string path = omni::fabric::toSdfPath(target).GetString();
        usdrt::UsdPrim prim = stage->GetPrimAtPath(target);
        if(!prim){
            return false;
        }

        std::regex pattern(R"(^mag_sensor_\d{2}$)");        
        if(prim.IsA(usdrt::UsdGeomXformable::_GetStaticTfType()) && (std::regex_match(prim.GetName().GetText(), pattern))){
            sensor_body_link_ = prim;
            setRtXformableAPI();
            if(prim.HasAPI(usdrt::UsdPhysicsRigidBodyAPI::_GetStaticTfType())){
                return initSensorBody();
            }
            return true;
        }else{
            return false;
        }
        CARB_LOG_WARN("%s is not a valid sensor body link or does not have Xformable API.", path.c_str());
        return false;
    }

    bool isSensorBodyPathEqual(omni::fabric::PathC& target){
        if(!sensor_body_link_){
            return false;
        }
        std::string target_path = omni::fabric::toSdfPath(target).GetString();
        std::string sensor_body_path = sensor_body_link_.GetPrimPath().GetString();
        return sensor_body_path == target_path;
    }

    const void getSensorBodyMotion(SensorBodyMotion& motions){
        if(!sensor_body_link_.IsValid()){
            return;
        }
        if(!sensor_rtxform_.HasWorldXform()){
            return;
        }
        // local coordinates
        sensor_body_link_.GetAttribute(usdrt::TfToken("xformOp:translate")).Get<usdrt::GfVec3d>(&motions.translate);
        sensor_body_link_.GetAttribute(usdrt::TfToken("xformOp:orient")).Get<usdrt::GfQuatd>(&motions.orient);
        // world coordinate based motions
        usdrt::GfVec3d pos;
        sensor_rtxform_.GetWorldPositionAttr().Get<usdrt::GfVec3d>(&pos);
        std::cout << "check pos: [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]" << std::endl;
    }


private:
    usdrt::UsdPrim sensor_body_link_;
    usdrt::RtXformable sensor_rtxform_;
    SensorBodyParamters parameters_;

    /**
     * Initializes the sensor body link params by setting up the required APIs, mass, inertia.
     *
     */
    bool initSensorBody(){
        setMassAndInertia();
        return true;
    }

    void setRtXformableAPI(){
        if(!sensor_body_link_.IsValid()){return;}
        sensor_rtxform_ = usdrt::RtXformable(sensor_body_link_);
        bool set_world_xform = sensor_rtxform_.SetWorldXformFromUsd();
        if(set_world_xform){
           CARB_LOG_INFO("World Xform from usd created world poses query for sensor enabled!"); 
        }else{
            CARB_LOG_WARN("Failed to initialize world transform from USD.");
        }
    }

    void setMassAndInertia(){
        if(!sensor_body_link_.IsValid()){
            return;
        }
        usdrt::TfToken mass = usdrt::TfToken("physics:mass");
        if(sensor_body_link_.HasAttribute(mass)){
            sensor_body_link_.GetAttribute(mass).Get<float>(&parameters_.mass);
        }
        usdrt::TfToken com = usdrt::TfToken("physics:center_of_mass");
        if(sensor_body_link_.HasAttribute(com)){
            sensor_body_link_.GetAttribute(com).Get<usdrt::GfVec3f>(&parameters_.center_of_mass);
        }
        usdrt::TfToken inertia = usdrt::TfToken("physics:diagonalInertia");
        if(sensor_body_link_.HasAttribute(inertia)){
            sensor_body_link_.GetAttribute(inertia).Get<usdrt::GfVec3f>(&parameters_.diagonal_inertias);
        }
    }

};
}
#endif