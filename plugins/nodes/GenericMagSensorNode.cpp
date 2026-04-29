#include <GenericMagSensorNodeDatabase.h>
#include "custom_mag_sensor.hpp"

#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>
#include <carb/events/EventsUtils.h>


using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

namespace isaac::generic::mag_sensor{
class GenericMagSensorNode{
public:
    static void initialize(const GraphContextObj &context, const NodeObj &nodeObj){
        constexpr u_char kDefaultSystemId = 1; // placeholder
        const auto graphInstanceIndex = nodeObj.iNode->getGraphInstanceID(nodeObj.nodeHandle, InstanceIndex{0});
        auto &state = GenericMagSensorNodeDatabase::sPerInstanceState<GenericMagSensorNode>(nodeObj, graphInstanceIndex);
        
        state.m_sensor_body_ = std::make_unique<SensorBodyUtils>();
        state.m_sensor_body_->reset();
        state.mag_sensor_ = std::make_unique<MagnetoSensor>();
        MagnetoSensor::MagParameters& mag_params = state.mag_sensor_->getParameters();
        // onValueCallback for timeline events (e.g., reset)
        auto cb = [](const omni::graph::core::AttributeObj &attr, const void *value){
            const NodeObj nodeObj = attr.iAttribute->getNode(attr);
            const auto graphInstanceIndex = nodeObj.iNode->getGraphInstanceID(nodeObj.nodeHandle, InstanceIndex{0});
            auto &state = GenericMagSensorNodeDatabase::sPerInstanceState<GenericMagSensorNode>(nodeObj, graphInstanceIndex);
            MagnetoSensor::MagParameters& mag_params = state.mag_sensor_->getParameters();
            // magnetometer attributes extraction
            if(attr.iAttribute->getNameToken(attr) == state::magnetometerNoiseDensity.token()){
                mag_params.magnetometer_noise_density = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::magnetometerRandomWalk.token()){
                mag_params.magnetometer_random_walk = *static_cast<const float *>(value);
            }
            if(attr.iAttribute->getNameToken(attr) == state::magnetometerBiasCorrelationTime.token()){
                mag_params.magnetometer_bias_correlation_time = *static_cast<const float *>(value);
            }
        };
        
        AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, state::systemId.token());
        attr.iAttribute->setDefaultValue(attr, omni::fabric::BaseDataType::eUChar, &kDefaultSystemId, 0);
        // Helper function to initialize attributes
        auto initializeAttribute = [&](const auto &token, omni::fabric::BaseDataType type, const void *defaultValue){
            AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, token);
            attr.iAttribute->setDefaultValue(attr, type, defaultValue, 0);
            attr.iAttribute->registerValueChangedCallback(attr, cb, true);
        };
        // initialize attributes
        initializeAttribute(state::magnetometerNoiseDensity.token(), omni::fabric::BaseDataType::eFloat, &mag_params.magnetometer_noise_density);
        initializeAttribute(state::magnetometerRandomWalk.token(), omni::fabric::BaseDataType::eFloat, &mag_params.magnetometer_random_walk);
        initializeAttribute(state::magnetometerBiasCorrelationTime.token(), omni::fabric::BaseDataType::eFloat, &mag_params.magnetometer_bias_correlation_time);
        if(auto timeline = omni::timeline::getTimeline()){
            state.m_timelineEventsSubscription = carb::events::createSubscriptionToPop(
                timeline->getTimelineEventStream(),
                [&state](carb::events::IEvent *timelineEvent){
                    if(static_cast<omni::timeline::TimelineEventType>(timelineEvent->type) == omni::timeline::TimelineEventType::eStop){
                        // Reset sensor's accumulated bias and readings on timeline stopped
                        state.mag_sensor_->reset();
                        state.m_sensor_body_->reset();
                        CARB_LOG_INFO("Reset sensor's accumulated bias and readings");
                    }
                });
        }
    }

    static bool compute(GenericMagSensorNodeDatabase& db){
        const auto& time = db.inputs.time();
        const auto& home = db.inputs.homeCoordinate(); // hardcoded for now the home of the drone is taken as the home coordinate for the mag sensor
        const auto &rate_mag = db.inputs.rateMag();
        const auto &sensor_in = db.inputs.sensorBodyLink();
        auto &mag_data_out = db.outputs.magneticFieldVector();

        // inputs sanity checks and fixes
        if (sensor_in.empty())
        {
            CARB_LOG_WARN("Sensor body link is not set. Please set the sensor body link to sample magnetometer readings.");
            return false;
        }
        if(time<=0){
            CARB_LOG_WARN("Invalid time input: %f. Time must be positive.", time);
            return false;
        }
        auto &state = db.internalState<GenericMagSensorNode>();
        omni::fabric::PathC sensorPath = sensor_in[0];
        if(!state.m_sensor_body_->isSensorBodyPathEqual(sensorPath)){
            if(!state.m_sensor_body_->loadSensorBody(sensorPath)){
                CARB_LOG_WARN("Failed to load sensor body link at path.");
                return false;
            }
        }

        // data processing part
        SensorBodyUtils::SensorBodyMotion motion;
        state.m_sensor_body_->getSensorBodyMotion(motion);
        uint64_t current_time_us = static_cast<uint64_t>(time * 1e6); // time in microseconds
        if(rate_mag > 0){
            uint64_t mag_period_us = static_cast<uint64_t>(1e6 / rate_mag);
            uint64_t mag_elapsed = current_time_us - state.mag_last_sampled_us_;
            if(mag_elapsed >= mag_period_us){
                double dt = mag_elapsed * 1e-6;
                state.mag_sensor_->sample(motion, home[0], home[1], dt);
                state.mag_last_sampled_us_ = current_time_us;
            }
        }
        auto mag_data = state.mag_sensor_->getMagReadings();
        for(int idx = 0; idx < 3; idx++){
            mag_data_out[idx] = mag_data.mag_field_noisy[idx];
        }
        return true;
    }

private:
    carb::ObjectPtr<carb::events::ISubscription> m_timelineEventsSubscription;
    std::unique_ptr<SensorBodyUtils> m_sensor_body_;
    uint64_t mag_last_sampled_us_ = 0U;
    std::unique_ptr<MagnetoSensor> mag_sensor_;
};
REGISTER_OGN_NODE()
}