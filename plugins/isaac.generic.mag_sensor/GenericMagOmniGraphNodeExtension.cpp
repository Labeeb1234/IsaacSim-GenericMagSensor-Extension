#define CARB_EXPORTS

#include <carb/PluginUtils.h>

#include <omni/ext/IExt.h>
#include <omni/kit/IApp.h>
#include <omni/graph/core/IGraphRegistry.h>
#include <omni/graph/core/ogn/Database.h>
#include <omni/graph/core/ogn/Registration.h>


const struct carb::PluginImplDesc pluginImplDesc = { 
    "isaac.generic.mag_sensor.plugin",
    "A generic simulated plugin for magnetometer sensor for isaac sim", "generic",
    carb::PluginHotReload::eEnabled, "dev" 
};

CARB_PLUGIN_IMPL_DEPS(
    omni::graph::core::IGraphRegistry, 
    omni::fabric::IPath, 
    omni::fabric::IToken, 
    omni::kit::IApp)

DECLARE_OGN_NODES()

namespace isaac::generic::mag_sensor{

class GenericMagOmnigraphNodeExtension : public omni::ext::IExt{
public:
    void onStartup(const char *extId) override{
        printf("Generic Magnetometer Sensor extension startup..... (ext_id: %s).\n", extId);
        INITIALIZE_OGN_NODES()
    }
    void onShutdown() override{
        printf("Generic Magnetometer Sensor extension shutdown\n");
        RELEASE_OGN_NODES()
    }

private:
};

}

CARB_PLUGIN_IMPL(pluginImplDesc, isaac::generic::mag_sensor::GenericMagOmnigraphNodeExtension)

void fillInterface(isaac::generic::mag_sensor::GenericMagOmnigraphNodeExtension& iface)
{
}