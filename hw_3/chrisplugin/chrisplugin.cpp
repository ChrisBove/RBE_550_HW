#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;

#include "chrisplugin.hpp"

class ChrisModule : public ModuleBase
{
public:
    ChrisModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&ChrisModule::MyCommand,this,_1,_2),
                        "This is an example command");
    }
    virtual ~ChrisModule() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "chrismodule" ) {
        return InterfaceBasePtr(new ChrisModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("ChrisModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

RRTNode::RRTNode(std::vector<float> configuration, RRTNode* parent){
    _configuration = configuration;
    _parent = parent;
}

RRTNode* RRTNode::getParent(){
    return _parent;
}

std::vector<float> RRTNode::getConfiguration(){
    return _configuration;
}

