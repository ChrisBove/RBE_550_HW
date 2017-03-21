#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;
#include <algorithm>
#include <math.h>
#include <vector>

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

void RRTNode::setParent(RRTNode* node){
    _parent = node;
}

void RRTNode::setConfiguration(std::vector<float> configuration){
    _configuration = configuration;
}

NodeTree::NodeTree(){

}

void NodeTree::addNode(RRTNode* node){
    _nodes.push_back(node);
}

void NodeTree::deleteNode(unsigned index){
    _nodes.erase(_nodes.begin()+index);
}

RRTNode* NodeTree::getNode(unsigned index){
    return _nodes[index];
}

std::vector<RRTNode*> NodeTree::getPath(unsigned index){
    std::vector<RRTNode*> path;
    RRTNode* currentNode = _nodes[index];

    path.push_back(currentNode);

    while(currentNode->getParent() != NULL){
        currentNode = currentNode->getParent();
        path.push_back(currentNode);
    }

    std::reverse(path.begin(), path.end());

    return path;
}

float HeapableRRTNode::getCost(){
	return _cost;
}

RRTNode* HeapableRRTNode::getRRTNode(){
	return _node;
}

// from http://stackoverflow.com/questions/14016921/comparator-for-min-heap-in-c
struct costComparitor{
  bool operator()(const HeapableRRTNode& a,const HeapableRRTNode& b) const{
    return a._cost > b._cost;
  }
};

//naive method - search all nodes for closest weighted neighbor
RRTNode* NodeTree::nearestNeighbor(std::vector<float> configuration, std::vector<float> weights){
    std::vector<HeapableRRTNode> neighbors;
    neighbors.reserve(_nodes.size()); // preallocate number of elements

    // compute distances from this configuration to the neighbors we have
    for (RRTNode* node : _nodes){
    	neighbors.emplace_back(node, weightedEuclidDistance(node->getConfiguration(),configuration,weights));
    }
    // heap them so that the shortest is at the top
    std::make_heap(neighbors.begin(), neighbors.end(), costComparitor());
    // pick the one at the top
    return neighbors.front().getRRTNode();
    
}

float NodeTree::weightedEuclidDistance(std::vector<float> configuration1, std::vector<float> configuration2, std::vector<float> weights){
    float sum = 0;
    for(unsigned i = 0; i < configuration1.size(); i++){
        sum += pow(weights[i]*(configuration1[i] - configuration2[i]),2);
    }
    return sqrt(sum);
}
