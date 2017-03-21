#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;
#include <algorithm>
#include <math.h>
#include <vector>
#include <sstream>
#include <string>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split

#include "chrisplugin.hpp"

class ChrisModule : public ModuleBase
{
public:
    ChrisModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&ChrisModule::MyCommand,this,_1,_2),
                        "This is an example command");
        RegisterCommand("FindPath",boost::bind(&ChrisModule::FindPath,this,_1,_2),
                                "This uses RRT to find a path to the given goal");
    }
    virtual ~ChrisModule() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }

    /**
     * produce path in under 3 minutes
     * allow goal bias adjustments from 1-96%
     * parse the input: [0.449,-0.201,-0.151,-0.11,0,-0.11,0][3.17104,2.75674,2.2325,1.78948,1.42903,0.809013,0.593084]
     *
     */
    bool FindPath(std::ostream& sout, std::istream& sinput){
    	std::string input;
    	sinput >> input;
    	std::cout << "I got this input: " << input << std::endl;

    	std::vector<std::string> arrays;
    	boost::split(arrays, input, boost::is_any_of("["), boost::token_compress_on);
    	std::cout << "And I parsed these arrays:" << std::endl;
    	for (auto i = arrays.begin(); i != arrays.end(); ++i){
    		std::cout << *i << ' ' << std::endl;
    		printedArrayToVector(*i);
    	}
//    	printedArrayToVector(arrays[0]);
//    	printedArrayToVector(arrays[1]);
    	return true;
    }

    std::vector<float> printedArrayToVector(std::string input){
    	std::cout << "I got this input: " << input << std::endl;
    	std::vector<float> nums;
//    	std::string::size_type sz;     // alias of size_t
//    	std::stringstream ss(input);
//    	float val;
//    	while (ss >> val){
//
//    		nums.push_back(val);
//    	}
    	// http://stackoverflow.com/questions/5607589/right-way-to-split-an-stdstring-into-a-vectorstring
    	std::vector<std::string> words;
    	boost::split(words, input, boost::is_any_of(", []"), boost::token_compress_on);
    	for (auto i = words.begin(); i != words.end(); ++i){
//    	    std::cout << *i << ' ';
//    	    nums.push_back(std::stof(*i));
    	}
    	return nums;
    }

private:
    NodeTree nodeTree;


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

