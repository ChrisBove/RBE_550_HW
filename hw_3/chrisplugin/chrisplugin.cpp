#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;
#include <algorithm>
#include <math.h>
#include <vector>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <cstdlib>


#include "chrisplugin.hpp"

class ChrisModule : public ModuleBase
{
private:
    NodeTree nodeTree;
    std::vector<float> goalConfiguration;
    std::vector<float> dofWeights;
    std::vector<float> dofLimUpper;
    std::vector<float> dofLimLower;
    RobotBasePtr robot;

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
    	parseArguments(sinput);
    	// now we have our weight and goal vector

    	// obtain starting configuration
    	std::vector<RobotBasePtr> robots;
    	GetEnv()->GetRobots(robots);
    	robot = robots[0];

    	std::vector<double> dJointVals;
    	robot.get()->GetActiveDOFValues(dJointVals);
    	std::vector<float> jointVals(dJointVals.begin(), dJointVals.end());

    	// create first node and add it to the tree
    	nodeTree.addNode(jointVals, NULL);

    	// save joint limits
    	std::vector<double> dUpperLim, dLowerLim;
    	robot.get()->GetActiveDOFLimits(dLowerLim, dUpperLim);
    	std::vector<float> tempLimLower(dLowerLim.begin(), dLowerLim.end());
    	dofLimLower = tempLimLower;
    	std::vector<float> tempLimUpper(dUpperLim.begin(), dUpperLim.end());
    	dofLimUpper = tempLimUpper;
    	std::cout << std::endl << "saved lower joint limits: " << std::endl;
    	for (auto i = dofLimLower.begin(); i != dofLimLower.end(); ++i){
    		std::cout << *i << ' ';
    	}
    	std::cout << std::endl << "saved upper joint limits: " << std::endl;
    	for (auto i = dofLimUpper.begin(); i != dofLimUpper.end(); ++i){
    		std::cout << *i << ' ';
    	}
    	std::cout << std::endl << std::endl;

    	for (unsigned k = 0; k < 2000; k++){
    		// sample randomly from c-space
    		std::vector<float> qRand;
    		getRandomConfig(&qRand);

//    		std::cout << std::endl << "a random joint value: " << std::endl;
//    		for (auto i = qRand.begin(); i != qRand.end(); ++i){
//    			std::cout << *i << ' ';
//    		}
//    		std::cout << std::endl << std::endl;

    		// find nearest neighbor
    		RRTNode* nearestNode = nodeTree.nearestNeighbor(qRand, dofWeights);


    		// extend - try to connect to tree
    		extend(qRand, nearestNode);

    	}
    	return true;
    }

    void parseArguments(std::istream& sinput){
    	std::string input;
    	sinput >> input;
    	std::cout << "Find Path input: " << input << std::endl;

    	std::vector<std::string> arrays;
    	boost::split(arrays, input, boost::is_any_of("[]"), boost::token_compress_on);
//		std::cout << "And I parsed these arrays:" << std::endl;

    	unsigned arrayCount = 0;
    	for (auto array : arrays){
//    		std::cout << "size: " << array.size() << " values: ";
//    		std::cout << array << std::endl;
    		if(array.size() != 0){
    			if (arrayCount == 0){
    				goalConfiguration = printedArrayToVector(array);
    				arrayCount = 1;
    			}
    			else if(arrayCount == 1) {
    				dofWeights = printedArrayToVector(array);
    				arrayCount = 2;
    			}
    		}
    	}

    	std::cout << std::endl << "saved goal configuration vector: " << std::endl;
    	for (auto i = goalConfiguration.begin(); i != goalConfiguration.end(); ++i){
    		std::cout << *i << ' ';
    	}
    	std::cout << std::endl << "saved dof weight vector: " << std::endl;
    	for (auto i = dofWeights.begin(); i != dofWeights.end(); ++i){
    		std::cout << *i << ' ';
    	}
    	std::cout << std::endl << std::endl;
    }

    std::vector<float> printedArrayToVector(std::string input){
//    	std::cout << "I got this input: " << input << std::endl;
    	std::vector<float> nums;

    	// http://stackoverflow.com/questions/5607589/right-way-to-split-an-stdstring-into-a-vectorstring
    	std::vector<std::string> words;
    	boost::split(words, input, boost::is_any_of(", []"), boost::token_compress_on);
    	for (auto i = words.begin(); i != words.end(); ++i){
//    	    std::cout << *i << ' ';
    	    nums.emplace_back(std::stof(*i));
    	}
    	return nums;
    }

    void getRandomConfig(std::vector<float>* config){
    	// random help from http://stackoverflow.com/questions/686353/c-random-float-number-generation
    	// seed rand
    	std::srand(static_cast <unsigned> (time(0)));

    	// for each joint, pick a random number between the limits
    	for (unsigned i = 0; i < dofLimLower.size(); i++){
    		float r3 = dofLimLower[i] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(dofLimUpper[i]-dofLimLower[i])));
    		config->push_back(r3);
    	}
    }

    bool isColliding(std::vector<float>* config){
//    	robot.get()->SetActiveDOFValues(*config);
    	return GetEnv()->CheckCollision(robot);
    }

    bool extend(std::vector<float> config, RRTNode* nearest){
    	// try to step towards the configuration, avoiding collisions


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

void NodeTree::addNode(std::vector<float> configuration, RRTNode* parent){
//	std::vector<float> config = configuration;
//	RRTNode* par = parent;
	_nodes.push_back(new RRTNode(configuration, parent));
//    _nodes.emplace_back(config, par); // faster, allocates in place
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

