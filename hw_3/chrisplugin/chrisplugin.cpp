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
#include <random>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>


#include "chrisplugin.hpp"

#define STEP_SIZE 0.5

// from http://stackoverflow.com/questions/26965508/infinite-while-loop-and-control-c
volatile sig_atomic_t stop;

void inthand(int signum) {
    stop = 1;
}

class ChrisModule : public ModuleBase
{
private:
    NodeTree nodeTree;
    std::vector<double> goalConfiguration;
    std::vector<double> dofWeights;
    std::vector<double> dofLimUpper;
    std::vector<double> dofLimLower;
    RobotBasePtr robot;

    const double stepSize = STEP_SIZE;

    // http://stackoverflow.com/questions/14638739/generating-a-random-double-between-a-range-of-values
    //Mersenne Twister: Good quality random number generator
    std::mt19937 rng;
    std::vector<std::uniform_real_distribution<double>> distributions;
    std::uniform_real_distribution<double> zeroToOneDist;

public:
    ChrisModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv), zeroToOneDist(0,1) {
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

    enum ExtendCodes { Reached, Trapped, Advanced};

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

    	std::vector<double> startingConfig;
    	robot.get()->GetActiveDOFValues(startingConfig);
    	if(isColliding(&startingConfig)){
    		std::cout << "ERROR: ROBOT IS ALREADY IN COLLISION STATE. ABORT." << std::endl;
    		return false;
    	}

    	if(isColliding(&goalConfiguration)){
    		std::cout << "ERROR: GOAL CONFIGURATION IN COLLISION STATE. ABORT." << std::endl;
    		return false;
    	}

    	// create first node and add it to the tree
    	nodeTree.addNode(startingConfig, NULL);

    	// save joint limits
    	robot.get()->GetActiveDOFLimits(dofLimLower, dofLimUpper);

    	std::cout << std::endl << "saved lower joint limits: " << std::endl;
    	for (auto i = dofLimLower.begin(); i != dofLimLower.end(); ++i){
    		std::cout << *i << ' ';
    	}
    	std::cout << std::endl << "saved upper joint limits: " << std::endl;
    	for (auto i = dofLimUpper.begin(); i != dofLimUpper.end(); ++i){
    		std::cout << *i << ' ';
    	}
    	std::cout << std::endl << std::endl;

    	// generate random uniform distributions from http://stackoverflow.com/questions/14638739/generating-a-random-double-between-a-range-of-values
    	//Initialize with non-deterministic seeds
    	rng.seed(std::random_device{}());
    	generateDistributions();

    	bool foundGoal = false;
    	unsigned iterations = 0;

    	while(!foundGoal && !stop){
    		iterations++;
    		std::vector<double> qRand;

    		// is it bias time?
    		if(zeroToOneDist(rng) <= 0.05) {
    			qRand = goalConfiguration;
    		}
    		else{
    			// sample randomly from c-space
    			getRandomConfig(&qRand);
    		}
//    		std::cout << std::endl << "a random joint value: " << std::endl;
//    		for (auto i = qRand.begin(); i != qRand.end(); ++i){
//    			std::cout << *i << ' ';
//    		}
//    		std::cout << std::endl << std::endl;

    		// find nearest neighbor
    		RRTNode* nearestNode = nodeTree.nearestNeighbor(qRand, dofWeights);

//    		std::cout << std::endl << "nearest neighbor joint value: " << std::endl;
//    		for (auto i = nearestNode->getConfiguration().begin(); i != nearestNode->getConfiguration().end(); ++i){
//    			std::cout << *i << ' ';
//    		}
//    		std::cout << std::endl << std::endl;

    		// connect - try to connect to tree
    		ExtendCodes code = connect(qRand, nearestNode);
    		if(code != ExtendCodes::Trapped){
    			if(code == ExtendCodes::Reached){
    				// return the path to the goal
    				std::cout << "WE REACHED THE GOAL" << std::endl;
    				foundGoal = true;
    				break;
    			}
    		}
    		// for bi-directional, this is where we switch which tree we grow

    	}
    	isColliding(&startingConfig);

    	std::cout << "took " << iterations << " iterations to find the goal" << std::endl;

    	if(foundGoal){
    		// save path
    		std::vector<std::vector<double>> path = nodeTree.getPath(nodeTree.getNumElems()-1); // the last element

    		auto manip = robot->GetActiveManipulator();

    		// display eef position as red points in environment
    		for (auto config : path){
    			robot.get()->SetActiveDOFValues(config);
    			auto eef = manip.get()->GetEndEffectorTransform();
//    			GetEnv()->plot3(eef.trans, 10);
//    			GetEnv()->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),1.0f);

//    			with robot: # lock environment and save robot state
//    			    robot.SetDOFValues([2.58, 0.547, 1.5, -0.7],[0,1,2,3]) # set the first 4 dof values
//    			    Tee = manip.GetEndEffectorTransform() # get end effector
//    			    ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
//    			    sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions
//
//    			h = env.plot3(Tee[0:3,3],10) # plot one point
    		}
    		// smooth the path

    		// execute the robot trajectory
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

    std::vector<double> printedArrayToVector(std::string input){
//    	std::cout << "I got this input: " << input << std::endl;
    	std::vector<double> nums;

    	// http://stackoverflow.com/questions/5607589/right-way-to-split-an-stdstring-into-a-vectorstring
    	std::vector<std::string> words;
    	boost::split(words, input, boost::is_any_of(", []"), boost::token_compress_on);
    	for (auto i = words.begin(); i != words.end(); ++i){
//    	    std::cout << *i << ' ';
    	    nums.emplace_back(std::stod(*i));
    	}
    	return nums;
    }

    void generateDistributions(){
    	// for each joint, generate the distribution
    	for (unsigned i = 0; i < dofLimLower.size(); i++){
    		if(abs(dofLimLower[i]) > 2*PI){
    			std::uniform_real_distribution<double> dist(-2*PI, 2*PI);
    			distributions.push_back(dist);
    		}
    		else{
    			std::uniform_real_distribution<double> dist(dofLimLower[i], dofLimUpper[i]);
    			distributions.push_back(dist);
    		}
    	}
    }

    void getRandomConfig(std::vector<double>* config){
    	bool successful = false;
    	while(!successful){
    		config->clear();
    		// for each joint, pick a random number between the limits
    		for (unsigned i = 0; i < dofLimLower.size(); i++){
    			config->push_back(distributions[i](rng));
    		}
    		if(!isColliding(config))
    			successful = true;
    	}
    }

    bool isColliding(std::vector<double>* config){
    	robot.get()->SetActiveDOFValues(*config);
    	return GetEnv()->CheckCollision(robot);
    }

    ExtendCodes connect(std::vector<double> config, RRTNode* nearest){
    	if (configsClose(config, nearest->getConfiguration())){
    		std::cout << "This node was too close to nearest - same thing" << std::endl;
    		return ExtendCodes::Trapped;
    	}
    	// try to step towards the configuration, avoiding collisions
    	std::vector<double> vNear = nearest->getConfiguration();
    	// calculate vector between config and nearest
    	Eigen::VectorXd vector(7);
    	vector << (config[0]-vNear[0]),(config[1]-vNear[1]),(config[2]-vNear[2]),(config[3]-vNear[3]),(config[4]-vNear[4]),(config[5]-vNear[5]),(config[6]-vNear[6]);
    	Eigen::VectorXd normalized = vector/vector.norm();

    	// save this, we need to add it back to the normalized vector to get a valid configuration
    	Eigen::VectorXd nearestVector(7);
    	nearestVector << (vNear[0]),(vNear[1]),(vNear[2]),(vNear[3]),(vNear[4]),(vNear[5]),(vNear[6]);

    	bool done = false;
    	unsigned stepCount = 1;
    	RRTNode* parent = nearest;
    	while(!done && !stop){
    		Eigen::VectorXd desired(7);
    		desired = (stepSize*stepCount)*normalized;
    		desired = desired + nearestVector;
    		std::vector<double> desiredConfig = eigenVectorToV(desired);

    		// if not in a collision
    		if(!isColliding(&desiredConfig)){
    			// add this node to the tree, set parent to the new node
    			parent = nodeTree.addNode(desiredConfig, parent);

    			// if this is the goal, yaay!
    			if(configsClose(goalConfiguration, desiredConfig)){
    				return ExtendCodes::Reached;
    			}
    			// if reached, report back
    			if(configsClose(config, desiredConfig)){
    				return ExtendCodes::Advanced;
    			}
    		}
    		else{
    			std::cout << "new config collided - trapped" << std::endl;
    			// we're done here - we are trapped
    			done = true;
    		}
    		stepCount++; // duh!
    	}
    	return ExtendCodes::Trapped;
    }

    std::vector<double> eigenVectorToV(Eigen::VectorXd eVect){
    	std::vector<double> vector;
    	for(unsigned i = 0; i < 7; i++) {
    		vector.push_back(eVect[i]);
    	}
    	return vector;
    }

    bool configsClose(std::vector<double> config1, std::vector<double> config2){
    	if(euclidDistance(config1, config2) < STEP_SIZE)
    		return true;
    	return false;
//    	for (unsigned i = 0; i < config1.size(); i++){
//    		if(std::abs(config1[i] - config2[i]) > 0.01)
//    			return false;
//    	}
//    	return true;
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

RRTNode::RRTNode(std::vector<double> configuration, RRTNode* parent){
    _configuration = configuration;
    _parent = parent;
}

RRTNode* RRTNode::getParent(){
    return _parent;
}

std::vector<double> RRTNode::getConfiguration(){
    return _configuration;
}

void RRTNode::setParent(RRTNode* node){
    _parent = node;
}

void RRTNode::setConfiguration(std::vector<double> configuration){
    _configuration = configuration;
}

NodeTree::NodeTree(){

}

void NodeTree::addNode(RRTNode* node){
    _nodes.push_back(node);
}

RRTNode* NodeTree::addNode(std::vector<double> configuration, RRTNode* parent){
//	std::vector<float> config = configuration;
//	RRTNode* par = parent;
	_nodes.push_back(new RRTNode(configuration, parent));
//    _nodes.emplace_back(config, par); // faster, allocates in place
	return _nodes.front();
}

void NodeTree::deleteNode(unsigned index){
    _nodes.erase(_nodes.begin()+index);
}

RRTNode* NodeTree::getNode(unsigned index){
    return _nodes[index];
}

unsigned NodeTree::getNumElems(){
	return _nodes.size();
}

std::vector<std::vector<double>> NodeTree::getPath(unsigned index){
    std::vector<std::vector<double>> path;
    RRTNode* currentNode = _nodes[index];

    path.push_back(currentNode->getConfiguration());

    while(currentNode->getParent() != NULL){
        currentNode = currentNode->getParent();
        path.push_back(currentNode->getConfiguration());
    }

    std::reverse(path.begin(), path.end());

    return path;
}

HeapableRRTNode::HeapableRRTNode(RRTNode* node, double cost){
	_cost = cost;
	_node = node;
}

double HeapableRRTNode::getCost(){
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
RRTNode* NodeTree::nearestNeighbor(std::vector<double> configuration, std::vector<double> weights){
    std::vector<HeapableRRTNode> neighbors;
    neighbors.reserve(_nodes.size()); // preallocate number of elements

    // compute distances from this configuration to the neighbors we have
    unsigned count = 0;
    double lowestDistance = weightedEuclidDistance(_nodes.front()->getConfiguration(),configuration,weights);
    RRTNode* closestNode = _nodes.front();
    for (RRTNode* node : _nodes){
    	double dist = weightedEuclidDistance(node->getConfiguration(),configuration,weights);
//    	std::cout << "Dist: " << dist << std::endl;
    	// only add nodes that have a chance of being lower
    	if (dist < lowestDistance){
//    		neighbors.emplace_back(node, dist);
    		lowestDistance = dist;
    		closestNode = node;
    	}
    	count++;
    }
    std::cout << "Counted " << count << " neighbors." << std::endl;
//    // heap them so that the shortest is at the top
//    std::make_heap(neighbors.begin(), neighbors.end(), costComparitor());
    // pick the one at the top
//    return neighbors.front().getRRTNode();
    return closestNode;
}

double weightedEuclidDistance(std::vector<double> configuration1, std::vector<double> configuration2, std::vector<double> weights){
    double sum = 0;
    for(unsigned i = 0; i < configuration1.size(); i++){
        sum += pow(weights[i]*(configuration1[i] - configuration2[i]),2);
    }
    return sqrt(sum);
}

double euclidDistance(std::vector<double> configuration1, std::vector<double> configuration2){
    double sum = 0;
    for(unsigned i = 0; i < configuration1.size(); i++){
        sum += pow(configuration1[i] - configuration2[i],2);
    }
    return sqrt(sum);
}
