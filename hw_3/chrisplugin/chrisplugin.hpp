#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <vector>
using namespace OpenRAVE;

class RRTNode
{
public:
	RRTNode(std::vector<double> configuration, RRTNode* parent);
	RRTNode* getParent();
	void setParent(RRTNode* node);
	std::vector<double> getConfiguration();
	void setConfiguration(std::vector<double> configuration);


private:
	std::vector<double> _configuration;
	RRTNode* _parent;
};

class HeapableRRTNode{
public:
	HeapableRRTNode(RRTNode* node, float cost);
	float getCost();
	RRTNode* getRRTNode();
	float _cost;
private:
	RRTNode* _node;

};

// make a struct for doing compares

class NodeTree
{
public:
	NodeTree();
	void addNode(RRTNode* node);
	void addNode(std::vector<double> configuration, RRTNode* parent);
	void deleteNode(unsigned node);
	RRTNode* getNode(unsigned index);
	std::vector<RRTNode*> getPath(unsigned index);

	RRTNode* nearestNeighbor(std::vector<double> configuration, std::vector<double> weights);

private:
	std::vector<RRTNode*> _nodes;

	float weightedEuclidDistance(std::vector<double> configuration1, std::vector<double> configuration2, std::vector<double> weights);

};
