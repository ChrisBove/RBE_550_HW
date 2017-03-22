#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <vector>
using namespace OpenRAVE;

class RRTNode
{
public:
	RRTNode(std::vector<float> configuration, RRTNode* parent);
	RRTNode* getParent();
	void setParent(RRTNode* node);
	std::vector<float> getConfiguration();
	void setConfiguration(std::vector<float> configuration);


private:
	std::vector<float> _configuration;
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
	void addNode(std::vector<float> configuration, RRTNode* parent);
	void deleteNode(unsigned node);
	RRTNode* getNode(unsigned index);
	std::vector<RRTNode*> getPath(unsigned index);

	RRTNode* nearestNeighbor(std::vector<float> configuration, std::vector<float> weights);

private:
	std::vector<RRTNode*> _nodes;

	float weightedEuclidDistance(std::vector<float> configuration1, std::vector<float> configuration2, std::vector<float> weights);

};
