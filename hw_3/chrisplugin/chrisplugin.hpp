#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <vector>
using namespace OpenRAVE;

class RRTNode
{
public:
	RRTNode(std::vector<float> configuration, RRTNode* parent);
	RRTNode* getParent();
	std::vector<float> getConfiguration();


private:
	std::vector<float> _configuration;
	RRTNode* _parent;
};

class NodeTree
{
public:
	NodeTree();
	bool addNode(RRTNode* node);
	bool deleteNode(RRTNode* node);
	RRTNode getNode(int index);
	std::vector<RRTNode*> getPath(int index);

private:
	std::vector<RRTNode*> _nodes;

};