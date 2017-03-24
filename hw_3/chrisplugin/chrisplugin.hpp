#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <vector>
using namespace OpenRAVE;

double weightedEuclidDistance(std::vector<double> configuration1, std::vector<double> configuration2, std::vector<double> weights);
double euclidDistance(std::vector<double> configuration1, std::vector<double> configuration2);

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

class NodeTree
{
public:
	NodeTree();
	void addNode(RRTNode* node);
	RRTNode* addNode(std::vector<double> configuration, RRTNode* parent);
	void deleteNode(unsigned node);
	void clear();
	RRTNode* getNode(unsigned index);
	RRTNode* getBack();
	std::vector<std::vector<double>> getPath(unsigned index);
	std::vector<std::vector<double>> getPath(unsigned parentI, unsigned childI);
	std::vector<std::vector<double>> getReversedPath(unsigned parentI, unsigned childI);

	unsigned getNumElems();

	RRTNode* nearestNeighbor(std::vector<double> configuration, std::vector<double> weights);

private:
	std::vector<RRTNode*> _nodes;

};
