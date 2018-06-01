#ifndef JP_SEARCH_H
#define JP_SEARCH_H
#include "astar.h"

class JP_Search:public Astar
{
public:
    JP_Search(float hweight, bool breakingties):Astar(hweight, breakingties){}
    ~JP_Search();

private:
    std::list<Node> findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath();
};

#endif // JP_SEARCH_H
