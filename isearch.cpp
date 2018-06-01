#include <algorithm>
#include <iostream>
#include "isearch.h"
#include "limits"
#include <time.h>

using namespace std;


ISearch::ISearch(){
    hweight = 1;
    breakingties = CN_SP_BT_GMAX;
}

ISearch::~ISearch(void) {}

int ISearch::open_count() {
    int count = 0;
    for (auto x: open) {
        count += x.size();
    }
    return count;
}

SearchResult ISearch::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options) {
    clock_t tStart = clock();
    Node curNode;
    curNode.i = map.start_i;
    curNode.j = map.start_j;
    curNode.H = computeHFromCellToCell(curNode.i, curNode.j, map.goal_i, map.goal_j, options);
    curNode.F = curNode.H * hweight;
    curNode.g = 0;
    curNode.parent = nullptr;
    open.resize(map.getMapHeight());
    add_or_update(curNode);
    bool pathfound = false;
    sresult.pathfound = false;

    while (open_count() != 0) {
        Node best;
        best.F = HUGE_VAL;
        vector<Node>::iterator to_erase;
        int tmp;
        for (int i = 0; i < open.size(); ++i) {
            vector<Node>::iterator it = open[i].begin();
            for (; it != open[i].end(); ++it) {
                if (it->F < best.F) {
                    best = *it;
                    to_erase = it;
                    tmp = i;
                } else if (it->F == best.F) {
                    if ((breakingties == CN_SP_BT_GMAX && it->g >= best.g) ||
                       (breakingties == CN_SP_BT_GMIN && it->g <= best.g)) {
                           best = *it;
                           to_erase = it;
                           tmp = i;
                       }
                }
            }
        }
        open[tmp].erase(to_erase);

        int x = curNode.i; 
        int y = curNode.j;
        curNode = best;

        close.insert({x * map.getMapWidth() + y, curNode});

        if (map.goalAchieved(x, y)) {
            pathfound = true;
            break;
        }

        auto successors = findSuccessors(curNode, map, options);
        auto parent = &close.at(x * map.getMapWidth() + y);

        for (auto node: successors) {
            node.parent = parent;
            node.H = computeHFromCellToCell(node.i, node.j, map.goal_i, map.goal_j, options);

            node.F = node.g + hweight * node.H;
            add_or_update(node);
        }
    }

    sresult.nodescreated = close.size() + open_count();
    sresult.numberofsteps = close.size();

    if (pathfound == true) {
        sresult.pathfound = true;
        makePrimaryPath(curNode);
        makeSecondaryPath();
    }

    sresult.time = (double)(clock() - tStart)/CLOCKS_PER_SEC;
    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    return sresult;
}

list<Node> ISearch::findSuccessors(Node curNode, const Map &map, const EnvironmentOptions &options) {
    list<Node> successors;
    Node next_node;

    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            int x = curNode.i, y = curNode.j;
            if (i == 0 && j == 0) { continue; }
            if (map.CellOnGrid(x+i, y+j) && map.CellIsTraversable(x+i, y+j)) {
                auto res = close.find((x+i)*map.getMapWidth() + y+j);
                if (res != close.end()) { continue; }

                if (i != 0 && j != 0) {
                    if (options.allowdiagonal == false) { continue; }
                    if (options.cutcorners == false) {
                        if (map.CellIsObstacle(x, y+j) || map.CellIsObstacle(x+i, y)) { continue; }
                    }
                    if (options.allowsqueeze == false) {
                        if (map.CellIsObstacle(x, y+j) && map.CellIsObstacle(x+i, y)) { continue; }
                    }
                }
                next_node.i = x + i;
                next_node.j = y + j;
                if (i == 0 || j == 0) {
                    next_node.g = curNode.g + 1;
                } else {
                    next_node.g = curNode.g + sqrt(2);
                }
                successors.push_back(next_node);
            }

        }
    }
    return successors;
}

void ISearch::makePrimaryPath(Node curNode) {
    Node cur_node = curNode;
    while (cur_node.parent) {
        lppath.push_front(cur_node);
        cur_node = *cur_node.parent;
    }
    lppath.push_front(cur_node);
}

void ISearch::makeSecondaryPath() {
    list<Node>::iterator it = lppath.begin();
    hppath.push_back(*it);
    for (; it != --lppath.end();) {
        int i = it->i, j = it->j;
        ++it; 
        int n_i = it->i, n_j = it->j;
        int di = n_i - i, dj = n_j - j;
        ++it;
        int dn_i = it->i - n_i, dn_j = it->j - n_j;
        if (dn_j != dj || dn_i != di) {
            --it;
            hppath.push_back(*it);
        } else {
            --it;
        }
    }
}

void ISearch::add_or_update(Node newNode) {
    int x = newNode.i;
    if (open[x].size() == 0) {
        open[x].push_back(newNode);
        return;
    }
    vector<Node>::iterator it = open[x].begin(), pos = open[x].end();
    bool found = false;
    for (; it != open[x].end(); ++it) {
        if (it->j == newNode.j) {
            if (newNode.F >= it->F) return;
            open[x].erase(it);
            break;
        }
    }
    open[newNode.i].push_back(newNode);
}
