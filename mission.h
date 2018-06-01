#ifndef MISSION_H
#define	MISSION_H

#include "map.h"
#include "config.h"
#include "isearch.h"
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "jp_search.h"
#include "astar.h"
#include "dijkstra.h"
#include "theta.h"
#include "xmllogger.h"

//That's the wrap up class that first creates all the needed objects (Map, Search etc.)
//and then runs the search and then cleans everything up.

//Hint: Create Mission object in the main() function and then use it 1) to retreive all the data from input XML
//2) run the search 3) flush the results to output XML

class Mission
{
    public:
        Mission();
        Mission (const char* fileName);
        ~Mission();

        bool getMap();
        bool getConfig();
        bool createLog();
        void createSearch();
        void createEnvironmentOptions();
        void startSearch();
        void printSearchResultsToConsole();
        void saveSearchResultsToLog();

    private:
        const char* getAlgorithmName();

        Map                     map;
        Config                  config;
        EnvironmentOptions      options;
        ISearch*                search;
        ILogger*                logger;
        const char*             fileName;
        SearchResult            sr;
};

#endif

