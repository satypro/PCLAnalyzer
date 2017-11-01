#ifndef SEARCHNEIGHBOURFACTORY_H
#define SEARCHNEIGHBOURFACTORY_H
#include "SearchNeighbourBase.h"
#include "SearchOptions.h"
#include <map>

class SearchNeighbourFactory
{
public:
    SearchNeighbourFactory();
    static SearchNeighbourBase* GetNeighbourSearchDataStructure(NeighbourSearchDataStructure dataStructure);
private:
    static std::map<NeighbourSearchDataStructure, SearchNeighbourBase*> _neighbourSearchMap;
    static SearchNeighbourBase* GetInstance(NeighbourSearchDataStructure dataStructure);
};

#endif // SEARCHNEIGHBOURFACTORY_H
