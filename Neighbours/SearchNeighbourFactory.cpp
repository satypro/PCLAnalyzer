#include "SearchNeighbourFactory.h"
#include "SearchNeighbourOctTree.h"

std::map<NeighbourSearchDataStructure, SearchNeighbourBase*> SearchNeighbourFactory::_neighbourSearchMap;

SearchNeighbourFactory::SearchNeighbourFactory()
{

}

SearchNeighbourBase* SearchNeighbourFactory::GetNeighbourSearchDataStructure(NeighbourSearchDataStructure dataStructure)
{
    std::map<NeighbourSearchDataStructure, SearchNeighbourBase*>::iterator it;
    it = SearchNeighbourFactory::_neighbourSearchMap.find(dataStructure);

    if (it != SearchNeighbourFactory::_neighbourSearchMap.end())
        return it->second;

    SearchNeighbourBase* instance = SearchNeighbourFactory::GetInstance(dataStructure);
    SearchNeighbourFactory::_neighbourSearchMap.insert(
                std::pair<NeighbourSearchDataStructure, SearchNeighbourBase*>(dataStructure, instance)
     );

    return instance;
}

SearchNeighbourBase* SearchNeighbourFactory::GetInstance(NeighbourSearchDataStructure dataStructure)
{
    SearchNeighbourBase* object;
    switch(dataStructure)
    {
        case KdTree:
            object = 0;
        break;
        case OctTree:
            object = new SearchNeighbourOctTree();
        default:
            object = new SearchNeighbourOctTree();
    }

    return object;
}
