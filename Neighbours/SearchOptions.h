#ifndef TYPES
#define TYPES

enum NeighbourSearchTypes
{
    Radius,
    Cyclinderical,
    KNearest,
    Voxel
};

enum NeighbourSearchDataStructure
{
  KdTree,
  OctTree
};

struct SearchParameter
{
    float radius;
    int kNearest;
    float resolution;
};

struct SearchOption
{
    NeighbourSearchDataStructure neighbourSearchDataStructure;
    NeighbourSearchTypes neighbourSearchTypes;
    SearchParameter searchParameter;
};
#endif // TYPES
