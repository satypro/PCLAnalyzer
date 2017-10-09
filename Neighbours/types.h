#ifndef TYPES
#define TYPES

enum NeighbourSearchTypes
{
    Radius,
    Cyclinderical,
    KNearest
};

struct SearchScale
{
    float radius;
    int kNearest;
    float resolution;
};

#endif // TYPES
