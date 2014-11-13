#ifndef __BARRIER_H__
#define __BARRIER_H__
#include <vector>


class Intersection;
class Section;
class AmuPoint;

class Barrier
{
  public:
    Barrier(Intersection* i0,
    Section* s1,
    Section* s2);
    double x(int i),y(int i),z(int i);
    Intersection* intersection(int i);

  private:
    // Barrierの4つの交点
    std::vector<AmuPoint*> _vertices;
    Section* _section[2];
    Intersection* _intersection; 
};

#endif //__BARRIER_H__
