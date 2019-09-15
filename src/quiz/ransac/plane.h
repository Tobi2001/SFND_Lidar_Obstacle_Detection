#ifndef PLANE_H
#define PLANE_H

#include "pcl/common/common.h"

namespace udacity_ransac
{

template<typename PointT>
class Plane
{

public:
    Plane(const PointT& p1, const PointT& p2, const PointT& p3);

    double distanceToPoint(const PointT& p) const;

private:
    double m_a;
    double m_b;
    double m_c;
    double m_d;
    double m_distDenom;
};

} // namespace udacity_ransac

#endif // PLANE_H
