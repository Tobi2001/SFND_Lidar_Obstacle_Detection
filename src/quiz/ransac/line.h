#ifndef LINE_H
#define LINE_H

#include <pcl/common/common.h>
#include <vector>

namespace udacity_ransac
{

class Line3D
{
public:
    Line3D(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

    double distanceToPoint(const pcl::PointXYZ& p) const;

private:
    double m_a;
    double m_b;
    double m_c;
    double m_distDenom;
};

} // namespace udacity_ransac

#endif // LINE_H
