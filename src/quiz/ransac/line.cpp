#include "line.h"

#include <cmath>
#include <limits>

namespace udacity_ransac
{


Line3D::Line3D(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{


    m_a = p1.y - p2.y;
    m_b = p2.x - p1.x;
    m_c = p1.x * p2.y - p2.x * p1.y;
    m_distDenom = std::sqrt(m_a * m_a + m_b * m_b);

    if (m_distDenom < std::numeric_limits<double>::epsilon())
    {
        throw std::runtime_error("Can't create line from equal points");
    }
}

double Line3D::distanceToPoint(const pcl::PointXYZ &p) const
{
    return std::fabs(m_a * p.x + m_b * p.y + m_c) / m_distDenom;
}



} // namespace udacity_ransac
