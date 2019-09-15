#include "plane.h"

#include <cmath>
#include <limits>

namespace udacity_ransac
{

template<typename PointT>
Plane<PointT>::Plane(const PointT& p1, const PointT& p2, const PointT& p3)
{
    m_a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
    m_b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
    m_c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

    m_distDenom = std::sqrt(m_a * m_a + m_b * m_b + m_c * m_c);
    if (m_distDenom < std::numeric_limits<double>::epsilon())
    {
        throw std::runtime_error("Can't create plane model from given points");
    }

    m_d = -1.0 * (m_a * p1.x + m_b * p1.y + m_c * p1.z);
}

template<typename PointT>
double Plane<PointT>::distanceToPoint(const PointT& p) const
{
    return std::fabs(m_a * p.x + m_b * p.y + m_c * p.z + m_d) / m_distDenom;
}

} // namespace udacity_ransac
