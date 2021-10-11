#define PCL_NO_PRECOMPILE
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct PointXYZIT
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;
    double gps_time;
    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
    inline PointXYZIT() :PointXYZIT(0, 0, 0, 0, 0) {}

    inline PointXYZIT(float _x, float _y, float _z, float _intensity, double _gps_time) {
        x = _x; y = _y; z = _z; intensity = _intensity, gps_time = _gps_time;
    }
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,           // here we assume a XYZ + "test" (as fields)
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, gps_time, gps_time)
)
