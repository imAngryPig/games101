//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()//初始化boundingbox pMax设成最小值，pMin设成最大值，方便后续函数min,max取最大最小
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    //返回包围盒的对角线由 min 指向 max
    Vector3f Diagonal() const { return pMax - pMin; }

    // 寻找包围盒对角线跨度最大的轴 0:x  1:y  2:z
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;          // x 跨度最大
        else if (d.y > d.z)
            return 1;           // y 跨度最大
        else
            return 2;           // z跨度最大
    }
    
    // 返回包围体的表面积
    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    // 返回包围体的体积
    double BoxVolunm() const
    {
        Vector3f d = Diagonal();
        return d.x * d.y * d.z;
    }

    //获取boundingbox的中心点
    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    // float t_Min_x = (pMin.x - ray.origin.x)*invDir[0];
    // float t_Min_y = (pMin.y - ray.origin.y)*invDir[1];
    // float t_Min_z = (pMin.z - ray.origin.z)*invDir[2];
    // float t_Max_x = (pMax.x - ray.origin.x)*invDir[0];
    // float t_Max_y = (pMax.y - ray.origin.y)*invDir[1];
    // float t_Max_z = (pMax.z - ray.origin.z)*invDir[2];

    // //如果发现射线的方向是反的，调换t_min和t_max的位置。
    // if(dirIsNeg[0])
    // {
    //     float t = t_Min_x;
    //     t_Min_x = t_Max_x;
    //     t_Max_x = t;
    // }
    // if(dirIsNeg[1])
    // {
    //     float t = t_Min_y;
    //     t_Min_y = t_Max_y;
    //     t_Max_y = t;
    // }
    // if(dirIsNeg[2])
    // {
    //     float t = t_Min_z;
    //     t_Min_z = t_Max_z;
    //     t_Max_z = t;
    // }
 
    // float t_enter = std::max(t_Min_x, std::max(t_Min_y, t_Min_z));
    // float t_exit  = std::min(t_Max_x, std::min(t_Max_y, t_Max_z));
    // if(t_enter < t_exit && t_exit >= 0)
    //     return true;
    // return false;

    Vector3f mMin=(pMin-ray.origin)*invDir;
    Vector3f mMax=(pMax-ray.origin)*invDir;
    Vector3f mmMin(std::min(mMin.x,mMax.x),std::min(mMin.y,mMax.y),std::min(mMin.z,mMax.z));
    Vector3f mmMax(std::max(mMin.x,mMax.x),std::max(mMin.y,mMax.y),std::max(mMin.z,mMax.z));
    float t_ent=std::max(std::max(mmMin.x,mmMin.y),mmMin.z);
    float t_exit=std::min(std::min(mmMax.x,mmMax.y),mmMax.z);
    if(t_ent < t_exit && t_exit >= 0)
        return true;
    return false;

}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2) //合并两个盒子
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p) //输入一个boundingbox,一个point，合并成一个boundingbox
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
