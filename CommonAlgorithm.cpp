#include "CommonAlgorithm.h"

#include "BvhAccelerator.h"
#include <BS_thread_pool.hpp>
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <algorithm>
#include <execution>
#include <glm/detail/type_quat.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/fwd.hpp>
#include <numeric>
#include <optional>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <spdlog/spdlog.h>

namespace Common
{

std::unique_ptr<BS::thread_pool<>> CommonAlgorithm::threadPool = nullptr;

void CommonAlgorithm::InitThreadPoolNum(const size_t num)
{
  threadPool = std::make_unique<BS::thread_pool<>>(num);
}

template <class T>
void CommonAlgorithm::ParallelFor(const std::vector<T>& array, const std::function<void(const size_t)>& func)
{
  const int64 maxSize = array.size();

  std::vector<int64> indexVec(maxSize);
  std::iota(indexVec.begin(), indexVec.end(), 0);

  // std::for_each(std::execution::par, indexVec.begin(), indexVec.end(), func);
  tbb::parallel_for(tbb::blocked_range<size_t>(0, indexVec.size()), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t i = r.begin(); i < r.end(); ++i)
    {
      func(i);
    }
  });
}

void CommonAlgorithm::ParallelFor(const size_t& maxSize, const std::function<void(const size_t)>& func, const bool useMt)
{
  if (useMt)
  {
    // 使用TBB并行遍历
    tbb::parallel_for(tbb::blocked_range<size_t>(0, maxSize), [&](const tbb::blocked_range<size_t>& r) {
      for (size_t i = r.begin(); i < r.end(); ++i)
      {
        func(i);
      }
    });
  }
  else
  {
    // 直接遍历索引，无需容器
    for (size_t i = 0; i < maxSize; ++i)
    {
      func(i);
    }
  }
}

void CommonAlgorithm::ParallelFor(const size_t& maxSize, const std::function<void(const size_t)>& func, const int32 maxThreadNum)
{
  std::vector<int64> indexVec(maxSize);
  std::iota(indexVec.begin(), indexVec.end(), 0);

  std::vector<std::future<void>> futures;
  futures.reserve(10 * maxThreadNum);  // 预分配，减少 reallocate

  for (size_t i = 0; i < maxSize; ++i)
  {
    // 按值捕获 i，func 用引用捕获
    futures.emplace_back(threadPool->submit_task([i, &func]() { func(i); }));

    if ((i + 1) % (10 * maxThreadNum) == 0 || i + 1 == maxSize)
    {
      for (auto& fut : futures)
      {
        if (fut.valid())
        {              // 防御式检查
          fut.wait();  // 或 fut.get();
        }
      }
      futures.clear();
    }
  }
}

glm::dvec3 CommonAlgorithm::CovertCartesian3ToLonLatHeight(const glm::dvec3& cart3)
{
  const std::optional<CesiumGeospatial::Cartographic> cartographic = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(cart3);
  const CesiumGeospatial::Cartographic cartographicValue           = cartographic.value_or(CesiumGeospatial::Cartographic(0, 0, 0));
  const glm::dvec3 point(CesiumUtility::Math::radiansToDegrees(cartographicValue.longitude),
                         CesiumUtility::Math::radiansToDegrees(cartographicValue.latitude), cartographicValue.height);

  return point;
}

glm::dvec3 CommonAlgorithm::CovertLonLatHeightToCartesian3(const glm::dvec3& lonLatHeight)
{
  const CesiumGeospatial::Cartographic cartographic = CesiumGeospatial::Cartographic::fromDegrees(lonLatHeight.x, lonLatHeight.y, lonLatHeight.z);

  return CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(cartographic);
}

bool CommonAlgorithm::IsInCube(const glm::dvec3& point, const glm::dvec3& min, const glm::dvec3& max)
{
  // 经度范围判断
  if (point.x < min.x || point.x > max.x)
  {
    return false;
  }
  // 纬度范围判断
  if (point.y < min.y || point.y > max.y)
  {
    return false;
  }
  // 判断高程范围
  if (point.z < min.z || point.z > max.z)
  {
    return false;
  }
  return true;
}

bool CommonAlgorithm::IsInRect(const glm::dvec3& point, const glm::dvec3& min, const glm::dvec3& max)
{
  // 经度范围判断
  if (point.x < min.x || point.x > max.x)
  {
    return false;
  }
  // 纬度范围判断
  if (point.y < min.y || point.y > max.y)
  {
    return false;
  }
  return true;
}

bool CommonAlgorithm::IsPointInSegment(const glm::dvec3& p0, const glm::dvec3& p1, const glm::dvec3& p2, const double epsilon)
{
  // 1. 共线检查：叉积接近0
  const glm::dvec3 v1 = p2 - p1;
  const glm::dvec3 v2 = p0 - p1;
  const double cross  = v1.x * v2.y - v1.y * v2.x;
  if (std::abs(cross) > epsilon)
    return false;

  // 2. 边界检查：投影在[0, 1]内
  const double dot   = glm::dot(v1, v2);
  const double lenSq = glm::dot(v1, v1);
  if (dot < -epsilon || dot > lenSq + epsilon)
  {
    return false;
  }

  return true;
}

bool CommonAlgorithm::IsPointInPolygon(const glm::dvec3& inPoint, const int32 numSides, const std::vector<glm::dvec3>& polygon, const double epsilon)
{
  int32 count = 0;

  for (int32 i = 0; i < numSides; ++i)
  {
    const glm::dvec3 startPoint = polygon[i];
    const glm::dvec3 endPoint   = polygon[i + 1 >= polygon.size() ? 0 : i + 1];

    if ((startPoint.y > inPoint.y && endPoint.y > inPoint.y) || (startPoint.y < inPoint.y && endPoint.y < inPoint.y))
    {
      continue;
    }
    const float x = static_cast<float>((inPoint.y - endPoint.y) * (startPoint.x - endPoint.x) / (startPoint.y - endPoint.y) + endPoint.x);

    if (x < inPoint.x)
    {
      count++;
    }
    else if (glm::abs(x - inPoint.x) < epsilon)
    {
      return true;
    }
  }

  return count % 2 != 0;
}

namespace
{
  // ---------- 基础工具 ----------
  static bool AabbOverlap(const glm::dvec3& amin, const glm::dvec3& aMax, const glm::dvec3& bMin, const glm::dvec3& bMax, const double eps = 1e-10)
  {
    return (amin.x - eps <= bMax.x && aMax.x + eps >= bMin.x) && (amin.y - eps <= bMax.y && aMax.y + eps >= bMin.y)
           && (amin.z - eps <= bMax.z && aMax.z + eps >= bMin.z);
  }

  static bool PointInBoxXy(const glm::dvec3& p, const glm::dvec3& boxMin, const glm::dvec3& boxMax)
  {
    return p.x >= boxMin.x && p.x <= boxMax.x && p.y >= boxMin.y && p.y <= boxMax.y;
  }

  // ---------- 三角形-矩形裁剪 ----------
  // 3D三角形 -> 2D投影（XY平面）
  struct Tri2D
  {
    glm::dvec2 v[3];
    double z[3];  // 原始Z
  };

  // 裁剪多边形：Sutherland-Hodgman，裁剪矩形四条边
  static std::vector<glm::dvec3> ClipTriangleToBox(const Tri2D& tri, const glm::dvec3& boxMin, const glm::dvec3& boxMax)
  {
    auto inside = [&](const glm::dvec2& p) { return p.x >= boxMin.x && p.x <= boxMax.x && p.y >= boxMin.y && p.y <= boxMax.y; };

    auto intersect = [&](const glm::dvec2& a, const glm::dvec2& b, const double edgeX, const bool vertical) {
      double t;
      if (vertical)
      {
        t = (edgeX - a.x) / (b.x - a.x);
      }
      else
      {
        t = (edgeX - a.y) / (b.y - a.y);
      }
      t = glm::clamp(t, 0.0, 1.0);

      const glm::dvec2 p = a + t * (b - a);
      const double z     = tri.z[0] + t * (tri.z[1] - tri.z[0]);  // 线性插值Z
      return glm::dvec3(p.x, p.y, z);
    };

    std::vector<glm::dvec3> poly;
    // 初始多边形 = 三角形
    poly.reserve(3);
    for (int32 i = 0; i < 3; ++i)
    {
      poly.emplace_back(tri.v[i].x, tri.v[i].y, tri.z[i]);
    }

    // 裁剪四条边
    const glm::dvec2 edge[4][2] = {
      { { boxMin.x, boxMin.y }, { boxMin.x, boxMax.y } },  // left
      { { boxMax.x, boxMin.y }, { boxMax.x, boxMax.y } },  // right
      { { boxMin.x, boxMin.y }, { boxMax.x, boxMin.y } },  // bottom
      { { boxMin.x, boxMax.y }, { boxMax.x, boxMax.y } }   // top
    };

    auto clip = [&](const std::vector<glm::dvec3>& src, const glm::dvec2& e1, const glm::dvec2& e2) {
      std::vector<glm::dvec3> res;
      const size_t n = src.size();
      for (size_t i = 0; i < n; ++i)
      {
        const size_t j = (i + 1) % n;
        glm::dvec2 p1(src[i].x, src[i].y);
        glm::dvec2 p2(src[j].x, src[j].y);

        const bool in1 = inside(p1);
        const bool in2 = inside(p2);

        if (in1)
          res.push_back(src[i]);
        if (in1 != in2)
        {
          // 计算交点
          glm::dvec3 inter;
          if (e1.x == e2.x)  // 垂直
            inter = intersect(p1, p2, e1.x, true);
          else  // 水平
            inter = intersect(p1, p2, e1.y, false);
          res.push_back(inter);
        }
      }
      return res;
    };

    // 依次裁剪四条边
    poly = clip(poly, edge[0][0], edge[0][1]);
    poly = clip(poly, edge[1][0], edge[1][1]);
    poly = clip(poly, edge[2][0], edge[2][1]);
    poly = clip(poly, edge[3][0], edge[3][1]);
    return poly;
  }

  // 判断三角形是否为竖直面
  static bool IsTriangleVertical(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c)
  {
    // 计算三角形法向量
    const glm::dvec3 ab = b - a;
    const glm::dvec3 ac = c - a;
    glm::dvec3 normal   = glm::cross(ab, ac);

    // 归一化
    const double length = glm::length(normal);
    if (length < 1e-10)
      return true;  // 退化三角形视为竖直面

    normal /= length;

    // 如果法向量接近水平（Z分量接近0），则为竖直面
    return std::abs(normal.z) < 0.1;  // 阈值可根据需要调整
  }

  // 处理竖直面三角形
  static void ProcessVerticalTriangle(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const double z0, const double z1,
                                      const double z2, const double left, const double right, const double bottom, const double top,
                                      std::vector<double>& candidateElevations)
  {
    // 1. 检查顶点是否在矩形内
    auto addIfInside = [&](const glm::dvec3& p, const double elevation) {
      if (p.x >= left && p.x <= right && p.y >= bottom && p.y <= top)
      {
        candidateElevations.push_back(elevation);
      }
    };

    addIfInside(a, z0);
    addIfInside(b, z1);
    addIfInside(c, z2);

    // 2. 检查边与矩形边界的交点
    auto processEdge = [&](const glm::dvec3& p1, const glm::dvec3& p2, const double z1Val, const double z2Val) {
      const glm::dvec2 dir = glm::dvec2(p2) - glm::dvec2(p1);
      constexpr double eps = 1e-10;

      // 与垂直边界（左/右）的交点
      if (std::abs(dir.x) > eps)
      {
        const double tLeft = (left - p1.x) / dir.x;
        if (tLeft >= 0.0 && tLeft <= 1.0)
        {
          const glm::dvec2 ip = glm::dvec2(p1) + tLeft * dir;
          if (ip.y >= bottom && ip.y <= top)
          {
            const double z = z1Val + tLeft * (z2Val - z1Val);
            candidateElevations.push_back(z);
          }
        }

        const double tRight = (right - p1.x) / dir.x;
        if (tRight >= 0.0 && tRight <= 1.0)
        {
          const glm::dvec2 ip = glm::dvec2(p1) + tRight * dir;
          if (ip.y >= bottom && ip.y <= top)
          {
            const double z = z1Val + tRight * (z2Val - z1Val);
            candidateElevations.push_back(z);
          }
        }
      }

      // 与水平边界（上/下）的交点
      if (std::abs(dir.y) > eps)
      {
        const double tBottom = (bottom - p1.y) / dir.y;
        if (tBottom >= 0.0 && tBottom <= 1.0)
        {
          const glm::dvec2 ip = glm::dvec2(p1) + tBottom * dir;
          if (ip.x >= left && ip.x <= right)
          {
            const double z = z1Val + tBottom * (z2Val - z1Val);
            candidateElevations.push_back(z);
          }
        }

        const double tTop = (top - p1.y) / dir.y;
        if (tTop >= 0.0 && tTop <= 1.0)
        {
          const glm::dvec2 ip = glm::dvec2(p1) + tTop * dir;
          if (ip.x >= left && ip.x <= right)
          {
            const double z = z1Val + tTop * (z2Val - z1Val);
            candidateElevations.push_back(z);
          }
        }
      }
    };

    processEdge(a, b, z0, z1);
    processEdge(b, c, z1, z2);
    processEdge(c, a, z2, z0);

    // 3. 处理完全包含在矩形内部的竖直面
    if (candidateElevations.empty())
    {
      // 计算三角形在矩形内的投影线段
      const glm::dvec2 projMin(glm::max(left, glm::min(glm::min(a.x, b.x), c.x)), glm::max(bottom, glm::min(glm::min(a.y, b.y), c.y)));
      const glm::dvec2 projMax(glm::min(right, glm::max(glm::max(a.x, b.x), c.x)), glm::min(top, glm::max(glm::max(a.y, b.y), c.y)));

      // 如果投影区域有效
      if (projMin.x <= projMax.x && projMin.y <= projMax.y)
      {
        // 竖直面的高程最大值就是其顶点的最大高程
        const double maxZ = std::max({ z0, z1, z2 });
        candidateElevations.push_back(maxZ);
      }
    }
  }

  // 处理一般三角形
  static void ProcessGeneralTriangle(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const double z0, const double z1, const double z2,
                                     const double left, const double right, const double bottom, const double top,
                                     std::vector<double>& candidateElevations)
  {
    // 1. 检查顶点是否在矩形内
    auto addIfInside = [&](const glm::dvec3& p, const double elevation) {
      if (p.x >= left && p.x <= right && p.y >= bottom && p.y <= top)
      {
        candidateElevations.push_back(elevation);
      }
    };

    addIfInside(a, z0);
    addIfInside(b, z1);
    addIfInside(c, z2);

    // 2. 计算边与矩形边界的交点
    auto processEdge = [&](const glm::dvec3& tp1, const glm::dvec3& tp2, const double z1Val, const double z2Val) {
      const glm::dvec2 dir = glm::dvec2(tp2) - glm::dvec2(tp1);

      // 与垂直边界（左/右）的交点
      if (dir.x != 0.0)
      {
        // 左边界
        const double tLeft = (left - tp1.x) / dir.x;
        if (tLeft >= 0.0 && tLeft <= 1.0)
        {
          const glm::dvec2 ip = glm::dvec2(tp1) + tLeft * dir;
          if (ip.y >= bottom && ip.y <= top)
          {
            const double z = z1Val + tLeft * (z2Val - z1Val);
            candidateElevations.push_back(z);
          }
        }

        // 右边界
        const double tRight = (right - tp1.x) / dir.x;
        if (tRight >= 0.0 && tRight <= 1.0)
        {
          const glm::dvec2 ip = glm::dvec2(tp1) + tRight * dir;
          if (ip.y >= bottom && ip.y <= top)
          {
            const double z = z1Val + tRight * (z2Val - z1Val);
            candidateElevations.push_back(z);
          }
        }
      }

      // 与水平边界（上/下）的交点
      if (dir.y != 0.0)
      {
        // 下边界
        const double tBottom = (bottom - tp1.y) / dir.y;
        if (tBottom >= 0.0 && tBottom <= 1.0)
        {
          const glm::dvec2 ip = glm::dvec2(tp1) + tBottom * dir;
          if (ip.x >= left && ip.x <= right)
          {
            const double z = z1Val + tBottom * (z2Val - z1Val);
            candidateElevations.push_back(z);
          }
        }

        // 上边界
        const double tTop = (top - tp1.y) / dir.y;
        if (tTop >= 0.0 && tTop <= 1.0)
        {
          const glm::dvec2 ip = glm::dvec2(tp1) + tTop * dir;
          if (ip.x >= left && ip.x <= right)
          {
            const double z = z1Val + tTop * (z2Val - z1Val);
            candidateElevations.push_back(z);
          }
        }
      }
    };

    // 3. 处理三条边
    processEdge(a, b, z0, z1);
    processEdge(b, c, z1, z2);
    processEdge(c, a, z2, z0);

    // 4. 检查矩形是否完全在三角形内部
    const glm::dvec3 p0(a.x, a.y, z0);
    const glm::dvec3 p1(b.x, b.y, z1);
    const glm::dvec3 p2(c.x, c.y, z2);
    const glm::dvec3 normal = glm::cross(p1 - p0, p2 - p0);
    if (glm::length(normal) > 1e-10)
    {
      const std::array<glm::dvec2, 4> corners = { glm::dvec2(left, bottom), glm::dvec2(left, top), glm::dvec2(right, bottom),
                                                  glm::dvec2(right, top) };
      bool allInside                          = true;
      for (const auto& corner : corners)
      {
        if (!CommonAlgorithm::IsPointInsideTriangle(corner, glm::dvec2(p0), glm::dvec2(p1), glm::dvec2(p2)))
        {
          allInside = false;
          break;
        }
      }
      if (allInside)
      {
        for (const auto& corner : corners)
        {
          // 计算高程: 从平面方程 normal.x*(x - p0.x) + normal.y*(y - p0.y) + normal.z*(z - p0.z) = 0
          // 所以: z = p0.z - (normal.x*(corner.x - p0.x) + normal.y*(corner.y - p0.y)) / normal.z;
          if (std::abs(normal.z) > 1e-10)
          {
            double z = p0.z - (normal.x * (corner.x - p0.x) + normal.y * (corner.y - p0.y)) / normal.z;
            candidateElevations.push_back(z);
          }
        }
      }
    }
  }

}  // namespace

// 判断三角形与矩形范围相交
double CommonAlgorithm::TrianglesIntersectRect2(const std::vector<glm::dvec3>& v, const std::vector<size_t>& indices, const glm::dvec3& rMin,
                                                const glm::dvec3& rMax, const glm::dmat4x4& transform)
{
  double maxZ = 0.0;

  const auto boxMin = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(rMin), 1.f));
  const auto boxMax = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(rMax), 1.f));

  // 并行 for，使用 reduction 做最大值归约
  for (size_t i = 0; i < indices.size(); i++)
  {
    if (i % 3 != 0)
    {
      continue;
    }

    double localMax = 0.0;

    const glm::dvec3 p0 = v[indices[i]];
    const glm::dvec3 p1 = v[indices[i + 1]];
    const glm::dvec3 p2 = v[indices[i + 2]];

    // 1. 三角形 AABB 粗筛
    const auto a = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(p0), 1.f));
    const auto b = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(p1), 1.f));
    const auto c = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(p2), 1.f));

    const glm::dvec3 triMin = glm::min(glm::min(a, b), c);
    const glm::dvec3 triMax = glm::max(glm::max(a, b), c);
    if (!AabbOverlap(triMin, triMax, boxMin, boxMax))
    {
      //// TODO: 后续有时间还有优化空间
      //// 解决极少量部分网格没有正确采集到的问题
      // if (const std::vector<glm::dvec3> tVertices = { a, b, c }; CommonAlgorithm::IsPointInPolygon(glm::dvec3(0.0), 3, tVertices))
      //{
      //   // maxZ     = glm::max(glm::max(glm::max(maxZ, p0.z), p1.z), p2.z);
      //   localMax = std::max({ localMax, p0.z, p1.z, p2.z });
      // }
      //// 将局部最大值写入归约
      // maxZ = std::max(maxZ, localMax);
      continue;
    }

    // 2. 投影到 XY
    // 这里要将高程替换为经纬度高程，否则在乘逆矩阵后，会导致实际高程发生变化
    const Tri2D tri2D{ { glm::dvec2(a.x, a.y), glm::dvec2(b.x, b.y), glm::dvec2(c.x, c.y) }, { p0.z, p1.z, p2.z } };

    // 3. 裁剪得到交集多边形
    const auto poly = ClipTriangleToBox(tri2D, boxMin, boxMax);
    if (poly.empty())
    {
      //// TODO: 后续有时间还有优化空间
      //// 解决极少量部分网格没有正确采集到的问题
      // if (const std::vector<glm::dvec3> tVertices = { a, b, c }; CommonAlgorithm::IsPointInPolygon(glm::dvec3(0.0), 3, tVertices))
      //{
      //   // maxZ = glm::max(glm::max(glm::max(maxZ, p0.z), p1.z), p2.z);
      //   localMax = std::max({ localMax, p0.z, p1.z, p2.z });
      // }
      // maxZ = std::max(maxZ, localMax);
      continue;
    }

    // 4. 取最大Z
    for (const auto& p : poly)
    {
      localMax = std::max(maxZ, p.z);
    }

    // 归约到 maxZ
    maxZ = std::max(maxZ, localMax);
  }
  return maxZ;
}

double CommonAlgorithm::TrianglesIntersectRect(const std::vector<glm::dvec3>& v, const std::vector<size_t>& indices, const glm::dvec3& rMin,
                                               const glm::dvec3& rMax, const glm::dmat4x4& transform)
{

  const auto boxMin = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(rMin), 1.f));
  const auto boxMax = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(rMax), 1.f));

  // 提取矩形边界（XY平面）
  const double left   = boxMin.x;
  const double right  = boxMax.x;
  const double bottom = boxMin.y;
  const double top    = boxMax.y;

  // 最小值不会小于此范围的最低高程
  double maxElevation = rMin.z;

  for (size_t i = 0; i + 2 < indices.size(); i += 3)
  {
    const glm::dvec3 p0 = v[indices[i]];
    const glm::dvec3 p1 = v[indices[i + 1]];
    const glm::dvec3 p2 = v[indices[i + 2]];

    // 获取原始高程值
    const double z0 = p0.z;
    const double z1 = p1.z;
    const double z2 = p2.z;

    // 处理为本地坐标
    const auto a = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(p0), 1.f));
    const auto b = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(p1), 1.f));
    const auto c = glm::dvec3(transform * glm::dvec4(CommonAlgorithm::CovertLonLatHeightToCartesian3(p2), 1.f));

    // 检查三角形类型（平面或竖直面）
    const bool isVertical = IsTriangleVertical(a, b, c);

    // 快速AABB检查
    const glm::dvec2 triMin = glm::min(glm::min(glm::dvec2(a), glm::dvec2(b)), glm::dvec2(c));
    const glm::dvec2 triMax = glm::max(glm::max(glm::dvec2(a), glm::dvec2(b)), glm::dvec2(c));

    if (triMax.x < left || triMin.x > right || triMax.y < bottom || triMin.y > top)
    {
      continue;  // 无重叠
    }

    std::vector<double> candidateElevations;

    // 处理竖直面三角形
    if (isVertical)
    {
      ProcessVerticalTriangle(a, b, c, z0, z1, z2, left, right, bottom, top, candidateElevations);
    }
    else
    {
      ProcessGeneralTriangle(a, b, c, z0, z1, z2, left, right, bottom, top, candidateElevations);
    }

    // 更新最大高程
    if (!candidateElevations.empty())
    {
      maxElevation = std::max(maxElevation, *std::max_element(candidateElevations.begin(), candidateElevations.end()));
    }
  }

  return maxElevation;
}

double CommonAlgorithm::TrianglesIntersectRectWithBvh(const BvhAccelerator& bvh, const std::vector<glm::dvec3>& vertices,
                                                      const std::vector<double>& heights, const std::vector<size_t>& indices,
                                                      const glm::dvec3& rectMin, const glm::dvec3& rectMax)
{
  // 1. 查询可能相交的三角形
  const std::vector<size_t> candidateTriangles = bvh.Query(rectMin, rectMax);

  double maxElevation = rectMin.z;

  const double left   = rectMin.x;
  const double right  = rectMax.x;
  const double bottom = rectMin.y;
  const double top    = rectMax.y;

  // 2. 处理候选三角形
  for (const size_t idx : candidateTriangles)
  {
    // 获取原始高程值
    const double z0 = heights[indices[idx]];
    const double z1 = heights[indices[idx + 1]];
    const double z2 = heights[indices[idx + 2]];

    // 转换到目标坐标系
    glm::dvec3 a = vertices[indices[idx]];
    glm::dvec3 b = vertices[indices[idx + 1]];
    glm::dvec3 c = vertices[indices[idx + 2]];

    // 检查三角形类型（平面或竖直面）
    const bool isVertical = IsTriangleVertical(a, b, c);

    // 快速AABB检查
    const glm::dvec2 triMin = glm::min(glm::min(glm::dvec2(a), glm::dvec2(b)), glm::dvec2(c));
    const glm::dvec2 triMax = glm::max(glm::max(glm::dvec2(a), glm::dvec2(b)), glm::dvec2(c));

    if (triMax.x < left || triMin.x > right || triMax.y < bottom || triMin.y > top)
    {
      continue;  // 无重叠
    }

    std::vector<double> candidateElevations;

    // 处理竖直面三角形
    if (isVertical)
    {
      ProcessVerticalTriangle(a, b, c, z0, z1, z2, left, right, bottom, top, candidateElevations);
    }
    else
    {
      ProcessGeneralTriangle(a, b, c, z0, z1, z2, left, right, bottom, top, candidateElevations);
    }

    // 更新最大高程
    if (!candidateElevations.empty())
    {
      maxElevation = std::max(maxElevation, *std::max_element(candidateElevations.begin(), candidateElevations.end()));
    }
  }

  return maxElevation;  // 默认值
}

std::vector<glm::dvec3> CommonAlgorithm::InterpolateWithAutoRoundCorners(const std::vector<glm::dvec3>& points, const double tubeRadius,
                                                                         const double minFilletFactor, const int32 minSegments,
                                                                         const int32 maxSegments)
{
  // 处理特殊情况
  if (points.size() < 3)
  {
    return points;
  }

  std::vector<glm::dvec3> result;
  result.reserve(points.size() * 3);  // 保守估计容量

  // 添加起点
  result.push_back(points[0]);

  for (size_t i = 1; i < points.size() - 1; ++i)
  {
    const glm::dvec3& a = points[i - 1];
    const glm::dvec3& b = points[i];
    const glm::dvec3& c = points[i + 1];

    // 计算方向向量和长度
    glm::dvec3 vecBa = glm::normalize(a - b);
    glm::dvec3 vecBc = glm::normalize(c - b);
    double lenBa     = glm::distance(a, b);
    double lenBc     = glm::distance(b, c);

    // 计算夹角
    double cosTheta            = glm::dot(vecBa, vecBc);
    cosTheta                   = glm::clamp(cosTheta, -1.0, 1.0);
    const double theta         = std::acos(cosTheta);
    const double exteriorAngle = glm::pi<double>() - theta;

    // 跳过小角度或接近180度的角度
    if (theta < 1e-5 || (glm::pi<double>() - theta) < 1e-5)
    {
      result.push_back(b);
      continue;
    }

    // 自动计算安全圆角半径 (考虑避免自相交)
    double maxPossibleRadius  = 0.5 * std::min(lenBa, lenBc);
    double minSafeRadius      = tubeRadius * minFilletFactor;
    const double filletRadius = std::min(maxPossibleRadius, minSafeRadius);

    // 如果圆角半径太小则跳过圆角
    if (filletRadius < tubeRadius * 1e-3)
    {
      result.push_back(b);
      continue;
    }

    // 自动计算插值段数 (基于弧长和圆管半径)
    const double arcLength = exteriorAngle * filletRadius;
    int32 numSegments      = static_cast<int32>(std::ceil(arcLength / (tubeRadius * 0.5)));
    numSegments            = std::clamp(numSegments, minSegments, maxSegments);

    // 计算切点距离
    double t = filletRadius / std::tan(theta / 2.0);
    t        = std::min(t, 0.5 * std::min(lenBa, lenBc));  // 确保不超出线段范围

    // 计算切点
    glm::dvec3 p1 = b + vecBa * t;
    glm::dvec3 p2 = b + vecBc * t;

    // 计算圆心
    glm::dvec3 bisector = glm::normalize(glm::normalize(vecBa) + glm::normalize(vecBc));
    const double d      = filletRadius / std::sin(theta / 2.0);
    glm::dvec3 o        = b + bisector * d;

    // 计算旋转轴和起始向量
    glm::dvec3 rotationAxis = glm::normalize(glm::cross(vecBc, vecBa));
    glm::dvec3 startVec     = p1 - o;

    // 添加直线段到第一个切点
    result.push_back(p1);

    // 生成圆弧插值点
    for (int32 j = 1; j < numSegments; j++)
    {
      const double fraction = static_cast<double>(j) / numSegments;
      double angle          = fraction * exteriorAngle;
      glm::dquat rot        = glm::angleAxis(angle, rotationAxis);
      glm::dvec3 arcPoint   = o + rot * startVec;
      result.push_back(arcPoint);
    }

    // 添加最后一个切点
    result.push_back(p2);
  }

  // 添加终点
  result.push_back(points.back());

  return result;
}

bool CommonAlgorithm::IsPointInsideTriangle(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c)
{
  // 计算向量和叉积
  const double cross1 = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
  const double cross2 = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
  const double cross3 = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);

  // 检查所有叉积是否同号（允许零）
  if ((cross1 >= 0 && cross2 >= 0 && cross3 >= 0) || (cross1 <= 0 && cross2 <= 0 && cross3 <= 0))
    return true;
  return false;
}

std::filesystem::path CommonAlgorithm::MakePath(const std::string& utf8Path)
{
#ifdef _WIN32
  return std::filesystem::u8path(utf8Path);
#else
  return std::filesystem::path(utf8_path);  // Linux/macOS 默认 UTF-8
#endif
}

std::string CommonAlgorithm::ConvertTransform2JsonString(const glm::dmat4& transform)
{
  rapidjson::Document doc;
  doc.SetObject();

  // 创建一个分配器用于内存分配
  rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();

  // 创建一个数组来存储矩阵元素
  rapidjson::Value matrixArray(rapidjson::kArrayType);
  for (int32 i = 0; i < 4; i++)
  {
    for (int32 j = 0; j < 4; j++)
    {
      // 将矩阵的每个元素添加到数组中
      // 这里假设矩阵元素是按行存储的，你可以根据需要进行调整
      matrixArray.PushBack(transform[i][j], allocator);
    }
  }

  // 将数组添加到文档对象中
  doc.AddMember("transform", matrixArray, allocator);

  // 创建一个字符串缓冲区和 writer
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

  // 将文档写入缓冲区
  doc.Accept(writer);

  // 返回结果字符串
  return buffer.GetString();
}

glm::dmat4 CommonAlgorithm::ConvertJsonString2Transform(const std::string& jsonString)
{
  // 创建一个 4x4 的双精度矩阵，默认值为 0
  glm::dmat4x4 mat(0.0);

  // 创建并解析 JSON 文档
  rapidjson::Document doc;
  doc.Parse(jsonString.c_str());

  // 检查 JSON 对象中是否存在 "matrix" 数组
  if (doc.HasMember("transform") && doc["transform"].IsArray())
  {
    const rapidjson::Value& matrixArray = doc["transform"];

    // 确保数组包含 16 个元素（4x4 矩阵）
    if (const size_t count = matrixArray.Size(); count == 16)
    {
      // 填充矩阵元素
      for (int32 i = 0; i < 4; i++)
      {
        for (int32 j = 0; j < 4; j++)
        {
          // 这里假设 JSON 数组中的元素是按行存储的，即先存储矩阵的第 0 行，然后第 1 行，以此类推
          // 如果 JSON 中的存储方式不同，需要调整索引的计算方式
          mat[i][j] = matrixArray[i * 4 + j].GetDouble();
        }
      }
    }
    else
    {
      // 如果数组大小不正确，返回默认矩阵
      SPDLOG_ERROR("Error: Matrix array size mismatch! Expected 16 elements, got {}", count);
    }
  }
  else
  {
    // 如果 JSON 中没有 "matrix" 键或者该键不是数组类型
    SPDLOG_ERROR("Error: JSON does not contain a valid transform array.");
  }

  return mat;
}

double CommonAlgorithm::LogX(const double base, const double x)
{
  return glm::log(x) / glm::log(base);
}

}  // namespace Common

// end
