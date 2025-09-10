#pragma once

#ifndef COMMON_ALGORITHM_H_
#define COMMON_ALGORITHM_H_

#include "BVHAccelerator.h"
#include <BS_thread_pool.hpp>
#include <cpp-httplib/httplib.h>
#include <filesystem>
#include <functional>
#include <glm/vec3.hpp>
#include <string>
#include <vector>

namespace Common
{

using namespace glm;

class CommonAlgorithm
{
public:
  /**
   * @brief 初始化线程池数量
   * @param num 线程池数量
   */
  static void InitThreadPoolNum(const size_t num);

  /**
   * @brief 并行处理For循环
   * @param array 数组
   * @param func 函数
   */
  template <class T>
  static void ParallelFor(const std::vector<T>& array, const std::function<void(const size_t)>& func);

  /**
   * @brief 并行处理For循环
   * @param maxSize 最大大小
   * @param func 函数
   * @param useMt 是否使用多线程
   */
  static void ParallelFor(const size_t& maxSize, const std::function<void(const size_t)>& func, const bool useMt = true);

  /**
   * @brief 并行处理For循环
   * @param maxSize 最大大小
   * @param func 函数
   * @param maxThreadNum 最大线程数量
   */
  static void ParallelFor(const size_t& maxSize, const std::function<void(const size_t)>& func, const int32 maxThreadNum);

  /**
   * @brief 笛卡尔坐标转经纬度
   * @param cart3 笛卡尔坐标
   * @return glm::dvec3 经纬度
   */
  static glm::dvec3 CovertCartesian3ToLonLatHeight(const glm::dvec3& cart3);

  /**
   * @brief 经纬度转笛卡尔坐标
   * @param lonLatHeight 经纬度
   * @return glm::dvec3 笛卡尔坐标
   */
  static glm::dvec3 CovertLonLatHeightToCartesian3(const glm::dvec3& lonLatHeight);

  /** 几何算法 **/

  /**
   * @brief 判断点是否在长方体内
   * @param point 点
   * @param min 长方体最小点
   * @param max 长方体最大点
   * @return true 点在长方体内, false 点不在长方体内
   */
  static bool IsInCube(const glm::dvec3& point, const glm::dvec3& min, const glm::dvec3& max);

  /**
   * @brief 判断点是否在平面矩形范围内
   * @param point 点
   * @param min 矩形最小点
   * @param max 矩形最大点
   * @return true 点在矩形范围内, false 点不在矩形范围内
   */
  static bool IsInRect(const glm::dvec3& point, const glm::dvec3& min, const glm::dvec3& max);

  /**
   * @brief 判断点 p0 是否在线段 p1, p2 上
   * @param p0 点
   * @param p1 线段点1
   * @param p2 线段点2
   * @param epsilon 误差
   * @return true 点在线段上, false 点不在线段上
   */
  static bool IsPointInSegment(const glm::dvec3& p0, const glm::dvec3& p1, const glm::dvec3& p2, const double epsilon = 1e-6);

  /**
   * @brief 判断点是否在多边形内
   * @param inPoint 点
   * @param numSides 多边形边数
   * @param polygon 多边形
   * @param epsilon 误差
   * @return true 点在多边形内, false 点不在多边形内
   */
  static bool IsPointInPolygon(const glm::dvec3& inPoint, const int32 numSides, const std::vector<glm::dvec3>& polygon, const double epsilon = 1e-10);

  /**
   * @brief 三角形与矩形相交，并返回区域内最大高程
   * @param v 顶点
   * @param indices 索引
   * @param rMin 矩形最小点
   * @param rMax 矩形最大点
   * @param transform 变换矩阵
   * @return double 相交区域的最大高程
   */
  static double TrianglesIntersectRect2(const std::vector<glm::dvec3>& v, const std::vector<size_t>& indices, const glm::dvec3& rMin,
                                        const glm::dvec3& rMax, const glm::dmat4x4& transform);

  /**
   * @brief 三角形与矩形相交，并返回区域内最大高程
   * @param v 顶点
   * @param indices 索引
   * @param rMin 矩形最小点
   * @param rMax 矩形最大点
   * @param transform 变换矩阵
   * @return double 相交区域的最大高程
   */
  static double TrianglesIntersectRect(const std::vector<glm::dvec3>& v, const std::vector<size_t>& indices, const glm::dvec3& rMin,
                                       const glm::dvec3& rMax, const glm::dmat4x4& transform);

  /**
   * @brief 三角形与矩形相交，并返回区域内最大高程
   * @param bvh 加速结构
   * @param vertices 顶点
   * @param heights 高程
   * @param indices 索引
   * @param rectMin 矩形最小点
   * @param rectMax 矩形最大点
   * @return double 相交区域的最大高程
   */
  static double TrianglesIntersectRectWithBvh(const BvhAccelerator& bvh, const std::vector<glm::dvec3>& vertices, const std::vector<double>& heights,
                                              const std::vector<size_t>& indices, const glm::dvec3& rectMin, const glm::dvec3& rectMax);

  /**
   * @brief 插值生成圆角曲线
   * @param points 点集
   * @param tubeRadius 半径
   * @param minFilletFactor 最小圆角因子
   * @param minSegments 最小分段数
   * @param maxSegments 最大分段数
   * @return std::vector<glm::dvec3> 插值点集
   */
  static std::vector<glm::dvec3> InterpolateWithAutoRoundCorners(const std::vector<glm::dvec3>& points, double tubeRadius,
                                                                 double minFilletFactor = 1.1, int32 minSegments = 4, int32 maxSegments = 32);

  static bool IsPointInsideTriangle(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c);

  /**
   * @brief 处理中文字符串
   *
   * @param utf8Path UTF-8 编码的字符串
   * @return std::filesystem::path
   */
  static std::filesystem::path MakePath(const std::string& utf8Path);

  // 矩阵转json字符串
  static std::string ConvertTransform2JsonString(const glm::dmat4& transform);

  // 字符串转矩阵
  static glm::dmat4 ConvertJsonString2Transform(const std::string& jsonString);

  /**
   * @brief 计算以指定底数的对数
   * @param base 对数的底数 (必须大于0且不等于1)
   * @param x 对数的真数 (必须大于0)
   * @return 返回 log_base(x) 的计算结果，即满足 base^result = x 的result值
   * @note 实现采用换底公式：log_base(x) = ln(x) / ln(base)
   * @warning 当 base<=0、base=1 或 x<=0 时，结果为未定义
   */
  static double LogX(const double base, const double x);

  /**
   * @brief 将 UTF-8 字符串转换为 GBK 编码
   *
   * @param utf8Str UTF-8 编码的字符串
   * @return std::string GBK 编码的字符串
   */
  static std::string Utf8ToGbk(const std::string& utf8Str)
  {
    if (utf8Str.empty())
      return "";

    // 第一步：将 UTF-8 转换为宽字符 (UTF-16)
    const glm::int32 wideSize = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, nullptr, 0);
    if (wideSize == 0)
      return "";

    std::wstring wideStr(wideSize, 0);
    MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, wideStr.data(), wideSize);

    // 第二步：将宽字符 (UTF-16) 转换为 GBK
    const glm::int32 gbkSize = WideCharToMultiByte(CP_ACP, 0, wideStr.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (gbkSize == 0)
      return "";

    std::string gbkStr(gbkSize, 0);
    WideCharToMultiByte(CP_ACP, 0, wideStr.c_str(), -1, gbkStr.data(), gbkSize, nullptr, nullptr);

    // 移除末尾的 null 字符
    if (!gbkStr.empty() && gbkStr.back() == '\0')
    {
      gbkStr.pop_back();
    }

    return gbkStr;
  }

  /**
   * @brief 将 GBK 字符串转换为 UTF-8 编码
   *
   * @param gbkStr GBK 编码的字符串
   * @return std::string UTF-8 编码的字符串
   */
  static std::string GbkToUtf8(const std::string& gbkStr)
  {
    if (gbkStr.empty())
      return "";

    // 第一步：将 GBK 转换为宽字符 (UTF-16)
    const glm::int32 wideSize = MultiByteToWideChar(CP_ACP, 0, gbkStr.c_str(), -1, nullptr, 0);
    if (wideSize == 0)
      return "";

    std::wstring wideStr(wideSize, 0);
    MultiByteToWideChar(CP_ACP, 0, gbkStr.c_str(), -1, &wideStr[0], wideSize);

    // 第二步：将宽字符 (UTF-16) 转换为 UTF-8
    const glm::int32 utf8Size = WideCharToMultiByte(CP_UTF8, 0, wideStr.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (utf8Size == 0)
      return "";

    std::string utf8Str(utf8Size, 0);
    WideCharToMultiByte(CP_UTF8, 0, wideStr.c_str(), -1, &utf8Str[0], utf8Size, nullptr, nullptr);

    // 移除末尾的 null 字符
    if (!utf8Str.empty() && utf8Str.back() == '\0')
    {
      utf8Str.pop_back();
    }

    return utf8Str;
  }

public:
  static std::unique_ptr<BS::thread_pool<>> threadPool;
};

}  // namespace Common

#endif
