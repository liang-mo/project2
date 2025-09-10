#pragma once

#ifndef BVH_ACCELERATOR_H_
#define BVH_ACCELERATOR_H_

#include <algorithm>
#include <atomic>
#include <glm/glm.hpp>
#include <limits>
#include <stack>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_group.h>
#include <thread>
#include <vector>

/**
 * @brief 紧凑的BVH节点结构
 */
struct BvhNode
{
  glm::dvec3 min;
  glm::dvec3 max;
  union
  {
    struct
    {
      uint32_t leftChild;   // 左子节点索引
      uint32_t rightChild;  // 右子节点索引
    };
    struct
    {
      uint32_t startIndex;  // 三角形起始索引
      uint32_t count;       // 三角形数量
    };
  };
  uint8_t flags;  // 标志位: 0x1=叶子节点
};

/**
 * @brief BVH加速结构类
 */
class BvhAccelerator
{
public:
  /**
   * @brief 构造函数
   * @param vertices 顶点
   * @param indices 索引
   */
  BvhAccelerator(const std::vector<glm::dvec3>& vertices, const std::vector<size_t>& indices) : _vertices(vertices), _indices(indices)
  {
    // 准备三角形索引列表
    std::vector<uint32_t> triangleIndices;
    triangleIndices.resize(indices.size() / 3);

    // 并行初始化三角形索引
    tbb::parallel_for(tbb::blocked_range<size_t>(0, triangleIndices.size()), [&](const tbb::blocked_range<size_t>& r) {
      for (size_t i = r.begin(); i < r.end(); ++i)
      {
        triangleIndices[i] = static_cast<uint32_t>(3 * i);
      }
    });

    // 构建BVH
    BuildBvh(triangleIndices);
  }

  /**
   * @brief 查询与矩形相交的三角形
   * @param rectMin 矩形最小点
   * @param rectMax 矩形最大点
   * @return std::vector<size_t> 相交三角形索引
   */
  std::vector<size_t> Query(const glm::dvec3& rectMin, const glm::dvec3& rectMax) const
  {
    tbb::concurrent_vector<size_t> concurrentResult;

    // 使用栈进行迭代查询
    std::stack<uint32_t> stack;
    stack.push(0);  // 根节点索引

    while (!stack.empty())
    {
      const uint32_t nodeIndex = stack.top();
      stack.pop();

      const BvhNode& node = _nodes[nodeIndex];

      // AABB不相交检查 (XY平面)
      if (node.max.x < rectMin.x || node.min.x > rectMax.x || node.max.y < rectMin.y || node.min.y > rectMax.y)
      {
        continue;
      }

      if (node.flags & 0x1)
      {  // 叶子节点
        for (uint32_t i = 0; i < node.count; ++i)
        {
          concurrentResult.push_back(_triangleIndices[node.startIndex + i]);
        }
      }
      else
      {
        // 使用启发式：先处理更可能相交的子节点
        const BvhNode& leftChild  = _nodes[node.leftChild];
        const BvhNode& rightChild = _nodes[node.rightChild];

        // 计算子节点与查询区域的相交面积
        const double leftArea  = ComputeOverlapArea(leftChild, rectMin, rectMax);
        const double rightArea = ComputeOverlapArea(rightChild, rectMin, rectMax);

        if (leftArea > rightArea)
        {
          stack.push(node.rightChild);
          stack.push(node.leftChild);
        }
        else
        {
          stack.push(node.leftChild);
          stack.push(node.rightChild);
        }
      }
    }

    // 转换为顺序结果
    return std::vector<size_t>(concurrentResult.begin(), concurrentResult.end());
  }

private:
  const std::vector<glm::dvec3>& _vertices;
  const std::vector<size_t>& _indices;
  std::vector<BvhNode> _nodes;
  std::vector<uint32_t> _triangleIndices;         // 存储所有三角形索引
  std::atomic<uint32_t> _nextTriangleIndex{ 0 };  // 原子三角形索引计数器

  // 计算包围盒与查询区域的相交面积
  double ComputeOverlapArea(const BvhNode& node, const glm::dvec3& rectMin, const glm::dvec3& rectMax) const
  {
    const double xMin = glm::max(node.min.x, rectMin.x);
    const double xMax = glm::min(node.max.x, rectMax.x);
    const double yMin = glm::max(node.min.y, rectMin.y);
    const double yMax = glm::min(node.max.y, rectMax.y);

    if (xMax < xMin || yMax < yMin)
      return 0.0;

    return (xMax - xMin) * (yMax - yMin);
  }

  // 计算三角形列表的AABB
  void ComputeAabb(const tbb::concurrent_vector<uint32_t>& triangleIndices, glm::dvec3& min, glm::dvec3& max) const
  {
    min = glm::dvec3(std::numeric_limits<double>::max());
    max = glm::dvec3(std::numeric_limits<double>::lowest());

    for (const uint32_t idx : triangleIndices)
    {
      const glm::dvec3& a = _vertices[_indices[idx]];
      const glm::dvec3& b = _vertices[_indices[idx + 1]];
      const glm::dvec3& c = _vertices[_indices[idx + 2]];

      min = glm::min(min, glm::min(a, glm::min(b, c)));
      max = glm::max(max, glm::max(a, glm::max(b, c)));
    }
  }

  // 构建BVH
  void BuildBvh(std::vector<uint32_t>& triangleIndices)
  {
    // 预分配节点空间
    _nodes.resize(triangleIndices.size() * 2 - 1);
    _triangleIndices.resize(triangleIndices.size());
    _nextTriangleIndex.store(0);

    tbb::concurrent_vector<uint32_t> tmpIndices(triangleIndices.begin(), triangleIndices.end());

    // 构建根节点
    uint32_t rootIndex = 0;
    _nodes[rootIndex]  = CreateNode(tmpIndices);

    // 使用线程安全队列
    tbb::concurrent_queue<BuildTask> taskQueue;
    taskQueue.push({ rootIndex, std::move(tmpIndices) });

    std::atomic<uint32_t> nextNodeIndex(1);
    std::atomic<uint32_t> activeTasks(1);  // 跟踪活跃任务

    // 使用TBB任务组进行并行处理
    tbb::task_group taskGroup;

    // 创建并行处理任务
    auto worker = [&] {
      BuildTask task;
      while (true)
      {
        if (taskQueue.try_pop(task))
        {
          // 处理当前任务
          ProcessNode(task, taskQueue, nextNodeIndex, activeTasks);

          // 任务完成，减少活跃任务计数
          if (--activeTasks == 0)
          {
            // 这是最后一个任务，通知其他线程退出
            break;
          }
        }
        else
        {
          // 队列为空时检查是否还有任务
          if (activeTasks.load() == 0)
          {
            break;  // 没有任务了，退出
          }
          // 短暂暂停让其他线程有机会添加任务
          std::this_thread::yield();
        }
      }
    };

    // 启动多个worker线程
    const size_t numThreads = std::thread::hardware_concurrency();
    for (size_t i = 0; i < numThreads; ++i)
    {
      taskGroup.run(worker);
    }

    // 等待所有任务完成
    taskGroup.wait();

    // 释放未使用内存
    _nodes.shrink_to_fit();
  }

  // 节点构建任务
  struct BuildTask
  {
    uint32_t nodeIndex;
    tbb::concurrent_vector<uint32_t> triangleIndices;

    BuildTask() = default;
    BuildTask(const uint32_t nodeIndex, tbb::concurrent_vector<uint32_t>&& triangleIndices)
      : nodeIndex(nodeIndex), triangleIndices(std::move(triangleIndices))
    {
    }

    // 禁止复制
    BuildTask(const BuildTask&)            = delete;
    BuildTask& operator=(const BuildTask&) = delete;

    // 允许移动
    BuildTask(BuildTask&&)            = default;
    BuildTask& operator=(BuildTask&&) = default;
  };

  // 创建节点
  BvhNode CreateNode(const tbb::concurrent_vector<uint32_t>& triangleIndices) const
  {
    BvhNode node;
    ComputeAabb(triangleIndices, node.min, node.max);
    node.flags = 0;
    return node;
  }

  // 处理节点构建 - 线程安全版本
  void ProcessNode(BuildTask& task, tbb::concurrent_queue<BuildTask>& taskQueue, std::atomic<uint32_t>& nextNodeIndex,
                   std::atomic<uint32_t>& activeTasks)
  {
    constexpr size_t leafThreshold = 4;

    if (task.triangleIndices.size() <= leafThreshold)
    {
      BvhNode& node = _nodes[task.nodeIndex];
      node.flags |= 0x1;  // 标记为叶子节点
      node.count = static_cast<uint32_t>(task.triangleIndices.size());

      // 原子分配三角形索引空间
      node.startIndex = _nextTriangleIndex.fetch_add(node.count);

      // 添加三角形索引
      for (uint32_t i = 0; i < node.count; ++i)
      {
        _triangleIndices[node.startIndex + i] = task.triangleIndices[i];
      }
      return;
    }

    // 选择划分轴：最长的轴
    const BvhNode& node     = _nodes[task.nodeIndex];
    const glm::dvec3 extent = node.max - node.min;
    const uint32_t axis     = (extent.x > extent.y) ? ((extent.x > extent.z) ? 0 : 2) : ((extent.y > extent.z) ? 1 : 2);

    // 排序 - 使用并行排序
    tbb::parallel_sort(task.triangleIndices.begin(), task.triangleIndices.end(), [&](uint32_t i1, uint32_t i2) {
      const glm::dvec3 center1 = (_vertices[_indices[i1]] + _vertices[_indices[i1 + 1]] + _vertices[_indices[i1 + 2]]) / 3.0;
      const glm::dvec3 center2 = (_vertices[_indices[i2]] + _vertices[_indices[i2 + 1]] + _vertices[_indices[i2 + 2]]) / 3.0;
      return center1[axis] < center2[axis];
    });

    // 划分为两个子集
    const size_t mid = task.triangleIndices.size() / 2;
    tbb::concurrent_vector<uint32_t> leftIndices(task.triangleIndices.begin(), task.triangleIndices.begin() + mid);
    tbb::concurrent_vector<uint32_t> rightIndices(task.triangleIndices.begin() + mid, task.triangleIndices.end());

    // 分配子节点
    const uint32_t leftIndex  = nextNodeIndex.fetch_add(1);
    const uint32_t rightIndex = nextNodeIndex.fetch_add(1);

    // 更新节点关系
    _nodes[task.nodeIndex].leftChild  = leftIndex;
    _nodes[task.nodeIndex].rightChild = rightIndex;

    // 创建子节点
    _nodes[leftIndex]  = CreateNode(leftIndices);
    _nodes[rightIndex] = CreateNode(rightIndices);

    // 添加新任务到队列并增加活跃任务计数
    if (!leftIndices.empty())
    {
      ++activeTasks;  // 增加活跃任务计数
      taskQueue.push({ leftIndex, std::move(leftIndices) });
    }

    if (!rightIndices.empty())
    {
      ++activeTasks;  // 增加活跃任务计数
      taskQueue.push({ rightIndex, std::move(rightIndices) });
    }
  }
};

#endif