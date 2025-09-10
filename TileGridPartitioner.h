#pragma once

#ifndef TILE_GRID_PARTITIONER_H_
#define TILE_GRID_PARTITIONER_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <glm/glm.hpp>
#include <memory>
#include <mutex>
#include <numeric>
#include <tbb/blocked_range.h>
#include <tbb/combinable.h>
#include <tbb/concurrent_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <vector>

namespace Zondy
{

using namespace glm;

/**
 * @brief 瓦片数据结构
 * @details 该结构体用于表示一个瓦片，包含瓦片的左下角坐标、右上角坐标和对应的文件名
 */
struct Tile
{
  glm::dvec2 min;        // 瓦片左下角坐标
  glm::dvec2 max;        // 瓦片右上角坐标
  std::string fileName;  // 瓦片对应的文件名
};

/**
 * @brief 网格单元结构
 * @details 该结构体用于表示一个网格单元，包含网格的左下角坐标、右上角坐标和包含的瓦片索引
 */
struct GridCell
{
  glm::dvec2 min;                              // 网格左下角坐标
  glm::dvec2 max;                              // 网格右上角坐标
  tbb::concurrent_vector<size_t> tileIndices;  // 包含的瓦片索引（线程安全）
  tbb::concurrent_set<std::string> fileNames;  // 包含的文件名（线程安全）
};

/**
 * @brief 四叉树节点结构
 * @details 该结构体用于表示四叉树的节点，包含节点的左下角坐标、右上角坐标和包含的瓦片索引
 */
struct QuadTreeNode
{
  glm::dvec2 min;
  glm::dvec2 max;
  std::vector<size_t> tileIndices;
  std::array<std::unique_ptr<QuadTreeNode>, 4> children;
};

/**
 * @brief 瓦片网格分区器
 * @details 该类用于将瓦片数据分区到自适应网格中
 */
class TileGridPartitioner
{
public:
  /**
   * @brief 构造函数
   * @param tiles 瓦片数据
   * @param globalMin 全局最小坐标
   * @param globalMax 全局最大坐标
   * @param maxTilesPerCell 每个网格单元的最大瓦片数量
   */
  TileGridPartitioner(const std::vector<Tile>& tiles, const glm::dvec2& globalMin, const glm::dvec2& globalMax, const size_t maxTilesPerCell = 2500)
    : _tiles(tiles), _globalMin(globalMin), _globalMax(globalMax), _maxTilesPerCell(maxTilesPerCell)
  {
    // 初始化时收集所有文件名
    InitializeFileNames();
  }

  /**
   * @brief 构建自适应网格
   * @return std::vector<GridCell>
   */
  std::vector<GridCell> BuildAdaptiveGrid()
  {
    // 创建根节点
    const auto root = std::make_unique<QuadTreeNode>();
    root->min       = _globalMin;
    root->max       = _globalMax;

    // 将所有瓦片添加到根节点 (更高效的方式)
    root->tileIndices.resize(_tiles.size());
    std::iota(root->tileIndices.begin(), root->tileIndices.end(), 0);

    // 递归构建四叉树
    BuildQuadTree(root.get());

    // 从四叉树提取网格单元
    tbb::concurrent_vector<GridCell> concurrentCells;
    ExtractGridCells(root.get(), concurrentCells);

    // 转换为标准vector
    return std::vector<GridCell>(concurrentCells.begin(), concurrentCells.end());
  }

  /**
   * @brief 构建均匀网格
   * @return std::vector<GridCell>
   */
  std::vector<GridCell> BuildUniformGrid() const
  {
    // 计算网格划分数量
    const double totalArea   = (_globalMax.x - _globalMin.x) * (_globalMax.y - _globalMin.y);
    const double avgTileArea = totalArea / static_cast<double>(_tiles.size());
    const double cellArea    = avgTileArea * static_cast<double>(_maxTilesPerCell);

    const size_t gridX      = static_cast<size_t>(std::ceil((_globalMax.x - _globalMin.x) / std::sqrt(cellArea)));
    const size_t gridY      = static_cast<size_t>(std::ceil((_globalMax.y - _globalMin.y) / std::sqrt(cellArea)));
    const size_t totalCells = gridX * gridY;

    // 并行初始化网格
    std::vector<GridCell> gridCells(totalCells);
    const double cellWidth  = (_globalMax.x - _globalMin.x) / static_cast<double>(gridX);
    const double cellHeight = (_globalMax.y - _globalMin.y) / static_cast<double>(gridY);

    // 并行初始化网格单元
    tbb::parallel_for(tbb::blocked_range<size_t>(0, totalCells), [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx)
      {
        const size_t y     = idx / gridX;
        const size_t x     = idx % gridX;
        gridCells[idx].min = { _globalMin.x + static_cast<double>(x) * cellWidth, _globalMin.y + static_cast<double>(y) * cellHeight };
        gridCells[idx].max = { gridCells[idx].min.x + cellWidth, gridCells[idx].min.y + cellHeight };
      }
    });

    // 并行分配瓦片到网格
    tbb::parallel_for(tbb::blocked_range<size_t>(0, _tiles.size()), [&](const tbb::blocked_range<size_t>& r) {
      for (size_t i = r.begin(); i < r.end(); ++i)
      {
        const Tile& tile = _tiles[i];

        // 计算瓦片所在的网格范围 (添加边界检查)
        const size_t startX = std::min(static_cast<size_t>((tile.min.x - _globalMin.x) / cellWidth), gridX - 1);
        const size_t endX   = std::min(static_cast<size_t>((tile.max.x - _globalMin.x) / cellWidth), gridX - 1);
        const size_t startY = std::min(static_cast<size_t>((tile.min.y - _globalMin.y) / cellHeight), gridY - 1);
        const size_t endY   = std::min(static_cast<size_t>((tile.max.y - _globalMin.y) / cellHeight), gridY - 1);

        // 将瓦片添加到覆盖的所有网格
        for (size_t y = startY; y <= endY; ++y)
        {
          for (size_t x = startX; x <= endX; ++x)
          {
            const size_t cellIdx = y * gridX + x;
            gridCells[cellIdx].tileIndices.push_back(i);
            gridCells[cellIdx].fileNames.insert(tile.fileName);
          }
        }
      }
    });

    return gridCells;
  }

  /**
   * @brief 获取所有文件名集合
   * @return const tbb::concurrent_set<std::string>&
   */
  const tbb::concurrent_set<std::string>& GetAllFileNames() const
  {
    return _totalFiles;
  }

  /**
   * @brief 验证网格分区
   * @param gridCells 网格单元
   * @param tiles 瓦片
   * @param maxTilesPerCell 最大瓦片数量
   * @return true 验证通过, false 验证失败
   */
  static bool ValidateGridPartition(const std::vector<GridCell>& gridCells, const std::vector<Tile>& tiles, size_t maxTilesPerCell)
  {
    // 检查所有瓦片是否都被分配
    std::vector<bool> tileAssigned(tiles.size(), false);

    for (const GridCell& cell : gridCells)
    {
      // 检查网格瓦片数量
      if (cell.tileIndices.size() > maxTilesPerCell)
      {
        SPDLOG_ERROR("Error: The grid contains {} tiles, which exceeds the maximum of {}.", cell.tileIndices.size(), maxTilesPerCell);
        return false;
      }

      // 检查瓦片是否在网格内
      for (const size_t tileIdx : cell.tileIndices)
      {
        if (tileIdx >= tiles.size())
        {
          SPDLOG_ERROR("Error: Invalid tile index {}.", tileIdx);
          return false;
        }

        const Tile& tile = tiles[tileIdx];
        if (tile.min.x < cell.min.x || tile.min.y < cell.min.y || tile.max.x > cell.max.x || tile.max.y > cell.max.y)
        {
          SPDLOG_ERROR("Error: Tile is outside the grid boundary.");
          return false;
        }

        tileAssigned[tileIdx] = true;
      }
    }

    // 检查所有瓦片是否都被分配
    for (size_t i = 0; i < tileAssigned.size(); ++i)
    {
      if (!tileAssigned[i])
      {
        SPDLOG_ERROR("Error: Tile {} is not assigned.", i);
        return false;
      }
    }

    return true;
  }

  /**
   * @brief 验证网格分区
   * @param gridCells 网格单元
   * @param tiles 瓦片
   * @return true 验证通过, false 验证失败
   */
  static bool ValidateGridPartition(const std::vector<GridCell>& gridCells, const std::vector<Tile>& tiles)
  {
    int64 gridsNum = 0;
    for (const GridCell& cell : gridCells)
    {
      gridsNum += cell.tileIndices.size();
    }

    if (gridsNum < tiles.size())
    {
      return false;
    }

    return true;
  }

  /**
   * @brief 验证网格分区
   * @param gridCells 网格单元
   * @param pTgp 瓦片网格分区器
   * @return true 验证通过, false 验证失败
   */
  static bool ValidateGridPartition(const std::vector<GridCell>& gridCells, const TileGridPartitioner* pTgp)
  {
    tbb::concurrent_set<std::string> totals;

    tbb::parallel_for(static_cast<size_t>(0), gridCells.size(), [&](const size_t i) {
      const auto& src = gridCells[i].fileNames;
      for (const auto& elem : src)
      {
        totals.insert(elem);
      }
    });

    SPDLOG_INFO("Partition file count: {}, Total files: {}", totals.size(), pTgp->GetAllFileNames().size());

    // 检查文件数量
    if (pTgp->GetAllFileNames().size() != totals.size())
    {
      return false;
    }

    // 检查文件内容
    for (const auto& file : pTgp->GetAllFileNames())
    {
      if (totals.find(file) == totals.end())
      {
        SPDLOG_ERROR("File missing in partition: {}", file);
        return false;
      }
    }

    return true;
  }

private:
  std::vector<Tile> _tiles;
  glm::dvec2 _globalMin;
  glm::dvec2 _globalMax;
  size_t _maxTilesPerCell;
  tbb::concurrent_set<std::string> _totalFiles;  // 存储所有文件名

  // 初始化时收集所有文件名
  void InitializeFileNames()
  {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, _tiles.size()), [&](const tbb::blocked_range<size_t>& r) {
      for (size_t i = r.begin(); i < r.end(); ++i)
      {
        _totalFiles.insert(_tiles[i].fileName);
      }
    });

    SPDLOG_INFO("Total files: {}", _totalFiles.size());
  }

  // 检查是否应停止分割
  bool ShouldStopPartitioning(const QuadTreeNode* node) const
  {
    return node->tileIndices.size() <= _maxTilesPerCell || (node->max.x - node->min.x) < 1e-6 || (node->max.y - node->min.y) < 1e-6;
  }

  // 四叉树递归构建函数
  void BuildQuadTree(QuadTreeNode* node)
  {
    if (ShouldStopPartitioning(node))
      return;

    // 计算中心点
    const glm::dvec2 center = (node->min + node->max) * 0.5;

    // 创建四个子节点
    node->children[0]      = std::make_unique<QuadTreeNode>();  // 左下
    node->children[0]->min = node->min;
    node->children[0]->max = center;

    node->children[1]      = std::make_unique<QuadTreeNode>();  // 右下
    node->children[1]->min = { center.x, node->min.y };
    node->children[1]->max = { node->max.x, center.y };

    node->children[2]      = std::make_unique<QuadTreeNode>();  // 左上
    node->children[2]->min = { node->min.x, center.y };
    node->children[2]->max = { center.x, node->max.y };

    node->children[3]      = std::make_unique<QuadTreeNode>();  // 右上
    node->children[3]->min = center;
    node->children[3]->max = node->max;

    // 分配瓦片到子节点
    std::array<std::vector<size_t>, 4> childTileIndices;
    static std::array<std::mutex, 4> mutexes;  // 静态互斥锁

    tbb::parallel_for(tbb::blocked_range<size_t>(0, node->tileIndices.size()), [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx)
      {
        const size_t tileIdx = node->tileIndices[idx];
        const Tile& tile     = _tiles[tileIdx];

        for (int32 i = 0; i < 4; ++i)
        {
          if (TileOverlapsCell(tile, node->children[i]->min, node->children[i]->max))
          {
            std::lock_guard<std::mutex> lock(mutexes[i]);
            childTileIndices[i].push_back(tileIdx);
          }
        }
      }
    });

    // 将分配的瓦片转移到子节点
    for (int32 i = 0; i < 4; ++i)
    {
      node->children[i]->tileIndices = std::move(childTileIndices[i]);
    }

    // 清空父节点瓦片列表
    node->tileIndices.clear();
    node->tileIndices.shrink_to_fit();

    // 并行递归处理子节点
    tbb::parallel_for(0, 4, [&](const int32 i) {
      if (node->children[i] && !node->children[i]->tileIndices.empty())
      {
        BuildQuadTree(node->children[i].get());
      }
    });
  }

  // 检查瓦片是否与网格单元重叠
  bool TileOverlapsCell(const Tile& tile, const glm::dvec2& cellMin, const glm::dvec2& cellMax) const
  {
    return !(tile.max.x < cellMin.x || tile.min.x > cellMax.x || tile.max.y < cellMin.y || tile.min.y > cellMax.y);
  }

  // 从四叉树提取网格单元
  void ExtractGridCells(const QuadTreeNode* node, tbb::concurrent_vector<GridCell>& gridCells)
  {
    if (!node)
      return;

    // 检查是否为叶节点
    bool isLeaf = true;
    for (int32 i = 0; i < 4; ++i)
    {
      if (node->children[i])
      {
        isLeaf = false;
        break;
      }
    }

    // 如果是叶节点且有瓦片，添加到结果
    if (isLeaf && !node->tileIndices.empty())
    {
      GridCell cell;
      cell.min = node->min;
      cell.max = node->max;

      // 添加瓦片索引和文件名
      for (size_t tileIdx : node->tileIndices)
      {
        cell.tileIndices.push_back(tileIdx);
        cell.fileNames.insert(_tiles[tileIdx].fileName);
      }

      gridCells.push_back(std::move(cell));
      return;
    }

    // 并行处理子节点
    tbb::parallel_for(0, 4, [&](const int32 i) {
      if (node->children[i])
      {
        ExtractGridCells(node->children[i].get(), gridCells);
      }
    });
  }
};

}  // namespace Zondy

#endif
