#pragma once
#pragma once

#ifndef BEIDOU_GRID_3D_H_
#define BEIDOU_GRID_3D_H_

#include "PositioningGrid.h"
#include <vector>

namespace Grid
{
    class BeiDouGrid3D final : public PositioningGrid
    {
    public:
        BeiDouGrid3D();
        ~BeiDouGrid3D() override;

        /**
         * @brief 初始化网格的初始信息
         */
        void Init() override;

        /**
         * @brief 获取网格类型
         * @return GridType 网格类型
         */
        [[nodiscard]] GridType GetGridType() const override;

        /**
         * @brief 获取网格中心坐标
         * @param gridCode 网格编码
         * @return glm::dvec3 网格中心坐标
         */
        [[nodiscard]] glm::dvec3 GetGridCenter(const std::string& gridCode) const override;

        /**
         * @brief 获取网格中心坐标
         * @param lonlatheight 点的坐标
         * @param level 网格级数
         * @return glm::dvec3 网格中心坐标
         */
        [[nodiscard]] glm::dvec3 GetGridCenter(const glm::dvec3& lonlatheight, const int32& level) const override;

        /**
         * @brief 获取二维网格中心坐标
         * @param gridCode 网格编码
         * @return glm::dvec3 网格中心坐标
         */
        [[nodiscard]] virtual glm::dvec3 GetGrid2DCenter(const std::string& gridCode) const override;

        /**
         * @brief 获取二维网格中心坐标
         * @param point 点的坐标
         * @param level 网格级数
         * @return glm::dvec3 网格中心坐标
         */
        [[nodiscard]] virtual glm::dvec3 GetGrid2DCenter(const glm::dvec3& point, const int32& level) const override;

        /**
         * @brief 构建点所在的二维网格编码
         * @param lonlatheight 点的坐标
         * @param level 北斗网格级别
         * @return std::string 网格编码
         */
        [[nodiscard]] std::string BuildGridCode2D(const glm::dvec3& lonlatheight, const int32& level) const override;

        /**
         * @brief 构建点所在的三维网格编码
         * @param lonlatheight 点的坐标
         * @param level 北斗网格级别
         * @return std::string 网格编码
         */
        [[nodiscard]] std::string BuildGridCode3D(const glm::dvec3& lonlatheight, const int32& level) const override;

        /**
         * @return 计算指定级别的网格在经度方向的个数
         * @param min 最小经度
         * @param max 最大经度
         * @param level 网格级别
         * @return 网格经度方向的个数
         */
        [[nodiscard]] virtual int64 GetLonMaxGrid(const double& min, const double& max, const int32& level) const override;

        /**
         * @return 计算指定级别的网格在纬度方向的个数
         * @param min 最小纬度
         * @param max 最大纬度
         * @param level 网格级别
         * @return 网格纬度方向的个数
         */
        [[nodiscard]] virtual int64 GetLatMaxGrid(const double& min, const double& max, const int32& level) const override;

        /**
         * @return 计算指定级别的网格在高度方向的个数
         * @param min 最小高度
         * @param max 最大高度
         * @param level 网格级别
         * @return 网格高度方向的个数
         */
        [[nodiscard]] virtual int64 GetHeightMaxGrid(const double& min, const double& max, const int32& level) const override;

        /**
         * @brief 获取包围盒范围内指定级别下的网格总量
         * @param min 包围盒最小角点
         * @param max 包围盒最大角点
         * @param level 网格级别
         * @return 网格总数
         */
        [[nodiscard]] virtual int64 GetMaxGridNumByBox(const glm::dvec3& min, const glm::dvec3& max, const int32& level) const override;

        /**
         * @brief 获取当个网格经度的跨度
         * @param level 网格级别
         * @return 单个网格经度的跨度
         */
        [[nodiscard]] virtual double GetStepLon(const int32& level) const override;

        /**
         * @brief 获取当个网格纬度的跨度
         * @param level 网格级别
         * @return 单个网格纬度的跨度
         */
        [[nodiscard]] virtual double GetStepLat(const int32& level) const override;

        /**
         * @brief 根据采样点计算被占用的全部网格
         * @param samplePoints 采样点
         * @param level 网格级别
         * @param aggregationLevel 需要聚合的网格级别
         * @param grids 生成的网格集合
         * @param minHeight 最小高程
         * @param maxHeight 最大高程
         */
        void GenOccupiedGridsWithSamplePoints(const std::vector<glm::dvec3>& samplePoints, const int32& level, const int32& aggregationLevel,
            std::map<std::string, std::vector<FGridInfo>>& grids, const double minHeight,
            const double maxHeight) const override;

        /**
         * @brief 根据二维网格码，和二维网格内的采样点，生成当前级别下的全量网格
         * @param parentCode2D 当前采样点所在的父级二维网格码
         * @param samplePoints 采样点
         * @param level 实际网格级别
         * @param minHeight 最小高程
         * @param maxHeight 最大高程
         * @param grids 生成的网格集合
         */
        virtual void GenFullGridsWithParentCode2D(const std::string& parentCode2D, const std::vector<glm::dvec3>& samplePoints, const int32& level,
            const double& minHeight, const double& maxHeight,
            std::map<std::string, std::vector<FGridInfo>>& grids) const override;

        /**
         * @brief 根据当前高度，获取对应的网格索引，索引从0开始计算
         * @param deltaLatDegree 纬度间隔
         * @param curHeight 当前高度
         * @return glm::int64 网格索引
         */
        static [[nodiscard]] glm::int64 GetBeidouGridHeightIndex(const double& deltaLatDegree, const double& curHeight);

        /**
         * @brief 获取指定索引的北斗网格的底面高度
         * @param deltaLatDegree 纬度间隔
         * @param heightIndex 网格索引，索引从0开始
         * @return double 网格底面高度
         */
        static [[nodiscard]] double GetBeiDouGridHeight(const double& deltaLatDegree, const int32& heightIndex);

        /**
         * @brief 获取北斗网格长或者高
         * @param deltaLatDegree 纬度间隔
         * @param heightIndex 高度索引
         * @return double 网格高度
         */
        static [[nodiscard]] double GetBeiDouGridZorYHeight(const double& deltaLatDegree, const int32& heightIndex);

        /**
         * @brief 网格校正
         * @param lon 经度
         * @param lat 纬度
         * @param level 级别
         * @param outLon 输出经度
         * @param outLat 输出纬度
         * @param bUseAnchorCorner 是否使用锚点角
         * @param bExtend 是否扩展
         */
        static void CorrectionCoordinates(const double lon, const double lat, const int32 level, double& outLon, double& outLat,
            const bool bUseAnchorCorner = true, const bool bExtend = false);

    private:
        /**
         * @brief 获取不同级别下的经度步长
         * @param level 北斗网格级别
         * @return 经度步长，角度值
         */
        [[nodiscard]] double GetStepLonDegree(const int32& level) const;

        /**
         * @brief 获取不同级别下的维度步长
         * @param level 北斗网格级别
         * @return 纬度步长，角度值
         */
        [[nodiscard]] double GetStepLatDegree(const int32& level) const;

        /**
         * @brief 获取不同级别下，用于计算纬度方向和高度方向的，经纬度步长
         * @param level 北斗网格级别
         * @return 第三级取经度步长，其他级别取纬度步长，角度值
         */
        [[nodiscard]] double GetStepZorYDegree(const int32& level) const;

        [[nodiscard]] std::vector<std::string> BuildBeiDouGrid2D(const glm::dvec3& lonlatheight, const int32& level) const;
        [[nodiscard]] std::vector<std::string> BuildBeiDouGridHeightCode(const double height, const int32& level) const;
        [[nodiscard]] std::string ConnectBeiDouGridCode(const std::vector<std::string>& code2d, const std::vector<std::string>& codeHeight) const;

        [[nodiscard]] int32 ConvertChar2Int32(const char c) const;

        [[nodiscard]] int64_t IndexOfVector(const std::vector<char>& v, char value) const;

        [[nodiscard]] int32 GetLevelByCode2D(const std::string_view& code2d) const;

        [[nodiscard]] int32 GetLevelByCode3D(const std::string_view& code3d) const;

    private:
        GridType _gridType = GridType::kBeiDouGrid;

        std::vector<double> _levelClassifiedStandardLon;
        std::vector<double> _levelClassifiedStandardLat;

        std::vector<char> _codeLetter;
        std::vector<char> _codeNumber;
    };

}  // namespace Grid

#endif

// end
