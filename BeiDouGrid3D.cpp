#include "BeiDouGrid3D.h"

#include "D:\Users\Administrator\Desktop\PostgreSQL\Common\CommonAlgorithm.h"
#include <bitset>
#include <glm/ext/scalar_constants.hpp>
#include <oneapi/tbb/parallel_for.h>
#include <spdlog/spdlog.h>
#include <tbb/concurrent_map.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for_each.h>

namespace Grid
{

    BeiDouGrid3D::BeiDouGrid3D()
    {
        Init();
    }

    BeiDouGrid3D::~BeiDouGrid3D() {}

    void BeiDouGrid3D::Init()
    {
        _levelClassifiedStandardLon = { 6.0,
                                        30.0 / 60.0,
                                        15.0 / 60.0,
                                        1.0 / 60.0,
                                        4.0 / 3600.0,
                                        2.0 / 3600.0,
                                        1.0 / (3600.0 * 4.0),
                                        1.0 / (3600.0 * 32.0),
                                        1.0 / (3600.0 * 256.0),
                                        1.0 / (3600.0 * 2048.0) };

        _levelClassifiedStandardLat = { 4.0,
                                        30.0 / 60.0,
                                        10.0 / 60.0,
                                        1.0 / 60.0,
                                        4.0 / 3600.0,
                                        2.0 / 3600.0,
                                        1.0 / (3600.0 * 4.0),
                                        1.0 / (3600.0 * 32.0),
                                        1.0 / (3600.0 * 256.0),
                                        1.0 / (3600.0 * 2048.0) };

        _codeLetter = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P', 'Q', 'R', 'S', 'T', 'U', 'V' };

        _codeNumber = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E' };
    }

    GridType BeiDouGrid3D::GetGridType() const
    {
        return _gridType;
    }

    glm::dvec3 BeiDouGrid3D::GetGridCenter(const std::string& gridCode) const
    {
        glm::vec3 nswe(1.0, 1.0, 1.0);

        // 根据级别拆分网格码
        // (1)(2)
        const char ns = gridCode[0];
        if (ns != 'S' && ns != 'N')
        {
            // 既不在南半球，也不在北半球
            return glm::dvec3(std::numeric_limits<double>::max());
        }
        const char floor = gridCode[1];
        if (floor != '0' && floor != '1')
        {
            // 既不在地面，也不在地下
            return glm::dvec3(std::numeric_limits<double>::max());
        }

        nswe.z = floor == '0' ? 1 : -1;

        glm::dvec3 preAnchorPoint(-180.0, -88.0, 0.0);

        int64 a = 0;
        int64 b = 0;
        int64 n = 0;

        size_t subStrLength = 0;

        glm::dvec3 newAnchorPoint(0);

        std::string nStr;
        const int32 realLevel = GetLevelByCode3D(gridCode);
        for (int32 l = 1; l < realLevel + 1; l++)
        {
            // (3,4,5)(6,7)
            if (l == 1 && gridCode.length() >= 7)
            {
                a = 10 * ConvertChar2Int32(gridCode[2]) + ConvertChar2Int32(gridCode[3]) - 1;
                b = IndexOfVector(_codeLetter, gridCode[4]);
                n = 10 * ConvertChar2Int32(gridCode[5]) + ConvertChar2Int32(gridCode[6]);

                subStrLength = 6;

                if (ns == 'S')
                {
                    b = (22 - b) % 22;
                    nswe.y = -1;
                }
                else
                {
                    b += 22;
                }

                nswe.x = a > 30 ? 1 : -1;
            }
            // (8,9)(10)
            else if (l == 2 && gridCode.length() >= 10)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[7])) * static_cast<int32>(glm::round(nswe.x)) + 12) % 12;
                b = ((IndexOfVector(_codeNumber, gridCode[8])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;

                n = IndexOfVector(_codeNumber, gridCode[9]);
                subStrLength = 3;
            }
            // (11)(12)
            else if (l == 3 && gridCode.length() >= 12)
            {
                const int64 aB = IndexOfVector(_codeNumber, gridCode[10]);
                a = aB % 2;
                b = (aB - a) / 2;
                n = IndexOfVector(_codeNumber, gridCode[11]);
                subStrLength = 1;
            }
            // (13,14)(15)
            else if (l == 4 && gridCode.length() >= 15)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[12])) * static_cast<int32>(glm::round(nswe.x)) + 15) % 15;
                b = ((IndexOfVector(_codeNumber, gridCode[13])) * static_cast<int32>(glm::round(nswe.y)) + 10) % 10;

                n = IndexOfVector(_codeNumber, gridCode[14]);
                subStrLength = 4;
            }
            // (16,17)(18)
            else if (l == 5 && gridCode.length() >= 18)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[15])) * static_cast<int32>(glm::round(nswe.x)) + 15) % 15;
                b = ((IndexOfVector(_codeNumber, gridCode[16])) * static_cast<int32>(glm::round(nswe.y)) + 15) % 15;

                n = IndexOfVector(_codeNumber, gridCode[17]);
                subStrLength = 4;
            }
            // (19)(20)
            else if (l == 6 && gridCode.length() >= 20)
            {
                const int64 aB = IndexOfVector(_codeNumber, gridCode[18]);
                a = aB % 2;
                b = (aB - a) / 2;
                n = IndexOfVector(_codeNumber, gridCode[19]);
                subStrLength = 1;
            }
            // (21,22)(23)
            else if (l == 7 && gridCode.length() >= 23)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[20])) * static_cast<int32>(glm::round(nswe.x)) + 8) % 8;
                b = ((IndexOfVector(_codeNumber, gridCode[21])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;

                n = IndexOfVector(_codeNumber, gridCode[22]);
                subStrLength = 3;
            }
            // (24,25)(26)
            else if (l == 8 && gridCode.length() >= 26)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[23])) * static_cast<int32>(glm::round(nswe.x)) + 8) % 8;
                b = ((IndexOfVector(_codeNumber, gridCode[24])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;

                n = IndexOfVector(_codeNumber, gridCode[25]);
                subStrLength = 3;
            }
            // (27,28)(29)
            else if (l == 9 && gridCode.length() >= 29)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[26])) * static_cast<int32>(glm::round(nswe.x)) + 8) % 8;
                b = ((IndexOfVector(_codeNumber, gridCode[27])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;

                n = IndexOfVector(_codeNumber, gridCode[28]);
                subStrLength = 3;
            }
            // (30,31)(32)
            else if (l == 10 && gridCode.length() >= 32)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[29])) * static_cast<int32>(glm::round(nswe.x)) + 8) % 8;
                b = ((IndexOfVector(_codeNumber, gridCode[30])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;

                n = IndexOfVector(_codeNumber, gridCode[31]);
                subStrLength = 3;
            }

            newAnchorPoint.x = preAnchorPoint.x + static_cast<double>(a) * GetStepLonDegree(l);
            newAnchorPoint.y = preAnchorPoint.y + static_cast<double>(b) * GetStepLatDegree(l);

            std::bitset<32> niBitset(n);
            std::string niStr = niBitset.to_string();
            niStr = niStr.substr(32 - subStrLength);
            nStr = niStr + nStr;

            preAnchorPoint = newAnchorPoint;

            if (newAnchorPoint.x > 180.0 || newAnchorPoint.y > 88.0 || newAnchorPoint.x < -180.0 || newAnchorPoint.y < -88.0)
            {
                return glm::dvec3(std::numeric_limits<double>::max());
            }
        }

        const std::bitset<32> nVal(nStr);
        n = nVal.to_ulong();

        const double deltaDegree = GetStepZorYDegree(realLevel);
        newAnchorPoint.z = GetBeiDouGridHeight(deltaDegree, n);

        const double halfLen = GetBeiDouGridZorYHeight(deltaDegree, n) * 0.5;

        return newAnchorPoint + glm::dvec3(GetStepLonDegree(realLevel) * 0.5, GetStepLatDegree(realLevel) * 0.5, halfLen);
    }

    glm::dvec3 BeiDouGrid3D::GetGridCenter(const glm::dvec3& lonlatheight, const int32& level) const
    {
        return GetGridCenter(BuildGridCode3D(lonlatheight, level));
    }

    glm::dvec3 BeiDouGrid3D::GetGrid2DCenter(const std::string& gridCode) const
    {
        glm::vec2 nswe(1.0, 1.0);

        // 根据级别拆分网格码
        // (1)
        const char ns = gridCode[0];
        if (ns != 'S' && ns != 'N')
        {
            // 既不在南半球，也不在北半球
            return glm::dvec3(std::numeric_limits<double>::max());
        }
        // const char floor = gridCode[1];
        // if (floor != '0' && floor != '1')
        //{
        //   // 既不在地面，也不在地下
        //   return glm::dvec3(std::numeric_limits<double>::max());
        // }

        // nswe.z = floor == '0' ? 1 : -1;

        glm::dvec3 preAnchorPoint(-180.0, -88.0, 0.0);

        int64 a = 0;
        int64 b = 0;

        glm::dvec3 newAnchorPoint(0);

        std::string nStr;
        const int32 realLevel = GetLevelByCode2D(gridCode);
        for (int32 l = 1; l < realLevel + 1; l++)
        {
            // (1,2,3)
            if (l == 1 && gridCode.length() >= 4)
            {
                a = 10 * ConvertChar2Int32(gridCode[1]) + ConvertChar2Int32(gridCode[2]) - 1;
                b = IndexOfVector(_codeLetter, gridCode[3]);

                if (ns == 'S')
                {
                    b = (22 - b) % 22;
                    nswe.y = -1;
                }
                else
                {
                    b += 22;
                }

                nswe.x = a > 30 ? 1 : -1;
            }
            // (4,5)
            else if (l == 2 && gridCode.length() >= 6)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[4])) * static_cast<int32>(glm::round(nswe.x)) + 12) % 12;
                b = ((IndexOfVector(_codeNumber, gridCode[5])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;
            }
            // (6)
            else if (l == 3 && gridCode.length() >= 7)
            {
                const int64 aB = IndexOfVector(_codeNumber, gridCode[6]);
                a = aB % 2;
                b = (aB - a) / 2;
            }
            // (7,8)
            else if (l == 4 && gridCode.length() >= 9)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[7])) * static_cast<int32>(glm::round(nswe.x)) + 15) % 15;
                b = ((IndexOfVector(_codeNumber, gridCode[8])) * static_cast<int32>(glm::round(nswe.y)) + 10) % 10;
            }
            // (9,10)
            else if (l == 5 && gridCode.length() >= 11)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[9])) * static_cast<int32>(glm::round(nswe.x)) + 15) % 15;
                b = ((IndexOfVector(_codeNumber, gridCode[10])) * static_cast<int32>(glm::round(nswe.y)) + 15) % 15;
            }
            // (11)
            else if (l == 6 && gridCode.length() >= 12)
            {
                const int64 aB = IndexOfVector(_codeNumber, gridCode[11]);
                a = aB % 2;
                b = (aB - a) / 2;
            }
            // (12,13)
            else if (l == 7 && gridCode.length() >= 14)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[12])) * static_cast<int32>(glm::round(nswe.x)) + 8) % 8;
                b = ((IndexOfVector(_codeNumber, gridCode[13])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;
            }
            // (14,15)
            else if (l == 8 && gridCode.length() >= 16)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[14])) * static_cast<int32>(glm::round(nswe.x)) + 8) % 8;
                b = ((IndexOfVector(_codeNumber, gridCode[15])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;
            }
            // (16,17)
            else if (l == 9 && gridCode.length() >= 18)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[16])) * static_cast<int32>(glm::round(nswe.x)) + 8) % 8;
                b = ((IndexOfVector(_codeNumber, gridCode[17])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;
            }
            // (18,19)
            else if (l == 10 && gridCode.length() >= 20)
            {
                a = ((IndexOfVector(_codeNumber, gridCode[18])) * static_cast<int32>(glm::round(nswe.x)) + 8) % 8;
                b = ((IndexOfVector(_codeNumber, gridCode[19])) * static_cast<int32>(glm::round(nswe.y)) + 8) % 8;
            }

            newAnchorPoint.x = preAnchorPoint.x + static_cast<double>(a) * GetStepLonDegree(l);
            newAnchorPoint.y = preAnchorPoint.y + static_cast<double>(b) * GetStepLatDegree(l);

            preAnchorPoint = newAnchorPoint;

            if (newAnchorPoint.x > 180.0 || newAnchorPoint.y > 88.0 || newAnchorPoint.x < -180.0 || newAnchorPoint.y < -88.0)
            {
                return glm::dvec3(std::numeric_limits<double>::max());
            }
        }

        return newAnchorPoint + glm::dvec3(GetStepLonDegree(realLevel) * 0.5, GetStepLatDegree(realLevel) * 0.5, 0.0);
    }

    glm::dvec3 BeiDouGrid3D::GetGrid2DCenter(const glm::dvec3& point, const int32& level) const
    {
        return GetGrid2DCenter(BuildGridCode2D(point, level));
    }

    std::string BeiDouGrid3D::BuildGridCode2D(const glm::dvec3& lonlatheight, const int32& level) const
    {
        std::string code2dStr;

        const std::vector<std::string> code2d = BuildBeiDouGrid2D(lonlatheight, level);
        for (const std::string& code : code2d)
        {
            code2dStr.append(code);
        }

        return code2dStr;
    }

    std::string BeiDouGrid3D::BuildGridCode3D(const glm::dvec3& lonlatheight, const int32& level) const
    {
        const std::vector<std::string> code2d = BuildBeiDouGrid2D(lonlatheight, level);
        const std::vector<std::string> codeHeight = BuildBeiDouGridHeightCode(lonlatheight.z, level);
        return ConnectBeiDouGridCode(code2d, codeHeight);
    }

    int64 BeiDouGrid3D::GetLonMaxGrid(const double& min, const double& max, const int32& level) const
    {
        const double lonStep = _levelClassifiedStandardLon[level - 1];

        double left = min - lonStep;
        double bottom = 0.0;
        double right = max + lonStep;
        double top = 0;

        CorrectionCoordinates(left, bottom, level, left, bottom, true);
        CorrectionCoordinates(right, top, level, right, top, true, true);

        return static_cast<int64>(std::ceil((right - left) / lonStep));
    }

    int64 BeiDouGrid3D::GetLatMaxGrid(const double& min, const double& max, const int32& level) const
    {
        const double lonStep = _levelClassifiedStandardLon[level - 1];
        const double latStep = _levelClassifiedStandardLat[level - 1];

        double left = 0.0;
        double bottom = min - latStep;
        double right = 0.0;
        double top = max + latStep;

        CorrectionCoordinates(left, bottom, level, left, bottom, true);
        CorrectionCoordinates(right, top, level, right, top, true, true);

        return static_cast<int64>(std::ceil((top - bottom) / latStep));
    }

    int64 BeiDouGrid3D::GetHeightMaxGrid(const double& min, const double& max, const int32& level) const
    {
        const double heightStep = GetBeiDouGridZorYHeight(GetStepZorYDegree(level), 0);

        const double minHeight = min - 2.0 > 0.0 ? min - 2.0 : 0.0;
        const double maxHeight = max + 10.0;

        return static_cast<int64>(std::ceil((maxHeight - minHeight) / heightStep));
    }

    int64 BeiDouGrid3D::GetMaxGridNumByBox(const glm::dvec3& min, const glm::dvec3& max, const int32& level) const
    {
        const double lonStep = _levelClassifiedStandardLon[level - 1];
        const double latStep = GetStepZorYDegree(level);
        // 低空下，偏差只有几毫米，可忽略不计
        const double heightStep = GetBeiDouGridZorYHeight(GetStepZorYDegree(level), 0);

        double left = min.x - lonStep;
        double bottom = min.y - latStep;
        double right = max.x + lonStep;
        double top = max.y + latStep;

        // 容差
        const double minHeight = min.z - 2.0 > 0.0 ? min.z - 2.0 : 0.0;
        const double maxHeight = max.z + 10.0;

        CorrectionCoordinates(left, bottom, level, left, bottom, false);
        CorrectionCoordinates(right, top, level, right, top, false);

        const int64 xCount = static_cast<int64>(std::ceil((right - left) / lonStep));
        const int64 yCount = static_cast<int64>(std::ceil((top - bottom) / latStep));
        const int64 zCount = static_cast<int64>(std::ceil((maxHeight - minHeight) / heightStep));

        return xCount * yCount * zCount;
    }

    double BeiDouGrid3D::GetStepLon(const int32& level) const
    {
        return GetStepLonDegree(level);
    }

    double BeiDouGrid3D::GetStepLat(const int32& level) const
    {
        return GetStepLatDegree(level);
    }

    void BeiDouGrid3D::GenOccupiedGridsWithSamplePoints(const std::vector<glm::dvec3>& samplePoints, const int32& level, const int32& aggregationLevel,
        std::map<std::string, std::vector<FGridInfo>>& grids, const double minHeight,
        const double maxHeight) const
    {
        tbb::concurrent_map<std::string, tbb::concurrent_vector<FGridInfo>> tmpGrids;

        const double lonStep = GetStepLonDegree(level);
        const double latStep = GetStepLatDegree(level);
        // 低空下，偏差只有几毫米，可忽略不计
        const double heightStep = GetBeiDouGridZorYHeight(GetStepZorYDegree(level), 1);

        tbb::parallel_for(static_cast<size_t>(0), samplePoints.size(), [&](const size_t& i) {
            // 按照高程点处理网格
            const glm::dvec3 lonlatheight = samplePoints[i];

            const int64 endIndex = GetBeidouGridHeightIndex(latStep, lonlatheight.z);
            const int64 beginIndex = GetBeidouGridHeightIndex(latStep, minHeight) < endIndex ? GetBeidouGridHeightIndex(latStep, minHeight) - 1 : endIndex;

            for (int64 j = beginIndex; j < endIndex + 1; j++)
            {
                const auto center = glm::dvec3(lonlatheight.x, lonlatheight.y, (static_cast<double>(j) + 0.5) * heightStep);

                // 计算父亲的网格码, 根据二维网格空间进行合并
                std::string parentCodeStr = BuildGridCode3D(center, aggregationLevel);

                FGridInfo grid = { center, BuildGridCode3D(center, level), true };

                tmpGrids[parentCodeStr].emplace_back(grid);
            }
            });

        grids.clear();
        // 并行地把 (key, vector) 收集到一个并发容器里
        tbb::concurrent_vector<std::pair<std::string, std::vector<FGridInfo>>> vecGrids;

        tbb::parallel_for_each(tmpGrids.range(), [&](const tbb::concurrent_map<std::string, tbb::concurrent_vector<FGridInfo>>::value_type& kv) {
            // concurrent_vector -> vector 直接区间构造
            vecGrids.emplace_back(kv.first, std::vector<FGridInfo>(kv.second.begin(), kv.second.end()));
            });

        // 串行地插到最终 map（std::map 非线程安全）
        grids.insert(vecGrids.begin(), vecGrids.end());
    }

    void BeiDouGrid3D::GenFullGridsWithParentCode2D(const std::string& parentCode2D, const std::vector<glm::dvec3>& samplePoints, const int32& level,
        const double& minHeight, const double& maxHeight,
        std::map<std::string, std::vector<FGridInfo>>& grids) const
    {
        // tbb::concurrent_map<std::string, tbb::concurrent_vector<FGridInfo>> tmpGrids;

        // 按照顺序，初始化父节点二维网格码中的全部网格
        const int32 parentLevel = level - 1;
        const double parentLonStep = GetStepLonDegree(parentLevel);
        const double parentLatStep = GetStepLatDegree(parentLevel);

        const int32 parentBeginIndex = static_cast<int32>(GetBeidouGridHeightIndex(GetStepZorYDegree(parentLevel), minHeight));
        const int32 parentEndIndex = static_cast<int32>(GetBeidouGridHeightIndex(GetStepZorYDegree(parentLevel), maxHeight));
        const double parentHeightStep = GetBeiDouGridZorYHeight(GetStepZorYDegree(parentLevel), parentBeginIndex);

        // 孩子节点的网格大小
        const double lonStep = GetStepLonDegree(level);
        const double latStep = GetStepLatDegree(level);
        const double heightStep = GetBeiDouGridZorYHeight(GetStepZorYDegree(level), 1);

        // 计算到父亲的角点位置
        const glm::dvec3 centerCoordinates = GetGrid2DCenter(parentCode2D);
        const glm::dvec3 cornerCoordinates =
            centerCoordinates + glm::dvec3(parentLonStep * -0.5, parentLatStep * -0.5, GetBeiDouGridHeight(parentBeginIndex, parentLevel));

        const int64 xCount = static_cast<int64>(glm::round(parentLonStep / lonStep));
        const int64 yCount = static_cast<int64>(glm::round(parentLatStep / latStep));
        const int64 zCount = static_cast<int64>(glm::round(parentHeightStep / heightStep));

        // 初始化全量网格
        for (int64 i = parentBeginIndex; i <= parentEndIndex; i++)
        {
            std::string code3d = BuildGridCode3D(centerCoordinates + glm::dvec3(0.0, 0.0, (static_cast<double>(i) + 0.5) * parentHeightStep), parentLevel);
            std::vector<FGridInfo> vecGrids;

            vecGrids.resize(xCount * yCount * zCount);

            // 处理 z 方向
            tbb::parallel_for(static_cast<size_t>(0), static_cast<size_t>(zCount), [&](const size_t& z) {
                // 处理 y 方向
                tbb::parallel_for(static_cast<size_t>(0), static_cast<size_t>(yCount), [&](const size_t& y) {
                    // 处理 x 方向
                    for (size_t x = 0; x < static_cast<size_t>(xCount); x++)
                    {
                        FGridInfo info;
                        info.center = cornerCoordinates
                            + glm::dvec3((static_cast<double>(xCount) + 0.5) * lonStep, (static_cast<double>(yCount) + 0.5) * latStep,
                                (static_cast<double>(zCount) + 0.5) * heightStep);
                        info.state = false;
                        info.code3d = BuildGridCode3D(info.center, level);

                        vecGrids[xCount * yCount * z + xCount * y + x] = info;
                    }
                    });
                });

            grids[code3d] = vecGrids;
        }

        // 根据采样点设置网格状态
        tbb::parallel_for(static_cast<size_t>(0), samplePoints.size(), [&](const size_t& i) {
            // 按照高程点处理网格
            const glm::dvec3 lonlatheight = samplePoints[i];

            const int64 endIndex = GetBeidouGridHeightIndex(latStep, lonlatheight.z > maxHeight ? maxHeight : lonlatheight.z);
            const int64 beginIndex = GetBeidouGridHeightIndex(latStep, minHeight) < endIndex ? GetBeidouGridHeightIndex(latStep, minHeight) : endIndex;

            for (int64 j = beginIndex; j < endIndex + 1; j++)
            {
                const auto center = glm::dvec3(lonlatheight.x, lonlatheight.y, (static_cast<double>(j) + 0.5) * heightStep);

                const int32 hIndex = static_cast<int32>(GetBeidouGridHeightIndex(GetStepZorYDegree(parentLevel), center.z));
                const double bottom = GetBeiDouGridHeight(GetStepZorYDegree(parentLevel), hIndex);

                // z 方向不一致，x、y 方向是一致的
                const int64 x = static_cast<int64>((center.x - cornerCoordinates.x) / lonStep);
                const int64 y = static_cast<int64>((center.y - cornerCoordinates.y) / latStep);
                const int64 z = static_cast<int64>((center.z - bottom) / heightStep);

                // 计算父亲的网格码, 根据二维网格空间进行合并
                std::string parentCodeStr = BuildGridCode3D(center, parentLevel);

                const FGridInfo grid = { center, BuildGridCode3D(center, level), true };

                grids[parentCodeStr][xCount * yCount * z + xCount * y + x] = grid;
            }
            });

        // grids.clear();
        //// 并行地把 (key, vector) 收集到一个并发容器里
        // tbb::concurrent_vector<std::pair<std::string, std::vector<FGridInfo>>> vecGrids;

        // tbb::parallel_for_each(tmpGrids.range(), [&](const tbb::concurrent_map<std::string, tbb::concurrent_vector<FGridInfo>>::value_type& kv) {
        //   // concurrent_vector -> vector 直接区间构造
        //   vecGrids.emplace_back(kv.first, std::vector<FGridInfo>(kv.second.begin(), kv.second.end()));
        // });

        //// 串行地插到最终 map（std::map 非线程安全）
        // grids.insert(vecGrids.begin(), vecGrids.end());
    }

    int64 BeiDouGrid3D::GetBeidouGridHeightIndex(const double& deltaLatDegree, const double& curHeight)
    {
        constexpr double r0 = 6378137.0000000000;
        constexpr double sta0 = glm::pi<double>() / 180.0000000;  // 度=0.017453292519943295弧度
        const double sta = deltaLatDegree * sta0;            // g_zdell[Level - 1] * 0.017453292519943295; // 该网格对应的经(纬)跨度差 单位为弧度
        const double rn = (r0 + curHeight) / r0;
        const double fg = (sta0 / sta) * Common::CommonAlgorithm::LogX(1.0 + sta0, rn);

        int32 rt = static_cast<int32>(glm::floor(fg));
        const float left = static_cast<float>(fg - static_cast<double>(rt));

        // 平衡误差，当点的高度差别0.99时，认为占据了全部格子，向上偏一个索引
        // TODO: 可能需要根据级别来平衡误差
        rt = left > 0.99 ? rt + 1 : rt;

        return rt;
    }

    double BeiDouGrid3D::GetBeiDouGridHeight(const double& deltaLatDegree, const int32& heightIndex)
    {
        constexpr double r0 = 6378137.0;
        constexpr double sta0 = glm::pi<double>() / 180.0;
        const double sta = deltaLatDegree * sta0;

        const double rn = glm::pow(1.0 + sta0, static_cast<double>(heightIndex) / (sta0 / sta));

        return rn * r0 - r0;
    }

    double BeiDouGrid3D::GetBeiDouGridZorYHeight(const double& deltaLatDegree, const int32& heightIndex)
    {
        return GetBeiDouGridHeight(deltaLatDegree, heightIndex + 1) - GetBeiDouGridHeight(deltaLatDegree, heightIndex);
    }

    void BeiDouGrid3D::CorrectionCoordinates(const double lon, const double lat, const int32 level, double& outLon, double& outLat,
        const bool bUseAnchorCorner, const bool bExtend)
    {
        const BeiDouGrid3D beiDou;

        const double rowWorldD = (lon - (-180.0)) / beiDou.GetStepLonDegree(level);
        const int64 rowWorld = bExtend ? static_cast<int64>(glm::ceil(rowWorldD)) : static_cast<int64>(glm::floor(rowWorldD));
        const double anchorLon = -180.0 + static_cast<double>(rowWorld) * beiDou.GetStepLonDegree(level);

        outLon = bUseAnchorCorner ? anchorLon : anchorLon + 0.5 * beiDou.GetStepLonDegree(level);

        const double columnWorldD = (lat - (-88.0)) / beiDou.GetStepLatDegree(level);
        const int64 columnWorld = bExtend ? static_cast<int64>(glm::ceil(columnWorldD)) : static_cast<int64>(glm::floor(columnWorldD));
        const double anchorLat = -88.0 + static_cast<double>(columnWorld) * beiDou.GetStepLatDegree(level);

        outLat = bUseAnchorCorner ? anchorLat : anchorLat + 0.5 * beiDou.GetStepLatDegree(level);
    }

#pragma region 私有方法

    double BeiDouGrid3D::GetStepLonDegree(const int32& level) const
    {
        return _levelClassifiedStandardLon[level - 1];
    }

    double BeiDouGrid3D::GetStepLatDegree(const int32& level) const
    {
        return _levelClassifiedStandardLat[level - 1];
    }

    double BeiDouGrid3D::GetStepZorYDegree(const int32& level) const
    {
        return level == 3 ? GetStepLonDegree(level) : GetStepLatDegree(level);
    }

    std::vector<std::string> BeiDouGrid3D::BuildBeiDouGrid2D(const glm::dvec3& lonlatheight, const int32& level) const
    {
        std::vector<std::string> code2d;

        const double lon = lonlatheight.x;
        const double lat = lonlatheight.y;

        // 判断南北半球, "N" 北半球, "S" 南半球
        std::string bit1 = lat >= 0.0 ? "N" : "S";

        code2d.emplace_back(bit1);

        // 索引与码元取值的映射关系，与点再东北，东南，西南，西北哪个半球有关
        const auto nswe = glm::vec2(lon >= 0 ? 1 : -1, lat >= 0 ? 1 : -1);

        double preLon = -180.0;
        double preLat = 0.0;

        for (int32 le = 1; le <= level; le++)
        {
            const double deltaLon = GetStepLonDegree(le);
            const double deltaLat = GetStepLatDegree(le);

            int32 aRow = 0;
            int32 bColumn = 0;
            int32 a = 0;
            int32 b = 0;

            if (le == 1)
            {
                a = aRow = glm::floor(glm::abs(lon - preLon) / deltaLon);
                b = bColumn = glm::floor(glm::abs(lat - preLat) / deltaLat);
            }
            else
            {
                const int32 splitCubeCountX = static_cast<int32>(glm::round(GetStepLonDegree(le - 1) / GetStepLonDegree(le)));
                const int32 splitCubeCountY = static_cast<int32>(glm::round(GetStepLatDegree(le - 1) / GetStepLatDegree(le)));

                const double aDouble = glm::abs(lon - preLon) / deltaLon;
                aRow = glm::floor(aDouble);
                a = nswe.x > 0 ? aRow : splitCubeCountX - aRow - 1;

                const double bDouble = glm::abs(lat - preLat) / deltaLat;
                bColumn = glm::floor(bDouble);
                b = nswe.y > 0 ? bColumn : splitCubeCountY - bColumn - 1;
            }

            a = a < 0 ? 0 : a;
            b = b < 0 ? 0 : b;

            if (le == 1)
            {
                a = a > 59 ? 59 : a;
                b = b > 21 ? 21 : b;

                std::string bit23;
                if (a < 10)
                {
                    bit23 += "0";
                }
                // 第1级索引是01-60，因此a需要加1
                bit23 += std::to_string(a + 1);  // 第2,3位：01 - 60
                std::string bit4 = { _codeLetter[b] };
                code2d.emplace_back(bit23 + bit4);

                preLat = -88.0;
                bColumn = glm::floor((lat - preLat) / deltaLat);
            }
            else if (le == 2 || le == 4 || le == 5 || le >= 7)
            {
                a = a > 14 ? 14 : a;
                b = b > 14 ? 14 : b;

                code2d.emplace_back(std::string({ _codeNumber[a], _codeNumber[b] }));
            }
            else if (le == 3)
            {
                a = a > 1 ? 1 : a;
                b = b > 2 ? 2 : b;

                code2d.emplace_back(std::to_string(b * 2 + a));
            }
            else if (le == 6)
            {
                a = a > 1 ? 1 : a;
                b = b > 1 ? 1 : b;

                code2d.emplace_back(std::to_string(b * 2 + a));
            }

            preLon = preLon + aRow * deltaLon;
            preLat = preLat + bColumn * deltaLat;
        };

        return code2d;
    }

    std::vector<std::string> BeiDouGrid3D::BuildBeiDouGridHeightCode(const double height, const int32& level) const
    {
        std::vector<std::string> hCode;
        // 地上：0， 地下：1
        std::string a0 = height >= 0 ? "0" : "1";
        hCode.emplace_back(a0);

        const double deltaDegree = GetStepZorYDegree(level);

        const int64 ffg = GetBeidouGridHeightIndex(deltaDegree, height);
        const std::bitset<32> ffgBitSet(ffg);
        const std::string ffgStr = ffgBitSet.to_string();

        std::vector<std::pair<int32, int32>> sePosList;
        sePosList.emplace_back(26, 31);
        sePosList.emplace_back(23, 25);
        sePosList.emplace_back(22, 22);
        sePosList.emplace_back(18, 21);
        sePosList.emplace_back(14, 17);
        sePosList.emplace_back(13, 13);
        sePosList.emplace_back(10, 12);
        sePosList.emplace_back(7, 9);
        sePosList.emplace_back(4, 6);
        sePosList.emplace_back(1, 3);

        for (int32 j = 1; j <= level; j++)
        {
            const auto& sePos = sePosList[j - 1];

            const int32 left = sePos.first;
            const int32 right = sePos.second;

            std::string nValStr = ffgStr.substr(left, right - left + 1);
            std::bitset<32> nValBitSet(nValStr);

            int32 nVal = static_cast<int32>(nValBitSet.to_ulong());
            nVal = nVal >= 0 ? nVal : 0;

            if (j == 1)
            {
                nVal = nVal > 63 ? 63 : nVal;
                if (nVal < 10)
                {
                    hCode.emplace_back("0" + std::to_string(nVal));
                }
                else
                {
                    hCode.emplace_back(std::to_string(nVal));
                }
            }
            else if (j == 2 || j >= 7)
            {
                nVal = nVal > 7 ? 7 : nVal;
            }
            else if (j == 3 || j == 6)
            {
                nVal = nVal > 1 ? 1 : nVal;
            }
            else if (j == 4 || j == 5)
            {
                nVal = nVal > 14 ? 14 : nVal;
                hCode.emplace_back(std::string({ _codeNumber[nVal] }));
            }

            if (j != 1 && j != 4 && j != 5)
            {
                hCode.emplace_back(std::to_string(nVal));
            }
        }

        return hCode;
    }

    std::string BeiDouGrid3D::ConnectBeiDouGridCode(const std::vector<std::string>& code2d, const std::vector<std::string>& codeHeight) const
    {
        std::string codeStr;

        for (size_t i = 0; i < code2d.size(); i++)
        {
            if (i >= codeHeight.size())
            {
                break;
            }

            codeStr.append(code2d[i]).append(codeHeight[i]);
        }

        return codeStr;
    }

    int32 BeiDouGrid3D::ConvertChar2Int32(const char c) const
    {
        if (std::isdigit(c))
        {
            return c - '0';
        }

        SPDLOG_CRITICAL("ERROR Char: {}", c);
        return -1;
    }

    int64 BeiDouGrid3D::IndexOfVector(const std::vector<char>& v, char value) const
    {
        const auto it = std::find(v.begin(), v.end(), value);
        return (it == v.end()) ? -1 : std::distance(v.begin(), it);
    }

    int32 BeiDouGrid3D::GetLevelByCode2D(const std::string_view& code2d) const
    {
        if (code2d.length() >= 20)
        {
            return 10;
        }
        else if (code2d.length() >= 18)
        {
            return 9;
        }
        else if (code2d.length() >= 16)
        {
            return 8;
        }
        else if (code2d.length() >= 14)
        {
            return 7;
        }
        else if (code2d.length() >= 12)
        {
            return 6;
        }
        else if (code2d.length() >= 11)
        {
            return 5;
        }
        else if (code2d.length() >= 9)
        {
            return 4;
        }
        else if (code2d.length() >= 7)
        {
            return 3;
        }
        else if (code2d.length() >= 6)
        {
            return 2;
        }
        else if (code2d.length() >= 4)
        {
            return 1;
        }

        return -1;
    }

    int32 BeiDouGrid3D::GetLevelByCode3D(const std::string_view& code3d) const
    {
        if (code3d.length() >= 32)
        {
            return 10;
        }
        else if (code3d.length() >= 29)
        {
            return 9;
        }
        else if (code3d.length() >= 26)
        {
            return 8;
        }
        else if (code3d.length() >= 23)
        {
            return 7;
        }
        else if (code3d.length() >= 20)
        {
            return 6;
        }
        else if (code3d.length() >= 18)
        {
            return 5;
        }
        else if (code3d.length() >= 15)
        {
            return 4;
        }
        else if (code3d.length() >= 12)
        {
            return 3;
        }
        else if (code3d.length() >= 10)
        {
            return 2;
        }
        else if (code3d.length() >= 7)
        {
            return 1;
        }

        return -1;
    }

#pragma endregion 私有方法

}  // namespace Grid

// end