#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <tuple>
#include <algorithm>
#include <limits>
#include <iostream>
#include <cmath>
#include <libpq-fe.h>

#include "BeiDouGrid3D.h" // 你之前提供的类（用于编码/解码/步长/高度函数）
#include <glm/glm.hpp>

namespace DronePathfinding {

    struct Point3D {
        int z, x, y;
        Point3D() : z(0), x(0), y(0) {}
        Point3D(int _z, int _x, int _y) : z(_z), x(_x), y(_y) {}
    };

    /// VoxelMap3D: 位压缩存储的三维体素地图（每个体素 1 bit）
    /// - 全局索引以“第7级北斗网格为单位”
    /// - 内部按 [z][blockX][y] 存储，blockX 表示 X 方向每 64 个为一块 (uint64_t)
    class VoxelMap3D {
    public:
        // 构造：height,width,length 指的是全局第7级格数 (Z, X, Y)
        // originGlobalX/Y/Z: 全局索引的起点（通常是扫描得到的最小 globalX/globalY/globalZ）
        VoxelMap3D(int height, int width, int length, int originGlobalZ = 0, int originGlobalX = 0, int originGlobalY = 0);

        // 访问
        bool isValidIndexGlobal(int gZ, int gX, int gY) const;     // 全局索引
        void setGlobal(int gZ, int gX, int gY, bool val);
        bool getGlobal(int gZ, int gX, int gY) const;

        // 获取尺寸与 origin
        int getHeight() const { return height_; } // Z
        int getWidth()  const { return width_; }  // X
        int getLength() const { return length_; } // Y

        int getOriginGlobalX() const { return originGlobalX_; }
        int getOriginGlobalY() const { return originGlobalY_; }
        int getOriginGlobalZ() const { return originGlobalZ_; }


        // 统计与导出
        void printStatistics() const;
        std::vector<Point3D> getObstacles(size_t limit = 0) const; // 若 limit>0 则最多返回 limit 个

        // 返回障碍物的真实地理坐标 (lon, lat, height[m])
        std::vector<glm::dvec3> getObstaclesGeo(size_t limit = 0) const;

        // 从 PostgreSQL 构建地图：会自动扫描 rows，计算所需全局范围，然后构建 map 并写入
        // 参数:
        //   conn - 已连接的 PGconn*
        //   sqlQuery - SQL 查询（必须返回 id, states, code3d），states 为 512 个 t/f（可带逗号或连续）
        //   limitRows - 若 >0 则限制读取行数（用于测试）
        // 返回：unique_ptr<VoxelMap3D>
        static std::unique_ptr<VoxelMap3D> buildFromPostgres(PGconn* conn, const std::string& sqlQuery, int limitRows = -1);

        // 根据全局第七级索引反推七级网格编码（使用 BeiDouGrid3D）
        // 返回：字符串编码（级别 7 的北斗编码）
        std::string getLevel7CodeFromGlobalIndex(int gZ, int gX, int gY) const;

        // 全局第7级索引 -> 真实地理坐标 (lon,lat,height[m])
        glm::dvec3 toGeoCoordinates(int gZ, int gX, int gY) const;

    private:
        // 内部存储布局
        int height_; // Z (number of level7 cells in Z)
        int width_;  // X (number of level7 cells in X)
        int length_; // Y (number of level7 cells in Y)
        int xBlocks_; // = ceil(width_/64.0)

        // origin: 全局索引最小值（加载时确定）
        int originGlobalX_;
        int originGlobalY_;
        int originGlobalZ_;

        // data_[z][blockX][y] -> uint64_t, bit represents X offset inside this block
        std::vector<std::vector<std::vector<uint64_t>>> data_;


    private:
        inline void ensureInRangeLocal(int lZ, int lX, int lY) const {}
        inline int toLocalX(int globalX) const { return globalX - originGlobalX_; }
        inline int toLocalY(int globalY) const { return globalY - originGlobalY_; }
        inline int toLocalZ(int globalZ) const { return globalZ - originGlobalZ_; }

        // parse states 字符串（支持 "t,t,f,..." 或 "tftftf..."）
        static std::vector<bool> parseStatesString(const char* s);

        // 将“六级北斗编码（code6）” 转换为该六级网格对应的七级网格起点全局索引 (baseZ, baseX, baseY)
        // 说明：这里使用 BeiDouGrid3D 来获取中心点、锚点与步长，然后用第7级步长把锚点映射为全局 level7 索引
        static bool beidou6ToLevel7BaseIndices(const std::string& code6, int& outBaseZ, int& outBaseX, int& outBaseY);

        // 小工具：把全局索引（gZ,gX,gY）写到 data_
        void setLocal(int lZ, int lX, int lY, bool val);

    }; // class VoxelMap3D

} // namespace DronePathfinding


// Implementation (append to same header or separate .cpp)

namespace DronePathfinding {

    VoxelMap3D::VoxelMap3D(int height, int width, int length, int originGlobalZ, int originGlobalX, int originGlobalY)
        : height_(height), width_(width), length_(length),
        originGlobalZ_(originGlobalZ), originGlobalX_(originGlobalX), originGlobalY_(originGlobalY)
    {
        if (height_ <= 0 || width_ <= 0 || length_ <= 0) {
            throw std::runtime_error("Invalid VoxelMap3D size");
        }
        xBlocks_ = static_cast<int>((width_ + 63) / 64);
        data_.resize(height_);
        for (int z = 0; z < height_; ++z) {
            data_[z].resize(xBlocks_);
            for (int bx = 0; bx < xBlocks_; ++bx) {
                data_[z][bx].resize(length_);
                std::fill(data_[z][bx].begin(), data_[z][bx].end(), 0ULL);
            }
        }
    }

    // portable popcount  (避免 __builtin_popcountll 在某些环境不可用的问题)
    static inline int popcount64(unsigned long long x) {
        int cnt = 0;
        while (x) {
            x &= (x - 1);  // 每次清掉最低位的 1
            cnt++;
        }
        return cnt;
    }

    bool VoxelMap3D::isValidIndexGlobal(int gZ, int gX, int gY) const {
        int lZ = toLocalZ(gZ), lX = toLocalX(gX), lY = toLocalY(gY);
        return lZ >= 0 && lZ < height_ && lX >= 0 && lX < width_ && lY >= 0 && lY < length_;
    }

    void VoxelMap3D::setLocal(int lZ, int lX, int lY, bool val) {
        if (lZ < 0 || lZ >= height_) return;
        if (lX < 0 || lX >= width_) return;
        if (lY < 0 || lY >= length_) return;
        int blockX = lX / 64;
        int bit = lX % 64;
        if (val) data_[lZ][blockX][lY] |= (1ULL << bit);
        else      data_[lZ][blockX][lY] &= ~(1ULL << bit);
    }

    void VoxelMap3D::setGlobal(int gZ, int gX, int gY, bool val) {
        if (!isValidIndexGlobal(gZ, gX, gY)) return;
        int lZ = toLocalZ(gZ), lX = toLocalX(gX), lY = toLocalY(gY);
        setLocal(lZ, lX, lY, val);
    }

    bool VoxelMap3D::getGlobal(int gZ, int gX, int gY) const {
        if (!isValidIndexGlobal(gZ, gX, gY)) return false;
        int lZ = toLocalZ(gZ), lX = toLocalX(gX), lY = toLocalY(gY);
        int blockX = lX / 64;
        int bit = lX % 64;
        return (data_[lZ][blockX][lY] >> bit) & 1ULL;
    }


    void VoxelMap3D::printStatistics() const {
        uint64_t cnt = 0;
        for (int z = 0; z < height_; ++z) {
            for (int bx = 0; bx < xBlocks_; ++bx) {
                for (int y = 0; y < length_; ++y) {
                    cnt += static_cast<uint64_t>(popcount64(data_[z][bx][y]));
                }
            }
        }
        std::cout << "VoxelMap3D: global origin (Z,X,Y)=("
            << originGlobalZ_ << "," << originGlobalX_ << "," << originGlobalY_ << ")\n";
        std::cout << "Dimensions (ZxXxY) = "
            << height_ << " x " << width_ << " x " << length_ << "\n";
        std::cout << "Total obstacle voxels: " << cnt << "\n";
    }

    std::vector<Point3D> VoxelMap3D::getObstacles(size_t limit) const {
        std::vector<Point3D> res;
        for (int z = 0; z < height_; ++z) {
            for (int bx = 0; bx < xBlocks_; ++bx) {
                for (int y = 0; y < length_; ++y) {
                    uint64_t val = data_[z][bx][y];
                    if (val == 0) continue;
                    for (int bit = 0; bit < 64; ++bit) {
                        int x = bx * 64 + bit;
                        if (x >= width_) break;
                        if (val & (1ULL << bit)) {
                            int gZ = originGlobalZ_ + z;
                            int gX = originGlobalX_ + x;
                            int gY = originGlobalY_ + y;
                            res.emplace_back(gZ, gX, gY);
                            if (limit > 0 && res.size() >= limit) return res;
                        }
                    }
                }
            }
        }
        return res;
    }

    std::vector<glm::dvec3> VoxelMap3D::getObstaclesGeo(size_t limit) const {
        std::vector<glm::dvec3> res;
        auto obs = getObstacles(limit == 0 ? 0 : limit); // if limit==0 get all (careful with memory)
        size_t added = 0;
        for (const auto& p : obs) {
            glm::dvec3 geo = toGeoCoordinates(p.z, p.x, p.y);
            res.push_back(geo);
            ++added;
            if (limit > 0 && added >= limit) break;
        }
        return res;
    }

    // parse "states" 字符串：支持 "t,t,f..." 或 "tttfff..."
    std::vector<bool> VoxelMap3D::parseStatesString(const char* s) {
        std::vector<bool> states;
        states.reserve(512);
        if (!s) return states;
        const char* p = s;
        // 如果有逗号，按逗号分隔；否则按字符解析
        bool hasComma = false;
        for (const char* q = p; *q; ++q) if (*q == ',') { hasComma = true; break; }
        if (hasComma) {
            std::string tmp(p);
            size_t pos = 0;
            while (pos < tmp.size() && states.size() < 512) {
                size_t next = tmp.find(',', pos);
                std::string token;
                if (next == std::string::npos) {
                    token = tmp.substr(pos);
                    pos = tmp.size();
                }
                else {
                    token = tmp.substr(pos, next - pos);
                    pos = next + 1;
                }
                // trim
                token.erase(0, token.find_first_not_of(" \t\r\n"));
                token.erase(token.find_last_not_of(" \t\r\n") + 1);
                if (!token.empty()) {
                    if (token == "t" || token == "T" || token == "1") states.push_back(true);
                    else states.push_back(false);
                }
            }
        }
        else {
            // 连续字符
            for (const char* q = p; *q && states.size() < 512; ++q) {
                char c = *q;
                if (c == 't' || c == 'T' || c == '1') states.push_back(true);
                else if (c == 'f' || c == 'F' || c == '0') states.push_back(false);
                else continue;
            }
        }
        // 不足 512 时补 false
        while (states.size() < 512) states.push_back(false);
        if (states.size() > 512) states.resize(512);
        return states;
    }

    /// 将六级网格 code6 映射为其第7级格子的起始全局索引 (baseZ, baseX, baseY)
    /// 说明与假设：
    ///   - 使用 BeiDouGrid3D 计算六级网格的中心和“锚点角（anchor corner）”
    ///   - 以第7级步长（经度/纬度）为单位把锚点量化成全局第7级索引（origin: lon=-180, lat=-88）
    ///   - 垂直方向：以第7级高度格为单位把中心高度映射为高度索引，取 centerIndex - 4 作为起始（使得 8 层以 center 居中）
    ///   - 这是一个工程实用的映射方式（对绝大多数场景能正确把 512 子格放入一个连续的 8×8×8 块）
    /// 返回 true 表示成功（生成了 base 索引）
    bool VoxelMap3D::beidou6ToLevel7BaseIndices(const std::string& code6, int& outBaseZ, int& outBaseX, int& outBaseY) {
        using namespace Grid;
        BeiDouGrid3D bd;
        // **我们假定数据库里的 code6 确实是 6 级码**，为了避免访问私有函数，直接设定 level = 6
        const int level = 6;

        // 取六级网格中心（三维）
        glm::dvec3 center = bd.GetGridCenter(code6);
        if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z)) return false;

        // 计算父（level）网格的左下角锚点（角点），使用静态 CorrectionCoordinates
        double anchorLon = 0.0, anchorLat = 0.0;
        BeiDouGrid3D::CorrectionCoordinates(center.x, center.y, level, anchorLon, anchorLat, true /*use anchor corner*/);

        // 第7级步长
        const int childLevel = 7;
        double step7_lon = bd.GetStepLon(childLevel); // 度
        double step7_lat = bd.GetStepLat(childLevel); // 度

        // origin (与 BeiDouGrid3D 中一致)
        const double originLon = -180.0;
        const double originLat = -88.0;

        // 量化 anchor 得到 baseX / baseY（第7级全局索引）
        double dx = (anchorLon - originLon) / step7_lon;
        double dy = (anchorLat - originLat) / step7_lat;
        // floor 更安全（锚点应当是格线）
        int baseX = static_cast<int>(std::floor(dx + 1e-9));
        int baseY = static_cast<int>(std::floor(dy + 1e-9));

        // 垂直方向：使用第7级高度步长与 GetBeidouGridHeightIndex
        double deltaDegChild = bd.GetStepLat(childLevel); // 对于高度计算，level!=3 的时候用纬度步长（与类中实现一致）
        // 将中心高度映射到第7级高度索引（这是一个近似但可用的方法）
        int centerHZIndex = static_cast<int>(bd.GetBeidouGridHeightIndex(deltaDegChild, center.z));
        // 以 center 居中分配 8 层：起始索引 = center - 4
        int baseZ = centerHZIndex - 4;
        if (baseZ < 0) baseZ = 0;

        outBaseX = baseX;
        outBaseY = baseY;
        outBaseZ = baseZ;
        return true;
    }

    // buildFromPostgres：两次扫描法
    std::unique_ptr<VoxelMap3D> VoxelMap3D::buildFromPostgres(PGconn* conn, const std::string& sqlQuery, int limitRows) {
        if (!conn) throw std::runtime_error("Null PGconn");

        // 执行查询
        PGresult* res = PQexec(conn, sqlQuery.c_str());
        if (PQresultStatus(res) != PGRES_TUPLES_OK) {
            std::string err = PQerrorMessage(conn);
            PQclear(res);
            throw std::runtime_error("Postgres query failed: " + err);
        }

        int rows = PQntuples(res);
        if (limitRows > 0 && limitRows < rows) rows = limitRows;

        struct RowEntry {
            std::string code6;
            std::vector<bool> states; // length 512
            int baseX, baseY, baseZ;
        };
        std::vector<RowEntry> entries;
        entries.reserve(rows);

        int globalMinX = std::numeric_limits<int>::max();
        int globalMinY = std::numeric_limits<int>::max();
        int globalMinZ = std::numeric_limits<int>::max();
        int globalMaxX = std::numeric_limits<int>::min();
        int globalMaxY = std::numeric_limits<int>::min();
        int globalMaxZ = std::numeric_limits<int>::min();

        // 第一遍：解析 states、计算 base 索引，收集 min/max（注意：base + 7 是 block 的末端）
        for (int i = 0; i < rows; ++i) {
            const char* code6_c = PQgetvalue(res, i, 2); // SELECT id, states, code3d
            const char* states_c = PQgetvalue(res, i, 1);
            if (!code6_c || !states_c) continue;

            RowEntry e;
            e.code6 = code6_c;
            e.states = parseStatesString(states_c); // 保证长度 512

            // 计算 base indices
            int baseZ, baseX, baseY;
            if (!beidou6ToLevel7BaseIndices(e.code6, baseZ, baseX, baseY)) {
                // 如果转换失败，跳过（也可以记录错误）
                std::cerr << "Warning: cannot map code6 to indices: " << e.code6 << "\n";
                continue;
            }
            e.baseX = baseX;
            e.baseY = baseY;
            e.baseZ = baseZ;
            entries.push_back(std::move(e));

            // 更新 min/max（每个六级块覆盖 base..base+7）
            globalMinX = std::min(globalMinX, baseX);
            globalMinY = std::min(globalMinY, baseY);
            globalMinZ = std::min(globalMinZ, baseZ);
            globalMaxX = std::max(globalMaxX, baseX + 7);
            globalMaxY = std::max(globalMaxY, baseY + 7);
            globalMaxZ = std::max(globalMaxZ, baseZ + 7);
        }

        if (entries.empty()) {
            PQclear(res);
            throw std::runtime_error("No valid rows parsed from DB");
        }

        // 计算地图大小（全局索引范围）
        int globalWidth = globalMaxX - globalMinX + 1;
        int globalLength = globalMaxY - globalMinY + 1;
        int globalHeight = globalMaxZ - globalMinZ + 1;

        // 为 X 方向对齐到 64 的倍数（可选，但便于内部块对齐）
        int padX = (64 - (globalWidth % 64)) % 64;
        globalWidth += padX;

        // 创建 Map（originGlobal = globalMin）
        auto map = std::make_unique<VoxelMap3D>(globalHeight, globalWidth, globalLength,
            globalMinZ, globalMinX, globalMinY);

        // 第二遍：写入数据到 map（按每条的 base 索引展开 8x8x8）
        for (const RowEntry& e : entries) {
            int baseX = e.baseX;
            int baseY = e.baseY;
            int baseZ = e.baseZ;
            // states 索引映射规则（与我们之前讨论一致）：
            // idx: 0..511
            // dz = idx / 64
            // dx = (idx % 64) / 8
            // dy = idx % 8
            for (int idx = 0; idx < 512; ++idx) {
                int dz = idx / 64;
                int rem = idx % 64;
                int dx = rem / 8;
                int dy = rem % 8;
                int gZ = baseZ + dz;
                int gX = baseX + dx;
                int gY = baseY + dy;
                bool isObstacle = e.states[idx];
                // bounds check (理论上不会越界，因为我们用 min/max 扩展过)
                if (gZ < map->originGlobalZ_ || gX < map->originGlobalX_ || gY < map->originGlobalY_) continue;
                if (gZ > map->originGlobalZ_ + map->height_ - 1) continue;
                if (gX > map->originGlobalX_ + map->width_ - 1) continue;
                if (gY > map->originGlobalY_ + map->length_ - 1) continue;
                map->setGlobal(gZ, gX, gY, isObstacle);
            }
        }

        PQclear(res);
        return map;
    }

    // 反推出第7级网格编码（使用 BeiDouGrid3D）
    std::string VoxelMap3D::getLevel7CodeFromGlobalIndex(int gZ, int gX, int gY) const {
        using namespace Grid;
        BeiDouGrid3D bd;
        // 第7級
        const int level = 7;
        double step_lon = bd.GetStepLon(level);
        double step_lat = bd.GetStepLat(level);
        // origin
        const double originLon = -180.0;
        const double originLat = -88.0;

        // 计算该第7级格中心经纬
        double lon = originLon + (static_cast<double>(gX) + 0.5) * step_lon;
        double lat = originLat + (static_cast<double>(gY) + 0.5) * step_lat;

        // 计算高度中心：
        double deltaDeg = bd.GetStepLat(level);
        double bottomH = bd.GetBeiDouGridHeight(deltaDeg, gZ);
        double hStep = bd.GetBeiDouGridZorYHeight(deltaDeg, gZ);
        double centerH = bottomH + 0.5 * hStep;

        glm::dvec3 center(lon, lat, centerH);
        return bd.BuildGridCode3D(center, level);
    }

    // 全局第7级索引 -> 真实地理坐标 (lon,lat,height[m])
    glm::dvec3 VoxelMap3D::toGeoCoordinates(int gZ, int gX, int gY) const {
        using namespace Grid;
        BeiDouGrid3D bd;
        const int level = 7;
        double stepLon = bd.GetStepLon(level);
        double stepLat = bd.GetStepLat(level);
        const double originLon = -180.0;
        const double originLat = -88.0;

        double lon = originLon + (static_cast<double>(gX) + 0.5) * stepLon;
        double lat = originLat + (static_cast<double>(gY) + 0.5) * stepLat;

        // 高度：用 gZ 反推实际高度（米）
        double deltaDeg = bd.GetStepLat(level);
        double bottomH = bd.GetBeiDouGridHeight(deltaDeg, gZ);
        double hStep = bd.GetBeiDouGridZorYHeight(deltaDeg, gZ);
        double height = bottomH + 0.5 * hStep;

        return glm::dvec3(lon, lat, height);
    }

} // namespace DronePathfinding
