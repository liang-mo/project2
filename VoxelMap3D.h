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

#include "BeiDouGrid3D.h" // ��֮ǰ�ṩ���ࣨ���ڱ���/����/����/�߶Ⱥ�����
#include <glm/glm.hpp>

namespace DronePathfinding {

    struct Point3D {
        int z, x, y;
        Point3D() : z(0), x(0), y(0) {}
        Point3D(int _z, int _x, int _y) : z(_z), x(_x), y(_y) {}
    };

    /// VoxelMap3D: λѹ���洢����ά���ص�ͼ��ÿ������ 1 bit��
    /// - ȫ�������ԡ���7����������Ϊ��λ��
    /// - �ڲ��� [z][blockX][y] �洢��blockX ��ʾ X ����ÿ 64 ��Ϊһ�� (uint64_t)
    class VoxelMap3D {
    public:
        // ���죺height,width,length ָ����ȫ�ֵ�7������ (Z, X, Y)
        // originGlobalX/Y/Z: ȫ����������㣨ͨ����ɨ��õ�����С globalX/globalY/globalZ��
        VoxelMap3D(int height, int width, int length, int originGlobalZ = 0, int originGlobalX = 0, int originGlobalY = 0);

        // ����
        bool isValidIndexGlobal(int gZ, int gX, int gY) const;     // ȫ������
        void setGlobal(int gZ, int gX, int gY, bool val);
        bool getGlobal(int gZ, int gX, int gY) const;

        // ��ȡ�ߴ��� origin
        int getHeight() const { return height_; } // Z
        int getWidth()  const { return width_; }  // X
        int getLength() const { return length_; } // Y

        int getOriginGlobalX() const { return originGlobalX_; }
        int getOriginGlobalY() const { return originGlobalY_; }
        int getOriginGlobalZ() const { return originGlobalZ_; }


        // ͳ���뵼��
        void printStatistics() const;
        std::vector<Point3D> getObstacles(size_t limit = 0) const; // �� limit>0 ����෵�� limit ��

        // �����ϰ������ʵ�������� (lon, lat, height[m])
        std::vector<glm::dvec3> getObstaclesGeo(size_t limit = 0) const;

        // �� PostgreSQL ������ͼ�����Զ�ɨ�� rows����������ȫ�ַ�Χ��Ȼ�󹹽� map ��д��
        // ����:
        //   conn - �����ӵ� PGconn*
        //   sqlQuery - SQL ��ѯ�����뷵�� id, states, code3d����states Ϊ 512 �� t/f���ɴ����Ż�������
        //   limitRows - �� >0 �����ƶ�ȡ���������ڲ��ԣ�
        // ���أ�unique_ptr<VoxelMap3D>
        static std::unique_ptr<VoxelMap3D> buildFromPostgres(PGconn* conn, const std::string& sqlQuery, int limitRows = -1);

        // ����ȫ�ֵ��߼����������߼�������루ʹ�� BeiDouGrid3D��
        // ���أ��ַ������루���� 7 �ı������룩
        std::string getLevel7CodeFromGlobalIndex(int gZ, int gX, int gY) const;

        // ȫ�ֵ�7������ -> ��ʵ�������� (lon,lat,height[m])
        glm::dvec3 toGeoCoordinates(int gZ, int gX, int gY) const;

    private:
        // �ڲ��洢����
        int height_; // Z (number of level7 cells in Z)
        int width_;  // X (number of level7 cells in X)
        int length_; // Y (number of level7 cells in Y)
        int xBlocks_; // = ceil(width_/64.0)

        // origin: ȫ��������Сֵ������ʱȷ����
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

        // parse states �ַ�����֧�� "t,t,f,..." �� "tftftf..."��
        static std::vector<bool> parseStatesString(const char* s);

        // ���������������루code6���� ת��Ϊ�����������Ӧ���߼��������ȫ������ (baseZ, baseX, baseY)
        // ˵��������ʹ�� BeiDouGrid3D ����ȡ���ĵ㡢ê���벽����Ȼ���õ�7��������ê��ӳ��Ϊȫ�� level7 ����
        static bool beidou6ToLevel7BaseIndices(const std::string& code6, int& outBaseZ, int& outBaseX, int& outBaseY);

        // С���ߣ���ȫ��������gZ,gX,gY��д�� data_
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

    // portable popcount  (���� __builtin_popcountll ��ĳЩ���������õ�����)
    static inline int popcount64(unsigned long long x) {
        int cnt = 0;
        while (x) {
            x &= (x - 1);  // ÿ��������λ�� 1
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

    // parse "states" �ַ�����֧�� "t,t,f..." �� "tttfff..."
    std::vector<bool> VoxelMap3D::parseStatesString(const char* s) {
        std::vector<bool> states;
        states.reserve(512);
        if (!s) return states;
        const char* p = s;
        // ����ж��ţ������ŷָ��������ַ�����
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
            // �����ַ�
            for (const char* q = p; *q && states.size() < 512; ++q) {
                char c = *q;
                if (c == 't' || c == 'T' || c == '1') states.push_back(true);
                else if (c == 'f' || c == 'F' || c == '0') states.push_back(false);
                else continue;
            }
        }
        // ���� 512 ʱ�� false
        while (states.size() < 512) states.push_back(false);
        if (states.size() > 512) states.resize(512);
        return states;
    }

    /// ���������� code6 ӳ��Ϊ���7�����ӵ���ʼȫ������ (baseZ, baseX, baseY)
    /// ˵������裺
    ///   - ʹ�� BeiDouGrid3D ����������������ĺ͡�ê��ǣ�anchor corner����
    ///   - �Ե�7������������/γ�ȣ�Ϊ��λ��ê��������ȫ�ֵ�7��������origin: lon=-180, lat=-88��
    ///   - ��ֱ�����Ե�7���߶ȸ�Ϊ��λ�����ĸ߶�ӳ��Ϊ�߶�������ȡ centerIndex - 4 ��Ϊ��ʼ��ʹ�� 8 ���� center ���У�
    ///   - ����һ������ʵ�õ�ӳ�䷽ʽ���Ծ��������������ȷ�� 512 �Ӹ����һ�������� 8��8��8 �飩
    /// ���� true ��ʾ�ɹ��������� base ������
    bool VoxelMap3D::beidou6ToLevel7BaseIndices(const std::string& code6, int& outBaseZ, int& outBaseX, int& outBaseY) {
        using namespace Grid;
        BeiDouGrid3D bd;
        // **���Ǽٶ����ݿ���� code6 ȷʵ�� 6 ����**��Ϊ�˱������˽�к�����ֱ���趨 level = 6
        const int level = 6;

        // ȡ�����������ģ���ά��
        glm::dvec3 center = bd.GetGridCenter(code6);
        if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z)) return false;

        // ���㸸��level����������½�ê�㣨�ǵ㣩��ʹ�þ�̬ CorrectionCoordinates
        double anchorLon = 0.0, anchorLat = 0.0;
        BeiDouGrid3D::CorrectionCoordinates(center.x, center.y, level, anchorLon, anchorLat, true /*use anchor corner*/);

        // ��7������
        const int childLevel = 7;
        double step7_lon = bd.GetStepLon(childLevel); // ��
        double step7_lat = bd.GetStepLat(childLevel); // ��

        // origin (�� BeiDouGrid3D ��һ��)
        const double originLon = -180.0;
        const double originLat = -88.0;

        // ���� anchor �õ� baseX / baseY����7��ȫ��������
        double dx = (anchorLon - originLon) / step7_lon;
        double dy = (anchorLat - originLat) / step7_lat;
        // floor ����ȫ��ê��Ӧ���Ǹ��ߣ�
        int baseX = static_cast<int>(std::floor(dx + 1e-9));
        int baseY = static_cast<int>(std::floor(dy + 1e-9));

        // ��ֱ����ʹ�õ�7���߶Ȳ����� GetBeidouGridHeightIndex
        double deltaDegChild = bd.GetStepLat(childLevel); // ���ڸ߶ȼ��㣬level!=3 ��ʱ����γ�Ȳ�����������ʵ��һ�£�
        // �����ĸ߶�ӳ�䵽��7���߶�����������һ�����Ƶ����õķ�����
        int centerHZIndex = static_cast<int>(bd.GetBeidouGridHeightIndex(deltaDegChild, center.z));
        // �� center ���з��� 8 �㣺��ʼ���� = center - 4
        int baseZ = centerHZIndex - 4;
        if (baseZ < 0) baseZ = 0;

        outBaseX = baseX;
        outBaseY = baseY;
        outBaseZ = baseZ;
        return true;
    }

    // buildFromPostgres������ɨ�跨
    std::unique_ptr<VoxelMap3D> VoxelMap3D::buildFromPostgres(PGconn* conn, const std::string& sqlQuery, int limitRows) {
        if (!conn) throw std::runtime_error("Null PGconn");

        // ִ�в�ѯ
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

        // ��һ�飺���� states������ base �������ռ� min/max��ע�⣺base + 7 �� block ��ĩ�ˣ�
        for (int i = 0; i < rows; ++i) {
            const char* code6_c = PQgetvalue(res, i, 2); // SELECT id, states, code3d
            const char* states_c = PQgetvalue(res, i, 1);
            if (!code6_c || !states_c) continue;

            RowEntry e;
            e.code6 = code6_c;
            e.states = parseStatesString(states_c); // ��֤���� 512

            // ���� base indices
            int baseZ, baseX, baseY;
            if (!beidou6ToLevel7BaseIndices(e.code6, baseZ, baseX, baseY)) {
                // ���ת��ʧ�ܣ�������Ҳ���Լ�¼����
                std::cerr << "Warning: cannot map code6 to indices: " << e.code6 << "\n";
                continue;
            }
            e.baseX = baseX;
            e.baseY = baseY;
            e.baseZ = baseZ;
            entries.push_back(std::move(e));

            // ���� min/max��ÿ�������鸲�� base..base+7��
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

        // �����ͼ��С��ȫ��������Χ��
        int globalWidth = globalMaxX - globalMinX + 1;
        int globalLength = globalMaxY - globalMinY + 1;
        int globalHeight = globalMaxZ - globalMinZ + 1;

        // Ϊ X ������뵽 64 �ı�������ѡ���������ڲ�����룩
        int padX = (64 - (globalWidth % 64)) % 64;
        globalWidth += padX;

        // ���� Map��originGlobal = globalMin��
        auto map = std::make_unique<VoxelMap3D>(globalHeight, globalWidth, globalLength,
            globalMinZ, globalMinX, globalMinY);

        // �ڶ��飺д�����ݵ� map����ÿ���� base ����չ�� 8x8x8��
        for (const RowEntry& e : entries) {
            int baseX = e.baseX;
            int baseY = e.baseY;
            int baseZ = e.baseZ;
            // states ����ӳ�����������֮ǰ����һ�£���
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
                // bounds check (�����ϲ���Խ�磬��Ϊ������ min/max ��չ��)
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

    // ���Ƴ���7��������루ʹ�� BeiDouGrid3D��
    std::string VoxelMap3D::getLevel7CodeFromGlobalIndex(int gZ, int gX, int gY) const {
        using namespace Grid;
        BeiDouGrid3D bd;
        // ��7��
        const int level = 7;
        double step_lon = bd.GetStepLon(level);
        double step_lat = bd.GetStepLat(level);
        // origin
        const double originLon = -180.0;
        const double originLat = -88.0;

        // ����õ�7�������ľ�γ
        double lon = originLon + (static_cast<double>(gX) + 0.5) * step_lon;
        double lat = originLat + (static_cast<double>(gY) + 0.5) * step_lat;

        // ����߶����ģ�
        double deltaDeg = bd.GetStepLat(level);
        double bottomH = bd.GetBeiDouGridHeight(deltaDeg, gZ);
        double hStep = bd.GetBeiDouGridZorYHeight(deltaDeg, gZ);
        double centerH = bottomH + 0.5 * hStep;

        glm::dvec3 center(lon, lat, centerH);
        return bd.BuildGridCode3D(center, level);
    }

    // ȫ�ֵ�7������ -> ��ʵ�������� (lon,lat,height[m])
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

        // �߶ȣ��� gZ ����ʵ�ʸ߶ȣ��ף�
        double deltaDeg = bd.GetStepLat(level);
        double bottomH = bd.GetBeiDouGridHeight(deltaDeg, gZ);
        double hStep = bd.GetBeiDouGridZorYHeight(deltaDeg, gZ);
        double height = bottomH + 0.5 * hStep;

        return glm::dvec3(lon, lat, height);
    }

} // namespace DronePathfinding
