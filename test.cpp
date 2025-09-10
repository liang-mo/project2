#include <iostream>
#include <string>
#include <libpq-fe.h>
#include "VoxelMap3D.h"

using namespace DronePathfinding;

int main() {
    // 1. 数据库连接
    const char* conninfo = "host=10.12.201.6 port=5432 dbname=postgres user=postgres password=postgres";
    PGconn* conn = PQconnectdb(conninfo);

    if (PQstatus(conn) != CONNECTION_OK) {
        std::cerr << "数据库连接失败: " << PQerrorMessage(conn) << std::endl;
        if (conn) PQfinish(conn);
        return -1;
    }
    std::cout << "数据库连接成功\n";

    // 2. SQL 查询
    std::string sql =
        "SELECT id, states, code3d "
        "FROM public.shaoxin_l7a6_fly_grid "
        "ORDER BY id ASC LIMIT 100";

    // 3. 构建 VoxelMap3D
    auto voxelMap = VoxelMap3D::buildFromPostgres(conn, sql);

    if (!voxelMap) {
        std::cerr << "❌ 构建 VoxelMap3D 失败" << std::endl;
        PQfinish(conn);
        return -1;
    }

    // 4. 打印统计信息
    voxelMap->printStatistics();

    // 5. 获取部分障碍体素
    auto obstacles = voxelMap->getObstaclesGeo(10);
    std::cout << "\n=== 障碍物示例 (最多 10 个) ===\n";
    for (size_t i = 0; i < obstacles.size(); i++) {
        auto& p = obstacles[i];
        std::cout << i + 1 << ": Lon=" << p.x
            << ", Lat=" << p.y
            << ", Height=" << p.z << "m\n";
    }

    PQfinish(conn);
    return 0;
}
