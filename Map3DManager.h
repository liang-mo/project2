#pragma once
// Map3DManager.h
#pragma once
#include <memory>
#include <mutex>
#include "map3d.h"

using namespace DronePathfinding;

class Map3DManager {
public:
	static Map3DManager& getInstance();

	void loadFromDatabase(PGconn* conn, const std::string& query);
	std::shared_ptr<Map3D> getMap();

private:
	Map3DManager() = default;
	std::shared_ptr<Map3D> map;
	std::mutex map_mutex;
};

