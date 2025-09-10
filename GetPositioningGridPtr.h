#pragma once

#ifndef GET_POSITIONING_GRID_PTR_H_
#define GET_POSITIONING_GRID_PTR_H_

#include "BeiDouGrid3D.h"
#include "PositioningGrid.h"
#include <memory>

namespace Grid
{
    static std::unique_ptr<PositioningGrid> GetPositioningGridByType(const GridType& type)
    {
        std::unique_ptr<PositioningGrid> ptr = nullptr;

        switch (type)
        {
        case GridType::kBeiDouGrid:
            ptr = std::make_unique<BeiDouGrid3D>();
            break;
        case GridType::kGeoGrid:
            break;
        }

        return std::move(ptr);
    }
}  // namespace Grid

#endif

// end of GetPositioningGridPtr.h