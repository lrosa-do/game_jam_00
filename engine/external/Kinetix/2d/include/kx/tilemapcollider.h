#pragma once

#include "body.h"
#include "world.h"

#include <ct/vector.hpp>

namespace kx
{

    class TileMapCollider
    {
    public:
        explicit TileMapCollider(World &world);
        ~TileMapCollider();

        TileMapCollider(const TileMapCollider &) = delete;
        TileMapCollider &operator=(const TileMapCollider &) = delete;

        void SetMapSize(int columns, int rows);
        void SetCellSize(const vec2 &size);
        void SetOffset(const vec2 &offset);
        void SetSolid(int x, int y, bool solid);
        bool IsSolid(int x, int y) const;
        void Clear();
        void Rebuild();

        int Columns() const { return mColumns; }
        int Rows() const { return mRows; }
        const vec2 &CellSize() const { return mCellSize; }
        const ct::Vector<Body *> &Bodies() const { return mBodies; }

    private:
        World &mWorld;
        int mColumns;
        int mRows;
        vec2 mCellSize;
        vec2 mOffset;
        ct::Vector<unsigned char> mSolid;
        ct::Vector<Body *> mBodies;
    };

}