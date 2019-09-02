/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.detour.crowd;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class ProximityGrid {

    private static class ItemKey {

        int x, y;

        public ItemKey(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public int hashCode() {
            final int prime = 31;
            int result = 1;
            result = prime * result + x;
            result = prime * result + y;
            return result;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            ItemKey other = (ItemKey) obj;
            if (x != other.x)
                return false;
            if (y != other.y)
                return false;
            return true;
        }

    };

    private final float m_cellSize;
    private final float m_invCellSize;
    private final Map<ItemKey, List<Integer>> items;
    int[] m_bounds = new int[4];

    public ProximityGrid(float m_cellSize) {
        this.m_cellSize = m_cellSize;
        m_invCellSize = 1.0f / m_cellSize;
        items = new HashMap<>();
    }

    void clear() {
        items.clear();
        m_bounds[0] = 0xffff;
        m_bounds[1] = 0xffff;
        m_bounds[2] = -0xffff;
        m_bounds[3] = -0xffff;
    }

    void addItem(int id, float minx, float miny, float maxx, float maxy) {
        int iminx = (int) Math.floor(minx * m_invCellSize);
        int iminy = (int) Math.floor(miny * m_invCellSize);
        int imaxx = (int) Math.floor(maxx * m_invCellSize);
        int imaxy = (int) Math.floor(maxy * m_invCellSize);

        m_bounds[0] = Math.min(m_bounds[0], iminx);
        m_bounds[1] = Math.min(m_bounds[1], iminy);
        m_bounds[2] = Math.min(m_bounds[2], imaxx);
        m_bounds[3] = Math.min(m_bounds[3], imaxy);

        for (int y = iminy; y <= imaxy; ++y) {
            for (int x = iminx; x <= imaxx; ++x) {
                ItemKey key = new ItemKey(x, y);
                List<Integer> ids = items.get(key);
                if (ids == null) {
                    ids = new ArrayList<>();
                    items.put(key, ids);
                }
                ids.add(id);
            }
        }
    }

    Set<Integer> queryItems(float minx, float miny, float maxx, float maxy) {
        int iminx = (int) Math.floor(minx * m_invCellSize);
        int iminy = (int) Math.floor(miny * m_invCellSize);
        int imaxx = (int) Math.floor(maxx * m_invCellSize);
        int imaxy = (int) Math.floor(maxy * m_invCellSize);

        Set<Integer> result = new HashSet<>();
        for (int y = iminy; y <= imaxy; ++y) {
            for (int x = iminx; x <= imaxx; ++x) {
                ItemKey key = new ItemKey(x, y);
                List<Integer> ids = items.get(key);
                if (ids != null) {
                    result.addAll(ids);
                }
            }
        }

        return result;
    }

    public int getItemCountAt(int x, int y) {
        ItemKey key = new ItemKey(x, y);
        List<Integer> ids = items.get(key);
        return ids != null ? ids.size() : 0;
    }

    public int[] getBounds() {
        return m_bounds;
    }

    public float getCellSize() {
        return m_cellSize;
    }
}
