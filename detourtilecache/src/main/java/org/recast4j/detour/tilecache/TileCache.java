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
package org.recast4j.detour.tilecache;

import static org.recast4j.detour.DetourCommon.ilog2;
import static org.recast4j.detour.DetourCommon.nextPow2;
import static org.recast4j.detour.DetourCommon.overlapBounds;
import static org.recast4j.detour.DetourCommon.vCopy;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import org.recast4j.detour.MeshData;
import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshBuilder;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.detour.tilecache.TileCacheObstacle.TileCacheObstacleType;
import org.recast4j.detour.tilecache.io.TileCacheLayerHeaderReader;

public class TileCache {

    int m_tileLutSize; /// < Tile hash lookup size (must be pot).
    int m_tileLutMask; /// < Tile hash lookup mask.

    private final CompressedTile[] m_posLookup; /// < Tile hash lookup.
    private CompressedTile m_nextFreeTile; /// < Freelist of tiles.
    private final CompressedTile[] m_tiles; /// < List of tiles. // TODO: (PP) replace with list

    private final int m_saltBits; /// < Number of salt bits in the tile ID.
    private final int m_tileBits; /// < Number of tile bits in the tile ID.

    private final NavMesh m_navmesh;
    private final TileCacheParams m_params;
    private final TileCacheStorageParams m_storageParams;

    private final TileCacheCompressor m_tcomp;
    private final TileCacheMeshProcess m_tmproc;

    private final List<TileCacheObstacle> m_obstacles = new ArrayList<>();
    private TileCacheObstacle m_nextFreeObstacle;

    private final List<ObstacleRequest> m_reqs = new ArrayList<>();
    private final List<Long> m_update = new ArrayList<>();

    private final TileCacheBuilder builder = new TileCacheBuilder();
    private final TileCacheLayerHeaderReader tileReader = new TileCacheLayerHeaderReader();

    private boolean contains(List<Long> a, long v) {
        return a.contains(v);
    }

    /// Encodes a tile id.
    private long encodeTileId(int salt, int it) {
        return ((long) salt << m_tileBits) | it;
    }

    /// Decodes a tile salt.
    private int decodeTileIdSalt(long ref) {
        long saltMask = (1L << m_saltBits) - 1;
        return (int) ((ref >> m_tileBits) & saltMask);
    }

    /// Decodes a tile id.
    private int decodeTileIdTile(long ref) {
        long tileMask = (1L << m_tileBits) - 1;
        return (int) (ref & tileMask);
    }

    /// Encodes an obstacle id.
    private long encodeObstacleId(int salt, int it) {
        return ((long) salt << 16) | it;
    }

    /// Decodes an obstacle salt.
    private int decodeObstacleIdSalt(long ref) {
        long saltMask = ((long) 1 << 16) - 1;
        return (int) ((ref >> 16) & saltMask);
    }

    /// Decodes an obstacle id.
    private int decodeObstacleIdObstacle(long ref) {
        long tileMask = ((long) 1 << 16) - 1;
        return (int) (ref & tileMask);
    }

    public TileCache(TileCacheParams params, TileCacheStorageParams storageParams, NavMesh navmesh,
            TileCacheCompressor tcomp, TileCacheMeshProcess tmprocs) {
        m_params = params;
        m_storageParams = storageParams;
        m_navmesh = navmesh;
        m_tcomp = tcomp;
        m_tmproc = tmprocs;

        m_tileLutSize = nextPow2(m_params.maxTiles / 4);
        if (m_tileLutSize == 0) {
            m_tileLutSize = 1;
        }
        m_tileLutMask = m_tileLutSize - 1;
        m_tiles = new CompressedTile[m_params.maxTiles];
        m_posLookup = new CompressedTile[m_tileLutSize];
        for (int i = m_params.maxTiles - 1; i >= 0; --i) {
            m_tiles[i] = new CompressedTile(i);
            m_tiles[i].next = m_nextFreeTile;
            m_nextFreeTile = m_tiles[i];
        }
        m_tileBits = ilog2(nextPow2(m_params.maxTiles));
        m_saltBits = Math.min(31, 32 - m_tileBits);
        if (m_saltBits < 10) {
            throw new RuntimeException("Too few salt bits: " + m_saltBits);
        }
    }

    public CompressedTile getTileByRef(long ref) {
        if (ref == 0) {
            return null;
        }
        int tileIndex = decodeTileIdTile(ref);
        int tileSalt = decodeTileIdSalt(ref);
        if (tileIndex >= m_params.maxTiles) {
            return null;
        }
        CompressedTile tile = m_tiles[tileIndex];
        if (tile.salt != tileSalt) {
            return null;
        }
        return tile;
    }

    public List<Long> getTilesAt(int tx, int ty) {
        List<Long> tiles = new ArrayList<>();

        // Find tile based on hash.
        int h = NavMesh.computeTileHash(tx, ty, m_tileLutMask);
        CompressedTile tile = m_posLookup[h];
        while (tile != null) {
            if (tile.header != null && tile.header.tx == tx && tile.header.ty == ty) {
                tiles.add(getTileRef(tile));
            }
            tile = tile.next;
        }

        return tiles;
    }

    CompressedTile getTileAt(int tx, int ty, int tlayer) {
        // Find tile based on hash.
        int h = NavMesh.computeTileHash(tx, ty, m_tileLutMask);
        CompressedTile tile = m_posLookup[h];
        while (tile != null) {
            if (tile.header != null && tile.header.tx == tx && tile.header.ty == ty && tile.header.tlayer == tlayer) {
                return tile;
            }
            tile = tile.next;
        }
        return null;
    }

    public long getTileRef(CompressedTile tile) {
        if (tile == null) {
            return 0;
        }
        int it = tile.index;
        return encodeTileId(tile.salt, it);
    }

    public long getObstacleRef(TileCacheObstacle ob) {
        if (ob == null) {
            return 0;
        }
        int idx = ob.index;
        return encodeObstacleId(ob.salt, idx);
    }

    public TileCacheObstacle getObstacleByRef(long ref) {
        if (ref == 0) {
            return null;
        }
        int idx = decodeObstacleIdObstacle(ref);
        if (idx >= m_obstacles.size()) {
            return null;
        }
        TileCacheObstacle ob = m_obstacles.get(idx);
        int salt = decodeObstacleIdSalt(ref);
        if (ob.salt != salt) {
            return null;
        }
        return ob;
    }

    public long addTile(byte[] data, int flags) throws IOException {
        // Make sure the data is in right format.
        ByteBuffer buf = ByteBuffer.wrap(data);
        buf.order(m_storageParams.byteOrder);
        TileCacheLayerHeader header = tileReader.read(buf, m_storageParams.cCompatibility);
        // Make sure the location is free.
        if (getTileAt(header.tx, header.ty, header.tlayer) != null) {
            return 0;
        }
        // Allocate a tile.
        CompressedTile tile = null;
        if (m_nextFreeTile != null) {
            tile = m_nextFreeTile;
            m_nextFreeTile = tile.next;
            tile.next = null;
        }

        // Make sure we could allocate a tile.
        if (tile == null) {
            throw new RuntimeException("Out of storage");
        }

        // Insert tile into the position lut.
        int h = NavMesh.computeTileHash(header.tx, header.ty, m_tileLutMask);
        tile.next = m_posLookup[h];
        m_posLookup[h] = tile;

        // Init tile.
        tile.header = header;
        tile.data = data;
        tile.compressed = align4(buf.position());
        tile.flags = flags;

        return getTileRef(tile);
    }

    private int align4(int i) {
        return (i + 3) & (~3);
    }

    public void removeTile(long ref) {
        if (ref == 0) {
            throw new RuntimeException("Invalid tile ref");
        }
        int tileIndex = decodeTileIdTile(ref);
        int tileSalt = decodeTileIdSalt(ref);
        if (tileIndex >= m_params.maxTiles) {
            throw new RuntimeException("Invalid tile index");
        }
        CompressedTile tile = m_tiles[tileIndex];
        if (tile.salt != tileSalt) {
            throw new RuntimeException("Invalid tile salt");
        }

        // Remove tile from hash lookup.
        int h = NavMesh.computeTileHash(tile.header.tx, tile.header.ty, m_tileLutMask);
        CompressedTile prev = null;
        CompressedTile cur = m_posLookup[h];
        while (cur != null) {
            if (cur == tile) {
                if (prev != null) {
                    prev.next = cur.next;
                } else {
                    m_posLookup[h] = cur.next;
                }
                break;
            }
            prev = cur;
            cur = cur.next;
        }

        tile.header = null;
        tile.data = null;
        tile.compressed = 0;
        tile.flags = 0;

        // Update salt, salt should never be zero.
        tile.salt = (tile.salt + 1) & ((1 << m_saltBits) - 1);
        if (tile.salt == 0) {
            tile.salt++;
        }

        // Add to free list.
        tile.next = m_nextFreeTile;
        m_nextFreeTile = tile;

    }

    // Cylinder obstacle
    public long addObstacle(float[] pos, float radius, float height) {
        TileCacheObstacle ob = allocObstacle();
        ob.type = TileCacheObstacleType.CYLINDER;

        vCopy(ob.pos, pos);
        ob.radius = radius;
        ob.height = height;

        return addObstacleRequest(ob).ref;
    }

    // Aabb obstacle
    public long addBoxObstacle(float[] bmin, float[] bmax) {
        TileCacheObstacle ob = allocObstacle();
        ob.type = TileCacheObstacleType.BOX;

        vCopy(ob.bmin, bmin);
        vCopy(ob.bmax, bmax);

        return addObstacleRequest(ob).ref;
    }

    // Box obstacle: can be rotated in Y
    public long addBoxObstacle(float[] center, float[] extents, float yRadians) {
        TileCacheObstacle ob = allocObstacle();
        ob.type = TileCacheObstacleType.ORIENTED_BOX;
        vCopy(ob.center, center);
        vCopy(ob.extents, extents);
        float coshalf = (float) Math.cos(0.5f * yRadians);
        float sinhalf = (float) Math.sin(-0.5f * yRadians);
        ob.rotAux[0] = coshalf * sinhalf;
        ob.rotAux[1] = coshalf * coshalf - 0.5f;
        return addObstacleRequest(ob).ref;
    }

    private ObstacleRequest addObstacleRequest(TileCacheObstacle ob) {
        ObstacleRequest req = new ObstacleRequest();
        req.action = ObstacleRequestAction.REQUEST_ADD;
        req.ref = getObstacleRef(ob);
        m_reqs.add(req);
        return req;
    }

    void removeObstacle(long ref) {
        if (ref == 0) {
            return;
        }

        ObstacleRequest req = new ObstacleRequest();
        req.action = ObstacleRequestAction.REQUEST_REMOVE;
        req.ref = ref;
        m_reqs.add(req);
    }

    private TileCacheObstacle allocObstacle() {
        TileCacheObstacle o = m_nextFreeObstacle;
        if (o == null) {
            o = new TileCacheObstacle(m_obstacles.size());
            m_obstacles.add(o);
        } else {
            m_nextFreeObstacle = o.next;
        }
        o.state = ObstacleState.DT_OBSTACLE_PROCESSING;
        o.touched.clear();
        o.pending.clear();
        o.next = null;
        return o;
    }

    List<Long> queryTiles(float[] bmin, float[] bmax) {
        List<Long> results = new ArrayList<>();
        float tw = m_params.width * m_params.cs;
        float th = m_params.height * m_params.cs;
        int tx0 = (int) Math.floor((bmin[0] - m_params.orig[0]) / tw);
        int tx1 = (int) Math.floor((bmax[0] - m_params.orig[0]) / tw);
        int ty0 = (int) Math.floor((bmin[2] - m_params.orig[2]) / th);
        int ty1 = (int) Math.floor((bmax[2] - m_params.orig[2]) / th);
        for (int ty = ty0; ty <= ty1; ++ty) {
            for (int tx = tx0; tx <= tx1; ++tx) {
                List<Long> tiles = getTilesAt(tx, ty);
                for (long i : tiles) {
                    CompressedTile tile = m_tiles[decodeTileIdTile(i)];
                    float[] tbmin = new float[3];
                    float[] tbmax = new float[3];
                    calcTightTileBounds(tile.header, tbmin, tbmax);
                    if (overlapBounds(bmin, bmax, tbmin, tbmax)) {
                        results.add(i);
                    }
                }
            }
        }
        return results;
    }

    /**
     * Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
     *
     * @return Returns true if the tile cache is fully up to date with obstacle requests and tile rebuilds. If the tile
     *         cache is up to date another (immediate) call to update will have no effect; otherwise another call will
     *         continue processing obstacle requests and tile rebuilds.
     */
    public boolean update() {
        if (m_update.isEmpty()) {
            // Process requests.
            for (ObstacleRequest req : m_reqs) {
                int idx = decodeObstacleIdObstacle(req.ref);
                if (idx >= m_obstacles.size()) {
                    continue;
                }
                TileCacheObstacle ob = m_obstacles.get(idx);
                int salt = decodeObstacleIdSalt(req.ref);
                if (ob.salt != salt) {
                    continue;
                }

                if (req.action == ObstacleRequestAction.REQUEST_ADD) {
                    // Find touched tiles.
                    float[] bmin = new float[3];
                    float[] bmax = new float[3];
                    getObstacleBounds(ob, bmin, bmax);
                    ob.touched = queryTiles(bmin, bmax);
                    // Add tiles to update list.
                    ob.pending.clear();
                    for (long j : ob.touched) {
                        if (!contains(m_update, j)) {
                            m_update.add(j);
                        }
                        ob.pending.add(j);
                    }
                } else if (req.action == ObstacleRequestAction.REQUEST_REMOVE) {
                    // Prepare to remove obstacle.
                    ob.state = ObstacleState.DT_OBSTACLE_REMOVING;
                    // Add tiles to update list.
                    ob.pending.clear();
                    for (long j : ob.touched) {
                        if (!contains(m_update, j)) {
                            m_update.add(j);
                        }
                        ob.pending.add(j);
                    }
                }
            }

            m_reqs.clear();
        }

        // Process updates
        if (!m_update.isEmpty()) {
            long ref = m_update.remove(0);
            // Build mesh
            buildNavMeshTile(ref);

            // Update obstacle states.
            for (int i = 0; i < m_obstacles.size(); ++i) {
                TileCacheObstacle ob = m_obstacles.get(i);
                if (ob.state == ObstacleState.DT_OBSTACLE_PROCESSING
                        || ob.state == ObstacleState.DT_OBSTACLE_REMOVING) {
                    // Remove handled tile from pending list.
                    ob.pending.remove(ref);

                    // If all pending tiles processed, change state.
                    if (ob.pending.isEmpty()) {
                        if (ob.state == ObstacleState.DT_OBSTACLE_PROCESSING) {
                            ob.state = ObstacleState.DT_OBSTACLE_PROCESSED;
                        } else if (ob.state == ObstacleState.DT_OBSTACLE_REMOVING) {
                            ob.state = ObstacleState.DT_OBSTACLE_EMPTY;
                            // Update salt, salt should never be zero.
                            ob.salt = (ob.salt + 1) & ((1 << 16) - 1);
                            if (ob.salt == 0) {
                                ob.salt++;
                            }
                            // Return obstacle to free list.
                            ob.next = m_nextFreeObstacle;
                            m_nextFreeObstacle = ob;
                        }
                    }
                }
            }
        }
        return m_update.isEmpty() && m_reqs.isEmpty();

    }

    public void buildNavMeshTile(long ref) {
        int idx = decodeTileIdTile(ref);
        if (idx > m_params.maxTiles) {
            throw new RuntimeException("Invalid tile index");
        }
        CompressedTile tile = m_tiles[idx];
        int salt = decodeTileIdSalt(ref);
        if (tile.salt != salt) {
            throw new RuntimeException("Invalid tile salt");
        }
        int walkableClimbVx = (int) (m_params.walkableClimb / m_params.ch);

        // Decompress tile layer data.
        TileCacheLayer layer = decompressTile(tile);

        // Rasterize obstacles.
        for (int i = 0; i < m_obstacles.size(); ++i) {
            TileCacheObstacle ob = m_obstacles.get(i);
            if (ob.state == ObstacleState.DT_OBSTACLE_EMPTY || ob.state == ObstacleState.DT_OBSTACLE_REMOVING) {
                continue;
            }
            if (contains(ob.touched, ref)) {
                if (ob.type == TileCacheObstacleType.CYLINDER) {
                    builder.markCylinderArea(layer, tile.header.bmin, m_params.cs, m_params.ch, ob.pos, ob.radius,
                            ob.height, 0);
                } else if (ob.type == TileCacheObstacleType.BOX) {
                    builder.markBoxArea(layer, tile.header.bmin, m_params.cs, m_params.ch, ob.bmin, ob.bmax, 0);
                } else if (ob.type == TileCacheObstacleType.ORIENTED_BOX) {
                    builder.markBoxArea(layer, tile.header.bmin, m_params.cs, m_params.ch, ob.center, ob.extents,
                            ob.rotAux, 0);
                }
            }
        }
        // Build navmesh
        builder.buildTileCacheRegions(layer, walkableClimbVx);
        TileCacheContourSet lcset = builder.buildTileCacheContours(layer, walkableClimbVx,
                m_params.maxSimplificationError);
        TileCachePolyMesh polyMesh = builder.buildTileCachePolyMesh(lcset, m_navmesh.getMaxVertsPerPoly());
        // Early out if the mesh tile is empty.
        if (polyMesh.npolys == 0) {
            m_navmesh.removeTile(m_navmesh.getTileRefAt(tile.header.tx, tile.header.ty, tile.header.tlayer));
            return;
        }
        NavMeshDataCreateParams params = new NavMeshDataCreateParams();
        params.verts = polyMesh.verts;
        params.vertCount = polyMesh.nverts;
        params.polys = polyMesh.polys;
        params.polyAreas = polyMesh.areas;
        params.polyFlags = polyMesh.flags;
        params.polyCount = polyMesh.npolys;
        params.nvp = m_navmesh.getMaxVertsPerPoly();
        params.walkableHeight = m_params.walkableHeight;
        params.walkableRadius = m_params.walkableRadius;
        params.walkableClimb = m_params.walkableClimb;
        params.tileX = tile.header.tx;
        params.tileY = tile.header.ty;
        params.tileLayer = tile.header.tlayer;
        params.cs = m_params.cs;
        params.ch = m_params.ch;
        params.buildBvTree = false;
        params.bmin = tile.header.bmin;
        params.bmax = tile.header.bmax;
        if (m_tmproc != null) {
            m_tmproc.process(params);
        }
        MeshData meshData = NavMeshBuilder.createNavMeshData(params);
        // Remove existing tile.
        m_navmesh.removeTile(m_navmesh.getTileRefAt(tile.header.tx, tile.header.ty, tile.header.tlayer));
        // Add new tile, or leave the location empty. if (navData) { // Let the
        if (meshData != null) {
            m_navmesh.addTile(meshData, 0, 0);
        }
    }

    public TileCacheLayer decompressTile(CompressedTile tile) {
        TileCacheLayer layer = builder.decompressTileCacheLayer(m_tcomp, tile.data, m_storageParams.byteOrder,
                m_storageParams.cCompatibility);
        return layer;
    }

    void calcTightTileBounds(TileCacheLayerHeader header, float[] bmin, float[] bmax) {
        float cs = m_params.cs;
        bmin[0] = header.bmin[0] + header.minx * cs;
        bmin[1] = header.bmin[1];
        bmin[2] = header.bmin[2] + header.miny * cs;
        bmax[0] = header.bmin[0] + (header.maxx + 1) * cs;
        bmax[1] = header.bmax[1];
        bmax[2] = header.bmin[2] + (header.maxy + 1) * cs;
    }

    void getObstacleBounds(TileCacheObstacle ob, float[] bmin, float[] bmax) {
        if (ob.type == TileCacheObstacleType.CYLINDER) {
            bmin[0] = ob.pos[0] - ob.radius;
            bmin[1] = ob.pos[1];
            bmin[2] = ob.pos[2] - ob.radius;
            bmax[0] = ob.pos[0] + ob.radius;
            bmax[1] = ob.pos[1] + ob.height;
            bmax[2] = ob.pos[2] + ob.radius;
        } else if (ob.type == TileCacheObstacleType.BOX) {
            vCopy(bmin, ob.bmin);
            vCopy(bmax, ob.bmax);
        } else if (ob.type == TileCacheObstacleType.ORIENTED_BOX) {
            float maxr = 1.41f * Math.max(ob.extents[0], ob.extents[2]);
            bmin[0] = ob.center[0] - maxr;
            bmax[0] = ob.center[0] + maxr;
            bmin[1] = ob.center[1] - ob.extents[1];
            bmax[1] = ob.center[1] + ob.extents[1];
            bmin[2] = ob.center[2] - maxr;
            bmax[2] = ob.center[2] + maxr;
        }
    }

    public TileCacheParams getParams() {
        return m_params;
    }

    public TileCacheCompressor getCompressor() {
        return m_tcomp;
    }

    public int getTileCount() {
        return m_params.maxTiles;
    }

    public CompressedTile getTile(int i) {
        return m_tiles[i];
    }

    public NavMesh getNavMesh() {
        return m_navmesh;
    }
}
