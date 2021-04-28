/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org

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

package org.recast4j.dynamic;

import static java.util.stream.Collectors.toList;
import static java.util.stream.Collectors.toSet;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicLong;

import org.recast4j.detour.NavMesh;
import org.recast4j.detour.NavMeshDataCreateParams;
import org.recast4j.detour.NavMeshParams;
import org.recast4j.dynamic.collider.Collider;
import org.recast4j.dynamic.io.VoxelFile;
import org.recast4j.dynamic.io.VoxelTile;
import org.recast4j.recast.Heightfield;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilder.RecastBuilderResult;
import org.recast4j.recast.Telemetry;

public class DynamicNavMesh {

    static final int MAX_VERTS_PER_POLY = 6;
    public final DynamicNavMeshConfig config;
    private final RecastBuilder builder;
    private final Map<Long, DynamicTile> tiles = new HashMap<>();
    private final Telemetry telemetry;
    private final NavMeshParams navMeshParams;
    private final Queue<UpdateQueueItem> updateQueue = new LinkedBlockingQueue<>();
    private final AtomicLong currentColliderId = new AtomicLong();
    private NavMesh navMesh;
    private boolean dirty = true;

    public DynamicNavMesh(VoxelFile voxelFile) {
        config = new DynamicNavMeshConfig(voxelFile.useTiles, voxelFile.tileSizeX, voxelFile.tileSizeZ, voxelFile.cellSize);
        config.walkableHeight = voxelFile.walkableHeight;
        config.walkableRadius = voxelFile.walkableRadius;
        config.walkableClimb = voxelFile.walkableClimb;
        config.walkableSlopeAngle = voxelFile.walkableSlopeAngle;
        config.maxSimplificationError = voxelFile.maxSimplificationError;
        config.maxEdgeLen = voxelFile.maxEdgeLen;
        config.minRegionArea = voxelFile.minRegionArea;
        config.regionMergeArea = voxelFile.regionMergeArea;
        config.vertsPerPoly = voxelFile.vertsPerPoly;
        config.buildDetailMesh = voxelFile.buildMeshDetail;
        config.detailSampleDistance = voxelFile.detailSampleDistance;
        config.detailSampleMaxError = voxelFile.detailSampleMaxError;
        builder = new RecastBuilder();
        navMeshParams = new NavMeshParams();
        navMeshParams.orig[0] = voxelFile.bounds[0];
        navMeshParams.orig[1] = voxelFile.bounds[1];
        navMeshParams.orig[2] = voxelFile.bounds[2];
        navMeshParams.tileWidth = voxelFile.cellSize * voxelFile.tileSizeX;
        navMeshParams.tileHeight = voxelFile.cellSize * voxelFile.tileSizeZ;
        navMeshParams.maxTiles = voxelFile.tiles.size();
        navMeshParams.maxPolys = 0x8000;
        voxelFile.tiles.forEach(t -> {
            tiles.put(lookupKey(t.tileX, t.tileZ), new DynamicTile(t));
        });
        telemetry = new Telemetry();
    }

    public NavMesh navMesh() {
        return navMesh;
    }

    /**
     * Voxel queries require checkpoints to be enabled in {@link DynamicNavMeshConfig}
     */
    public VoxelQuery voxelQuery() {
        return new VoxelQuery(navMeshParams.orig, navMeshParams.tileWidth, navMeshParams.tileHeight, this::lookupHeightfield);
    }

    private Optional<Heightfield> lookupHeightfield(int x, int z) {
        return Optional.ofNullable(getTileAt(x, z)).map(t -> t.checkpoint).map(c -> c.heightfield);
    }

    public long addCollider(Collider collider) {
        long cid = currentColliderId.incrementAndGet();
        updateQueue.add(new AddColliderQueueItem(cid, collider, getTiles(collider.bounds())));
        return cid;
    }

    public void removeCollider(long colliderId) {
        updateQueue.add(new RemoveColliderQueueItem(colliderId, getTilesByCollider(colliderId)));
    }

    /**
     * Perform full build of the nav mesh
     */
    public void build() {
        processQueue();
        rebuild(tiles.values());
    }

    /**
     * Perform incremental update of the nav mesh
     */
    public boolean update() {
        return rebuild(processQueue());
    }

    private boolean rebuild(Collection<DynamicTile> stream) {
        stream.forEach(this::rebuild);
        return updateNavMesh();
    }

    private Collection<DynamicTile> processQueue() {
        return consumeQueue().stream().peek(this::process).flatMap(i -> i.affectedTiles().stream()).collect(toSet());
    }

    private List<UpdateQueueItem> consumeQueue() {
        List<UpdateQueueItem> items = new ArrayList<>();
        while (true) {
            UpdateQueueItem item = updateQueue.poll();
            if (item == null) {
                break;
            }
            items.add(item);
        }
        return items;
    }

    private void process(UpdateQueueItem item) {
        item.affectedTiles().forEach(item::process);
    }

    /**
     * Perform full build concurrently using the given {@link ExecutorService}
     */
    public CompletableFuture<Boolean> build(ExecutorService executor) {
        processQueue();
        return rebuild(tiles.values(), executor);
    }

    /**
     * Perform incremental update concurrently using the given {@link ExecutorService}
     */
    public CompletableFuture<Boolean> update(ExecutorService executor) {
        return rebuild(processQueue(), executor);
    }

    private CompletableFuture<Boolean> rebuild(Collection<DynamicTile> tiles, ExecutorService executor) {
        CompletableFuture<Boolean> future = CompletableFuture.allOf(tiles.stream()
                .map(tile -> CompletableFuture.runAsync(() -> rebuild(tile), executor)).toArray(CompletableFuture[]::new))
                .thenApply(__ -> updateNavMesh());
        return future;
    }

    private Collection<DynamicTile> getTiles(float[] bounds) {
        if (bounds == null) {
            return tiles.values();
        }
        int minx = (int) Math.floor((bounds[0] - navMeshParams.orig[0]) / navMeshParams.tileWidth);
        int minz = (int) Math.floor((bounds[2] - navMeshParams.orig[2]) / navMeshParams.tileHeight);
        int maxx = (int) Math.floor((bounds[3] - navMeshParams.orig[0]) / navMeshParams.tileWidth);
        int maxz = (int) Math.floor((bounds[5] - navMeshParams.orig[2]) / navMeshParams.tileHeight);
        List<DynamicTile> tiles = new ArrayList<>();
        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                DynamicTile tile = getTileAt(x, z);
                if (tile != null) {
                    tiles.add(tile);
                }
            }
        }
        return tiles;
    }

    private Collection<DynamicTile> getTilesByCollider(long cid) {
        return tiles.values().stream().filter(t -> t.containsCollider(cid)).collect(toList());
    }

    private void rebuild(DynamicTile tile) {
        NavMeshDataCreateParams params = new NavMeshDataCreateParams();
        params.walkableHeight = config.walkableHeight;
        dirty = dirty | tile.build(builder, config, telemetry);
    }

    private boolean updateNavMesh() {
        if (dirty) {
            NavMesh navMesh = new NavMesh(navMeshParams, MAX_VERTS_PER_POLY);
            tiles.values().forEach(t -> t.addTo(navMesh));
            this.navMesh = navMesh;
            dirty = false;
            return true;
        }
        return false;
    }

    private DynamicTile getTileAt(int x, int z) {
        return tiles.get(lookupKey(x, z));
    }

    private long lookupKey(long x, long z) {
        return (z << 32) | x;
    }

    public List<VoxelTile> voxelTiles() {
        return tiles.values().stream().map(t -> t.voxelTile).collect(toList());
    }

    public List<RecastBuilderResult> recastResults() {
        return tiles.values().stream().map(t -> t.recastResult).collect(toList());
    }

}
