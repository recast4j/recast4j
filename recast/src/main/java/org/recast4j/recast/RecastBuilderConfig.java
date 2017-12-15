package org.recast4j.recast;

import static org.recast4j.recast.RecastVectors.copy;

public class RecastBuilderConfig {

    public final RecastConfig cfg;

    /** The width of the field along the x-axis. [Limit: >= 0] [Units: vx] **/
    public final int width;

    /** The height of the field along the z-axis. [Limit: >= 0] [Units: vx] **/
    public final int height;

    /** The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu] **/
    public final float[] bmin = new float[3];

    /** The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu] **/
    public final float[] bmax = new float[3];

    /** The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx] **/
    public final int borderSize;

    /** Set to true for tiled build **/
    public final boolean tiled;

    /** Set to false to disable building detailed mesh **/
    public final boolean buildMeshDetail;

    public RecastBuilderConfig(RecastConfig cfg, float[] bmin, float[] bmax) {
        this(cfg, bmin, bmax, 0, 0, false);
    }

    public RecastBuilderConfig(RecastConfig cfg, float[] bmin, float[] bmax, int tx, int ty, boolean tiled) {
        this(cfg, bmin, bmax, tx, ty, tiled, true);
    }

    public RecastBuilderConfig(RecastConfig cfg, float[] bmin, float[] bmax, int tx, int ty, boolean tiled, boolean buildMeshDetail) {
        this.cfg = cfg;
        this.tiled = tiled;
        this.buildMeshDetail = buildMeshDetail;
        copy(this.bmin, bmin);
        copy(this.bmax, bmax);
        if (tiled) {
            float ts = cfg.tileSize * cfg.cs;
            this.bmin[0] += tx * ts;
            this.bmin[2] += ty * ts;
            this.bmax[0] = this.bmin[0] + ts;
            this.bmax[2] = this.bmin[2] + ts;
            // Expand the heighfield bounding box by border size to find the extents of geometry we need to build this
            // tile.
            //
            // This is done in order to make sure that the navmesh tiles connect correctly at the borders,
            // and the obstacles close to the border work correctly with the dilation process.
            // No polygons (or contours) will be created on the border area.
            //
            // IMPORTANT!
            //
            // :''''''''':
            // : +-----+ :
            // : | | :
            // : | |<--- tile to build
            // : | | :
            // : +-----+ :<-- geometry needed
            // :.........:
            //
            // You should use this bounding box to query your input geometry.
            //
            // For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
            // you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8
            // neighbours,
            // or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
            this.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
            this.bmin[0] -= this.borderSize * cfg.cs;
            this.bmin[2] -= this.borderSize * cfg.cs;
            this.bmax[0] += this.borderSize * cfg.cs;
            this.bmax[2] += this.borderSize * cfg.cs;
            this.width = cfg.tileSize + this.borderSize * 2;
            this.height = cfg.tileSize + this.borderSize * 2;
        } else {
            int[] wh = Recast.calcGridSize(this.bmin, this.bmax, cfg.cs);
            this.width = wh[0];
            this.height = wh[1];
            this.borderSize = 0;
        }
    }

}
