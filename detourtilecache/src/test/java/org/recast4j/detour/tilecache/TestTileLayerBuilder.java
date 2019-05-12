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

import static org.recast4j.detour.DetourCommon.vCopy;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

import org.recast4j.recast.HeightfieldLayerSet;
import org.recast4j.recast.HeightfieldLayerSet.HeightfieldLayer;
import org.recast4j.recast.Recast;
import org.recast4j.recast.RecastBuilder;
import org.recast4j.recast.RecastBuilderConfig;
import org.recast4j.recast.RecastConfig;
import org.recast4j.recast.RecastConstants.PartitionType;
import org.recast4j.recast.geom.InputGeomProvider;

public class TestTileLayerBuilder extends AbstractTileLayersBuilder {

    private final float m_cellSize = 0.3f;
    private final float m_cellHeight = 0.2f;
    private final float m_agentHeight = 2.0f;
    private final float m_agentRadius = 0.6f;
    private final float m_agentMaxClimb = 0.9f;
    private final float m_agentMaxSlope = 45.0f;
    private final int m_regionMinSize = 8;
    private final int m_regionMergeSize = 20;
    private final float m_edgeMaxLen = 12.0f;
    private final float m_edgeMaxError = 1.3f;
    private final int m_vertsPerPoly = 6;
    private final float m_detailSampleDist = 6.0f;
    private final float m_detailSampleMaxError = 1.0f;
    private final RecastConfig rcConfig;
    private final int m_tileSize = 48;
    protected final InputGeomProvider geom;
    private final int tw;
    private final int th;

    public TestTileLayerBuilder(InputGeomProvider geom) {
        this.geom = geom;
        rcConfig = new RecastConfig(PartitionType.WATERSHED, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
                m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError,
                m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, m_tileSize,
                SampleAreaModifications.SAMPLE_AREAMOD_GROUND);
        float[] bmin = geom.getMeshBoundsMin();
        float[] bmax = geom.getMeshBoundsMax();
        int[] twh = Recast.calcTileCount(bmin, bmax, m_cellSize, m_tileSize);
        tw = twh[0];
        th = twh[1];
    }

    public List<byte[]> build(ByteOrder order, boolean cCompatibility, int threads) {
        return build(order, cCompatibility, threads, tw, th);
    }

    public int getTw() {
        return tw;
    }

    public int getTh() {
        return th;
    }

    @Override
    public List<byte[]> build(int tx, int ty, ByteOrder order, boolean cCompatibility) {
        HeightfieldLayerSet lset = getHeightfieldSet(tx, ty);
        List<byte[]> result = new ArrayList<>();
        if (lset != null) {
            TileCacheBuilder builder = new TileCacheBuilder();
            for (int i = 0; i < lset.layers.length; ++i) {
                HeightfieldLayer layer = lset.layers[i];

                // Store header
                TileCacheLayerHeader header = new TileCacheLayerHeader();
                header.magic = TileCacheLayerHeader.DT_TILECACHE_MAGIC;
                header.version = TileCacheLayerHeader.DT_TILECACHE_VERSION;

                // Tile layer location in the navmesh.
                header.tx = tx;
                header.ty = ty;
                header.tlayer = i;
                vCopy(header.bmin, layer.bmin);
                vCopy(header.bmax, layer.bmax);

                // Tile info.
                header.width = layer.width;
                header.height = layer.height;
                header.minx = layer.minx;
                header.maxx = layer.maxx;
                header.miny = layer.miny;
                header.maxy = layer.maxy;
                header.hmin = layer.hmin;
                header.hmax = layer.hmax;
                result.add(builder.compressTileCacheLayer(header, layer.heights, layer.areas, layer.cons, order,
                        cCompatibility));
            }
        }
        return result;
    }

    protected HeightfieldLayerSet getHeightfieldSet(int tx, int ty) {
        RecastBuilder rcBuilder = new RecastBuilder();
        float[] bmin = geom.getMeshBoundsMin();
        float[] bmax = geom.getMeshBoundsMax();
        RecastBuilderConfig cfg = new RecastBuilderConfig(rcConfig, bmin, bmax, tx, ty, true);
        HeightfieldLayerSet lset = rcBuilder.buildLayers(geom, cfg);
        return lset;
    }
}
