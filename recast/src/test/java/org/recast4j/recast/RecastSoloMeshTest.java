/*
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.recast;

import static org.assertj.core.api.Assertions.assertThat;
import static org.recast4j.recast.RecastConstants.RC_MESH_NULL_IDX;

import java.io.File;
import java.io.FileWriter;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.recast4j.recast.RecastConstants.PartitionType;
import org.recast4j.recast.geom.InputGeomProvider;
import org.recast4j.recast.geom.TriMesh;

public class RecastSoloMeshTest {

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
    private PartitionType m_partitionType = PartitionType.WATERSHED;

    @Test
    public void testPerformance() {
        for (int i = 0; i < 10; i++) {
            testBuild("dungeon.obj", PartitionType.WATERSHED, 52, 16, 15, 223, 118, 118, 513, 291);
            testBuild("dungeon.obj", PartitionType.MONOTONE, 0, 17, 16, 210, 100, 100, 453, 264);
            testBuild("dungeon.obj", PartitionType.LAYERS, 0, 5, 5, 203, 97, 97, 446, 266);
        }
    }

    @Test
    public void testDungeonWatershed() {
        testBuild("dungeon.obj", PartitionType.WATERSHED, 52, 16, 15, 223, 118, 118, 513, 291);
    }

    @Test
    public void testDungeonMonotone() {
        testBuild("dungeon.obj", PartitionType.MONOTONE, 0, 17, 16, 210, 100, 100, 453, 264);
    }

    @Test
    public void testDungeonLayers() {
        testBuild("dungeon.obj", PartitionType.LAYERS, 0, 5, 5, 203, 97, 97, 446, 266);
    }

    @Test
    public void testWatershed() {
        testBuild("nav_test.obj", PartitionType.WATERSHED, 60, 48, 47, 349, 155, 155, 812, 561);
    }

    @Test
    public void testMonotone() {
        testBuild("nav_test.obj", PartitionType.MONOTONE, 0, 50, 49, 341, 186, 186, 878, 567);
    }

    @Test
    public void testLayers() {
        testBuild("nav_test.obj", PartitionType.LAYERS, 0, 19, 32, 310, 150, 150, 773, 526);
    }

    public void testBuild(String filename, PartitionType partitionType, int expDistance, int expRegions,
            int expContours, int expVerts, int expPolys, int expDetMeshes, int expDetVerts, int expDetTris) {
        m_partitionType = partitionType;
        ObjImporter importer = new ObjImporter();
        InputGeomProvider geomProvider = importer.load(getClass().getResourceAsStream(filename));
        long time = System.nanoTime();
        float[] bmin = geomProvider.getMeshBoundsMin();
        float[] bmax = geomProvider.getMeshBoundsMax();
        Telemetry m_ctx = new Telemetry();
        //
        // Step 1. Initialize build config.
        //

        // Init build configuration from GUI
        RecastConfig cfg = new RecastConfig(partitionType, m_cellSize, m_cellHeight, m_agentHeight, m_agentRadius,
                m_agentMaxClimb, m_agentMaxSlope, m_regionMinSize, m_regionMergeSize, m_edgeMaxLen, m_edgeMaxError,
                m_vertsPerPoly, m_detailSampleDist, m_detailSampleMaxError, SampleAreaModifications.SAMPLE_AREAMOD_GROUND);
        RecastBuilderConfig bcfg = new RecastBuilderConfig(cfg, bmin, bmax);
        //
        // Step 2. Rasterize input polygon soup.
        //

        // Allocate voxel heightfield where we rasterize our input data to.
        Heightfield m_solid = new Heightfield(bcfg.width, bcfg.height, bcfg.bmin, bcfg.bmax, cfg.cs, cfg.ch, cfg.borderSize);

        for (TriMesh geom : geomProvider.meshes()) {
            float[] verts = geom.getVerts();
            int[] tris = geom.getTris();
            int ntris = tris.length / 3;

            // Allocate array that can hold triangle area types.
            // If you have multiple meshes you need to process, allocate
            // and array which can hold the max number of triangles you need to
            // process.

            // Find triangles which are walkable based on their slope and rasterize
            // them.
            // If your input data is multiple meshes, you can transform them here,
            // calculate
            // the are type for each of the meshes and rasterize them.
            int[] m_triareas = Recast.markWalkableTriangles(m_ctx, cfg.walkableSlopeAngle, verts, tris, ntris,
                    cfg.walkableAreaMod);
            RecastRasterization.rasterizeTriangles(m_solid, verts, tris, m_triareas, ntris, cfg.walkableClimb, m_ctx);
            //
            // Step 3. Filter walkables surfaces.
            //
        }
        // Once all geometry is rasterized, we do initial pass of filtering to
        // remove unwanted overhangs caused by the conservative rasterization
        // as well as filter spans where the character cannot possibly stand.
        RecastFilter.filterLowHangingWalkableObstacles(m_ctx, cfg.walkableClimb, m_solid);
        RecastFilter.filterLedgeSpans(m_ctx, cfg.walkableHeight, cfg.walkableClimb, m_solid);
        RecastFilter.filterWalkableLowHeightSpans(m_ctx, cfg.walkableHeight, m_solid);

        //
        // Step 4. Partition walkable surface to simple regions.
        //

        // Compact the heightfield so that it is faster to handle from now on.
        // This will result more cache coherent data as well as the neighbours
        // between walkable cells will be calculated.
        CompactHeightfield m_chf = RecastCompact.buildCompactHeightfield(m_ctx, cfg.walkableHeight, cfg.walkableClimb,
                m_solid);

        // Erode the walkable area by agent radius.
        RecastArea.erodeWalkableArea(m_ctx, cfg.walkableRadius, m_chf);

        // (Optional) Mark areas.
        /*
         * ConvexVolume vols = m_geom->getConvexVolumes(); for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
         * rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned
         * char)vols[i].area, *m_chf);
         */

        // Partition the heightfield so that we can use simple algorithm later
        // to triangulate the walkable areas.
        // There are 3 martitioning methods, each with some pros and cons:
        // 1) Watershed partitioning
        // - the classic Recast partitioning
        // - creates the nicest tessellation
        // - usually slowest
        // - partitions the heightfield into nice regions without holes or
        // overlaps
        // - the are some corner cases where this method creates produces holes
        // and overlaps
        // - holes may appear when a small obstacles is close to large open area
        // (triangulation can handle this)
        // - overlaps may occur if you have narrow spiral corridors (i.e
        // stairs), this make triangulation to fail
        // * generally the best choice if you precompute the nacmesh, use this
        // if you have large open areas
        // 2) Monotone partioning
        // - fastest
        // - partitions the heightfield into regions without holes and overlaps
        // (guaranteed)
        // - creates long thin polygons, which sometimes causes paths with
        // detours
        // * use this if you want fast navmesh generation
        // 3) Layer partitoining
        // - quite fast
        // - partitions the heighfield into non-overlapping regions
        // - relies on the triangulation code to cope with holes (thus slower
        // than monotone partitioning)
        // - produces better triangles than monotone partitioning
        // - does not have the corner cases of watershed partitioning
        // - can be slow and create a bit ugly tessellation (still better than
        // monotone)
        // if you have large open areas with small obstacles (not a problem if
        // you use tiles)
        // * good choice to use for tiled navmesh with medium and small sized
        // tiles
        long time3 = System.nanoTime();

        if (m_partitionType == PartitionType.WATERSHED) {
            // Prepare for region partitioning, by calculating distance field
            // along the walkable surface.
            RecastRegion.buildDistanceField(m_ctx, m_chf);
            // Partition the walkable surface into simple regions without holes.
            RecastRegion.buildRegions(m_ctx, m_chf, cfg.minRegionArea, cfg.mergeRegionArea);
        } else if (m_partitionType == PartitionType.MONOTONE) {
            // Partition the walkable surface into simple regions without holes.
            // Monotone partitioning does not need distancefield.
            RecastRegion.buildRegionsMonotone(m_ctx, m_chf, cfg.minRegionArea, cfg.mergeRegionArea);
        } else {
            // Partition the walkable surface into simple regions without holes.
            RecastRegion.buildLayerRegions(m_ctx, m_chf, cfg.minRegionArea);
        }

        assertThat(m_chf.maxDistance).as("maxDistance").isEqualTo(expDistance);
        assertThat(m_chf.maxRegions).as("Regions").isEqualTo(expRegions);
        //
        // Step 5. Trace and simplify region contours.
        //

        // Create contours.
        ContourSet m_cset = RecastContour.buildContours(m_ctx, m_chf, cfg.maxSimplificationError, cfg.maxEdgeLen,
                RecastConstants.RC_CONTOUR_TESS_WALL_EDGES);

        assertThat(m_cset.conts).as("Contours").hasSize(expContours);
        //
        // Step 6. Build polygons mesh from contours.
        //

        // Build polygon navmesh from the contours.
        PolyMesh m_pmesh = RecastMesh.buildPolyMesh(m_ctx, m_cset, cfg.maxVertsPerPoly);
        assertThat(m_pmesh.nverts).as("Mesh Verts").isEqualTo(expVerts);
        assertThat(m_pmesh.npolys).as("Mesh Polys").isEqualTo(expPolys);

        //
        // Step 7. Create detail mesh which allows to access approximate height
        // on each polygon.
        //

        PolyMeshDetail m_dmesh = RecastMeshDetail.buildPolyMeshDetail(m_ctx, m_pmesh, m_chf, cfg.detailSampleDist,
                cfg.detailSampleMaxError);
        assertThat(m_dmesh.nmeshes).as("Mesh Detail Meshes").isEqualTo(expDetMeshes);
        assertThat(m_dmesh.nverts).as("Mesh Detail Verts").isEqualTo(expDetVerts);
        assertThat(m_dmesh.ntris).as("Mesh Detail Tris").isEqualTo(expDetTris);
        long time2 = System.nanoTime();
        System.out.println(filename + " : " + partitionType + "  " + (time2 - time) / 1000000 + " ms");
        System.out.println("           " + (time3 - time) / 1000000 + " ms");
        saveObj(filename.substring(0, filename.lastIndexOf('.')) + "_" + partitionType + "_detail.obj", m_dmesh);
        saveObj(filename.substring(0, filename.lastIndexOf('.')) + "_" + partitionType + ".obj", m_pmesh);
        m_ctx.print();
    }

    private void saveObj(String filename, PolyMesh mesh) {
        try {
            File file = Path.of("test-output", filename).toFile();
            file.getParentFile().mkdirs();
            FileWriter fw = new FileWriter(file);
            for (int v = 0; v < mesh.nverts; v++) {
                fw.write("v " + (mesh.bmin[0] + mesh.verts[v * 3] * mesh.cs) + " "
                        + (mesh.bmin[1] + mesh.verts[v * 3 + 1] * mesh.ch) + " "
                        + (mesh.bmin[2] + mesh.verts[v * 3 + 2] * mesh.cs) + "\n");
            }

            for (int i = 0; i < mesh.npolys; i++) {
                int p = i * mesh.nvp * 2;
                fw.write("f ");
                for (int j = 0; j < mesh.nvp; ++j) {
                    int v = mesh.polys[p + j];
                    if (v == RC_MESH_NULL_IDX) {
                        break;
                    }
                    fw.write((v + 1) + " ");
                }
                fw.write("\n");
            }
            fw.close();
        } catch (Exception e) {
        }
    }

    private void saveObj(String filename, PolyMeshDetail dmesh) {
        try {
            File file = Path.of("test-output", filename).toFile();
            file.getParentFile().mkdirs();
            FileWriter fw = new FileWriter(file);
            for (int v = 0; v < dmesh.nverts; v++) {
                fw.write(
                        "v " + dmesh.verts[v * 3] + " " + dmesh.verts[v * 3 + 1] + " " + dmesh.verts[v * 3 + 2] + "\n");
            }

            for (int m = 0; m < dmesh.nmeshes; m++) {
                int vfirst = dmesh.meshes[m * 4];
                int tfirst = dmesh.meshes[m * 4 + 2];
                for (int f = 0; f < dmesh.meshes[m * 4 + 3]; f++) {
                    fw.write("f " + (vfirst + dmesh.tris[(tfirst + f) * 4] + 1) + " "
                            + (vfirst + dmesh.tris[(tfirst + f) * 4 + 1] + 1) + " "
                            + (vfirst + dmesh.tris[(tfirst + f) * 4 + 2] + 1) + "\n");
                }
            }
            fw.close();
        } catch (Exception e) {
        }
    }
}
