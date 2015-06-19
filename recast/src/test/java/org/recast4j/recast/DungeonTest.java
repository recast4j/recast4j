package org.recast4j.recast;

import java.io.File;
import java.io.FileWriter;

import org.junit.Assert;
import org.junit.Test;
import org.recast4j.recast.RecastConstants.PartitionType;

public class DungeonTest {

	private float m_cellSize;
	private float m_cellHeight;
	private float m_agentHeight;
	private float m_agentRadius;
	private float m_agentMaxClimb;
	private float m_agentMaxSlope;
	private int m_regionMinSize;
	private int m_regionMergeSize;
	private float m_edgeMaxLen;
	private float m_edgeMaxError;
	private float m_vertsPerPoly;
	private float m_detailSampleDist;
	private PartitionType m_partitionType;
	private float m_detailSampleMaxError;

	void resetCommonSettings() {
		m_cellSize = 0.3f;
		m_cellHeight = 0.2f;
		m_agentHeight = 2.0f;
		m_agentRadius = 0.6f;
		m_agentMaxClimb = 0.9f;
		m_agentMaxSlope = 45.0f;
		m_regionMinSize = 8;
		m_regionMergeSize = 20;
		m_edgeMaxLen = 12.0f;
		m_edgeMaxError = 1.3f;
		m_vertsPerPoly = 6.0f;
		m_detailSampleDist = 6.0f;
		m_detailSampleMaxError = 1.0f;
		m_partitionType = PartitionType.WATERSHED;
	}

	@Test
	public void testBuildWatershed() {
		testBuild(PartitionType.WATERSHED, 52, 16, 15, 223, 118, 118, 512, 289);
	}

	@Test
	public void testBuildMonotone() {
		testBuild(PartitionType.MONOTONE, 0, 17, 16, 210, 100, 100, 453, 264);
	}

	@Test
	public void testBuildLayers() {
		testBuild(PartitionType.LAYERS, 0, 5, 5, 203, 97, 97, 447, 268);
	}

	public void testBuild(PartitionType partitionType, int expDisatnce, int expRegions, int expContours, int expVerts, int expPolys, Object expDetMeshes, Object expDetVerts, Object expDetTRis) {
		resetCommonSettings();
		m_partitionType = partitionType;
		ObjImporter importer = new ObjImporter();
		InputGeom m_geom = importer.load(getClass().getResourceAsStream("dungeon.obj"));
		float[] bmin = m_geom.getMeshBoundsMin();
		float[] bmax = m_geom.getMeshBoundsMax();
		float[] verts = m_geom.getVerts();
		int nverts = verts.length / 3;
		int[] tris = m_geom.getTris();
		int ntris = tris.length / 3;
		//
		// Step 1. Initialize build config.
		//

		// Init build configuration from GUI
		RecastConfig m_cfg = new RecastConfig();
		m_cfg.cs = m_cellSize;
		m_cfg.ch = m_cellHeight;
		m_cfg.walkableSlopeAngle = m_agentMaxSlope;
		m_cfg.walkableHeight = (int) Math.ceil(m_agentHeight / m_cfg.ch);
		m_cfg.walkableClimb = (int) Math.floor(m_agentMaxClimb / m_cfg.ch);
		m_cfg.walkableRadius = (int) Math.ceil(m_agentRadius / m_cfg.cs);
		m_cfg.maxEdgeLen = (int) (m_edgeMaxLen / m_cellSize);
		m_cfg.maxSimplificationError = m_edgeMaxError;
		m_cfg.minRegionArea = m_regionMinSize * m_regionMinSize; // Note:
																	// area
																	// =
																	// size*size
		m_cfg.mergeRegionArea = m_regionMergeSize * m_regionMergeSize; // Note:
																		// area
																		// =
																		// size*size
		m_cfg.maxVertsPerPoly = (int) m_vertsPerPoly;
		m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
		m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

		// Set the area where the navigation will be build.
		// Here the bounds of the input mesh are used, but the
		// area could be specified by an user defined box, etc.
		m_cfg.bmin = bmin;
		m_cfg.bmax = bmax;

		int[] wh = Recast.calcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs);
		m_cfg.width = wh[0];
		m_cfg.height = wh[1];

		Context m_ctx = new Context();
		//
		// Step 2. Rasterize input polygon soup.
		//

		// Allocate voxel heightfield where we rasterize our input data to.
		Heightfield m_solid = new Heightfield(m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch);

		// Allocate array that can hold triangle area types.
		// If you have multiple meshes you need to process, allocate
		// and array which can hold the max number of triangles you need to
		// process.
		int[] m_triareas = new int[ntris];

		// Find triangles which are walkable based on their slope and rasterize
		// them.
		// If your input data is multiple meshes, you can transform them here,
		// calculate
		// the are type for each of the meshes and rasterize them.
		Recast.markWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
		RecastRasterization.rasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, m_solid,
				m_cfg.walkableClimb);
		//
		// Step 3. Filter walkables surfaces.
		//

		// Once all geometry is rasterized, we do initial pass of filtering to
		// remove unwanted overhangs caused by the conservative rasterization
		// as well as filter spans where the character cannot possibly stand.
		RecastFilter.filterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, m_solid);
		RecastFilter.filterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, m_solid);
		RecastFilter.filterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, m_solid);

		//
		// Step 4. Partition walkable surface to simple regions.
		//

		// Compact the heightfield so that it is faster to handle from now on.
		// This will result more cache coherent data as well as the neighbours
		// between walkable cells will be calculated.
		CompactHeightfield m_chf = new CompactHeightfield();
		Recast.buildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, m_solid, m_chf);

		// Erode the walkable area by agent radius.
		RecastArea.erodeWalkableArea(m_ctx, m_cfg.walkableRadius, m_chf);

		// (Optional) Mark areas.
		/*
		 * ConvexVolume vols = m_geom->getConvexVolumes(); for (int i = 0; i <
		 * m_geom->getConvexVolumeCount(); ++i) rcMarkConvexPolyArea(m_ctx,
		 * vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned
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

		if (m_partitionType == PartitionType.WATERSHED) {
			// Prepare for region partitioning, by calculating distance field
			// along the walkable surface.
			RecastRegion.buildDistanceField(m_ctx, m_chf);
			// Partition the walkable surface into simple regions without holes.
			RecastRegion.buildRegions(m_ctx, m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea);
		} else if (m_partitionType == PartitionType.MONOTONE) {
			// Partition the walkable surface into simple regions without holes. 
			// Monotone partitioning does not need distancefield. 
			RecastRegion.buildRegionsMonotone(m_ctx, m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea);
		} else {
			// Partition the walkable surface into simple regions without holes. 
			RecastRegion.buildLayerRegions(m_ctx, m_chf, 0, m_cfg.minRegionArea);
		}

		Assert.assertEquals("maxDisatance", expDisatnce, m_chf.maxDistance);
		Assert.assertEquals("Regions", expRegions, m_chf.maxRegions);
		//
		// Step 5. Trace and simplify region contours.
		//

		// Create contours.
		ContourSet m_cset = new ContourSet();
		RecastContour.buildContours(m_ctx, m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, m_cset,
				RecastConstants.RC_CONTOUR_TESS_WALL_EDGES);

		Assert.assertEquals("Contours", expContours, m_cset.conts.size());
		//
		// Step 6. Build polygons mesh from contours.
		//

		// Build polygon navmesh from the contours.
		PolyMesh m_pmesh = new PolyMesh();
		RecastMesh.buildPolyMesh(m_ctx, m_cset, m_cfg.maxVertsPerPoly, m_pmesh);
		Assert.assertEquals("Mesh Verts", expVerts, m_pmesh.nverts);
		Assert.assertEquals("Mesh Polys", expPolys, m_pmesh.npolys);

		//
		// Step 7. Create detail mesh which allows to access approximate height
		// on each polygon.
		//

		PolyMeshDetail m_dmesh = new PolyMeshDetail();

		RecastMeshDetail.buildPolyMeshDetail(m_ctx, m_pmesh, m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError,
				m_dmesh);

		Assert.assertEquals("Mesh Detail Meshes", expDetMeshes, m_dmesh.nmeshes);
		Assert.assertEquals("Mesh Detail Verts", expDetVerts, m_dmesh.nverts);
		Assert.assertEquals("Mesh Detail Tris", expDetTRis, m_dmesh.ntris);
		saveObj(m_dmesh);
	}

	private void saveObj(PolyMeshDetail m_dmesh) {
		try {
			File file = new File("dungeon_nmesh.obj");
			FileWriter fw = new FileWriter(file);
			for (int v = 0; v < m_dmesh.nverts; v++) {
				fw.write("v " + m_dmesh.verts[v * 3] + " " + m_dmesh.verts[v * 3 + 1] + " " + m_dmesh.verts[v * 3 + 2]
						+ "\n");
			}

			for (int m = 0; m < m_dmesh.nmeshes; m++) {
				int vfirst = m_dmesh.meshes[m * 4];
				int tfirst = m_dmesh.meshes[m * 4 + 2];
				for (int f = 0; f < m_dmesh.meshes[m * 4 + 3]; f++) {
					fw.write("f " + (vfirst + m_dmesh.tris[(tfirst + f) * 4] + 1) + " "
							+ (vfirst + m_dmesh.tris[(tfirst + f) * 4 + 1] + 1) + " "
							+ (vfirst + m_dmesh.tris[(tfirst + f) * 4 + 2] + 1) + "\n");
				}
			}
			fw.close();
		} catch (Exception e) {
		}
	}
}
