/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
Recast4J Copyright (c) 2015 Piotr Piastucki piotr@jtilia.org

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

import java.util.List;

import org.recast4j.recast.ChunkyTriMesh.ChunkyTriMeshNode;
import org.recast4j.recast.RecastConstants.PartitionType;

public class RecastBuilder {

	private InputGeom m_geom;
	private PolyMesh m_pmesh;
	private PolyMeshDetail m_dmesh;

	public RecastBuilder(InputGeom m_geom) {
		this.m_geom = m_geom;
	}

	public void build(RecastConfig m_cfg) {

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

		// Find triangles which are walkable based on their slope and rasterize
		// them.
		// If your input data is multiple meshes, you can transform them here,
		// calculate
		// the are type for each of the meshes and rasterize them.
		float[] verts = m_geom.getVerts();
		boolean tiled = m_cfg.tileSize > 0;
		int totaltris = 0;
		if (tiled) {
			ChunkyTriMesh chunkyMesh = m_geom.getChunkyMesh();
			float[] tbmin = new float[2];
			float[] tbmax = new float[2];
			tbmin[0] = m_cfg.bmin[0];
			tbmin[1] = m_cfg.bmin[2];
			tbmax[0] = m_cfg.bmax[0];
			tbmax[1] = m_cfg.bmax[2];
			List<ChunkyTriMeshNode> nodes = chunkyMesh.getChunksOverlappingRect(tbmin, tbmax);
			for (ChunkyTriMeshNode node : nodes) {
				int[] tris = node.tris;
				int ntris = tris.length / 3;
				totaltris += ntris;
				int[] m_triareas = Recast.markWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, tris, ntris);
				RecastRasterization.rasterizeTriangles(m_ctx, verts, tris, m_triareas, ntris, m_solid,
						m_cfg.walkableClimb);
			}
		} else {
			int[] tris = m_geom.getTris();
			int ntris = tris.length / 3;
			int[] m_triareas = Recast.markWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, tris, ntris);
			totaltris = ntris;
			RecastRasterization.rasterizeTriangles(m_ctx, verts, tris, m_triareas, ntris, m_solid, m_cfg.walkableClimb);
		}
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
		CompactHeightfield m_chf = Recast.buildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb,
				m_solid);

		// Erode the walkable area by agent radius.
		RecastArea.erodeWalkableArea(m_ctx, m_cfg.walkableRadius, m_chf);

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

		if (m_cfg.partitionType == PartitionType.WATERSHED) {
			// Prepare for region partitioning, by calculating distance field
			// along the walkable surface.
			RecastRegion.buildDistanceField(m_ctx, m_chf);
			// Partition the walkable surface into simple regions without holes.
			RecastRegion.buildRegions(m_ctx, m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea);
		} else if (m_cfg.partitionType == PartitionType.MONOTONE) {
			// Partition the walkable surface into simple regions without holes.
			// Monotone partitioning does not need distancefield.
			RecastRegion.buildRegionsMonotone(m_ctx, m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea);
		} else {
			// Partition the walkable surface into simple regions without holes.
			RecastRegion.buildLayerRegions(m_ctx, m_chf, m_cfg.borderSize, m_cfg.minRegionArea);
		}

		//
		// Step 5. Trace and simplify region contours.
		//

		// Create contours.
		ContourSet m_cset = RecastContour.buildContours(m_ctx, m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen,
				RecastConstants.RC_CONTOUR_TESS_WALL_EDGES);

		//
		// Step 6. Build polygons mesh from contours.
		//

		m_pmesh = RecastMesh.buildPolyMesh(m_ctx, m_cset, m_cfg.maxVertsPerPoly);

		//
		// Step 7. Create detail mesh which allows to access approximate height
		// on each polygon.
		//

		m_dmesh = RecastMeshDetail.buildPolyMeshDetail(m_ctx, m_pmesh, m_chf, m_cfg.detailSampleDist,
				m_cfg.detailSampleMaxError);

	}

	public PolyMesh getMesh() {
		return m_pmesh;
	}

	public PolyMeshDetail getMeshDetail() {
		return m_dmesh;
	}

}
