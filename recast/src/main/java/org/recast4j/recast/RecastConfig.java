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

import static org.recast4j.recast.RecastVectors.copy;

import org.recast4j.recast.RecastConstants.PartitionType;

public class RecastConfig {
	public final PartitionType partitionType;

	/** The width of the field along the x-axis. [Limit: >= 0] [Units: vx] **/
	public final int width;

	/** The height of the field along the z-axis. [Limit: >= 0] [Units: vx] **/
	public final int height;

	/** The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx] **/
	public final int tileSize;

	/** The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx] **/
	public final int borderSize;

	/** The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] **/
	public final float cs;

	/** The y-axis cell size to use for fields. [Limit: > 0] [Units: wu] **/
	public final float ch;

	/** The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu] **/
	public final float[] bmin = new float[3];

	/** The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu] **/
	public final float[] bmax = new float[3];

	/** The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] **/
	public final float walkableSlopeAngle;

	/**
	 * Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3]
	 * [Units: vx]
	 **/
	public final int walkableHeight;

	/** Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx] **/
	public final int walkableClimb;

	/**
	 * The distance to erode/shrink the walkable area of the heightfield away from obstructions. [Limit: >=0] [Units:
	 * vx]
	 **/
	public final int walkableRadius;

	/** The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] **/
	public final int maxEdgeLen;

	/**
	 * The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0]
	 * [Units: vx]
	 **/
	public final float maxSimplificationError;

	/** The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx] **/
	public final int minRegionArea;

	/**
	 * Any regions with a span count smaller than this value will, if possible, be merged with larger regions. [Limit:
	 * >=0] [Units: vx]
	 **/
	public final int mergeRegionArea;

	/**
	 * The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process.
	 * [Limit: >= 3]
	 **/
	public final int maxVertsPerPoly;

	/**
	 * Sets the sampling distance to use when generating the detail mesh. (For height detail only.) [Limits: 0 or >=
	 * 0.9] [Units: wu]
	 **/
	public final float detailSampleDist;

	/**
	 * The maximum distance the detail mesh surface should deviate from heightfield data. (For height detail only.)
	 * [Limit: >=0] [Units: wu]
	 **/
	public final float detailSampleMaxError;

	public RecastConfig(PartitionType partitionType, float cellSize, float cellHeight, float agentHeight, float agentRadius, float agentMaxClimb,
			float agentMaxSlope, int regionMinSize, int regionMergeSize, float edgeMaxLen, float edgeMaxError,
			int vertsPerPoly, float detailSampleDist, float detailSampleMaxError, float[] bmin, float[] bmax) {
		this(partitionType, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, regionMinSize,
				regionMergeSize, edgeMaxLen, edgeMaxError, vertsPerPoly, detailSampleDist, detailSampleMaxError, bmin,
				bmax, 0, 0, 0, false);
	}

	public RecastConfig(PartitionType partitionType, float cellSize, float cellHeight, float agentHeight, float agentRadius, float agentMaxClimb,
			float agentMaxSlope, int regionMinSize, int regionMergeSize, float edgeMaxLen, float edgeMaxError,
			int vertsPerPoly, float detailSampleDist, float detailSampleMaxError, float[] bmin, float[] bmax,
			int tileSize, int tx, int ty) {
		this(partitionType, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, regionMinSize,
				regionMergeSize, edgeMaxLen, edgeMaxError, vertsPerPoly, detailSampleDist, detailSampleMaxError, bmin,
				bmax, tileSize, tx, ty, true);
	}

	private RecastConfig(PartitionType partitionType, float cellSize, float cellHeight, float agentHeight, float agentRadius, float agentMaxClimb,
			float agentMaxSlope, int regionMinSize, int regionMergeSize, float edgeMaxLen, float edgeMaxError,
			int vertsPerPoly, float detailSampleDist, float detailSampleMaxError, float[] bmin, float[] bmax,
			int tileSize, int tx, int ty, boolean tiled) {
		this.partitionType = partitionType;
		this.cs = cellSize;
		this.ch = cellHeight;
		this.walkableSlopeAngle = agentMaxSlope;
		this.walkableHeight = (int) Math.ceil(agentHeight / ch);
		this.walkableClimb = (int) Math.floor(agentMaxClimb / ch);
		this.walkableRadius = (int) Math.ceil(agentRadius / cs);
		this.maxEdgeLen = (int) (edgeMaxLen / cellSize);
		this.maxSimplificationError = edgeMaxError;
		this.minRegionArea = regionMinSize * regionMinSize; // Note: area = size*size
		this.mergeRegionArea = regionMergeSize * regionMergeSize; // Note: area = size*size
		this.maxVertsPerPoly = vertsPerPoly;
		this.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
		this.detailSampleMaxError = cellHeight * detailSampleMaxError;
		this.tileSize = tileSize;
		copy(this.bmin, bmin);
		copy(this.bmax, bmax);
		if (tileSize > 0) {
			float ts = tileSize * cellSize;
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
			this.borderSize = this.walkableRadius + 3; // Reserve enough padding.
			this.bmin[0] -= this.borderSize * this.cs;
			this.bmin[2] -= this.borderSize * this.cs;
			this.bmax[0] += this.borderSize * this.cs;
			this.bmax[2] += this.borderSize * this.cs;
			this.width = this.tileSize + this.borderSize * 2;
			this.height = this.tileSize + this.borderSize * 2;
		} else {
			int[] wh = Recast.calcGridSize(this.bmin, this.bmax, this.cs);
			this.width = wh[0];
			this.height = wh[1];
			this.borderSize = 0;
		}

	}
};
