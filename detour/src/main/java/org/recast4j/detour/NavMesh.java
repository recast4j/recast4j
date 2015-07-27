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
package org.recast4j.detour;

import static org.recast4j.detour.DetourCommon.*;

import java.util.ArrayList;
import java.util.List;

public class NavMesh {

	static int DT_SALT_BITS = 16;
	static int DT_TILE_BITS = 28;
	static int DT_POLY_BITS = 20;

	/** The maximum number of vertices per navigation polygon. */
	public static int DT_VERTS_PER_POLYGON = 6;

	/// A magic number used to detect compatibility of navigation tile data.
	static int DT_NAVMESH_MAGIC = 'D' << 24 | 'N' << 16 | 'A' << 8 | 'V';

	/// A version number used to detect compatibility of navigation tile data.
	static int DT_NAVMESH_VERSION = 7;

	/// A magic number used to detect the compatibility of navigation tile states.
	static int DT_NAVMESH_STATE_MAGIC = 'D' << 24 | 'N' << 16 | 'M' << 8 | 'S';

	/// A version number used to detect compatibility of navigation tile states.
	static int DT_NAVMESH_STATE_VERSION = 1;

	/// A flag that indicates that an entity links to an external entity.
	/// (E.g. A polygon edge is a portal that links to another polygon.)
	static int DT_EXT_LINK = 0x8000;

	/// A value that indicates the entity does not link to anything.
	static int DT_NULL_LINK = 0xffffffff;

	/// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
	static int DT_OFFMESH_CON_BIDIR = 1;

	/// The maximum number of user defined area ids.
	static int DT_MAX_AREAS = 64;

	/// Limit raycasting during any angle pahfinding
	/// The limit is given as a multiple of the character radius
	static float DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0f;

	/// The maximum number of tiles supported by the navigation mesh.
	/// @return The maximum number of tiles supported by the navigation mesh.
	int getMaxTiles() {
		return m_maxTiles;
	}

	MeshTile getTile(int i) {
		return m_tiles[i];
	}

	/// Gets the polygon reference for the tile's base polygon.
	/// @param[in] tile The tile.
	/// @return The polygon reference for the base polygon in the specified tile.
	/// @par
	///
	/// Example use case:
	/// @code
	///
	/// const dtPolyRef base = navmesh->getPolyRefBase(tile);
	/// for (int i = 0; i < tile->header->polyCount; ++i)
	/// {
	/// const dtPoly* p = &tile->polys[i];
	/// const dtPolyRef ref = base | (dtPolyRef)i;
	///
	/// // Use the reference to access the polygon data.
	/// }
	/// @endcode
	public long getPolyRefBase(MeshTile tile) {
		if (tile == null)
			return 0;
		int it = tile.index;
		return encodePolyId(tile.salt, it, 0);
	}

	/// Derives a standard polygon reference.
	/// @note This function is generally meant for internal use only.
	/// @param[in] salt The tile's salt value.
	/// @param[in] it The index of the tile.
	/// @param[in] ip The index of the polygon within the tile.
	static long encodePolyId(int salt, int it, int ip) {
		return (((long) salt) << (DT_POLY_BITS + DT_TILE_BITS)) | ((long) it << DT_POLY_BITS) | (long) ip;
	}

	/// Decodes a standard polygon reference.
	/// @note This function is generally meant for internal use only.
	/// @param[in] ref The polygon reference to decode.
	/// @param[out] salt The tile's salt value.
	/// @param[out] it The index of the tile.
	/// @param[out] ip The index of the polygon within the tile.
	/// @see #encodePolyId
	static int[] decodePolyId(long ref) {
		int salt;
		int it;
		int ip;
		long saltMask = (1L << DT_SALT_BITS) - 1;
		long tileMask = (1L << DT_TILE_BITS) - 1;
		long polyMask = (1L << DT_POLY_BITS) - 1;
		salt = (int) ((ref >> (DT_POLY_BITS + DT_TILE_BITS)) & saltMask);
		it = (int) ((ref >> DT_POLY_BITS) & tileMask);
		ip = (int) (ref & polyMask);
		return new int[] { salt, it, ip };
	}

	/// Extracts a tile's salt value from the specified polygon reference.
	/// @note This function is generally meant for internal use only.
	/// @param[in] ref The polygon reference.
	/// @see #encodePolyId
	static int decodePolyIdSalt(long ref) {
		long saltMask = (1L << DT_SALT_BITS) - 1;
		return (int) ((ref >> (DT_POLY_BITS + DT_TILE_BITS)) & saltMask);
	}

	/// Extracts the tile's index from the specified polygon reference.
	/// @note This function is generally meant for internal use only.
	/// @param[in] ref The polygon reference.
	/// @see #encodePolyId
	static int decodePolyIdTile(long ref) {
		long tileMask = (1L << DT_TILE_BITS) - 1;
		return (int) ((ref >> DT_POLY_BITS) & tileMask);
	}

	/// Extracts the polygon's index (within its tile) from the specified polygon reference.
	/// @note This function is generally meant for internal use only.
	/// @param[in] ref The polygon reference.
	/// @see #encodePolyId
	static int decodePolyIdPoly(long ref) {
		long polyMask = (1L << DT_POLY_BITS) - 1;
		return (int) (ref & polyMask);
	}

	int allocLink(MeshTile tile) {
		Link link = new Link();
		link.next = DT_NULL_LINK;
		tile.links.add(link);
		return tile.links.size() - 1;
	}

	NavMeshParams m_params; /// < Current initialization params. TODO: do not store this info twice.
	private float[] m_orig; /// < Origin of the tile (0,0)
	// float m_orig[3]; ///< Origin of the tile (0,0)
	float m_tileWidth, m_tileHeight; /// < Dimensions of each tile.
	int m_maxTiles; /// < Max number of tiles.
	int m_tileLutSize; /// < Tile hash lookup size (must be pot).
	int m_tileLutMask; /// < Tile hash lookup mask.

	// MeshTile** m_posLookup; ///< Tile hash lookup.
	// MeshTile[] m_nextFree; ///< Freelist of tiles.
	MeshTile[] m_posLookup; /// < Tile hash lookup.
	MeshTile m_nextFree; /// < Freelist of tiles.
	MeshTile[] m_tiles; /// < List of tiles.

	int[] calcTileLoc(float[] pos) {
		int tx = (int) Math.floor((pos[0] - m_orig[0]) / m_tileWidth);
		int ty = (int) Math.floor((pos[2] - m_orig[2]) / m_tileHeight);
		return new int[] { tx, ty };
	}

	public Tupple2<MeshTile, Poly> getTileAndPolyByRef(long ref) {
		if (ref == 0) {
			throw new IllegalArgumentException("ref = 0");
		}
		int[] saltitip = decodePolyId(ref);
		int salt = saltitip[0];
		int it = saltitip[1];
		int ip = saltitip[2];
		if (it >= m_maxTiles)
			throw new IllegalArgumentException("tile > m_maxTiles");
		if (m_tiles[it].salt != salt || m_tiles[it].header == null)
			throw new IllegalArgumentException("Invalid salt or header");
		if (ip >= m_tiles[it].header.polyCount)
			throw new IllegalArgumentException("poly > polyCount");
		return new Tupple2<>(m_tiles[it], m_tiles[it].polys[ip]);
	}

	/// @par
	///
	/// @warning Only use this function if it is known that the provided polygon
	/// reference is valid. This function is faster than #getTileAndPolyByRef, but
	/// it does not validate the reference.
	Tupple2<MeshTile, Poly> getTileAndPolyByRefUnsafe(long ref) {
		int[] saltitip = decodePolyId(ref);
		int it = saltitip[1];
		int ip = saltitip[2];
		return new Tupple2<>(m_tiles[it], m_tiles[it].polys[ip]);
	}

	boolean isValidPolyRef(long ref) {
		if (ref == 0)
			return false;
		int[] saltitip = decodePolyId(ref);
		int salt = saltitip[0];
		int it = saltitip[1];
		int ip = saltitip[2];
		if (it >= m_maxTiles)
			return false;
		if (m_tiles[it].salt != salt || m_tiles[it].header == null)
			return false;
		if (ip >= m_tiles[it].header.polyCount)
			return false;
		return true;
	}

	public void init(NavMeshParams params) {
		this.m_params = params;
		m_orig = params.orig;
		m_tileWidth = params.tileWidth;
		m_tileHeight = params.tileHeight;
		// Init tiles
		m_maxTiles = params.maxTiles;
		m_tileLutSize = nextPow2(params.maxTiles / 4);
		if (m_tileLutSize == 0)
			m_tileLutSize = 1;
		m_tileLutMask = m_tileLutSize - 1;
		m_tiles = new MeshTile[m_maxTiles];
		m_posLookup = new MeshTile[m_tileLutSize];
		m_nextFree = null;
		for (int i = m_maxTiles - 1; i >= 0; --i) {
			m_tiles[i] = new MeshTile(i);
			m_tiles[i].salt = 1;
			m_tiles[i].next = m_nextFree;
			m_nextFree = m_tiles[i];
		}

	}

	public void init(NavMeshData data, int flags) {
		init(getNavMeshParams(data));
		addTile(data, flags, 0);
	}

	private static NavMeshParams getNavMeshParams(NavMeshData data) {
		NavMeshParams params = new NavMeshParams();
		params.orig = data.header.bmin;
		params.tileWidth = data.header.bmax[0] - data.header.bmin[0];
		params.tileHeight = data.header.bmax[2] - data.header.bmin[2];
		params.maxTiles = 1;
		params.maxPolys = data.header.polyCount;
		return params;
	}

	List<Long> queryPolygonsInTile(MeshTile tile, float[] qmin, float[] qmax) {
		List<Long> polys = new ArrayList<>();
		if (tile.bvTree != null) {
			int nodeIndex = 0;
			float[] tbmin = tile.header.bmin;
			float[] tbmax = tile.header.bmax;
			float qfac = tile.header.bvQuantFactor;
			// Calculate quantized box
			int[] bmin = new int[3];
			int[] bmax = new int[3];
			// dtClamp query box to world box.
			float minx = clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
			float miny = clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
			float minz = clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
			float maxx = clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
			float maxy = clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
			float maxz = clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
			// Quantize
			bmin[0] = (int) (qfac * minx) & 0xfffe;
			bmin[1] = (int) (qfac * miny) & 0xfffe;
			bmin[2] = (int) (qfac * minz) & 0xfffe;
			bmax[0] = (int) (qfac * maxx + 1) | 1;
			bmax[1] = (int) (qfac * maxy + 1) | 1;
			bmax[2] = (int) (qfac * maxz + 1) | 1;

			// Traverse tree
			long base = getPolyRefBase(tile);
			int end = tile.header.bvNodeCount;
			while (nodeIndex < end) {
				BVNode node = tile.bvTree[nodeIndex];
				boolean overlap = overlapQuantBounds(bmin, bmax, node.bmin, node.bmax);
				boolean isLeafNode = node.i >= 0;

				if (isLeafNode && overlap) {
					polys.add(base | node.i);
				}

				if (overlap || isLeafNode)
					nodeIndex++;
				else {
					int escapeIndex = -node.i;
					nodeIndex += escapeIndex;
				}
			}

			return polys;
		} else {
			float[] bmin = new float[3];
			float[] bmax = new float[3];
			long base = getPolyRefBase(tile);
			for (int i = 0; i < tile.header.polyCount; ++i) {
				Poly p = tile.polys[i];
				// Do not return off-mesh connection polygons.
				if (p.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					continue;
				// Calc polygon bounds.
				int v = p.verts[0] * 3;
				vCopy(bmin, tile.verts, v);
				vCopy(bmax, tile.verts, v);
				for (int j = 1; j < p.vertCount; ++j) {
					v = p.verts[j] * 3;
					vMin(bmin, tile.verts, v);
					vMax(bmax, tile.verts, v);
				}
				if (overlapBounds(qmin, qmax, bmin, bmax)) {
					polys.add(base | i);
				}
			}
			return polys;
		}
	}

	/// @par
	///
	/// The add operation will fail if the data is in the wrong format, the allocated tile
	/// space is full, or there is a tile already at the specified reference.
	///
	/// The lastRef parameter is used to restore a tile with the same tile
	/// reference it had previously used. In this case the #dtPolyRef's for the
	/// tile will be restored to the same values they were before the tile was
	/// removed.
	///
	/// @see dtCreateNavMeshData, #removeTile
	long addTile(NavMeshData data, int flags, long lastRef) {
		// Make sure the data is in right format.
		MeshHeader header = data.header;

		// Make sure the location is free.
		if (getTileAt(header.x, header.y, header.layer) != null)
			throw new RuntimeException("Tile already exists");

		// Allocate a tile.
		MeshTile tile = null;
		if (lastRef == 0) {
			if (m_nextFree != null) {
				tile = m_nextFree;
				m_nextFree = tile.next;
				tile.next = null;
			}
		} else {
			// Try to relocate the tile to specific index with same salt.
			int tileIndex = (int) decodePolyIdTile(lastRef);
			if (tileIndex >= m_maxTiles)
				throw new RuntimeException("Tile index too high");
			// Try to find the specific tile id from the free list.
			MeshTile target = m_tiles[tileIndex];
			MeshTile prev = null;
			tile = m_nextFree;
			while (tile != null && tile != target) {
				prev = tile;
				tile = tile.next;
			}
			// Could not find the correct location.
			if (tile != target)
				throw new RuntimeException("Could not find tile");
			// Remove from freelist
			if (prev == null)
				m_nextFree = tile.next;
			else
				prev.next = tile.next;

			// Restore salt.
			tile.salt = decodePolyIdSalt(lastRef);
		}

		// Make sure we could allocate a tile.
		if (tile == null)
			throw new RuntimeException("Could not allocate a tile");

		// Insert tile into the position lut.
		int h = computeTileHash(header.x, header.y, m_tileLutMask);
		tile.next = m_posLookup[h];
		m_posLookup[h] = tile;

		// Patch header pointers.

		tile.verts = data.navVerts;
		tile.polys = data.navPolys;
		tile.links = new ArrayList<>();
		tile.detailMeshes = data.navDMeshes;
		tile.detailVerts = data.navDVerts;
		tile.detailTris = data.navDTris;
		tile.bvTree = data.navBvtree;
		tile.offMeshCons = data.offMeshCons;

		// If there are no items in the bvtree, reset the tree pointer.
		if (tile.bvTree != null && tile.bvTree.length == 0)
			tile.bvTree = null;

		// Init tile.
		tile.header = header;
		tile.data = data;
		tile.flags = flags;

		connectIntLinks(tile);
		baseOffMeshLinks(tile);

		// Connect with layers in current tile.
		List<MeshTile> neis = getTilesAt(header.x, header.y);
		for (int j = 0; j < neis.size(); ++j) {
			if (neis.get(j) != tile) {
				connectExtLinks(tile, neis.get(j), -1);
				connectExtLinks(neis.get(j), tile, -1);
			}
			connectExtOffMeshLinks(tile, neis.get(j), -1);
			connectExtOffMeshLinks(neis.get(j), tile, -1);
		}

		// Connect with neighbour tiles.
		for (int i = 0; i < 8; ++i) {
			neis = getNeighbourTilesAt(header.x, header.y, i);
			for (int j = 0; j < neis.size(); ++j) {
				connectExtLinks(tile, neis.get(j), i);
				connectExtLinks(neis.get(j), tile, oppositeTile(i));
				connectExtOffMeshLinks(tile, neis.get(j), i);
				connectExtOffMeshLinks(neis.get(j), tile, oppositeTile(i));
			}
		}

		return getTileRef(tile);
	}

	void connectIntLinks(MeshTile tile) {
		if (tile == null)
			return;

		long base = getPolyRefBase(tile);

		for (int i = 0; i < tile.header.polyCount; ++i) {
			Poly poly = tile.polys[i];
			poly.firstLink = DT_NULL_LINK;

			if (poly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			// Build edge links backwards so that the links will be
			// in the linked list from lowest index to highest.
			for (int j = poly.vertCount - 1; j >= 0; --j) {
				// Skip hard and non-internal edges.
				if (poly.neis[j] == 0 || (poly.neis[j] & DT_EXT_LINK) != 0)
					continue;

				int idx = allocLink(tile);
				Link link = tile.links.get(idx);
				link.ref = base | (poly.neis[j] - 1);
				link.edge = j;
				link.side = 0xff;
				link.bmin = link.bmax = 0;
				// Add to linked list.
				link.next = poly.firstLink;
				poly.firstLink = idx;
			}
		}
	}

	void connectExtLinks(MeshTile tile, MeshTile target, int side) {
		if (tile == null)
			return;

		// Connect border links.
		for (int i = 0; i < tile.header.polyCount; ++i) {
			Poly poly = tile.polys[i];

			// Create new links.
			// unsigned short m = DT_EXT_LINK | (unsigned short)side;

			int nv = poly.vertCount;
			for (int j = 0; j < nv; ++j) {
				// Skip non-portal edges.
				if ((poly.neis[j] & DT_EXT_LINK) == 0)
					continue;

				int dir = (int) (poly.neis[j] & 0xff);
				if (side != -1 && dir != side)
					continue;

				// Create new links
				int va = poly.verts[j] * 3;
				int vb = poly.verts[(j + 1) % nv] * 3;
				Tupple3<long[], float[], Integer> connectedPolys = findConnectingPolys(tile.verts, va, vb, target,
						oppositeTile(dir), 4);
				long[] nei = connectedPolys.first;
				float[] neia = connectedPolys.second;
				int nnei = connectedPolys.third;
				for (int k = 0; k < nnei; ++k) {
					int idx = allocLink(tile);
					Link link = tile.links.get(idx);
					link.ref = nei[k];
					link.edge = j;
					link.side = dir;

					link.next = poly.firstLink;
					poly.firstLink = idx;

					// Compress portal limits to a byte value.
					if (dir == 0 || dir == 4) {
						float tmin = (neia[k * 2 + 0] - tile.verts[va + 2])
								/ (tile.verts[vb + 2] - tile.verts[va + 2]);
						float tmax = (neia[k * 2 + 1] - tile.verts[va + 2])
								/ (tile.verts[vb + 2] - tile.verts[va + 2]);
						if (tmin > tmax) {
							float temp = tmin;
							tmin = tmax;
							tmax = temp;
						}
						link.bmin = (int) (clamp(tmin, 0.0f, 1.0f) * 255.0f);
						link.bmax = (int) (clamp(tmax, 0.0f, 1.0f) * 255.0f);
					} else if (dir == 2 || dir == 6) {
						float tmin = (neia[k * 2 + 0] - tile.verts[va]) / (tile.verts[vb] - tile.verts[va]);
						float tmax = (neia[k * 2 + 1] - tile.verts[va]) / (tile.verts[vb] - tile.verts[va]);
						if (tmin > tmax) {
							float temp = tmin;
							tmin = tmax;
							tmax = temp;
						}
						link.bmin = (int) (clamp(tmin, 0.0f, 1.0f) * 255.0f);
						link.bmax = (int) (clamp(tmax, 0.0f, 1.0f) * 255.0f);
					}
				}
			}
		}
	}

	void connectExtOffMeshLinks(MeshTile tile, MeshTile target, int side) {
		if (tile == null)
			return;

		// Connect off-mesh links.
		// We are interested on links which land from target tile to this tile.
		int oppositeSide = (side == -1) ? 0xff : oppositeTile(side);

		for (int i = 0; i < target.header.offMeshConCount; ++i) {
			OffMeshConnection targetCon = target.offMeshCons[i];
			if (targetCon.side != oppositeSide)
				continue;

			Poly targetPoly = target.polys[targetCon.poly];
			// Skip off-mesh connections which start location could not be connected at all.
			if (targetPoly.firstLink == DT_NULL_LINK)
				continue;

			float[] ext = new float[] { targetCon.rad, target.header.walkableClimb, targetCon.rad };

			// Find polygon to connect to.
			float[] p = new float[3];
			p[0] = targetCon.pos[3];
			p[1] = targetCon.pos[4];
			p[2] = targetCon.pos[5];
			Tupple2<Long, float[]> nearest = findNearestPolyInTile(tile, p, ext);
			long ref = nearest.first;
			if (ref == 0)
				continue;
			float[] nearestPt = nearest.second;
			// findNearestPoly may return too optimistic results, further check to make sure.

			if (sqr(nearestPt[0] - p[0]) + sqr(nearestPt[2] - p[2]) > sqr(targetCon.rad))
				continue;
			// Make sure the location is on current mesh.
			target.verts[targetPoly.verts[1] * 3] = nearestPt[0];
			target.verts[targetPoly.verts[1] * 3 + 1] = nearestPt[1];
			target.verts[targetPoly.verts[1] * 3 + 2] = nearestPt[2];

			// Link off-mesh connection to target poly.
			int idx = allocLink(target);
			Link link = target.links.get(idx);
			link.ref = ref;
			link.edge = 1;
			link.side = oppositeSide;
			link.bmin = link.bmax = 0;
			// Add to linked list.
			link.next = targetPoly.firstLink;
			targetPoly.firstLink = idx;

			// Link target poly to off-mesh connection.
			if ((targetCon.flags & DT_OFFMESH_CON_BIDIR) != 0) {
				int tidx = allocLink(tile);
				int landPolyIdx = decodePolyIdPoly(ref);
				Poly landPoly = tile.polys[landPolyIdx];
				link = tile.links.get(tidx);
				link.ref = getPolyRefBase(target) | (targetCon.poly);
				link.edge = 0xff;
				link.side = (side == -1 ? 0xff : side);
				link.bmin = link.bmax = 0;
				// Add to linked list.
				link.next = landPoly.firstLink;
				landPoly.firstLink = tidx;
			}
		}
	}

	Tupple3<long[], float[], Integer> findConnectingPolys(float[] verts, int va, int vb, MeshTile tile, int side,
			int maxcon) {
		if (tile == null)
			return new Tupple3<>(null, null, 0);
		long[] con = new long[maxcon];
		float[] conarea = new float[maxcon * 2];
		float[] amin = new float[2];
		float[] amax = new float[2];
		calcSlabEndPoints(verts, va, vb, amin, amax, side);
		float apos = getSlabCoord(verts, va, side);

		// Remove links pointing to 'side' and compact the links array.
		float[] bmin = new float[2];
		float[] bmax = new float[2];
		int m = DT_EXT_LINK | side;
		int n = 0;
		long base = getPolyRefBase(tile);

		for (int i = 0; i < tile.header.polyCount; ++i) {
			Poly poly = tile.polys[i];
			int nv = poly.vertCount;
			for (int j = 0; j < nv; ++j) {
				// Skip edges which do not point to the right side.
				if (poly.neis[j] != m)
					continue;
				int vc = poly.verts[j] * 3;
				int vd = poly.verts[(j + 1) % nv] * 3;
				float bpos = getSlabCoord(tile.verts, vc, side);
				// Segments are not close enough.
				if (Math.abs(apos - bpos) > 0.01f)
					continue;

				// Check if the segments touch.
				calcSlabEndPoints(tile.verts, vc, vd, bmin, bmax, side);

				if (!overlapSlabs(amin, amax, bmin, bmax, 0.01f, tile.header.walkableClimb))
					continue;

				// Add return value.
				if (n < maxcon) {
					conarea[n * 2 + 0] = Math.max(amin[0], bmin[0]);
					conarea[n * 2 + 1] = Math.min(amax[0], bmax[0]);
					con[n] = base | i;
					n++;
				}
				break;
			}
		}
		return new Tupple3<long[], float[], Integer>(con, conarea, n);
	}

	static float getSlabCoord(float[] verts, int va, int side) {
		if (side == 0 || side == 4)
			return verts[va];
		else if (side == 2 || side == 6)
			return verts[va + 2];
		return 0;
	}

	static void calcSlabEndPoints(float[] verts, int va, int vb, float[] bmin, float[] bmax, int side) {
		if (side == 0 || side == 4) {
			if (verts[va + 2] < verts[vb + 2]) {
				bmin[0] = verts[va + 2];
				bmin[1] = verts[va + 1];
				bmax[0] = verts[vb + 2];
				bmax[1] = verts[vb + 1];
			} else {
				bmin[0] = verts[vb + 2];
				bmin[1] = verts[vb + 1];
				bmax[0] = verts[va + 2];
				bmax[1] = verts[va + 1];
			}
		} else if (side == 2 || side == 6) {
			if (verts[va + 0] < verts[vb + 0]) {
				bmin[0] = verts[va + 0];
				bmin[1] = verts[va + 1];
				bmax[0] = verts[vb + 0];
				bmax[1] = verts[vb + 1];
			} else {
				bmin[0] = verts[vb + 0];
				bmin[1] = verts[vb + 1];
				bmax[0] = verts[va + 0];
				bmax[1] = verts[va + 1];
			}
		}
	}

	boolean overlapSlabs(float[] amin, float[] amax, float[] bmin, float[] bmax, float px, float py) {
		// Check for horizontal overlap.
		// The segment is shrunken a little so that slabs which touch
		// at end points are not connected.
		float minx = Math.max(amin[0] + px, bmin[0] + px);
		float maxx = Math.min(amax[0] - px, bmax[0] - px);
		if (minx > maxx)
			return false;

		// Check vertical overlap.
		float ad = (amax[1] - amin[1]) / (amax[0] - amin[0]);
		float ak = amin[1] - ad * amin[0];
		float bd = (bmax[1] - bmin[1]) / (bmax[0] - bmin[0]);
		float bk = bmin[1] - bd * bmin[0];
		float aminy = ad * minx + ak;
		float amaxy = ad * maxx + ak;
		float bminy = bd * minx + bk;
		float bmaxy = bd * maxx + bk;
		float dmin = bminy - aminy;
		float dmax = bmaxy - amaxy;

		// Crossing segments always overlap.
		if (dmin * dmax < 0)
			return true;

		// Check for overlap at endpoints.
		float thr = (py * 2) * (py * 2);
		if (dmin * dmin <= thr || dmax * dmax <= thr)
			return true;

		return false;
	}

	void baseOffMeshLinks(MeshTile tile) {
		if (tile == null)
			return;

		long base = getPolyRefBase(tile);

		// Base off-mesh connection start points.
		for (int i = 0; i < tile.header.offMeshConCount; ++i) {
			OffMeshConnection con = tile.offMeshCons[i];
			Poly poly = tile.polys[con.poly];

			float[] ext = new float[] { con.rad, tile.header.walkableClimb, con.rad };

			// Find polygon to connect to.
			Tupple2<Long, float[]> nearestPoly = findNearestPolyInTile(tile, con.pos, ext);
			long ref = nearestPoly.first;
			if (ref == 0)
				continue;
			float[] p = con.pos; // First vertex
			float[] nearestPt = nearestPoly.second;
			// findNearestPoly may return too optimistic results, further check to make sure.
			float dx = nearestPt[0] - p[0];
			float dz = nearestPt[2] - p[2];
			float dr = con.rad;
			if (dx * dx + dz * dz > dr * dr)
				continue;
			// Make sure the location is on current mesh.
			System.arraycopy(nearestPoly, 0, tile.verts, poly.verts[0] * 3, 3);

			// Link off-mesh connection to target poly.
			int idx = allocLink(tile);
			Link link = tile.links.get(idx);
			link.ref = ref;
			link.edge = 0;
			link.side = 0xff;
			link.bmin = link.bmax = 0;
			// Add to linked list.
			link.next = poly.firstLink;
			poly.firstLink = idx;

			// Start end-point is always connect back to off-mesh connection.
			int tidx = allocLink(tile);
			int landPolyIdx = decodePolyIdPoly(ref);
			Poly landPoly = tile.polys[landPolyIdx];
			link = tile.links.get(tidx);
			link.ref = base | (con.poly);
			link.edge = 0xff;
			link.side = 0xff;
			link.bmin = link.bmax = 0;
			// Add to linked list.
			link.next = landPoly.firstLink;
			landPoly.firstLink = tidx;
		}
	}

	Tupple2<Boolean, float[]> closestPointOnPoly(long ref, float[] pos) {
		Tupple2<MeshTile, Poly> tileAndPoly = getTileAndPolyByRefUnsafe(ref);
		MeshTile tile = tileAndPoly.first;
		Poly poly = tileAndPoly.second;
		// Off-mesh connections don't have detail polygons.
		if (poly.getType() == Poly.DT_POLYTYPE_OFFMESH_CONNECTION) {
			int v0 = poly.verts[0] * 3;
			int v1 = poly.verts[1] * 3;
			float d0 = vDist(pos, tile.verts, v0);
			float d1 = vDist(pos, tile.verts, v1);
			float u = d0 / (d0 + d1);
			float[] closest = vLerp(tile.verts, v0, v1, u);
			return new Tupple2<>(false, closest);
		}

		int ip = poly.index;
		PolyDetail pd = tile.detailMeshes[ip];

		// Clamp point to be inside the polygon.
		float[] verts = new float[DT_VERTS_PER_POLYGON * 3];
		float[] edged = new float[DT_VERTS_PER_POLYGON];
		float[] edget = new float[DT_VERTS_PER_POLYGON];
		int nv = poly.vertCount;
		for (int i = 0; i < nv; ++i)
			System.arraycopy(tile.verts, poly.verts[i] * 3, verts, i * 3, 3);

		boolean posOverPoly = false;
		float[] closest = new float[3];
		vCopy(closest, pos);
		if (!distancePtPolyEdgesSqr(pos, verts, nv, edged, edget)) {
			// Point is outside the polygon, dtClamp to nearest edge.
			float dmin = Float.MAX_VALUE;
			int imin = -1;
			for (int i = 0; i < nv; ++i) {
				if (edged[i] < dmin) {
					dmin = edged[i];
					imin = i;
				}
			}
			int va = imin * 3;
			int vb = ((imin + 1) % nv) * 3;
			closest = vLerp(verts, va, vb, edget[imin]);
			posOverPoly = false;
		} else {
			posOverPoly = true;
		}

		// Find height at the location.
		VectorPtr posV = new VectorPtr(pos);
		for (int j = 0; j < pd.triCount; ++j) {
			int t = (pd.triBase + j) * 4;
			VectorPtr[] v = new VectorPtr[3];
			for (int k = 0; k < 3; ++k) {
				if (tile.detailTris[t + k] < poly.vertCount)
					v[k] = new VectorPtr(tile.verts, poly.verts[tile.detailTris[t + k]] * 3);
				else
					v[k] = new VectorPtr(tile.detailVerts,
							(pd.vertBase + (tile.detailTris[t + k] - poly.vertCount)) * 3);
			}
			Tupple2<Boolean, Float> clp = closestHeightPointTriangle(posV, v[0], v[1], v[2]);
			if (clp.first) {
				closest[1] = clp.second;
				break;
			}
		}
		return new Tupple2<>(posOverPoly, closest);
	}

	Tupple2<Long, float[]> findNearestPolyInTile(MeshTile tile, float[] center, float[] extents) {
		float[] nearestPt = null;
		float[] bmin = vSub(center, extents);
		float[] bmax = vAdd(center, extents);

		// Get nearby polygons from proximity grid.
		List<Long> polys = queryPolygonsInTile(tile, bmin, bmax);

		// Find nearest polygon amongst the nearby polygons.
		long nearest = 0;
		float nearestDistanceSqr = Float.MAX_VALUE;
		for (int i = 0; i < polys.size(); ++i) {
			long ref = polys.get(i);
			float d;
			Tupple2<Boolean, float[]> cpp = closestPointOnPoly(ref, center);
			boolean posOverPoly = cpp.first;
			float[] closestPtPoly = cpp.second;

			// If a point is directly over a polygon and closer than
			// climb height, favor that instead of straight line nearest point.
			float[] diff = vSub(center, closestPtPoly);
			if (posOverPoly) {
				d = Math.abs(diff[1]) - tile.header.walkableClimb;
				d = d > 0 ? d * d : 0;
			} else {
				d = vLenSqr(diff);
			}
			if (d < nearestDistanceSqr) {
				nearestPt = closestPtPoly;
				nearestDistanceSqr = d;
				nearest = ref;
			}
		}
		return new Tupple2<>(nearest, nearestPt);
	}

	MeshTile getTileAt(int x, int y, int layer) {
		// Find tile based on hash.
		int h = computeTileHash(x, y, m_tileLutMask);
		MeshTile tile = m_posLookup[h];
		while (tile != null) {
			if (tile.header != null && tile.header.x == x && tile.header.y == y && tile.header.layer == layer) {
				return tile;
			}
			tile = tile.next;
		}
		return null;
	}

	List<MeshTile> getNeighbourTilesAt(int x, int y, int side) {
		int nx = x, ny = y;
		switch (side) {
		case 0:
			nx++;
			break;
		case 1:
			nx++;
			ny++;
			break;
		case 2:
			ny++;
			break;
		case 3:
			nx--;
			ny++;
			break;
		case 4:
			nx--;
			break;
		case 5:
			nx--;
			ny--;
			break;
		case 6:
			ny--;
			break;
		case 7:
			nx++;
			ny--;
			break;
		}
		return getTilesAt(nx, ny);
	}

	List<MeshTile> getTilesAt(int x, int y) {
		List<MeshTile> tiles = new ArrayList<>();
		// Find tile based on hash.
		int h = computeTileHash(x, y, m_tileLutMask);
		MeshTile tile = m_posLookup[h];
		while (tile != null) {
			if (tile.header != null && tile.header.x == x && tile.header.y == y) {
				tiles.add(tile);
			}
			tile = tile.next;
		}
		return tiles;
	}

	MeshTile getTileByRef(long ref) {
		if (ref == 0)
			return null;
		int tileIndex = decodePolyIdTile(ref);
		int tileSalt = decodePolyIdSalt(ref);
		if ((int) tileIndex >= m_maxTiles)
			return null;
		MeshTile tile = m_tiles[tileIndex];
		if (tile.salt != tileSalt)
			return null;
		return tile;
	}

	long getTileRef(MeshTile tile) {
		if (tile == null)
			return 0;
		int it = tile.index;
		return encodePolyId(tile.salt, it, 0);
	}

	static int computeTileHash(int x, int y, int mask) {
		int h1 = 0x8da6b343; // Large multiplicative constants;
		int h2 = 0xd8163841; // here arbitrarily chosen primes
		int n = h1 * x + h2 * y;
		return (int) (n & mask);
	}

}
