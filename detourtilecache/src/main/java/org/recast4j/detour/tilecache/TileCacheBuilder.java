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
package org.recast4j.detour.tilecache;

import static org.recast4j.detour.DetourCommon.sqr;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import org.recast4j.detour.Tupple2;
import org.recast4j.detour.tilecache.io.TileCacheLayerHeaderReader;
import org.recast4j.detour.tilecache.io.TileCacheLayerHeaderWriter;

public class TileCacheBuilder {

	static final int DT_TILECACHE_NULL_AREA = 0;
	static final int DT_TILECACHE_WALKABLE_AREA = 63;
	static final int DT_TILECACHE_NULL_IDX = 0xffff;

	class LayerSweepSpan {
		int ns; // number samples
		int id; // region id
		int nei; // neighbour id
	};

	static final int DT_LAYER_MAX_NEIS = 16;

	class LayerMonotoneRegion {
		int area;
		int[] neis = new int[DT_LAYER_MAX_NEIS];
		int nneis;
		int regId;
		int areaId;
	};

	class TempContour {
		int[] verts;
		int nverts;
		int cverts;
		int[] poly;
		int npoly;
		int cpoly;

		// FIXME: (PP) switch to List
		TempContour(int[] vbuf, int nvbuf, int[] pbuf, int npbuf) {

			verts = vbuf;
			nverts = 0;
			cverts = nvbuf;
			poly = pbuf;
			npoly = 0;
			cpoly = npbuf;
		}

	};

	class Edge {
		int[] vert = new int[2];
		int[] polyEdge = new int[2];
		int[] poly = new int[2];
	};

	TileCacheLayerHeaderReader reader;

	void buildTileCacheRegions(TileCacheLayer layer, int walkableClimb) {

		int w = layer.header.width;
		int h = layer.header.height;

		Arrays.fill(layer.regs, (byte) 0xFF);
		int nsweeps = w;
		LayerSweepSpan[] sweeps = new LayerSweepSpan[nsweeps];
		for (int i = 0; i < sweeps.length; i++) {
			sweeps[i] = new LayerSweepSpan();
		}
		// Partition walkable area into monotone regions.
		int[] prevCount = new int[256];
		int regId = 0;

		for (int y = 0; y < h; ++y) {
			if (regId > 0) {
				Arrays.fill(prevCount, 0, regId, 0);
			}
			// memset(prevCount,0,sizeof(char)*regId);
			int sweepId = 0;

			for (int x = 0; x < w; ++x) {
				int idx = x + y * w;
				if (layer.areas[idx] == DT_TILECACHE_NULL_AREA)
					continue;

				int sid = 0xff;

				// -x
				int xidx = (x - 1) + y * w;
				if (x > 0 && isConnected(layer, idx, xidx, walkableClimb)) {
					if (layer.regs[xidx] != 0xff)
						sid = layer.regs[xidx];
				}

				if (sid == 0xff) {
					sid = sweepId++;
					sweeps[sid].nei = 0xff;
					sweeps[sid].ns = 0;
				}

				// -y
				int yidx = x + (y - 1) * w;
				if (y > 0 && isConnected(layer, idx, yidx, walkableClimb)) {
					int nr = layer.regs[yidx];
					if (nr != 0xff) {
						// Set neighbour when first valid neighbour is
						// encoutered.
						if (sweeps[sid].ns == 0)
							sweeps[sid].nei = nr;

						if (sweeps[sid].nei == nr) {
							// Update existing neighbour
							sweeps[sid].ns++;
							prevCount[nr]++;
						} else {
							// This is hit if there is nore than one neighbour.
							// Invalidate the neighbour.
							sweeps[sid].nei = 0xff;
						}
					}
				}

				layer.regs[idx] = (byte) sid;
			}

			// Create unique ID.
			for (int i = 0; i < sweepId; ++i) {
				// If the neighbour is set and there is only one continuous
				// connection to it,
				// the sweep will be merged with the previous one, else new
				// region is created.
				if (sweeps[i].nei != 0xff && prevCount[sweeps[i].nei] == sweeps[i].ns) {
					sweeps[i].id = sweeps[i].nei;
				} else {
					if (regId == 255) {
						// Region ID's overflow.
						throw new RuntimeException("Buffer too small");
					}
					sweeps[i].id = regId++;
				}
			}

			// Remap local sweep ids to region ids.
			for (int x = 0; x < w; ++x) {
				int idx = x + y * w;
				if (layer.regs[idx] != 0xff)
					layer.regs[idx] = (byte) sweeps[layer.regs[idx]].id;
			}
		}

		// Allocate and init layer regions.
		int nregs = regId;
		LayerMonotoneRegion[] regs = new LayerMonotoneRegion[nregs];

		for (int i = 0; i < nregs; ++i) {
			regs[i] = new LayerMonotoneRegion();
			regs[i].regId = 0xff;
		}

		// Find region neighbours.
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				int idx = x + y * w;
				int ri = layer.regs[idx];
				if (ri == 0xff)
					continue;

				// Update area.
				regs[ri].area++;
				regs[ri].areaId = layer.areas[idx];

				// Update neighbours
				int ymi = x + (y - 1) * w;
				if (y > 0 && isConnected(layer, idx, ymi, walkableClimb)) {
					int rai = layer.regs[ymi];
					if (rai != 0xff && rai != ri) {
						regs[ri].nneis = addUniqueLast(regs[ri].neis, regs[ri].nneis, rai);
						regs[rai].nneis = addUniqueLast(regs[rai].neis, regs[rai].nneis, ri);
					}
				}
			}
		}

		for (int i = 0; i < nregs; ++i)
			regs[i].regId = i;

		for (int i = 0; i < nregs; ++i) {
			LayerMonotoneRegion reg = regs[i];

			int merge = -1;
			int mergea = 0;
			for (int j = 0; j < reg.nneis; ++j) {
				int nei = reg.neis[j];
				LayerMonotoneRegion regn = regs[nei];
				if (reg.regId == regn.regId)
					continue;
				if (reg.areaId != regn.areaId)
					continue;
				if (regn.area > mergea) {
					if (canMerge(reg.regId, regn.regId, regs, nregs)) {
						mergea = regn.area;
						merge = nei;
					}
				}
			}
			if (merge != -1) {
				int oldId = reg.regId;
				int newId = regs[merge].regId;
				for (int j = 0; j < nregs; ++j)
					if (regs[j].regId == oldId)
						regs[j].regId = newId;
			}
		}

		// Compact ids.
		int[] remap = new int[256];
		// Find number of unique regions.
		regId = 0;
		for (int i = 0; i < nregs; ++i)
			remap[regs[i].regId] = 1;
		for (int i = 0; i < 256; ++i)
			if (remap[i] != 0)
				remap[i] = regId++;
		// Remap ids.
		for (int i = 0; i < nregs; ++i)
			regs[i].regId = remap[regs[i].regId];

		layer.regCount = regId;

		for (int i = 0; i < w * h; ++i) {
			if (layer.regs[i] != 0xff)
				layer.regs[i] = (byte) regs[layer.regs[i]].regId;
		}

	}

	int addUniqueLast(int[] a, int an, int v) {
		int n = an;
		if (n > 0 && a[n - 1] == v)
			return an;
		a[an] = v;
		an++;
		return an;
	}

	boolean isConnected(TileCacheLayer layer, int ia, int ib, int walkableClimb) {
		if (layer.areas[ia] != layer.areas[ib])
			return false;
		if (Math.abs(layer.heights[ia] - layer.heights[ib]) > walkableClimb)
			return false;
		return true;
	}

	boolean canMerge(int oldRegId, int newRegId, LayerMonotoneRegion[] regs, int nregs) {
		int count = 0;
		for (int i = 0; i < nregs; ++i) {
			LayerMonotoneRegion reg = regs[i];
			if (reg.regId != oldRegId)
				continue;
			int nnei = reg.nneis;
			for (int j = 0; j < nnei; ++j) {
				if (regs[reg.neis[j]].regId == newRegId)
					count++;
			}
		}
		return count == 1;
	}

	private boolean appendVertex(TempContour cont, int x, int y, int z, int r) {
		// Try to merge with existing segments.
		if (cont.nverts > 1) {
			int pa = (cont.nverts - 2) * 4;
			int pb = (cont.nverts - 1) * 4;
			if (cont.verts[pb + 3] == r) {
				if (cont.verts[pa] == cont.verts[pb] && cont.verts[pb] == x) {
					// The verts are aligned aling x-axis, update z.
					cont.verts[pb + 1] = y;
					cont.verts[pb + 2] = z;
					return true;
				} else if (cont.verts[pa + 2] == cont.verts[pb + 2] && cont.verts[pb + 2] == z) {
					// The verts are aligned aling z-axis, update x.
					cont.verts[pb] = x;
					cont.verts[pb + 1] = y;
					return true;
				}
			}
		}

		// Add new point.
		if (cont.nverts + 1 > cont.cverts)
			return false;

		cont.verts[cont.nverts * 4] = x;
		cont.verts[cont.nverts * 4 + 1] = y;
		cont.verts[cont.nverts * 4 + 2] = z;
		cont.verts[cont.nverts * 4 + 3] = r;
		cont.nverts++;

		return true;
	}

	private int getNeighbourReg(TileCacheLayer layer, int ax, int ay, int dir) {
		int w = layer.header.width;
		int ia = ax + ay * w;

		int con = layer.cons[ia] & 0xf;
		int portal = layer.cons[ia] >> 4;
		int mask = 1 << dir;

		if ((con & mask) == 0) {
			// No connection, return portal or hard edge.
			if ((portal & mask) != 0)
				return 0xf8 + dir;
			return 0xff;
		}

		int bx = ax + getDirOffsetX(dir);
		int by = ay + getDirOffsetY(dir);
		int ib = bx + by * w;

		return layer.regs[ib];
	}

	private int getDirOffsetX(int dir) {
		int[] offset = new int[] { -1, 0, 1, 0, };
		return offset[dir & 0x03];
	}

	private int getDirOffsetY(int dir) {
		int[] offset = new int[] { 0, 1, 0, -1 };
		return offset[dir & 0x03];
	}

	private boolean walkContour(TileCacheLayer layer, int x, int y, TempContour cont) {
		int w = layer.header.width;
		int h = layer.header.height;

		cont.nverts = 0;

		int startX = x;
		int startY = y;
		int startDir = -1;

		for (int i = 0; i < 4; ++i) {
			int dir = (i + 3) & 3;
			int rn = getNeighbourReg(layer, x, y, dir);
			if (rn != layer.regs[x + y * w]) {
				startDir = dir;
				break;
			}
		}
		if (startDir == -1)
			return true;

		int dir = startDir;
		int maxIter = w * h;

		int iter = 0;
		while (iter < maxIter) {
			int rn = getNeighbourReg(layer, x, y, dir);

			int nx = x;
			int ny = y;
			int ndir = dir;

			if (rn != layer.regs[x + y * w]) {
				// Solid edge.
				int px = x;
				int pz = y;
				switch (dir) {
				case 0:
					pz++;
					break;
				case 1:
					px++;
					pz++;
					break;
				case 2:
					px++;
					break;
				}

				// Try to merge with previous vertex.
				if (!appendVertex(cont, px, layer.heights[x + y * w], pz, rn))
					return false;

				ndir = (dir + 1) & 0x3; // Rotate CW
			} else {
				// Move to next.
				nx = x + getDirOffsetX(dir);
				ny = y + getDirOffsetY(dir);
				ndir = (dir + 3) & 0x3; // Rotate CCW
			}

			if (iter > 0 && x == startX && y == startY && dir == startDir)
				break;

			x = nx;
			y = ny;
			dir = ndir;

			iter++;
		}

		// Remove last vertex if it is duplicate of the first one.
		int pa = (cont.nverts - 1) * 4;
		int pb = 0;
		if (cont.verts[pa] == cont.verts[pb] && cont.verts[pa + 2] == cont.verts[pb + 2])
			cont.nverts--;

		return true;
	}

	private float distancePtSeg(int x, int z, int px, int pz, int qx, int qz) {
		float pqx = qx - px;
		float pqz = qz - pz;
		float dx = x - px;
		float dz = z - pz;
		float d = pqx * pqx + pqz * pqz;
		float t = pqx * dx + pqz * dz;
		if (d > 0)
			t /= d;
		if (t < 0)
			t = 0;
		else if (t > 1)
			t = 1;

		dx = px + t * pqx - x;
		dz = pz + t * pqz - z;

		return dx * dx + dz * dz;
	}

	private void simplifyContour(TempContour cont, float maxError) {
		cont.npoly = 0;

		for (int i = 0; i < cont.nverts; ++i) {
			int j = (i + 1) % cont.nverts;
			// Check for start of a wall segment.
			int ra = j * 4 + 3;
			int rb = i * 4 + 3;
			if (cont.verts[ra] != cont.verts[rb])
				cont.poly[cont.npoly++] = (short) i;
		}
		if (cont.npoly < 2) {
			// If there is no transitions at all,
			// create some initial points for the simplification process.
			// Find lower-left and upper-right vertices of the contour.
			int llx = cont.verts[0];
			int llz = cont.verts[2];
			int lli = 0;
			int urx = cont.verts[0];
			int urz = cont.verts[2];
			int uri = 0;
			for (int i = 1; i < cont.nverts; ++i) {
				int x = cont.verts[i * 4 + 0];
				int z = cont.verts[i * 4 + 2];
				if (x < llx || (x == llx && z < llz)) {
					llx = x;
					llz = z;
					lli = i;
				}
				if (x > urx || (x == urx && z > urz)) {
					urx = x;
					urz = z;
					uri = i;
				}
			}
			cont.npoly = 0;
			cont.poly[cont.npoly++] = lli;
			cont.poly[cont.npoly++] = uri;
		}

		// Add points until all raw points are within
		// error tolerance to the simplified shape.
		for (int i = 0; i < cont.npoly;) {
			int ii = (i + 1) % cont.npoly;

			int ai = cont.poly[i];
			int ax = cont.verts[ai * 4 + 0];
			int az = cont.verts[ai * 4 + 2];

			int bi = cont.poly[ii];
			int bx = cont.verts[bi * 4 + 0];
			int bz = cont.verts[bi * 4 + 2];

			// Find maximum deviation from the segment.
			float maxd = 0;
			int maxi = -1;
			int ci, cinc, endi;

			// Traverse the segment in lexilogical order so that the
			// max deviation is calculated similarly when traversing
			// opposite segments.
			if (bx > ax || (bx == ax && bz > az)) {
				cinc = 1;
				ci = (ai + cinc) % cont.nverts;
				endi = bi;
			} else {
				cinc = cont.nverts - 1;
				ci = (bi + cinc) % cont.nverts;
				endi = ai;
			}

			// Tessellate only outer edges or edges between areas.
			while (ci != endi) {
				float d = distancePtSeg(cont.verts[ci * 4 + 0], cont.verts[ci * 4 + 2], ax, az, bx, bz);
				if (d > maxd) {
					maxd = d;
					maxi = ci;
				}
				ci = (ci + cinc) % cont.nverts;
			}

			// If the max deviation is larger than accepted error,
			// add new point, else continue to next segment.
			if (maxi != -1 && maxd > (maxError * maxError)) {
				cont.npoly++;
				for (int j = cont.npoly - 1; j > i; --j)
					cont.poly[j] = cont.poly[j - 1];
				cont.poly[i + 1] = maxi;
			} else {
				++i;
			}
		}

		// Remap vertices
		int start = 0;
		for (int i = 1; i < cont.npoly; ++i)
			if (cont.poly[i] < cont.poly[start])
				start = i;

		cont.nverts = 0;
		for (int i = 0; i < cont.npoly; ++i) {
			int j = (start + i) % cont.npoly;
			int src = cont.poly[j] * 4;
			int dst = cont.nverts * 4;
			cont.verts[dst] = cont.verts[src];
			cont.verts[dst + 1] = cont.verts[src + 1];
			cont.verts[dst + 2] = cont.verts[src + 2];
			cont.verts[dst + 3] = cont.verts[src + 3];
			cont.nverts++;
		}
	}

	static Tupple2<Integer, Boolean> getCornerHeight(TileCacheLayer layer, int x, int y, int z, int walkableClimb) {
		int w = layer.header.width;
		int h = layer.header.height;

		int n = 0;

		int portal = 0xf;
		int height = 0;
		int preg = 0xff;
		boolean allSameReg = true;

		for (int dz = -1; dz <= 0; ++dz) {
			for (int dx = -1; dx <= 0; ++dx) {
				int px = x + dx;
				int pz = z + dz;
				if (px >= 0 && pz >= 0 && px < w && pz < h) {
					int idx = px + pz * w;
					int lh = layer.heights[idx];
					if (Math.abs(lh - y) <= walkableClimb && layer.areas[idx] != DT_TILECACHE_NULL_AREA) {
						height = Math.max(height, (char) lh);
						portal &= (layer.cons[idx] >> 4);
						if (preg != 0xff && preg != layer.regs[idx])
							allSameReg = false;
						preg = layer.regs[idx];
						n++;
					}
				}
			}
		}

		int portalCount = 0;
		for (int dir = 0; dir < 4; ++dir)
			if ((portal & (1 << dir)) != 0)
				portalCount++;

		boolean shouldRemove = false;
		if (n > 1 && portalCount == 1 && allSameReg) {
			shouldRemove = true;
		}

		return new Tupple2<Integer, Boolean>(height, shouldRemove);
	}

	// TODO: move this somewhere else, once the layer meshing is done.
	TileCacheContourSet buildTileCacheContours(TileCacheLayer layer, int walkableClimb, float maxError) {
		int w = layer.header.width;
		int h = layer.header.height;

		TileCacheContourSet lcset = new TileCacheContourSet();
		lcset.nconts = layer.regCount;
		lcset.conts = new TileCacheContour[lcset.nconts];
		for (int i = 0; i < lcset.nconts; i++) {
			lcset.conts[i] = new TileCacheContour();
		}

		// Allocate temp buffer for contour tracing.
		int maxTempVerts = (w + h) * 2 * 2; // Twice around the layer.
		int[] tempVerts = new int[maxTempVerts * 4];
		int[] tempPoly = new int[maxTempVerts];
		TempContour temp = new TempContour(tempVerts, maxTempVerts, tempPoly, maxTempVerts);

		// Find contours.
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				int idx = x + y * w;
				int ri = layer.regs[idx];
				if (ri == 0xff)
					continue;

				TileCacheContour cont = lcset.conts[ri];

				if (cont.nverts > 0)
					continue;

				cont.reg = ri;
				cont.area = layer.areas[idx];

				if (!walkContour(layer, x, y, temp)) {
					// Too complex contour.
					// Note: If you hit here ofte, try increasing
					// 'maxTempVerts'.
					throw new RuntimeException("Too complex contour"); // FIXME:
																		// (PP)
																		// use
																		// List
																		// instead
				}

				simplifyContour(temp, maxError);

				// Store contour.
				cont.nverts = temp.nverts;
				if (cont.nverts > 0) {
					cont.verts = new int[4 * temp.nverts];

					for (int i = 0, j = temp.nverts - 1; i < temp.nverts; j = i++) {
						int dst = j * 4;
						int v = j * 4;
						int vn = i * 4;
						int nei = temp.verts[vn + 3]; // The neighbour reg is
														// stored at segment
														// vertex of a segment.
						Tupple2<Integer, Boolean> res = getCornerHeight(layer, temp.verts[v], temp.verts[v + 1],
								temp.verts[v + 2], walkableClimb);
						int lh = res.first;
						boolean shouldRemove = res.second;
						cont.verts[dst + 0] = temp.verts[v];
						cont.verts[dst + 1] = lh;
						cont.verts[dst + 2] = temp.verts[v + 2];

						// Store portal direction and remove status to the
						// fourth component.
						cont.verts[dst + 3] = 0x0f;
						if (nei != 0xff && nei >= 0xf8)
							cont.verts[dst + 3] = nei - 0xf8;
						if (shouldRemove)
							cont.verts[dst + 3] |= 0x80;
					}
				}
			}
		}
		return lcset;

	}

	static final int VERTEX_BUCKET_COUNT2 = (1 << 8);

	private int computeVertexHash2(int x, int y, int z) {
		int h1 = 0x8da6b343; // Large multiplicative constants;
		int h2 = 0xd8163841; // here arbitrarily chosen primes
		int h3 = 0xcb1ab31f;
		int n = h1 * x + h2 * y + h3 * z;
		return n & (VERTEX_BUCKET_COUNT2 - 1);
	}

	private int addVertex(int x, int y, int z, int[] verts, int[] firstVert, int[] nextVert, int nv) {
		int bucket = computeVertexHash2(x, 0, z);
		int i = firstVert[bucket];
		while (i != DT_TILECACHE_NULL_IDX) {
			int v = i * 3;
			if (verts[v] == x && verts[v + 2] == z && (Math.abs(verts[v + 1] - y) <= 2))
				return i;
			i = nextVert[i]; // next
		}

		// Could not find, create new.
		i = nv;
		nv++;
		int v = i * 3;
		verts[v] = x;
		verts[v + 1] = y;
		verts[v + 2] = z;
		nextVert[i] = firstVert[bucket];
		firstVert[bucket] = i;

		return i;
	}

	static final int MAX_VERTS_PER_POLY = 6; // TODO: use the
												// DT_VERTS_PER_POLYGON
	static final int MAX_REM_EDGES = 48; // TODO: make this an expression.

	private void buildMeshAdjacency(int[] polys, int npolys, int[] verts, int nverts, TileCacheContourSet lcset) {
		// Based on code by Eric Lengyel from:
		// http://www.terathon.com/code/edges.php

		int maxEdgeCount = npolys * MAX_VERTS_PER_POLY;

		int[] firstEdge = new int[nverts + maxEdgeCount];
		int nextEdge = nverts;
		int edgeCount = 0;

		Edge[] edges = new Edge[maxEdgeCount];

		for (int i = 0; i < nverts; i++)
			firstEdge[i] = DT_TILECACHE_NULL_IDX;

		for (int i = 0; i < npolys; ++i) {
			int t = i * MAX_VERTS_PER_POLY * 2;
			for (int j = 0; j < MAX_VERTS_PER_POLY; ++j) {
				if (polys[t + j] == DT_TILECACHE_NULL_IDX)
					break;
				int v0 = polys[t + j];
				int v1 = (j + 1 >= MAX_VERTS_PER_POLY || polys[t + j + 1] == DT_TILECACHE_NULL_IDX) ? polys[t]
						: polys[t + j + 1];
				if (v0 < v1) {
					Edge edge = edges[edgeCount];
					edge.vert[0] = v0;
					edge.vert[1] = v1;
					edge.poly[0] = i;
					edge.polyEdge[0] = j;
					edge.poly[1] = i;
					edge.polyEdge[1] = 0xff;
					// Insert edge
					firstEdge[nextEdge + edgeCount] = firstEdge[v0];
					firstEdge[v0] = (short) edgeCount;
					edgeCount++;
				}
			}
		}

		for (int i = 0; i < npolys; ++i) {
			int t = i * MAX_VERTS_PER_POLY * 2;
			for (int j = 0; j < MAX_VERTS_PER_POLY; ++j) {
				if (polys[t + j] == DT_TILECACHE_NULL_IDX)
					break;
				int v0 = polys[t + j];
				int v1 = (j + 1 >= MAX_VERTS_PER_POLY || polys[t + j + 1] == DT_TILECACHE_NULL_IDX) ? polys[t]
						: polys[t + j + 1];
				if (v0 > v1) {
					boolean found = false;
					for (int e = firstEdge[v1]; e != DT_TILECACHE_NULL_IDX; e = firstEdge[nextEdge + e]) {
						Edge edge = edges[e];
						if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1]) {
							edge.poly[1] = i;
							edge.polyEdge[1] = j;
							found = true;
							break;
						}
					}
					if (!found) {
						// Matching edge not found, it is an open edge, add it.
						Edge edge = edges[edgeCount];
						edge.vert[0] = v1;
						edge.vert[1] = v0;
						edge.poly[0] = (short) i;
						edge.polyEdge[0] = (short) j;
						edge.poly[1] = (short) i;
						edge.polyEdge[1] = 0xff;
						// Insert edge
						firstEdge[nextEdge + edgeCount] = firstEdge[v1];
						firstEdge[v1] = (short) edgeCount;
						edgeCount++;
					}
				}
			}
		}

		// Mark portal edges.
		for (int i = 0; i < lcset.nconts; ++i) {
			TileCacheContour cont = lcset.conts[i];
			if (cont.nverts < 3)
				continue;

			for (int j = 0, k = cont.nverts - 1; j < cont.nverts; k = j++) {
				int va = k * 4;
				int vb = j * 4;
				int dir = cont.verts[va + 3] & 0xf;
				if (dir == 0xf)
					continue;

				if (dir == 0 || dir == 2) {
					// Find matching vertical edge
					int x = cont.verts[va];
					int zmin = cont.verts[va + 2];
					int zmax = cont.verts[vb + 2];
					if (zmin > zmax) {
						int tmp = zmin;
						zmin = zmax;
						zmax = tmp;
					}

					for (int m = 0; m < edgeCount; ++m) {
						Edge e = edges[m];
						// Skip connected edges.
						if (e.poly[0] != e.poly[1])
							continue;
						int eva = e.vert[0] * 3;
						int evb = e.vert[1] * 3;
						if (verts[eva] == x && verts[evb] == x) {
							int ezmin = verts[eva + 2];
							int ezmax = verts[evb + 2];
							if (ezmin > ezmax) {
								int tmp = ezmin;
								ezmin = ezmax;
								ezmax = tmp;
							}
							if (overlapRangeExl(zmin, zmax, ezmin, ezmax)) {
								// Reuse the other polyedge to store dir.
								e.polyEdge[1] = dir;
							}
						}
					}
				} else {
					// Find matching vertical edge
					int z = cont.verts[va + 2];
					int xmin = cont.verts[va];
					int xmax = cont.verts[vb];
					if (xmin > xmax) {

					}
					int tmp = xmin;
					xmin = xmax;
					xmax = tmp;
					for (int m = 0; m < edgeCount; ++m) {
						Edge e = edges[m];
						// Skip connected edges.
						if (e.poly[0] != e.poly[1])
							continue;
						int eva = e.vert[0] * 3;
						int evb = e.vert[1] * 3;
						if (verts[eva + 2] == z && verts[evb + 2] == z) {
							int exmin = verts[eva];
							int exmax = verts[evb];
							if (exmin > exmax) {
								tmp = exmin;
								exmin = exmax;
								exmax = tmp;
							}
							if (overlapRangeExl(xmin, xmax, exmin, exmax)) {
								// Reuse the other polyedge to store dir.
								e.polyEdge[1] = dir;
							}
						}
					}
				}
			}
		}

		// Store adjacency
		for (int i = 0; i < edgeCount; ++i) {
			Edge e = edges[i];
			if (e.poly[0] != e.poly[1]) {
				int p0 = e.poly[0] * MAX_VERTS_PER_POLY * 2;
				int p1 = e.poly[1] * MAX_VERTS_PER_POLY * 2;
				polys[p0 + MAX_VERTS_PER_POLY + e.polyEdge[0]] = e.poly[1];
				polys[p1 + MAX_VERTS_PER_POLY + e.polyEdge[1]] = e.poly[0];
			} else if (e.polyEdge[1] != 0xff) {
				int p0 = e.poly[0] * MAX_VERTS_PER_POLY * 2;
				polys[p0 + MAX_VERTS_PER_POLY + e.polyEdge[0]] = 0x8000 | (short) e.polyEdge[1];
			}

		}
	}

	private boolean overlapRangeExl(int amin, int amax, int bmin, int bmax) {
		return (amin >= bmax || amax <= bmin) ? false : true;
	}

	private int prev(int i, int n) {
		return i - 1 >= 0 ? i - 1 : n - 1;
	}

	private int next(int i, int n) {
		return i + 1 < n ? i + 1 : 0;
	}

	private int area2(int[] verts, int a, int b, int c) {
		return (verts[b] - verts[a]) * (verts[c + 2] - verts[a + 2])
				- (verts[c] - verts[a]) * (verts[b + 2] - verts[a + 2]);
	}

	// Returns true iff c is strictly to the left of the directed
	// line through a to b.
	private boolean left(int[] verts, int a, int b, int c) {
		return area2(verts, a, b, c) < 0;
	}

	private boolean leftOn(int[] verts, int a, int b, int c) {
		return area2(verts, a, b, c) <= 0;
	}

	private boolean collinear(int[] verts, int a, int b, int c) {
		return area2(verts, a, b, c) == 0;
	}

	// Returns true iff ab properly intersects cd: they share
	// a point interior to both segments. The properness of the
	// intersection is ensured by using strict leftness.
	private boolean intersectProp(int[] verts, int a, int b, int c, int d) {
		// Eliminate improper cases.
		if (collinear(verts, a, b, c) || collinear(verts, a, b, d) || collinear(verts, c, d, a)
				|| collinear(verts, c, d, b))
			return false;

		return (left(verts, a, b, c) ^ left(verts, a, b, d)) && (left(verts, c, d, a) ^ left(verts, c, d, b));
	}

	// Returns T iff (a,b,c) are collinear and point c lies
	// on the closed segement ab.
	private boolean between(int[] verts, int a, int b, int c) {
		if (!collinear(verts, a, b, c))
			return false;
		// If ab not vertical, check betweenness on x; else on y.
		if (verts[a] != verts[b])
			return ((verts[a] <= verts[c]) && (verts[c] <= verts[b]))
					|| ((verts[a] >= verts[c]) && (verts[c] >= verts[b]));
		else
			return ((verts[a + 2] <= verts[c + 2]) && (verts[c + 2] <= verts[b + 2]))
					|| ((verts[a + 2] >= verts[c + 2]) && (verts[c + 2] >= verts[b + 2]));
	}

	// Returns true iff segments ab and cd intersect, properly or improperly.
	private boolean intersect(int[] verts, int a, int b, int c, int d) {
		if (intersectProp(verts, a, b, c, d))
			return true;
		else if (between(verts, a, b, c) || between(verts, a, b, d) || between(verts, c, d, a)
				|| between(verts, c, d, b))
			return true;
		else
			return false;
	}

	private boolean vequal(int[] verts, int a, int b) {
		return verts[a] == verts[b] && verts[a + 2] == verts[b + 2];
	}

	// Returns T iff (v_i, v_j) is a proper internal *or* external
	// diagonal of P, *ignoring edges incident to v_i and v_j*.
	private boolean diagonalie(int i, int j, int n, int[] verts, int[] indices) {
		int d0 = (indices[i] & 0x7fff) * 4;
		int d1 = (indices[j] & 0x7fff) * 4;

		// For each edge (k,k+1) of P
		for (int k = 0; k < n; k++) {
			int k1 = next(k, n);
			// Skip edges incident to i or j
			if (!((k == i) || (k1 == i) || (k == j) || (k1 == j))) {
				int p0 = (indices[k] & 0x7fff) * 4;
				int p1 = (indices[k1] & 0x7fff) * 4;

				if (vequal(verts, d0, p0) || vequal(verts, d1, p0) || vequal(verts, d0, p1) || vequal(verts, d1, p1))
					continue;

				if (intersect(verts, d0, d1, p0, p1))
					return false;
			}
		}
		return true;
	}

	// Returns true iff the diagonal (i,j) is strictly internal to the
	// polygon P in the neighborhood of the i endpoint.
	private boolean inCone(int i, int j, int n, int[] verts, int[] indices) {
		int pi = (indices[i] & 0x7fff) * 4;
		int pj = (indices[j] & 0x7fff) * 4;
		int pi1 = (indices[next(i, n)] & 0x7fff) * 4;
		int pin1 = (indices[prev(i, n)] & 0x7fff) * 4;

		// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
		if (leftOn(verts, pin1, pi, pi1))
			return left(verts, pi, pj, pin1) && left(verts, pj, pi, pi1);
		// Assume (i-1,i,i+1) not collinear.
		// else P[i] is reflex.
		return !(leftOn(verts, pi, pj, pi1) && leftOn(verts, pj, pi, pin1));
	}

	// Returns T iff (v_i, v_j) is a proper internal
	// diagonal of P.
	private boolean diagonal(int i, int j, int n, int[] verts, int[] indices) {
		return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
	}

	private int triangulate(int n, int[] verts, int[] indices, int[] tris) {
		int ntris = 0;
		int dst = 0;// tris;

		// The last bit of the index is used to indicate if the vertex can be
		// removed.
		for (int i = 0; i < n; i++) {
			int i1 = next(i, n);
			int i2 = next(i1, n);
			if (diagonal(i, i2, n, verts, indices))
				indices[i1] |= 0x8000;
		}

		while (n > 3) {
			int minLen = -1;
			int mini = -1;
			for (int i = 0; i < n; i++) {
				int i1 = next(i, n);
				if ((indices[i1] & 0x8000) != 0) {
					int p0 = (indices[i] & 0x7fff) * 4;
					int p2 = (indices[next(i1, n)] & 0x7fff) * 4;

					int dx = verts[p2] - verts[p0];
					int dz = verts[p2 + 2] - verts[p0 + 2];
					int len = dx * dx + dz * dz;
					if (minLen < 0 || len < minLen) {
						minLen = len;
						mini = i;
					}
				}
			}

			if (mini == -1) {
				// Should not happen.
				/*
				 * printf("mini == -1 ntris=%d n=%d\n", ntris, n); for (int i =
				 * 0; i < n; i++) { printf("%d ", indices[i] & 0x0fffffff); }
				 * printf("\n");
				 */
				return -ntris;
			}

			int i = mini;
			int i1 = next(i, n);
			int i2 = next(i1, n);

			tris[dst++] = indices[i] & 0x7fff;
			tris[dst++] = indices[i1] & 0x7fff;
			tris[dst++] = indices[i2] & 0x7fff;
			ntris++;

			// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
			n--;
			for (int k = i1; k < n; k++)
				indices[k] = indices[k + 1];

			if (i1 >= n)
				i1 = 0;
			i = prev(i1, n);
			// Update diagonal flags.
			if (diagonal(prev(i, n), i1, n, verts, indices))
				indices[i] |= 0x8000;
			else
				indices[i] &= 0x7fff;

			if (diagonal(i, next(i1, n), n, verts, indices))
				indices[i1] |= 0x8000;
			else
				indices[i1] &= 0x7fff;
		}

		// Append the remaining triangle.
		tris[dst++] = indices[0] & 0x7fff;
		tris[dst++] = indices[1] & 0x7fff;
		tris[dst++] = indices[2] & 0x7fff;
		ntris++;

		return ntris;
	}

	private int countPolyVerts(int[] polys, int p) {
		for (int i = 0; i < MAX_VERTS_PER_POLY; ++i)
			if (polys[p + i] == DT_TILECACHE_NULL_IDX)
				return i;
		return MAX_VERTS_PER_POLY;
	}

	private boolean uleft(int[] verts, int a, int b, int c) {
		return (verts[b] - verts[a]) * (verts[c + 2] - verts[a + 2])
				- (verts[c] - verts[a]) * (verts[b + 2] - verts[a + 2]) < 0;
	}

	private int[] getPolyMergeValue(int[] polys, int pa, int pb, int[] verts) {
		int na = countPolyVerts(polys, pa);
		int nb = countPolyVerts(polys, pb);

		// If the merged polygon would be too big, do not merge.
		if (na + nb - 2 > MAX_VERTS_PER_POLY)
			return new int[] { -1, 0, 0 };

		// Check if the polygons share an edge.
		int ea = -1;
		int eb = -1;

		for (int i = 0; i < na; ++i) {
			int va0 = polys[pa + i];
			int va1 = polys[pa + (i + 1) % na];
			if (va0 > va1) {
				int tmp = va0;
				va0 = va1;
				va1 = tmp;
			}
			for (int j = 0; j < nb; ++j) {
				int vb0 = polys[pb + j];
				int vb1 = polys[pb + (j + 1) % nb];
				if (vb0 > vb1) {
					int tmp = vb0;
					vb0 = vb1;
					vb1 = tmp;
				}
				if (va0 == vb0 && va1 == vb1) {
					ea = i;
					eb = j;
					break;
				}
			}
		}

		// No common edge, cannot merge.
		if (ea == -1 || eb == -1)
			return new int[] { -1, ea, eb };

		// Check to see if the merged polygon would be convex.
		int va, vb, vc;

		va = polys[pa + (ea + na - 1) % na];
		vb = polys[pa + ea];
		vc = polys[pb + (eb + 2) % nb];
		if (!uleft(verts, va * 3, vb * 3, vc * 3))
			return new int[] { -1, ea, eb };

		va = polys[pb + (eb + nb - 1) % nb];
		vb = polys[pb + eb];
		vc = polys[pa + (ea + 2) % na];
		if (!uleft(verts, va * 3, vb * 3, vc * 3))
			return new int[] { -1, ea, eb };

		va = polys[pa + ea];
		vb = polys[pa + (ea + 1) % na];

		int dx = verts[va * 3 + 0] - verts[vb * 3 + 0];
		int dy = verts[va * 3 + 2] - verts[vb * 3 + 2];

		return new int[] { dx * dx + dy * dy, ea, eb };
	}

	private void mergePolys(int[] polys, int pa, int pb, int ea, int eb) {
		int[] tmp = new int[MAX_VERTS_PER_POLY * 2];

		int na = countPolyVerts(polys, pa);
		int nb = countPolyVerts(polys, pb);

		// Merge polygons.
		Arrays.fill(tmp, 0xFFFF);
		int n = 0;
		// Add pa
		for (int i = 0; i < na - 1; ++i)
			tmp[n++] = polys[pa + (ea + 1 + i) % na];
		// Add pb
		for (int i = 0; i < nb - 1; ++i)
			tmp[n++] = polys[pb + (eb + 1 + i) % nb];
		System.arraycopy(tmp, 0, polys, pa, MAX_VERTS_PER_POLY);
	}

	private int pushFront(int v, int[] arr, int an) {
		an++;
		for (int i = an - 1; i > 0; --i)
			arr[i] = arr[i - 1];
		arr[0] = v;
		return an;
	}

	private int pushBack(int v, int[] arr, int an) {
		arr[an] = v;
		an++;
		return an;
	}

	private boolean canRemoveVertex(TileCachePolyMesh mesh, int rem) {
		// Count number of polygons to remove.
		int numRemovedVerts = 0;
		int numTouchedVerts = 0;
		int numRemainingEdges = 0;
		for (int i = 0; i < mesh.npolys; ++i) {
			int p = i * MAX_VERTS_PER_POLY * 2;
			int nv = countPolyVerts(mesh.polys, p);
			int numRemoved = 0;
			int numVerts = 0;
			for (int j = 0; j < nv; ++j) {
				if (mesh.polys[p + j] == rem) {
					numTouchedVerts++;
					numRemoved++;
				}
				numVerts++;
			}
			if (numRemoved != 0) {
				numRemovedVerts += numRemoved;
				numRemainingEdges += numVerts - (numRemoved + 1);
			}
		}

		// There would be too few edges remaining to create a polygon.
		// This can happen for example when a tip of a triangle is marked
		// as deletion, but there are no other polys that share the vertex.
		// In this case, the vertex should not be removed.
		if (numRemainingEdges <= 2)
			return false;

		// Check that there is enough memory for the test.
		int maxEdges = numTouchedVerts * 2;
		if (maxEdges > MAX_REM_EDGES)
			return false;

		// Find edges which share the removed vertex.
		int[] edges = new int[MAX_REM_EDGES];
		int nedges = 0;

		for (int i = 0; i < mesh.npolys; ++i) {
			int p = i * MAX_VERTS_PER_POLY * 2;
			int nv = countPolyVerts(mesh.polys, p);

			// Collect edges which touches the removed vertex.
			for (int j = 0, k = nv - 1; j < nv; k = j++) {
				if (mesh.polys[p + j] == rem || mesh.polys[p + k] == rem) {
					// Arrange edge so that a=rem.
					int a = mesh.polys[p + j], b = mesh.polys[p + k];
					if (b == rem) {
						int tmp = a;
						a = b;
						b = tmp;
					}

					// Check if the edge exists
					boolean exists = false;
					for (int m = 0; m < nedges; ++m) {
						int e = m * 3;
						if (edges[e + 1] == b) {
							// Exists, increment vertex share count.
							edges[e + 2]++;
							exists = true;
						}
					}
					// Add new edge.
					if (!exists) {
						int e = nedges * 3;
						edges[e] = a;
						edges[e + 1] = b;
						edges[e + 2] = 1;
						nedges++;
					}
				}
			}
		}

		// There should be no more than 2 open edges.
		// This catches the case that two non-adjacent polygons
		// share the removed vertex. In that case, do not remove the vertex.
		int numOpenEdges = 0;
		for (int i = 0; i < nedges; ++i) {
			if (edges[i * 3 + 2] < 2)
				numOpenEdges++;
		}
		if (numOpenEdges > 2)
			return false;

		return true;
	}

	private void removeVertex(TileCachePolyMesh mesh, int rem, int maxTris) {
		// Count number of polygons to remove.
		int numRemovedVerts = 0;
		for (int i = 0; i < mesh.npolys; ++i) {
			int p = i * MAX_VERTS_PER_POLY * 2;
			int nv = countPolyVerts(mesh.polys, p);
			for (int j = 0; j < nv; ++j) {
				if (mesh.polys[p + j] == rem)
					numRemovedVerts++;
			}
		}

		int nedges = 0;
		int[] edges = new int[MAX_REM_EDGES * 3];
		int nhole = 0;
		int[] hole = new int[MAX_REM_EDGES];
		int nharea = 0;
		int[] harea = new int[MAX_REM_EDGES];

		for (int i = 0; i < mesh.npolys; ++i) {
			int p = i * MAX_VERTS_PER_POLY * 2;
			int nv = countPolyVerts(mesh.polys, p);
			boolean hasRem = false;
			for (int j = 0; j < nv; ++j)
				if (mesh.polys[p + j] == rem)
					hasRem = true;
			if (hasRem) {
				// Collect edges which does not touch the removed vertex.
				for (int j = 0, k = nv - 1; j < nv; k = j++) {
					if (mesh.polys[p + j] != rem && mesh.polys[p + k] != rem) {
						if (nedges >= MAX_REM_EDGES) { // FIXME: (PP) use lists
														// instead
							throw new RuntimeException("Buffer too small");
						}
						int e = nedges * 3;
						edges[e] = mesh.polys[p + k];
						edges[e + 1] = mesh.polys[p + j];
						edges[e + 2] = mesh.areas[i];
						nedges++;
					}
				}
				// Remove the polygon.
				int p2 = (mesh.npolys - 1) * MAX_VERTS_PER_POLY * 2;
				System.arraycopy(mesh.polys, p2, mesh.polys, p, MAX_VERTS_PER_POLY);
				Arrays.fill(mesh.polys, p + MAX_VERTS_PER_POLY, p + 2 * MAX_VERTS_PER_POLY, 0xFFFF);
				mesh.areas[i] = mesh.areas[mesh.npolys - 1];
				mesh.npolys--;
				--i;
			}
		}

		// Remove vertex.
		for (int i = rem; i < mesh.nverts; ++i) {
			mesh.verts[i * 3 + 0] = mesh.verts[(i + 1) * 3 + 0];
			mesh.verts[i * 3 + 1] = mesh.verts[(i + 1) * 3 + 1];
			mesh.verts[i * 3 + 2] = mesh.verts[(i + 1) * 3 + 2];
		}
		mesh.nverts--;

		// Adjust indices to match the removed vertex layout.
		for (int i = 0; i < mesh.npolys; ++i) {
			int p = i * MAX_VERTS_PER_POLY * 2;
			int nv = countPolyVerts(mesh.polys, p);
			for (int j = 0; j < nv; ++j)
				if (mesh.polys[p + j] > rem)
					mesh.polys[p + j]--;
		}
		for (int i = 0; i < nedges; ++i) {
			if (edges[i * 3 + 0] > rem)
				edges[i * 3 + 0]--;
			if (edges[i * 3 + 1] > rem)
				edges[i * 3 + 1]--;
		}

		if (nedges == 0)
			return;

		// Start with one vertex, keep appending connected
		// segments to the start and end of the hole.
		nhole = pushBack(edges[0], hole, nhole);
		nharea = pushBack(edges[2], harea, nharea);

		while (nedges != 0) {
			boolean match = false;

			for (int i = 0; i < nedges; ++i) {
				int ea = edges[i * 3 + 0];
				int eb = edges[i * 3 + 1];
				int a = edges[i * 3 + 2];
				boolean add = false;
				if (hole[0] == eb) {
					// The segment matches the beginning of the hole boundary.
					if (nhole >= MAX_REM_EDGES)
						throw new RuntimeException("Buffer too small");
					nhole = pushFront(ea, hole, nhole);
					nharea = pushFront(a, harea, nharea);
					add = true;
				} else if (hole[nhole - 1] == ea) {
					// The segment matches the end of the hole boundary.
					if (nhole >= MAX_REM_EDGES)
						throw new RuntimeException("Buffer too small");
					nhole = pushBack(eb, hole, nhole);
					nharea = pushBack(a, harea, nharea);
					add = true;
				}
				if (add) {
					// The edge segment was added, remove it.
					edges[i * 3 + 0] = edges[(nedges - 1) * 3 + 0];
					edges[i * 3 + 1] = edges[(nedges - 1) * 3 + 1];
					edges[i * 3 + 2] = edges[(nedges - 1) * 3 + 2];
					--nedges;
					match = true;
					--i;
				}
			}

			if (!match)
				break;
		}

		int[] tris = new int[MAX_REM_EDGES * 3];
		int[] tverts = new int[MAX_REM_EDGES * 3];
		int[] tpoly = new int[MAX_REM_EDGES * 3];

		// Generate temp vertex array for triangulation.
		for (int i = 0; i < nhole; ++i) {
			int pi = hole[i];
			tverts[i * 4 + 0] = mesh.verts[pi * 3 + 0];
			tverts[i * 4 + 1] = mesh.verts[pi * 3 + 1];
			tverts[i * 4 + 2] = mesh.verts[pi * 3 + 2];
			tverts[i * 4 + 3] = 0;
			tpoly[i] = (short) i;
		}

		// Triangulate the hole.
		int ntris = triangulate(nhole, tverts, tpoly, tris);
		if (ntris < 0) {
			// TODO: issue warning!
			ntris = -ntris;
		}

		if (ntris > MAX_REM_EDGES) // FIXME: (PP) Use lists instead
			throw new RuntimeException("Buffer too small");

		int[] polys = new int[MAX_REM_EDGES * MAX_VERTS_PER_POLY];
		int[] pareas = new int[MAX_REM_EDGES];

		// Build initial polygons.
		int npolys = 0;
		Arrays.fill(polys, 0, ntris * MAX_VERTS_PER_POLY, 0xFFFF);
		for (int j = 0; j < ntris; ++j) {
			int t = j * 3;
			if (tris[t] != tris[t + 1] && tris[t] != tris[t + 2] && tris[t + 1] != tris[t + 2]) {
				polys[npolys * MAX_VERTS_PER_POLY + 0] = hole[tris[t]];
				polys[npolys * MAX_VERTS_PER_POLY + 1] = hole[tris[t + 1]];
				polys[npolys * MAX_VERTS_PER_POLY + 2] = hole[tris[t + 2]];
				pareas[npolys] = harea[tris[t]];
				npolys++;
			}
		}
		if (npolys == 0)
			return;

		// Merge polygons.
		int maxVertsPerPoly = MAX_VERTS_PER_POLY;
		if (maxVertsPerPoly > 3) {
			for (;;) {
				// Find best polygons to merge.
				int bestMergeVal = 0;
				int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

				for (int j = 0; j < npolys - 1; ++j) {
					int pj = j * MAX_VERTS_PER_POLY;
					for (int k = j + 1; k < npolys; ++k) {
						int pk = k * MAX_VERTS_PER_POLY;
						int[] pm = getPolyMergeValue(polys, pj, pk, mesh.verts);
						int v = pm[0];
						int ea = pm[1];
						int eb = pm[2];
						if (v > bestMergeVal) {
							bestMergeVal = v;
							bestPa = j;
							bestPb = k;
							bestEa = ea;
							bestEb = eb;
						}
					}
				}

				if (bestMergeVal > 0) {
					// Found best, merge.
					int pa = bestPa * MAX_VERTS_PER_POLY;
					int pb = bestPb * MAX_VERTS_PER_POLY;
					mergePolys(polys, pa, pb, bestEa, bestEb);
					System.arraycopy(polys, (npolys - 1) * MAX_VERTS_PER_POLY, polys, pb, MAX_VERTS_PER_POLY);
					pareas[bestPb] = pareas[npolys - 1];
					npolys--;
				} else {
					// Could not merge any polygons, stop.
					break;
				}
			}
		}

		// Store polygons.
		for (int i = 0; i < npolys; ++i) {
			if (mesh.npolys >= maxTris)
				break;
			int p = mesh.npolys * MAX_VERTS_PER_POLY * 2;
			Arrays.fill(mesh.polys, p, p + MAX_VERTS_PER_POLY * 2, 0xFFFF);
			for (int j = 0; j < MAX_VERTS_PER_POLY; ++j)
				mesh.polys[p + j] = polys[i * MAX_VERTS_PER_POLY + j];
			mesh.areas[mesh.npolys] = pareas[i];
			mesh.npolys++;
			if (mesh.npolys > maxTris) {
				throw new RuntimeException("Buffer too small");
			}
		}

	}

	TileCachePolyMesh buildTileCachePolyMesh(TileCacheContourSet lcset) {
		TileCachePolyMesh mesh = new TileCachePolyMesh();

		int maxVertices = 0;
		int maxTris = 0;
		int maxVertsPerCont = 0;
		for (int i = 0; i < lcset.nconts; ++i) {
			// Skip null contours.
			if (lcset.conts[i].nverts < 3)
				continue;
			maxVertices += lcset.conts[i].nverts;
			maxTris += lcset.conts[i].nverts - 2;
			maxVertsPerCont = Math.max(maxVertsPerCont, lcset.conts[i].nverts);
		}

		// TODO: warn about too many vertices?

		mesh.nvp = MAX_VERTS_PER_POLY;

		int[] vflags = new int[maxVertices];

		mesh.verts = new int[maxVertices * 3];
		mesh.polys = new int[maxTris * MAX_VERTS_PER_POLY * 2];
		mesh.areas = new int[maxTris];
		// Just allocate and clean the mesh flags array. The user is resposible
		// for filling it.
		mesh.flags = new int[maxTris];

		mesh.nverts = 0;
		mesh.npolys = 0;

		int[] firstVert = new int[VERTEX_BUCKET_COUNT2];
		for (int i = 0; i < VERTEX_BUCKET_COUNT2; ++i)
			firstVert[i] = DT_TILECACHE_NULL_IDX;

		int[] nextVert = new int[maxVertices];
		int[] indices = new int[maxVertsPerCont];
		int[] tris = new int[maxVertsPerCont];
		int[] polys = new int[maxVertsPerCont * MAX_VERTS_PER_POLY];
		Arrays.fill(polys, 0xFFFF);

		for (int i = 0; i < lcset.nconts; ++i) {
			TileCacheContour cont = lcset.conts[i];

			// Skip null contours.
			if (cont.nverts < 3)
				continue;

			// Triangulate contour
			for (int j = 0; j < cont.nverts; ++j)
				indices[j] = j;

			int ntris = triangulate(cont.nverts, cont.verts, indices, tris);
			if (ntris <= 0) {
				// TODO: issue warning!
				ntris = -ntris;
			}

			// Add and merge vertices.
			for (int j = 0; j < cont.nverts; ++j) {
				int v = j * 4;
				indices[j] = addVertex(cont.verts[v], cont.verts[v + 1], cont.verts[v + 2], mesh.verts, firstVert,
						nextVert, mesh.nverts);
				if ((cont.verts[v + 3] & 0x80) != 0) {
					// This vertex should be removed.
					vflags[indices[j]] = 1;
				}
			}

			// Build initial polygons.
			int npolys = 0;

			for (int j = 0; j < ntris; ++j) {
				int t = j * 3;
				if (tris[t] != tris[t + 1] && tris[t] != tris[t + 2] && tris[t + 1] != tris[t + 2]) {
					polys[npolys * MAX_VERTS_PER_POLY + 0] = indices[tris[t]];
					polys[npolys * MAX_VERTS_PER_POLY + 1] = indices[tris[t + 1]];
					polys[npolys * MAX_VERTS_PER_POLY + 2] = indices[tris[t + 2]];
					npolys++;
				}
			}
			if (npolys == 0)
				continue;

			// Merge polygons.
			int maxVertsPerPoly = MAX_VERTS_PER_POLY;
			if (maxVertsPerPoly > 3) {
				for (;;) {
					// Find best polygons to merge.
					int bestMergeVal = 0;
					int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

					for (int j = 0; j < npolys - 1; ++j) {
						int pj = j * MAX_VERTS_PER_POLY;
						for (int k = j + 1; k < npolys; ++k) {
							int pk = k * MAX_VERTS_PER_POLY;
							int[] pm = getPolyMergeValue(polys, pj, pk, mesh.verts);
							int v = pm[0];
							int ea = pm[1];
							int eb = pm[2];
							if (v > bestMergeVal) {
								bestMergeVal = v;
								bestPa = j;
								bestPb = k;
								bestEa = ea;
								bestEb = eb;
							}
						}
					}

					if (bestMergeVal > 0) {
						// Found best, merge.
						int pa = bestPa * MAX_VERTS_PER_POLY;
						int pb = bestPb * MAX_VERTS_PER_POLY;
						mergePolys(polys, pa, pb, bestEa, bestEb);
						System.arraycopy(polys, (npolys - 1) * MAX_VERTS_PER_POLY, polys, pb, MAX_VERTS_PER_POLY);
						npolys--;
					} else {
						// Could not merge any polygons, stop.
						break;
					}
				}
			}

			// Store polygons.
			for (int j = 0; j < npolys; ++j) {
				int p = mesh.npolys * MAX_VERTS_PER_POLY * 2;
				int q = j * MAX_VERTS_PER_POLY;
				for (int k = 0; k < MAX_VERTS_PER_POLY; ++k)
					mesh.polys[p + k] = polys[q + k];
				mesh.areas[mesh.npolys] = cont.area;
				mesh.npolys++;
				if (mesh.npolys > maxTris)
					throw new RuntimeException("Buffer too small");
			}
		}

		// Remove edge vertices.
		for (int i = 0; i < mesh.nverts; ++i) {
			if (vflags[i] != 0) {
				if (!canRemoveVertex(mesh, (short) i))
					continue;
				removeVertex(mesh, (short) i, maxTris);
				// Remove vertex
				// Note: mesh.nverts is already decremented inside
				// removeVertex()!
				for (int j = i; j < mesh.nverts; ++j)
					vflags[j] = vflags[j + 1];
				--i;
			}
		}

		// Calculate adjacency.
		buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.verts, mesh.nverts, lcset);

		return mesh;
	}

	public void markCylinderArea(TileCacheLayer layer, float[] orig, float cs, float ch, float[] pos, float radius,
			float height, int areaId) {
		float[] bmin = new float[3];
		float[] bmax = new float[3];
		bmin[0] = pos[0] - radius;
		bmin[1] = pos[1];
		bmin[2] = pos[2] - radius;
		bmax[0] = pos[0] + radius;
		bmax[1] = pos[1] + height;
		bmax[2] = pos[2] + radius;
		float r2 = sqr(radius / cs + 0.5f);

		int w = layer.header.width;
		int h = layer.header.height;
		float ics = 1.0f / cs;
		float ich = 1.0f / ch;

		float px = (pos[0] - orig[0]) * ics;
		float pz = (pos[2] - orig[2]) * ics;

		int minx = (int) Math.floor((bmin[0] - orig[0]) * ics);
		int miny = (int) Math.floor((bmin[1] - orig[1]) * ich);
		int minz = (int) Math.floor((bmin[2] - orig[2]) * ics);
		int maxx = (int) Math.floor((bmax[0] - orig[0]) * ics);
		int maxy = (int) Math.floor((bmax[1] - orig[1]) * ich);
		int maxz = (int) Math.floor((bmax[2] - orig[2]) * ics);

		if (maxx < 0)
			return;
		if (minx >= w)
			return;
		if (maxz < 0)
			return;
		if (minz >= h)
			return;

		if (minx < 0)
			minx = 0;
		if (maxx >= w)
			maxx = w - 1;
		if (minz < 0)
			minz = 0;
		if (maxz >= h)
			maxz = h - 1;

		for (int z = minz; z <= maxz; ++z) {
			for (int x = minx; x <= maxx; ++x) {
				float dx = x + 0.5f - px;
				float dz = z + 0.5f - pz;
				if (dx * dx + dz * dz > r2)
					continue;
				int y = layer.heights[x + z * w];
				if (y < miny || y > maxy)
					continue;
				layer.areas[x + z * w] = (byte) areaId;
			}
		}

	}

	public byte[] buildTileCacheLayer(TileCacheCompressor comp, TileCacheLayerHeader header, int[] heights, int[]areas, int[] cons, ByteOrder order, boolean cCompatibility) {
		int gridSize = header.width * header.height;
		ByteArrayOutputStream baos = new ByteArrayOutputStream();
		TileCacheLayerHeaderWriter hw = new TileCacheLayerHeaderWriter();
		try {
			hw.write(baos, header, order, cCompatibility);
			baos.write(gridSize);
			byte[] buffer = new byte[gridSize * 3];
			for (int i = 0; i < gridSize; i++) {
				buffer[i] = (byte) heights[i];
				buffer[gridSize + i] = (byte) areas[i];
				buffer[gridSize * 2 + i] = (byte) cons[i];
			}
			baos.write(comp.compress(buffer));
			return baos.toByteArray();
		} catch (IOException e) {
			throw new RuntimeException(e.getMessage(), e);
		}
	}

	TileCacheLayer decompressTileCacheLayer(TileCacheCompressor comp, byte[] compressed, ByteOrder order, boolean cCompatibility) {
		ByteBuffer buf = ByteBuffer.wrap(compressed);
		buf.order(order);
		TileCacheLayer layer = new TileCacheLayer();
		try {
			layer.header = reader.readLayerHeader(buf, cCompatibility);
		} catch (IOException e) {
			throw new RuntimeException(e.getMessage(), e);
		}

		int gridSize = layer.header.width * layer.header.height;
		byte[] grids = comp.decompress(compressed, buf.position(), compressed.length - buf.position(), gridSize * 3);
		layer.heights = Arrays.copyOfRange(grids, 0, gridSize);
		layer.areas = Arrays.copyOfRange(grids, gridSize, gridSize * 2);
		layer.cons = Arrays.copyOfRange(grids, gridSize * 2, gridSize * 3);
		layer.regs = new byte[gridSize];
		return layer;

	}

}
