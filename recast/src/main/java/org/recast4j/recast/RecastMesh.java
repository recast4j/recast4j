/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
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

import static org.recast4j.recast.RecastConstants.RC_BORDER_VERTEX;
import static org.recast4j.recast.RecastConstants.RC_MESH_NULL_IDX;
import static org.recast4j.recast.RecastConstants.RC_MULTIPLE_REGS;

import java.util.Arrays;

public class RecastMesh {

    static int VERTEX_BUCKET_COUNT = (1 << 12);

    private static class Edge {
        int vert[] = new int[2];
        int polyEdge[] = new int[2];
        int poly[] = new int[2];

    }

    private static void buildMeshAdjacency(int[] polys, int npolys, int nverts, int vertsPerPoly) {
        // Based on code by Eric Lengyel from:
        // http://www.terathon.com/code/edges.php

        int maxEdgeCount = npolys * vertsPerPoly;
        int[] firstEdge = new int[nverts + maxEdgeCount];
        int nextEdge = nverts;
        int edgeCount = 0;

        Edge[] edges = new Edge[maxEdgeCount];

        for (int i = 0; i < nverts; i++)
            firstEdge[i] = RC_MESH_NULL_IDX;

        for (int i = 0; i < npolys; ++i) {
            int t = i * vertsPerPoly * 2;
            for (int j = 0; j < vertsPerPoly; ++j) {
                if (polys[t + j] == RC_MESH_NULL_IDX)
                    break;
                int v0 = polys[t + j];
                int v1 = (j + 1 >= vertsPerPoly || polys[t + j + 1] == RC_MESH_NULL_IDX) ? polys[t + 0]
                        : polys[t + j + 1];
                if (v0 < v1) {
                    Edge edge = new Edge();
                    edges[edgeCount] = edge;
                    edge.vert[0] = v0;
                    edge.vert[1] = v1;
                    edge.poly[0] = i;
                    edge.polyEdge[0] = j;
                    edge.poly[1] = i;
                    edge.polyEdge[1] = 0;
                    // Insert edge
                    firstEdge[nextEdge + edgeCount] = firstEdge[v0];
                    firstEdge[v0] = edgeCount;
                    edgeCount++;
                }
            }
        }

        for (int i = 0; i < npolys; ++i) {
            int t = i * vertsPerPoly * 2;
            for (int j = 0; j < vertsPerPoly; ++j) {
                if (polys[t + j] == RC_MESH_NULL_IDX)
                    break;
                int v0 = polys[t + j];
                int v1 = (j + 1 >= vertsPerPoly || polys[t + j + 1] == RC_MESH_NULL_IDX) ? polys[t + 0]
                        : polys[t + j + 1];
                if (v0 > v1) {
                    for (int e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = firstEdge[nextEdge + e]) {
                        Edge edge = edges[e];
                        if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1]) {
                            edge.poly[1] = i;
                            edge.polyEdge[1] = j;
                            break;
                        }
                    }
                }
            }
        }

        // Store adjacency
        for (int i = 0; i < edgeCount; ++i) {
            Edge e = edges[i];
            if (e.poly[0] != e.poly[1]) {
                int p0 = e.poly[0] * vertsPerPoly * 2;
                int p1 = e.poly[1] * vertsPerPoly * 2;
                polys[p0 + vertsPerPoly + e.polyEdge[0]] = e.poly[1];
                polys[p1 + vertsPerPoly + e.polyEdge[1]] = e.poly[0];
            }
        }

    }

    private static int computeVertexHash(int x, int y, int z) {
        int h1 = 0x8da6b343; // Large multiplicative constants;
        int h2 = 0xd8163841; // here arbitrarily chosen primes
        int h3 = 0xcb1ab31f;
        int n = h1 * x + h2 * y + h3 * z;
        return n & (VERTEX_BUCKET_COUNT - 1);
    }

    private static int[] addVertex(int x, int y, int z, int[] verts, int[] firstVert, int[] nextVert, int nv) {
        int bucket = computeVertexHash(x, 0, z);
        int i = firstVert[bucket];

        while (i != -1) {
            int v = i * 3;
            if (verts[v + 0] == x && (Math.abs(verts[v + 1] - y) <= 2) && verts[v + 2] == z)
                return new int[] { i, nv };
            i = nextVert[i]; // next
        }

        // Could not find, create new.
        i = nv;
        nv++;
        int v = i * 3;
        verts[v + 0] = x;
        verts[v + 1] = y;
        verts[v + 2] = z;
        nextVert[i] = firstVert[bucket];
        firstVert[bucket] = i;

        return new int[] { i, nv };
    }

    static int prev(int i, int n) {
        return i - 1 >= 0 ? i - 1 : n - 1;
    }

    static int next(int i, int n) {
        return i + 1 < n ? i + 1 : 0;
    }

    private static int area2(int[] verts, int a, int b, int c) {
        return (verts[b + 0] - verts[a + 0]) * (verts[c + 2] - verts[a + 2])
                - (verts[c + 0] - verts[a + 0]) * (verts[b + 2] - verts[a + 2]);
    }

    // Returns true iff c is strictly to the left of the directed
    // line through a to b.
    static boolean left(int[] verts, int a, int b, int c) {
        return area2(verts, a, b, c) < 0;
    }

    static boolean leftOn(int[] verts, int a, int b, int c) {
        return area2(verts, a, b, c) <= 0;
    }

    private static boolean collinear(int[] verts, int a, int b, int c) {
        return area2(verts, a, b, c) == 0;
    }

    // Returns true iff ab properly intersects cd: they share
    // a point interior to both segments. The properness of the
    // intersection is ensured by using strict leftness.
    private static boolean intersectProp(int[] verts, int a, int b, int c, int d) {
        // Eliminate improper cases.
        if (collinear(verts, a, b, c) || collinear(verts, a, b, d) || collinear(verts, c, d, a)
                || collinear(verts, c, d, b))
            return false;

        return (left(verts, a, b, c) ^ left(verts, a, b, d)) && (left(verts, c, d, a) ^ left(verts, c, d, b));
    }

    // Returns T iff (a,b,c) are collinear and point c lies
    // on the closed segement ab.
    private static boolean between(int[] verts, int a, int b, int c) {
        if (!collinear(verts, a, b, c))
            return false;
        // If ab not vertical, check betweenness on x; else on y.
        if (verts[a + 0] != verts[b + 0])
            return ((verts[a + 0] <= verts[c + 0]) && (verts[c + 0] <= verts[b + 0]))
                    || ((verts[a + 0] >= verts[c + 0]) && (verts[c + 0] >= verts[b + 0]));
        else
            return ((verts[a + 2] <= verts[c + 2]) && (verts[c + 2] <= verts[b + 2]))
                    || ((verts[a + 2] >= verts[c + 2]) && (verts[c + 2] >= verts[b + 2]));
    }

    // Returns true iff segments ab and cd intersect, properly or improperly.
    static boolean intersect(int[] verts, int a, int b, int c, int d) {
        if (intersectProp(verts, a, b, c, d))
            return true;
        else if (between(verts, a, b, c) || between(verts, a, b, d) || between(verts, c, d, a)
                || between(verts, c, d, b))
            return true;
        else
            return false;
    }

    static boolean vequal(int[] verts, int a, int b) {
        return verts[a + 0] == verts[b + 0] && verts[a + 2] == verts[b + 2];
    }

    // Returns T iff (v_i, v_j) is a proper internal *or* external
    // diagonal of P, *ignoring edges incident to v_i and v_j*.
    private static boolean diagonalie(int i, int j, int n, int[] verts, int[] indices) {
        int d0 = (indices[i] & 0x0fffffff) * 4;
        int d1 = (indices[j] & 0x0fffffff) * 4;

        // For each edge (k,k+1) of P
        for (int k = 0; k < n; k++) {
            int k1 = next(k, n);
            // Skip edges incident to i or j
            if (!((k == i) || (k1 == i) || (k == j) || (k1 == j))) {
                int p0 = (indices[k] & 0x0fffffff) * 4;
                int p1 = (indices[k1] & 0x0fffffff) * 4;

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
    private static boolean inCone(int i, int j, int n, int[] verts, int[] indices) {
        int pi = (indices[i] & 0x0fffffff) * 4;
        int pj = (indices[j] & 0x0fffffff) * 4;
        int pi1 = (indices[next(i, n)] & 0x0fffffff) * 4;
        int pin1 = (indices[prev(i, n)] & 0x0fffffff) * 4;
        // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
        if (leftOn(verts, pin1, pi, pi1)) {
            return left(verts, pi, pj, pin1) && left(verts, pj, pi, pi1);
        }
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
        return !(leftOn(verts, pi, pj, pi1) && leftOn(verts, pj, pi, pin1));
    }

    // Returns T iff (v_i, v_j) is a proper internal
    // diagonal of P.
    private static boolean diagonal(int i, int j, int n, int[] verts, int[] indices) {
        return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
    }

    private static boolean diagonalieLoose(int i, int j, int n, int[] verts, int[] indices) {
        int d0 = (indices[i] & 0x0fffffff) * 4;
        int d1 = (indices[j] & 0x0fffffff) * 4;

        // For each edge (k,k+1) of P
        for (int k = 0; k < n; k++) {
            int k1 = next(k, n);
            // Skip edges incident to i or j
            if (!((k == i) || (k1 == i) || (k == j) || (k1 == j))) {
                int p0 = (indices[k] & 0x0fffffff) * 4;
                int p1 = (indices[k1] & 0x0fffffff) * 4;

                if (vequal(verts, d0, p0) || vequal(verts, d1, p0) || vequal(verts, d0, p1) || vequal(verts, d1, p1))
                    continue;

                if (intersectProp(verts, d0, d1, p0, p1))
                    return false;
            }
        }
        return true;
    }

    private static boolean inConeLoose(int i, int j, int n, int[] verts, int[] indices) {
        int pi = (indices[i] & 0x0fffffff) * 4;
        int pj = (indices[j] & 0x0fffffff) * 4;
        int pi1 = (indices[next(i, n)] & 0x0fffffff) * 4;
        int pin1 = (indices[prev(i, n)] & 0x0fffffff) * 4;

        // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
        if (leftOn(verts, pin1, pi, pi1))
            return leftOn(verts, pi, pj, pin1) && leftOn(verts, pj, pi, pi1);
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
        return !(leftOn(verts, pi, pj, pi1) && leftOn(verts, pj, pi, pin1));
    }

    private static boolean diagonalLoose(int i, int j, int n, int[] verts, int[] indices) {
        return inConeLoose(i, j, n, verts, indices) && diagonalieLoose(i, j, n, verts, indices);
    }

    private static int triangulate(int n, int[] verts, int[] indices, int[] tris) {
        int ntris = 0;

        // The last bit of the index is used to indicate if the vertex can be removed.
        for (int i = 0; i < n; i++) {
            int i1 = next(i, n);
            int i2 = next(i1, n);
            if (diagonal(i, i2, n, verts, indices)) {
                indices[i1] |= 0x80000000;
            }
        }

        while (n > 3) {
            int minLen = -1;
            int mini = -1;
            for (int i = 0; i < n; i++) {
                int i1 = next(i, n);
                if ((indices[i1] & 0x80000000) != 0) {
                    int p0 = (indices[i] & 0x0fffffff) * 4;
                    int p2 = (indices[next(i1, n)] & 0x0fffffff) * 4;

                    int dx = verts[p2 + 0] - verts[p0 + 0];
                    int dy = verts[p2 + 2] - verts[p0 + 2];
                    int len = dx * dx + dy * dy;

                    if (minLen < 0 || len < minLen) {
                        minLen = len;
                        mini = i;
                    }
                }
            }

            if (mini == -1) {
                // We might get here because the contour has overlapping segments, like this:
                //
                // A o-o=====o---o B
                // / |C D| \
                // o o o o
                // : : : :
                // We'll try to recover by loosing up the inCone test a bit so that a diagonal
                // like A-B or C-D can be found and we can continue.
                minLen = -1;
                mini = -1;
                for (int i = 0; i < n; i++) {
                    int i1 = next(i, n);
                    int i2 = next(i1, n);
                    if (diagonalLoose(i, i2, n, verts, indices)) {
                        int p0 = (indices[i] & 0x0fffffff) * 4;
                        int p2 = (indices[next(i2, n)] & 0x0fffffff) * 4;
                        int dx = verts[p2 + 0] - verts[p0 + 0];
                        int dy = verts[p2 + 2] - verts[p0 + 2];
                        int len = dx * dx + dy * dy;

                        if (minLen < 0 || len < minLen) {
                            minLen = len;
                            mini = i;
                        }
                    }
                }
                if (mini == -1) {
                    // The contour is messed up. This sometimes happens
                    // if the contour simplification is too aggressive.
                    return -ntris;
                }
            }

            int i = mini;
            int i1 = next(i, n);
            int i2 = next(i1, n);

            tris[ntris * 3] = indices[i] & 0x0fffffff;
            tris[ntris * 3 + 1] = indices[i1] & 0x0fffffff;
            tris[ntris * 3 + 2] = indices[i2] & 0x0fffffff;
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
                indices[i] |= 0x80000000;
            else
                indices[i] &= 0x0fffffff;

            if (diagonal(i, next(i1, n), n, verts, indices))
                indices[i1] |= 0x80000000;
            else
                indices[i1] &= 0x0fffffff;
        }

        // Append the remaining triangle.
        tris[ntris * 3] = indices[0] & 0x0fffffff;
        tris[ntris * 3 + 1] = indices[1] & 0x0fffffff;
        tris[ntris * 3 + 2] = indices[2] & 0x0fffffff;
        ntris++;

        return ntris;
    }

    private static int countPolyVerts(int[] p, int j, int nvp) {
        for (int i = 0; i < nvp; ++i)
            if (p[i + j] == RC_MESH_NULL_IDX)
                return i;
        return nvp;
    }

    private static boolean uleft(int[] verts, int a, int b, int c) {
        return (verts[b + 0] - verts[a + 0]) * (verts[c + 2] - verts[a + 2])
                - (verts[c + 0] - verts[a + 0]) * (verts[b + 2] - verts[a + 2]) < 0;
    }

    private static int[] getPolyMergeValue(int[] polys, int pa, int pb, int[] verts, int nvp) {
        int ea = -1;
        int eb = -1;
        int na = countPolyVerts(polys, pa, nvp);
        int nb = countPolyVerts(polys, pb, nvp);

        // If the merged polygon would be too big, do not merge.
        if (na + nb - 2 > nvp)
            return new int[] { -1, ea, eb };

        // Check if the polygons share an edge.

        for (int i = 0; i < na; ++i) {
            int va0 = polys[pa + i];
            int va1 = polys[pa + (i + 1) % na];
            if (va0 > va1) {
                int temp = va0;
                va0 = va1;
                va1 = temp;
            }
            for (int j = 0; j < nb; ++j) {
                int vb0 = polys[pb + j];
                int vb1 = polys[pb + (j + 1) % nb];
                if (vb0 > vb1) {
                    int temp = vb0;
                    vb0 = vb1;
                    vb1 = temp;
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

    private static void mergePolyVerts(int[] polys, int pa, int pb, int ea, int eb, int tmp, int nvp) {
        int na = countPolyVerts(polys, pa, nvp);
        int nb = countPolyVerts(polys, pb, nvp);

        // Merge polygons.
        Arrays.fill(polys, tmp, tmp + nvp, RC_MESH_NULL_IDX);
        int n = 0;
        // Add pa
        for (int i = 0; i < na - 1; ++i) {
            polys[tmp + n] = polys[pa + (ea + 1 + i) % na];
            n++;
        }
        // Add pb
        for (int i = 0; i < nb - 1; ++i) {
            polys[tmp + n] = polys[pb + (eb + 1 + i) % nb];
            n++;
        }
        System.arraycopy(polys, tmp, polys, pa, nvp);
    }

    private static int pushFront(int v, int[] arr, int an) {
        an++;
        for (int i = an - 1; i > 0; --i)
            arr[i] = arr[i - 1];
        arr[0] = v;
        return an;
    }

    private static int pushBack(int v, int[] arr, int an) {
        arr[an] = v;
        an++;
        return an;
    }

    private static boolean canRemoveVertex(Context ctx, PolyMesh mesh, int rem) {
        int nvp = mesh.nvp;

        // Count number of polygons to remove.
        int numTouchedVerts = 0;
        int numRemainingEdges = 0;
        for (int i = 0; i < mesh.npolys; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVerts(mesh.polys, p, nvp);
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
                numRemainingEdges += numVerts - (numRemoved + 1);
            }
        }
        // There would be too few edges remaining to create a polygon.
        // This can happen for example when a tip of a triangle is marked
        // as deletion, but there are no other polys that share the vertex.
        // In this case, the vertex should not be removed.
        if (numRemainingEdges <= 2)
            return false;

        // Find edges which share the removed vertex.
        int maxEdges = numTouchedVerts * 2;
        int nedges = 0;
        int[] edges = new int[maxEdges * 3];

        for (int i = 0; i < mesh.npolys; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVerts(mesh.polys, p, nvp);

            // Collect edges which touches the removed vertex.
            for (int j = 0, k = nv - 1; j < nv; k = j++) {
                if (mesh.polys[p + j] == rem || mesh.polys[p + k] == rem) {
                    // Arrange edge so that a=rem.
                    int a = mesh.polys[p + j], b = mesh.polys[p + k];
                    if (b == rem) {
                        int temp = a;
                        a = b;
                        b = temp;
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
                        edges[e + 0] = a;
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

    private static void removeVertex(Context ctx, PolyMesh mesh, int rem, int maxTris) {
        int nvp = mesh.nvp;

        // Count number of polygons to remove.
        int numRemovedVerts = 0;
        for (int i = 0; i < mesh.npolys; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVerts(mesh.polys, p, nvp);
            for (int j = 0; j < nv; ++j) {
                if (mesh.polys[p + j] == rem)
                    numRemovedVerts++;
            }
        }

        int nedges = 0;
        int[] edges = new int[numRemovedVerts * nvp * 4];

        int nhole = 0;
        int[] hole = new int[numRemovedVerts * nvp];

        int nhreg = 0;
        int[] hreg = new int[numRemovedVerts * nvp];

        int nharea = 0;
        int[] harea = new int[numRemovedVerts * nvp];

        for (int i = 0; i < mesh.npolys; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVerts(mesh.polys, p, nvp);
            boolean hasRem = false;
            for (int j = 0; j < nv; ++j)
                if (mesh.polys[p + j] == rem)
                    hasRem = true;
            if (hasRem) {
                // Collect edges which does not touch the removed vertex.
                for (int j = 0, k = nv - 1; j < nv; k = j++) {
                    if (mesh.polys[p + j] != rem && mesh.polys[p + k] != rem) {
                        int e = nedges * 4;
                        edges[e + 0] = mesh.polys[p + k];
                        edges[e + 1] = mesh.polys[p + j];
                        edges[e + 2] = mesh.regs[i];
                        edges[e + 3] = mesh.areas[i];
                        nedges++;
                    }
                }
                // Remove the polygon.
                int p2 = (mesh.npolys - 1) * nvp * 2;
                if (p != p2) {
                    System.arraycopy(mesh.polys, p2, mesh.polys, p, nvp);
                }
                Arrays.fill(mesh.polys, p + nvp, p + nvp + nvp, RC_MESH_NULL_IDX);
                mesh.regs[i] = mesh.regs[mesh.npolys - 1];
                mesh.areas[i] = mesh.areas[mesh.npolys - 1];
                mesh.npolys--;
                --i;
            }
        }

        // Remove vertex.
        for (int i = rem; i < mesh.nverts - 1; ++i) {
            mesh.verts[i * 3 + 0] = mesh.verts[(i + 1) * 3 + 0];
            mesh.verts[i * 3 + 1] = mesh.verts[(i + 1) * 3 + 1];
            mesh.verts[i * 3 + 2] = mesh.verts[(i + 1) * 3 + 2];
        }
        mesh.nverts--;

        // Adjust indices to match the removed vertex layout.
        for (int i = 0; i < mesh.npolys; ++i) {
            int p = i * nvp * 2;
            int nv = countPolyVerts(mesh.polys, p, nvp);
            for (int j = 0; j < nv; ++j)
                if (mesh.polys[p + j] > rem)
                    mesh.polys[p + j]--;
        }
        for (int i = 0; i < nedges; ++i) {
            if (edges[i * 4 + 0] > rem)
                edges[i * 4 + 0]--;
            if (edges[i * 4 + 1] > rem)
                edges[i * 4 + 1]--;
        }

        if (nedges == 0)
            return;

        // Start with one vertex, keep appending connected
        // segments to the start and end of the hole.
        pushBack(edges[0], hole, nhole);
        pushBack(edges[2], hreg, nhreg);
        pushBack(edges[3], harea, nharea);

        while (nedges != 0) {
            boolean match = false;

            for (int i = 0; i < nedges; ++i) {
                int ea = edges[i * 4 + 0];
                int eb = edges[i * 4 + 1];
                int r = edges[i * 4 + 2];
                int a = edges[i * 4 + 3];
                boolean add = false;
                if (hole[0] == eb) {
                    // The segment matches the beginning of the hole boundary.
                    pushFront(ea, hole, nhole);
                    pushFront(r, hreg, nhreg);
                    pushFront(a, harea, nharea);
                    add = true;
                } else if (hole[nhole - 1] == ea) {
                    // The segment matches the end of the hole boundary.
                    nhole = pushBack(eb, hole, nhole);
                    nhreg = pushBack(r, hreg, nhreg);
                    nharea = pushBack(a, harea, nharea);
                    add = true;
                }
                if (add) {
                    // The edge segment was added, remove it.
                    edges[i * 4 + 0] = edges[(nedges - 1) * 4 + 0];
                    edges[i * 4 + 1] = edges[(nedges - 1) * 4 + 1];
                    edges[i * 4 + 2] = edges[(nedges - 1) * 4 + 2];
                    edges[i * 4 + 3] = edges[(nedges - 1) * 4 + 3];
                    --nedges;
                    match = true;
                    --i;
                }
            }

            if (!match)
                break;
        }

        int[] tris = new int[nhole * 3];

        int[] tverts = new int[nhole * 4];

        int[] thole = new int[nhole];

        // Generate temp vertex array for triangulation.
        for (int i = 0; i < nhole; ++i) {
            int pi = hole[i];
            tverts[i * 4 + 0] = mesh.verts[pi * 3 + 0];
            tverts[i * 4 + 1] = mesh.verts[pi * 3 + 1];
            tverts[i * 4 + 2] = mesh.verts[pi * 3 + 2];
            tverts[i * 4 + 3] = 0;
            thole[i] = i;
        }

        // Triangulate the hole.
        int ntris = triangulate(nhole, tverts, thole, tris);
        if (ntris < 0) {
            ntris = -ntris;
            ctx.warn("removeVertex: triangulate() returned bad results.");
        }

        // Merge the hole triangles back to polygons.
        int[] polys = new int[(ntris + 1) * nvp];
        int[] pregs = new int[ntris];
        int[] pareas = new int[ntris];

        int tmpPoly = ntris * nvp;

        // Build initial polygons.
        int npolys = 0;
        Arrays.fill(polys, 0, ntris * nvp, RC_MESH_NULL_IDX);
        for (int j = 0; j < ntris; ++j) {
            int t = j * 3;
            if (tris[t + 0] != tris[t + 1] && tris[t + 0] != tris[t + 2] && tris[t + 1] != tris[t + 2]) {
                polys[npolys * nvp + 0] = hole[tris[t + 0]];
                polys[npolys * nvp + 1] = hole[tris[t + 1]];
                polys[npolys * nvp + 2] = hole[tris[t + 2]];

                // If this polygon covers multiple region types then
                // mark it as such
                if (hreg[tris[t + 0]] != hreg[tris[t + 1]] || hreg[tris[t + 1]] != hreg[tris[t + 2]])
                    pregs[npolys] = RC_MULTIPLE_REGS;
                else
                    pregs[npolys] = hreg[tris[t + 0]];

                pareas[npolys] = harea[tris[t + 0]];
                npolys++;
            }
        }
        if (npolys == 0)
            return;

        // Merge polygons.
        if (nvp > 3) {
            for (;;) {
                // Find best polygons to merge.
                int bestMergeVal = 0;
                int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

                for (int j = 0; j < npolys - 1; ++j) {
                    int pj = j * nvp;
                    for (int k = j + 1; k < npolys; ++k) {
                        int pk = k * nvp;
                        int[] veaeb = getPolyMergeValue(polys, pj, pk, mesh.verts, nvp);
                        int v = veaeb[0];
                        int ea = veaeb[1];
                        int eb = veaeb[2];
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
                    int pa = bestPa * nvp;
                    int pb = bestPb * nvp;
                    mergePolyVerts(polys, pa, pb, bestEa, bestEb, tmpPoly, nvp);
                    if (pregs[bestPa] != pregs[bestPb])
                        pregs[bestPa] = RC_MULTIPLE_REGS;
                    int last = (npolys - 1) * nvp;
                    if (pb != last) {
                        System.arraycopy(polys, last, polys, pb, nvp);
                    }
                    pregs[bestPb] = pregs[npolys - 1];
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
            int p = mesh.npolys * nvp * 2;
            Arrays.fill(mesh.polys, p, p + nvp * 2, RC_MESH_NULL_IDX);
            for (int j = 0; j < nvp; ++j)
                mesh.polys[p + j] = polys[i * nvp + j];
            mesh.regs[mesh.npolys] = pregs[i];
            mesh.areas[mesh.npolys] = pareas[i];
            mesh.npolys++;
            if (mesh.npolys > maxTris) {
                throw new RuntimeException("removeVertex: Too many polygons " + mesh.npolys + " (max:" + maxTris + ".");
            }
        }

    }

    /// @par
    ///
    /// @note If the mesh data is to be used to construct a Detour navigation mesh, then the upper
    /// limit must be retricted to <= #DT_VERTS_PER_POLYGON.
    ///
    /// @see rcAllocPolyMesh, rcContourSet, rcPolyMesh, rcConfig
    public static PolyMesh buildPolyMesh(Context ctx, ContourSet cset, int nvp) {
        ctx.startTimer("BUILD_POLYMESH");
        PolyMesh mesh = new PolyMesh();
        RecastVectors.copy(mesh.bmin, cset.bmin, 0);
        RecastVectors.copy(mesh.bmax, cset.bmax, 0);
        mesh.cs = cset.cs;
        mesh.ch = cset.ch;
        mesh.borderSize = cset.borderSize;
        mesh.maxEdgeError = cset.maxError;

        int maxVertices = 0;
        int maxTris = 0;
        int maxVertsPerCont = 0;
        for (int i = 0; i < cset.conts.size(); ++i) {
            // Skip null contours.
            if (cset.conts.get(i).nverts < 3)
                continue;
            maxVertices += cset.conts.get(i).nverts;
            maxTris += cset.conts.get(i).nverts - 2;
            maxVertsPerCont = Math.max(maxVertsPerCont, cset.conts.get(i).nverts);
        }
        if (maxVertices >= 0xfffe) {
            throw new RuntimeException("rcBuildPolyMesh: Too many vertices " + maxVertices);
        }
        int[] vflags = new int[maxVertices];

        mesh.verts = new int[maxVertices * 3];
        mesh.polys = new int[maxTris * nvp * 2];
        Arrays.fill(mesh.polys, RC_MESH_NULL_IDX);
        mesh.regs = new int[maxTris];
        mesh.areas = new int[maxTris];

        mesh.nverts = 0;
        mesh.npolys = 0;
        mesh.nvp = nvp;
        mesh.maxpolys = maxTris;

        int[] nextVert = new int[maxVertices];

        int[] firstVert = new int[VERTEX_BUCKET_COUNT];
        for (int i = 0; i < VERTEX_BUCKET_COUNT; ++i)
            firstVert[i] = -1;

        int[] indices = new int[maxVertsPerCont];
        int[] tris = new int[maxVertsPerCont * 3];
        int[] polys = new int[(maxVertsPerCont + 1) * nvp];

        int tmpPoly = maxVertsPerCont * nvp;

        for (int i = 0; i < cset.conts.size(); ++i) {
            Contour cont = cset.conts.get(i);

            // Skip null contours.
            if (cont.nverts < 3)
                continue;

            // Triangulate contour
            for (int j = 0; j < cont.nverts; ++j)
                indices[j] = j;
            int ntris = triangulate(cont.nverts, cont.verts, indices, tris);
            if (ntris <= 0) {
                // Bad triangulation, should not happen.
                ctx.warn("buildPolyMesh: Bad triangulation Contour " + i + ".");
                ntris = -ntris;
            }

            // Add and merge vertices.
            for (int j = 0; j < cont.nverts; ++j) {
                int v = j * 4;
                int[] inv = addVertex(cont.verts[v + 0], cont.verts[v + 1], cont.verts[v + 2], mesh.verts, firstVert,
                        nextVert, mesh.nverts);
                indices[j] = inv[0];
                mesh.nverts = inv[1];
                if ((cont.verts[v + 3] & RC_BORDER_VERTEX) != 0) {
                    // This vertex should be removed.
                    vflags[indices[j]] = 1;
                }
            }

            // Build initial polygons.
            int npolys = 0;
            Arrays.fill(polys, RC_MESH_NULL_IDX);
            for (int j = 0; j < ntris; ++j) {
                int t = j * 3;
                if (tris[t + 0] != tris[t + 1] && tris[t + 0] != tris[t + 2] && tris[t + 1] != tris[t + 2]) {
                    polys[npolys * nvp + 0] = indices[tris[t + 0]];
                    polys[npolys * nvp + 1] = indices[tris[t + 1]];
                    polys[npolys * nvp + 2] = indices[tris[t + 2]];
                    npolys++;
                }
            }
            if (npolys == 0)
                continue;

            // Merge polygons.
            if (nvp > 3) {
                for (;;) {
                    // Find best polygons to merge.
                    int bestMergeVal = 0;
                    int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

                    for (int j = 0; j < npolys - 1; ++j) {
                        int pj = j * nvp;
                        for (int k = j + 1; k < npolys; ++k) {
                            int pk = k * nvp;
                            int[] veaeb = getPolyMergeValue(polys, pj, pk, mesh.verts, nvp);
                            int v = veaeb[0];
                            int ea = veaeb[1];
                            int eb = veaeb[2];
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
                        int pa = bestPa * nvp;
                        int pb = bestPb * nvp;
                        mergePolyVerts(polys, pa, pb, bestEa, bestEb, tmpPoly, nvp);
                        int lastPoly = (npolys - 1) * nvp;
                        if (pb != lastPoly) {
                            System.arraycopy(polys, lastPoly, polys, pb, nvp);
                        }
                        npolys--;
                    } else {
                        // Could not merge any polygons, stop.
                        break;
                    }
                }
            }

            // Store polygons.
            for (int j = 0; j < npolys; ++j) {
                int p = mesh.npolys * nvp * 2;
                int q = j * nvp;
                for (int k = 0; k < nvp; ++k)
                    mesh.polys[p + k] = polys[q + k];
                mesh.regs[mesh.npolys] = cont.reg;
                mesh.areas[mesh.npolys] = cont.area;
                mesh.npolys++;
                if (mesh.npolys > maxTris) {
                    throw new RuntimeException(
                            "rcBuildPolyMesh: Too many polygons " + mesh.npolys + " (max:" + maxTris + ").");
                }
            }
        }

        // Remove edge vertices.
        for (int i = 0; i < mesh.nverts; ++i) {
            if (vflags[i] != 0) {
                if (!canRemoveVertex(ctx, mesh, i))
                    continue;
                removeVertex(ctx, mesh, i, maxTris);
                // Remove vertex
                // Note: mesh.nverts is already decremented inside removeVertex()!
                // Fixup vertex flags
                for (int j = i; j < mesh.nverts; ++j)
                    vflags[j] = vflags[j + 1];
                --i;
            }
        }

        // Calculate adjacency.
        buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, nvp);

        // Find portal edges
        if (mesh.borderSize > 0) {
            int w = cset.width;
            int h = cset.height;
            for (int i = 0; i < mesh.npolys; ++i) {
                int p = i * 2 * nvp;
                for (int j = 0; j < nvp; ++j) {
                    if (mesh.polys[p + j] == RC_MESH_NULL_IDX)
                        break;
                    // Skip connected edges.
                    if (mesh.polys[p + nvp + j] != RC_MESH_NULL_IDX)
                        continue;
                    int nj = j + 1;
                    if (nj >= nvp || mesh.polys[p + nj] == RC_MESH_NULL_IDX)
                        nj = 0;
                    int va = mesh.polys[p + j] * 3;
                    int vb = mesh.polys[p + nj] * 3;

                    if (mesh.verts[va + 0] == 0 && mesh.verts[vb + 0] == 0)
                        mesh.polys[p + nvp + j] = 0x8000 | 0;
                    else if (mesh.verts[va + 2] == h && mesh.verts[vb + 2] == h)
                        mesh.polys[p + nvp + j] = 0x8000 | 1;
                    else if (mesh.verts[va + 0] == w && mesh.verts[vb + 0] == w)
                        mesh.polys[p + nvp + j] = 0x8000 | 2;
                    else if (mesh.verts[va + 2] == 0 && mesh.verts[vb + 2] == 0)
                        mesh.polys[p + nvp + j] = 0x8000 | 3;
                }
            }
        }

        // Just allocate the mesh flags array. The user is resposible to fill it.
        mesh.flags = new int[mesh.npolys];

        if (mesh.nverts > 0xffff) {
            throw new RuntimeException("rcBuildPolyMesh: The resulting mesh has too many vertices " + mesh.nverts
                    + " (max " + 0xffff + "). Data can be corrupted.");
        }
        if (mesh.npolys > 0xffff) {
            throw new RuntimeException("rcBuildPolyMesh: The resulting mesh has too many polygons " + mesh.npolys
                    + " (max " + 0xffff + "). Data can be corrupted.");
        }

        ctx.stopTimer("BUILD_POLYMESH");
        return mesh;

    }

    /// @see rcAllocPolyMesh, rcPolyMesh
    public static PolyMesh mergePolyMeshes(Context ctx, PolyMesh[] meshes, int nmeshes) {

        if (nmeshes == 0 || meshes == null)
            return null;

        ctx.startTimer("MERGE_POLYMESH");
        PolyMesh mesh = new PolyMesh();
        mesh.nvp = meshes[0].nvp;
        mesh.cs = meshes[0].cs;
        mesh.ch = meshes[0].ch;
        RecastVectors.copy(mesh.bmin, meshes[0].bmin, 0);
        RecastVectors.copy(mesh.bmax, meshes[0].bmax, 0);

        int maxVerts = 0;
        int maxPolys = 0;
        int maxVertsPerMesh = 0;
        for (int i = 0; i < nmeshes; ++i) {
            RecastVectors.min(mesh.bmin, meshes[i].bmin, 0);
            RecastVectors.max(mesh.bmax, meshes[i].bmax, 0);
            maxVertsPerMesh = Math.max(maxVertsPerMesh, meshes[i].nverts);
            maxVerts += meshes[i].nverts;
            maxPolys += meshes[i].npolys;
        }

        mesh.nverts = 0;
        mesh.verts = new int[maxVerts * 3];

        mesh.npolys = 0;
        mesh.polys = new int[maxPolys * 2 * mesh.nvp];
        Arrays.fill(mesh.polys, 0, mesh.polys.length, RC_MESH_NULL_IDX);
        mesh.regs = new int[maxPolys];
        mesh.areas = new int[maxPolys];
        mesh.flags = new int[maxPolys];

        int[] nextVert = new int[maxVerts];

        int[] firstVert = new int[VERTEX_BUCKET_COUNT];
        for (int i = 0; i < VERTEX_BUCKET_COUNT; ++i)
            firstVert[i] = -1;

        int[] vremap = new int[maxVertsPerMesh];

        for (int i = 0; i < nmeshes; ++i) {
            PolyMesh pmesh = meshes[i];

            int ox = (int) Math.floor((pmesh.bmin[0] - mesh.bmin[0]) / mesh.cs + 0.5f);
            int oz = (int) Math.floor((pmesh.bmin[2] - mesh.bmin[2]) / mesh.cs + 0.5f);

            boolean isMinX = (ox == 0);
            boolean isMinZ = (oz == 0);
            boolean isMaxX = (Math.floor((mesh.bmax[0] - pmesh.bmax[0]) / mesh.cs + 0.5f)) == 0;
            boolean isMaxZ = (Math.floor((mesh.bmax[2] - pmesh.bmax[2]) / mesh.cs + 0.5f)) == 0;
            boolean isOnBorder = (isMinX || isMinZ || isMaxX || isMaxZ);

            for (int j = 0; j < pmesh.nverts; ++j) {
                int v = j * 3;
                int[] inv = addVertex(pmesh.verts[v + 0] + ox, pmesh.verts[v + 1], pmesh.verts[v + 2] + oz, mesh.verts,
                        firstVert, nextVert, mesh.nverts);

                vremap[j] = inv[0];
                mesh.nverts = inv[1];
            }

            for (int j = 0; j < pmesh.npolys; ++j) {
                int tgt = mesh.npolys * 2 * mesh.nvp;
                int src = j * 2 * mesh.nvp;
                mesh.regs[mesh.npolys] = pmesh.regs[j];
                mesh.areas[mesh.npolys] = pmesh.areas[j];
                mesh.flags[mesh.npolys] = pmesh.flags[j];
                mesh.npolys++;
                for (int k = 0; k < mesh.nvp; ++k) {
                    if (pmesh.polys[src + k] == RC_MESH_NULL_IDX)
                        break;
                    mesh.polys[tgt + k] = vremap[pmesh.polys[src + k]];
                }

                if (isOnBorder) {
                    for (int k = mesh.nvp; k < mesh.nvp * 2; ++k) {
                        if ((pmesh.polys[src + k] & 0x8000) != 0 && pmesh.polys[src + k] != 0xffff) {
                            int dir = pmesh.polys[src + k] & 0xf;
                            switch (dir) {
                            case 0: // Portal x-
                                if (isMinX)
                                    mesh.polys[tgt + k] = pmesh.polys[src + k];
                                break;
                            case 1: // Portal z+
                                if (isMaxZ)
                                    mesh.polys[tgt + k] = pmesh.polys[src + k];
                                break;
                            case 2: // Portal x+
                                if (isMaxX)
                                    mesh.polys[tgt + k] = pmesh.polys[src + k];
                                break;
                            case 3: // Portal z-
                                if (isMinZ)
                                    mesh.polys[tgt + k] = pmesh.polys[src + k];
                                break;
                            }
                        }
                    }
                }
            }
        }

        // Calculate adjacency.
        buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, mesh.nvp);
        if (mesh.nverts > 0xffff) {
            throw new RuntimeException("rcBuildPolyMesh: The resulting mesh has too many vertices " + mesh.nverts
                    + " (max " + 0xffff + "). Data can be corrupted.");
        }
        if (mesh.npolys > 0xffff) {
            throw new RuntimeException("rcBuildPolyMesh: The resulting mesh has too many polygons " + mesh.npolys
                    + " (max " + 0xffff + "). Data can be corrupted.");
        }

        ctx.stopTimer("MERGE_POLYMESH");

        return mesh;
    }

    public static PolyMesh copyPolyMesh(Context ctx, PolyMesh src) {
        PolyMesh dst = new PolyMesh();

        dst.nverts = src.nverts;
        dst.npolys = src.npolys;
        dst.maxpolys = src.npolys;
        dst.nvp = src.nvp;
        RecastVectors.copy(dst.bmin, src.bmin, 0);
        RecastVectors.copy(dst.bmax, src.bmax, 0);
        dst.cs = src.cs;
        dst.ch = src.ch;
        dst.borderSize = src.borderSize;
        dst.maxEdgeError = src.maxEdgeError;

        dst.verts = new int[src.nverts * 3];
        System.arraycopy(src.verts, 0, dst.verts, 0, dst.verts.length);
        dst.polys = new int[src.npolys * 2 * src.nvp];
        System.arraycopy(src.polys, 0, dst.polys, 0, dst.polys.length);
        dst.regs = new int[src.npolys];
        System.arraycopy(src.regs, 0, dst.regs, 0, dst.regs.length);
        dst.areas = new int[src.npolys];
        System.arraycopy(src.areas, 0, dst.areas, 0, dst.areas.length);
        dst.flags = new int[src.npolys];
        System.arraycopy(src.flags, 0, dst.flags, 0, dst.flags.length);
        return dst;
    }
}
