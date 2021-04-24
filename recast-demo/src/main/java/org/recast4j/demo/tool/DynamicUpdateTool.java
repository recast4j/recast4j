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

package org.recast4j.demo.tool;

import static org.lwjgl.nuklear.Nuklear.*;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.recast4j.demo.math.DemoMath.vCross;
import static org.recast4j.detour.DetourCommon.vNormalize;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.nuklear.NkContext;
import org.lwjgl.system.MemoryStack;
import org.lwjgl.util.tinyfd.TinyFileDialogs;
import org.recast4j.demo.RecastBuilderThreadFactory;
import org.recast4j.demo.builder.SampleAreaModifications;
import org.recast4j.demo.draw.GLU;
import org.recast4j.demo.draw.NavMeshRenderer;
import org.recast4j.demo.geom.DemoInputGeomProvider;
import org.recast4j.demo.io.ObjImporter;
import org.recast4j.demo.sample.Sample;
import org.recast4j.demo.ui.NuklearUIHelper;
import org.recast4j.detour.Tupple2;
import org.recast4j.dynamic.DynamicNavMesh;
import org.recast4j.dynamic.collider.BoxCollider;
import org.recast4j.dynamic.collider.CapsuleCollider;
import org.recast4j.dynamic.collider.Collider;
import org.recast4j.dynamic.collider.CompositeCollider;
import org.recast4j.dynamic.collider.CylinderCollider;
import org.recast4j.dynamic.collider.SphereCollider;
import org.recast4j.dynamic.collider.TrimeshCollider;
import org.recast4j.dynamic.io.VoxelFile;
import org.recast4j.dynamic.io.VoxelFileReader;
import org.recast4j.dynamic.io.VoxelFileWriter;
import org.recast4j.recast.RecastConstants.PartitionType;

public class DynamicUpdateTool implements Tool {

    private enum ColliderShape {
        SPHERE, CAPSULE, BOX, CYLINDER, COMPOSITE, CONVEX, TRIMESH_BRIDGE, TRIMESH_HOUSE
    }

    private Sample sample;
    private final FloatBuffer cellSize = BufferUtils.createFloatBuffer(1).put(0, 0.3f);
    private PartitionType partitioning = PartitionType.WATERSHED;
    private boolean filterLowHangingObstacles = true;
    private boolean filterLedgeSpans = true;
    private boolean filterWalkableLowHeightSpans = true;
    private final FloatBuffer walkableHeight = BufferUtils.createFloatBuffer(1).put(0, 2f);
    private final FloatBuffer walkableRadius = BufferUtils.createFloatBuffer(1).put(0, 0.6f);
    private final FloatBuffer walkableClimb = BufferUtils.createFloatBuffer(1).put(0, 0.9f);
    private final FloatBuffer walkableSlopeAngle = BufferUtils.createFloatBuffer(1).put(0, 45f);
    private final FloatBuffer minRegionArea = BufferUtils.createFloatBuffer(1).put(0, 6);
    private final FloatBuffer regionMergeSize = BufferUtils.createFloatBuffer(1).put(0, 36);
    private final FloatBuffer maxEdgeLen = BufferUtils.createFloatBuffer(1).put(0, 12f);
    private final FloatBuffer maxSimplificationError = BufferUtils.createFloatBuffer(1).put(0, 1.3f);
    private final IntBuffer vertsPerPoly = BufferUtils.createIntBuffer(1).put(0, 6);
    private boolean buildDetailMesh = true;
    private final FloatBuffer detailSampleDist = BufferUtils.createFloatBuffer(1).put(0, 6f);
    private final FloatBuffer detailSampleMaxError = BufferUtils.createFloatBuffer(1).put(0, 1f);
    private boolean showColliders = false;
    private long buildTime;
    private ColliderShape colliderShape = ColliderShape.SPHERE;

    private DynamicNavMesh dynaMesh;
    private final ExecutorService executor;
    private final Map<Long, Collider> colliders = new HashMap<>();
    private final Map<Long, ColliderGizmo> colliderGizmos = new HashMap<>();
    private final Random random = new Random();
    private final DemoInputGeomProvider bridgeGeom;
    private final DemoInputGeomProvider houseGeom;
    private final DemoInputGeomProvider convexGeom;

    public DynamicUpdateTool() {
        executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors() / 2, new RecastBuilderThreadFactory());
        bridgeGeom = new ObjImporter().load(getClass().getClassLoader().getResourceAsStream("bridge.obj"));
        houseGeom = new ObjImporter().load(getClass().getClassLoader().getResourceAsStream("house.obj"));
        convexGeom = new ObjImporter().load(getClass().getClassLoader().getResourceAsStream("convex.obj"));
    }

    @Override
    public void setSample(Sample sample) {
        this.sample = sample;
    }

    @Override
    public void handleClick(float[] s, float[] p, boolean shift) {
        if (!shift) {
            Tupple2<Collider, ColliderGizmo> colliderWithGizmo = null;
            if (dynaMesh != null) {
                if (colliderShape == ColliderShape.SPHERE) {
                    colliderWithGizmo = sphereCollider(p);
                } else if (colliderShape == ColliderShape.CAPSULE) {
                    colliderWithGizmo = capsuleCollider(p);
                } else if (colliderShape == ColliderShape.BOX) {
                    colliderWithGizmo = boxCollider(p);
                } else if (colliderShape == ColliderShape.CYLINDER) {
                    colliderWithGizmo = cylinderCollider(p);
                } else if (colliderShape == ColliderShape.COMPOSITE) {
                    colliderWithGizmo = compositeCollider(p);
                } else if (colliderShape == ColliderShape.TRIMESH_BRIDGE) {
                    colliderWithGizmo = trimeshBridge(p);
                } else if (colliderShape == ColliderShape.TRIMESH_HOUSE) {
                    colliderWithGizmo = trimeshHouse(p);
                } else if (colliderShape == ColliderShape.CONVEX) {
                    colliderWithGizmo = convexTrimesh(p);
                }
            }
            if (colliderWithGizmo != null) {
                long id = dynaMesh.addCollider(colliderWithGizmo.first);
                colliders.put(id, colliderWithGizmo.first);
                colliderGizmos.put(id, colliderWithGizmo.second);
            }
        }
    }

    private Tupple2<Collider, ColliderGizmo> sphereCollider(float[] p) {
        float radius = 1 + random.nextFloat() * 10;
        return new Tupple2<>(
                new SphereCollider(p, radius, SampleAreaModifications.SAMPLE_POLYAREA_TYPE_WATER, dynaMesh.config.walkableClimb),
                ColliderGizmo.sphere(p, radius));
    }

    private Tupple2<Collider, ColliderGizmo> capsuleCollider(float[] p) {
        float radius = 0.4f + random.nextFloat() * 4f;
        float[] a = new float[] { (1f - 2 * random.nextFloat()), 0.01f + random.nextFloat(), (1f - 2 * random.nextFloat()) };
        vNormalize(a);
        float len = 1f + random.nextFloat() * 20f;
        a[0] *= len;
        a[1] *= len;
        a[2] *= len;
        float[] start = new float[] { p[0], p[1], p[2] };
        float[] end = new float[] { p[0] + a[0], p[1] + a[1], p[2] + a[2] };
        return new Tupple2<>(new CapsuleCollider(start, end, radius, SampleAreaModifications.SAMPLE_POLYAREA_TYPE_WATER,
                dynaMesh.config.walkableClimb), ColliderGizmo.capsule(start, end, radius));
    }

    private Tupple2<Collider, ColliderGizmo> boxCollider(float[] p) {
        float[] extent = new float[] { 0.5f + random.nextFloat() * 6f, 0.5f + random.nextFloat() * 6f,
                0.5f + random.nextFloat() * 6f };
        float[] forward = new float[] { (1f - 2 * random.nextFloat()), 0, (1f - 2 * random.nextFloat()) };
        float[] up = new float[] { (1f - 2 * random.nextFloat()), 0.01f + random.nextFloat(), (1f - 2 * random.nextFloat()) };
        float[][] halfEdges = BoxCollider.getHalfEdges(up, forward, extent);
        return new Tupple2<>(
                new BoxCollider(p, halfEdges, SampleAreaModifications.SAMPLE_POLYAREA_TYPE_WATER, dynaMesh.config.walkableClimb),
                ColliderGizmo.box(p, halfEdges));
    }

    private Tupple2<Collider, ColliderGizmo> cylinderCollider(float[] p) {
        float radius = 0.7f + random.nextFloat() * 4f;
        float[] a = new float[] { (1f - 2 * random.nextFloat()), 0.01f + random.nextFloat(), (1f - 2 * random.nextFloat()) };
        vNormalize(a);
        float len = 2f + random.nextFloat() * 20f;
        a[0] *= len;
        a[1] *= len;
        a[2] *= len;
        float[] start = new float[] { p[0], p[1], p[2] };
        float[] end = new float[] { p[0] + a[0], p[1] + a[1], p[2] + a[2] };
        return new Tupple2<>(new CylinderCollider(start, end, radius, SampleAreaModifications.SAMPLE_POLYAREA_TYPE_WATER,
                dynaMesh.config.walkableClimb), ColliderGizmo.cylinder(start, end, radius));
    }

    private Tupple2<Collider, ColliderGizmo> compositeCollider(float[] p) {
        float[] baseExtent = new float[] { 5, 3, 8 };
        float[] baseCenter = new float[] { p[0], p[1] + 3, p[2] };
        float[] baseUp = new float[] { 0, 1, 0 };
        float[] forward = new float[] { (1f - 2 * random.nextFloat()), 0, (1f - 2 * random.nextFloat()) };
        vNormalize(forward);
        float[] side = vCross(forward, baseUp);
        BoxCollider base = new BoxCollider(baseCenter, BoxCollider.getHalfEdges(baseUp, forward, baseExtent),
                SampleAreaModifications.SAMPLE_POLYAREA_TYPE_ROAD, dynaMesh.config.walkableClimb);
        float[] roofExtent = new float[] { 4.5f, 4.5f, 8f };
        float[] rx = GLU.build_4x4_rotation_matrix(45, forward[0], forward[1], forward[2]);
        float[] roofUp = mulMatrixVector(new float[3], rx, baseUp);
        float[] roofCenter = new float[] { p[0], p[1] + 6, p[2] };
        BoxCollider roof = new BoxCollider(roofCenter, BoxCollider.getHalfEdges(roofUp, forward, roofExtent),
                SampleAreaModifications.SAMPLE_POLYAREA_TYPE_ROAD, dynaMesh.config.walkableClimb);
        float[] trunkStart = new float[] { baseCenter[0] - forward[0] * 15 + side[0] * 6, p[1],
                baseCenter[2] - forward[2] * 15 + side[2] * 6 };
        float[] trunkEnd = new float[] { trunkStart[0], trunkStart[1] + 10, trunkStart[2] };
        CapsuleCollider trunk = new CapsuleCollider(trunkStart, trunkEnd, 0.5f, SampleAreaModifications.SAMPLE_POLYAREA_TYPE_ROAD,
                dynaMesh.config.walkableClimb);
        float[] crownCenter = new float[] { baseCenter[0] - forward[0] * 15 + side[0] * 6, p[1] + 10,
                baseCenter[2] - forward[2] * 15 + side[2] * 6 };
        SphereCollider crown = new SphereCollider(crownCenter, 4f, SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GRASS,
                dynaMesh.config.walkableClimb);
        CompositeCollider collider = new CompositeCollider(base, roof, trunk, crown);
        ColliderGizmo baseGizmo = ColliderGizmo.box(baseCenter, BoxCollider.getHalfEdges(baseUp, forward, baseExtent));
        ColliderGizmo roofGizmo = ColliderGizmo.box(roofCenter, BoxCollider.getHalfEdges(roofUp, forward, roofExtent));
        ColliderGizmo trunkGizmo = ColliderGizmo.capsule(trunkStart, trunkEnd, 0.5f);
        ColliderGizmo crownGizmo = ColliderGizmo.sphere(crownCenter, 4f);
        ColliderGizmo gizmo = ColliderGizmo.composite(baseGizmo, roofGizmo, trunkGizmo, crownGizmo);
        return new Tupple2<>(collider, gizmo);
    }

    private Tupple2<Collider, ColliderGizmo> trimeshBridge(float[] p) {
        return trimeshCollider(p, bridgeGeom, 0);
    }

    private Tupple2<Collider, ColliderGizmo> trimeshHouse(float[] p) {
        return trimeshCollider(p, houseGeom, 0);
    }

    private Tupple2<Collider, ColliderGizmo> convexTrimesh(float[] p) {
        return trimeshCollider(p, convexGeom, 360);
    }

    private Tupple2<Collider, ColliderGizmo> trimeshCollider(float[] p, DemoInputGeomProvider geom, float ax) {
        float[] rx = GLU.build_4x4_rotation_matrix(random.nextFloat() * ax, 1, 0, 0);
        float[] ry = GLU.build_4x4_rotation_matrix(random.nextFloat() * 360, 0, 1, 0);
        float[] m = GLU.mul(rx, ry);
        float[] verts = new float[geom.vertices.length];
        float[] v = new float[3];
        float[] vr = new float[3];
        float[] bounds = new float[] { Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY };
        for (int i = 0; i < geom.vertices.length; i += 3) {
            v[0] = geom.vertices[i];
            v[1] = geom.vertices[i + 1];
            v[2] = geom.vertices[i + 2];
            mulMatrixVector(vr, m, v);
            vr[0] += p[0];
            vr[1] += p[1] - 0.1f;
            vr[2] += p[2];
            verts[i] = vr[0];
            verts[i + 1] = vr[1];
            verts[i + 2] = vr[2];
            bounds[0] = Math.min(bounds[0], vr[0]);
            bounds[1] = Math.min(bounds[1], vr[1]);
            bounds[2] = Math.min(bounds[2], vr[2]);
            bounds[3] = Math.max(bounds[3], vr[0]);
            bounds[4] = Math.max(bounds[4], vr[1]);
            bounds[5] = Math.max(bounds[5], vr[2]);
        }
        TrimeshCollider collider = new TrimeshCollider(verts, geom.faces, bounds,
                SampleAreaModifications.SAMPLE_POLYAREA_TYPE_ROAD, dynaMesh.config.walkableClimb * 10);
        return new Tupple2<>(collider, ColliderGizmo.trimesh(verts, geom.faces));
    }

    private float[] mulMatrixVector(float[] resultvector, float[] matrix, float[] pvector) {
        resultvector[0] = matrix[0] * pvector[0] + matrix[4] * pvector[1] + matrix[8] * pvector[2];
        resultvector[1] = matrix[1] * pvector[0] + matrix[5] * pvector[1] + matrix[9] * pvector[2];
        resultvector[2] = matrix[2] * pvector[0] + matrix[6] * pvector[1] + matrix[10] * pvector[2];
        return resultvector;
    }

    @Override
    public void handleClickRay(float[] start, float[] dir, boolean shift) {
        if (shift) {
            for (Entry<Long, Collider> e : colliders.entrySet()) {
                if (hit(start, dir, e.getValue().bounds())) {
                    dynaMesh.removeCollider(e.getKey());
                    colliders.remove(e.getKey());
                    colliderGizmos.remove(e.getKey());
                    break;
                }
            }
        }
    }

    private boolean hit(float[] point, float[] dir, float[] bounds) {
        float cx = 0.5f * (bounds[0] + bounds[3]);
        float cy = 0.5f * (bounds[1] + bounds[4]);
        float cz = 0.5f * (bounds[2] + bounds[5]);
        float dx = 0.5f * (bounds[3] - bounds[0]);
        float dy = 0.5f * (bounds[4] - bounds[1]);
        float dz = 0.5f * (bounds[5] - bounds[2]);
        float rSqr = dx * dx + dy * dy + dz * dz;
        float mx = point[0] - cx;
        float my = point[1] - cy;
        float mz = point[2] - cz;
        float c = mx * mx + my * my + mz * mz - rSqr;
        if (c <= 0.0f) {
            return true;
        }
        float b = mx * dir[0] + my * dir[1] + mz * dir[2];
        if (b > 0.0f) {
            return false;
        }
        float disc = b * b - c;
        return disc >= 0.0f;
    }

    @Override
    public void handleRender(NavMeshRenderer renderer) {
        if (showColliders) {
            colliderGizmos.values().forEach(g -> g.render(renderer.getDebugDraw()));
        }
    }

    @Override
    public void handleUpdate(float dt) {
        if (dynaMesh != null) {
            updateDynaMesh();
        }
    }

    private void updateDynaMesh() {
        long t = System.nanoTime();
        try {
            boolean updated = dynaMesh.update(executor).get();
            if (updated) {
                buildTime = (System.nanoTime() - t) / 1_000_000;
                sample.update(null, dynaMesh.recastResults(), dynaMesh.navMesh());
                sample.setChanged(false);
            }
        } catch (InterruptedException | ExecutionException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void layout(NkContext ctx) {

        nk_layout_row_dynamic(ctx, 18, 1);
        if (nk_button_text(ctx, "Load Voxels...")) {
            load();
        }
        if (dynaMesh != null) {
            if (nk_button_text(ctx, "Save Voxels...")) {
                save();
            }
        }
        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, "Rasterization", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 18, 2);
        nk_label(ctx, "Cell Size", NK_TEXT_ALIGN_LEFT);
        nk_label(ctx, String.format("%.2f", cellSize.get(0)), NK_TEXT_ALIGN_RIGHT);
        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, "Agent", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Height", 0f, walkableHeight, 5f, 0.01f, 0.01f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Radius", 0f, walkableRadius, 10f, 0.01f, 0.01f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Max Climb", 0f, walkableClimb, 10f, 0.01f, 0.01f);
        nk_layout_row_dynamic(ctx, 18, 2);
        nk_label(ctx, "Max Slope", NK_TEXT_ALIGN_LEFT);
        nk_label(ctx, String.format("%.0f", walkableSlopeAngle.get(0)), NK_TEXT_ALIGN_RIGHT);

        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, "Partitioning", NK_TEXT_ALIGN_LEFT);
        partitioning = NuklearUIHelper.nk_radio(ctx, PartitionType.values(), partitioning,
                p -> p.name().substring(0, 1) + p.name().substring(1).toLowerCase());

        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, "Filtering", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 18, 1);
        filterLowHangingObstacles = nk_option_text(ctx, "Low Hanging Obstacles", filterLowHangingObstacles);
        nk_layout_row_dynamic(ctx, 18, 1);
        filterLedgeSpans = nk_option_text(ctx, "Ledge Spans", filterLedgeSpans);
        nk_layout_row_dynamic(ctx, 18, 1);
        filterWalkableLowHeightSpans = nk_option_text(ctx, "Walkable Low Height Spans", filterWalkableLowHeightSpans);

        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, "Region", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Min Region Size", 0, minRegionArea, 150, 0.1f, 0.1f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Merged Region Size", 0, regionMergeSize, 400, 0.1f, 0.1f);

        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, "Polygonization", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Max Edge Length", 0f, maxEdgeLen, 50f, 0.1f, 0.1f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Max Edge Error", 0.1f, maxSimplificationError, 10f, 0.1f, 0.1f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_int(ctx, "Verts Per Poly", 3, vertsPerPoly, 12, 1, 1);

        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, "Detail Mesh", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        buildDetailMesh = nk_check_text(ctx, "Enable", buildDetailMesh);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Sample Distance", 0f, detailSampleDist, 16f, 0.1f, 0.1f);
        nk_layout_row_dynamic(ctx, 20, 1);
        nk_property_float(ctx, "Max Sample Error", 0f, detailSampleMaxError, 16f, 0.1f, 0.1f);

        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, String.format("Build Time: %d ms", buildTime), NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_button_text(ctx, "Build")) {
            if (dynaMesh != null) {
                buildDynaMesh();
                sample.setChanged(false);
            }
        }

        nk_layout_row_dynamic(ctx, 1, 1);
        nk_spacing(ctx, 1);
        nk_layout_row_dynamic(ctx, 18, 1);
        nk_label(ctx, "Colliders", NK_TEXT_ALIGN_LEFT);
        nk_layout_row_dynamic(ctx, 20, 1);
        showColliders = nk_check_text(ctx, "Show", showColliders);
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_option_label(ctx, "Sphere", colliderShape == ColliderShape.SPHERE)) {
            colliderShape = ColliderShape.SPHERE;
        }
        nk_layout_row_dynamic(ctx, 18, 1);
        if (nk_option_label(ctx, "Capsule", colliderShape == ColliderShape.CAPSULE)) {
            colliderShape = ColliderShape.CAPSULE;
        }
        nk_layout_row_dynamic(ctx, 18, 1);
        if (nk_option_label(ctx, "Box", colliderShape == ColliderShape.BOX)) {
            colliderShape = ColliderShape.BOX;
        }
        nk_layout_row_dynamic(ctx, 18, 1);
        if (nk_option_label(ctx, "Cylinder", colliderShape == ColliderShape.CYLINDER)) {
            colliderShape = ColliderShape.CYLINDER;
        }
        nk_layout_row_dynamic(ctx, 18, 1);
        if (nk_option_label(ctx, "Composite", colliderShape == ColliderShape.COMPOSITE)) {
            colliderShape = ColliderShape.COMPOSITE;
        }
        nk_layout_row_dynamic(ctx, 18, 1);
        if (nk_option_label(ctx, "Convex Trimesh", colliderShape == ColliderShape.CONVEX)) {
            colliderShape = ColliderShape.CONVEX;
        }
        nk_layout_row_dynamic(ctx, 18, 1);
        if (nk_option_label(ctx, "Trimesh Bridge", colliderShape == ColliderShape.TRIMESH_BRIDGE)) {
            colliderShape = ColliderShape.TRIMESH_BRIDGE;
        }
        nk_layout_row_dynamic(ctx, 18, 1);
        if (nk_option_label(ctx, "Trimesh House", colliderShape == ColliderShape.TRIMESH_HOUSE)) {
            colliderShape = ColliderShape.TRIMESH_HOUSE;
        }

    }

    private void load() {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer aFilterPatterns = stack.mallocPointer(1);
            aFilterPatterns.put(stack.UTF8("*.voxels"));
            aFilterPatterns.flip();
            String filename = TinyFileDialogs.tinyfd_openFileDialog("Open Voxel File", "", aFilterPatterns, "Voxel File", false);
            if (filename != null) {
                load(filename);
            }
        }

    }

    private void load(String filename) {
        File file = new File(filename);
        if (file.exists()) {
            VoxelFileReader reader = new VoxelFileReader();
            try (FileInputStream fis = new FileInputStream(file)) {
                VoxelFile voxelFile = reader.read(fis);
                dynaMesh = new DynamicNavMesh(voxelFile);
                dynaMesh.config.keepIntermediateResults = true;
                updateUI();
                buildDynaMesh();
                colliders.clear();
            } catch (Exception e) {
                e.printStackTrace();
                dynaMesh = null;
            }
        }
    }

    private void save() {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer aFilterPatterns = stack.mallocPointer(1);
            aFilterPatterns.put(stack.UTF8("*.voxels"));
            aFilterPatterns.flip();
            String filename = TinyFileDialogs.tinyfd_saveFileDialog("Save Voxel File", "", aFilterPatterns, "Voxel File");
            if (filename != null) {
                save(filename);
            }
        }

    }

    private void save(String filename) {
        File file = new File(filename);
        try (FileOutputStream fos = new FileOutputStream(file)) {
            VoxelFile voxelFile = VoxelFile.from(dynaMesh);
            VoxelFileWriter writer = new VoxelFileWriter();
            writer.write(fos, voxelFile);
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    private void buildDynaMesh() {
        configDynaMesh();
        long t = System.nanoTime();
        try {
            dynaMesh.build(executor).get();
        } catch (InterruptedException | ExecutionException e) {
            e.printStackTrace();
        }
        buildTime = (System.nanoTime() - t) / 1_000_000;
        sample.update(null, dynaMesh.recastResults(), dynaMesh.navMesh());
    }

    private void configDynaMesh() {
        dynaMesh.config.partitionType = partitioning;
        dynaMesh.config.walkableHeight = walkableHeight.get(0);
        dynaMesh.config.walkableSlopeAngle = walkableSlopeAngle.get(0);
        dynaMesh.config.walkableRadius = walkableRadius.get(0);
        dynaMesh.config.walkableClimb = walkableClimb.get(0);
        dynaMesh.config.filterLowHangingObstacles = filterLowHangingObstacles;
        dynaMesh.config.filterLedgeSpans = filterLedgeSpans;
        dynaMesh.config.filterWalkableLowHeightSpans = filterWalkableLowHeightSpans;
        dynaMesh.config.minRegionArea = minRegionArea.get(0);
        dynaMesh.config.regionMergeArea = regionMergeSize.get(0);
        dynaMesh.config.maxEdgeLen = maxEdgeLen.get(0);
        dynaMesh.config.maxSimplificationError = maxSimplificationError.get(0);
        dynaMesh.config.vertsPerPoly = vertsPerPoly.get(0);
        dynaMesh.config.buildDetailMesh = buildDetailMesh;
        dynaMesh.config.detailSampleDistance = detailSampleDist.get(0);
        dynaMesh.config.detailSampleMaxError = detailSampleMaxError.get(0);
    }

    private void updateUI() {
        cellSize.put(0, dynaMesh.config.cellSize);
        partitioning = dynaMesh.config.partitionType;
        walkableHeight.put(0, dynaMesh.config.walkableHeight);
        walkableSlopeAngle.put(0, dynaMesh.config.walkableSlopeAngle);
        walkableRadius.put(0, dynaMesh.config.walkableRadius);
        walkableClimb.put(0, dynaMesh.config.walkableClimb);
        minRegionArea.put(0, dynaMesh.config.minRegionArea);
        regionMergeSize.put(0, dynaMesh.config.regionMergeArea);
        maxEdgeLen.put(0, dynaMesh.config.maxEdgeLen);
        maxSimplificationError.put(0, dynaMesh.config.maxSimplificationError);
        vertsPerPoly.put(0, dynaMesh.config.vertsPerPoly);
        buildDetailMesh = dynaMesh.config.buildDetailMesh;
        detailSampleDist.put(0, dynaMesh.config.detailSampleDistance);
        detailSampleMaxError.put(0, dynaMesh.config.detailSampleMaxError);
        filterLowHangingObstacles = dynaMesh.config.filterLowHangingObstacles;
        filterLedgeSpans = dynaMesh.config.filterLedgeSpans;
        filterWalkableLowHeightSpans = dynaMesh.config.filterWalkableLowHeightSpans;
    }

    @Override
    public String getName() {
        return "Dynamic Updates";
    }

}
