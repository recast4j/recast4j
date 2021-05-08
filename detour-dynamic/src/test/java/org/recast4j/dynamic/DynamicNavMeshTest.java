package org.recast4j.dynamic;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;

import org.junit.Test;
import org.recast4j.detour.DefaultQueryFilter;
import org.recast4j.detour.FindNearestPolyResult;
import org.recast4j.detour.NavMeshQuery;
import org.recast4j.detour.QueryFilter;
import org.recast4j.dynamic.collider.Collider;
import org.recast4j.dynamic.collider.SphereCollider;
import org.recast4j.dynamic.io.VoxelFile;
import org.recast4j.dynamic.io.VoxelFileReader;

public class DynamicNavMeshTest {

    private static final float[] START_POS = new float[] { 70.87453f, 0.0010070801f, 86.69021f };
    private static final float[] END_POS = new float[] { -50.22061f, 0.0010070801f, -70.761444f };
    private static final float[] EXTENT = new float[] { 0.1f, 0.1f, 0.1f };
    private static final float[] SPHERE_POS = new float[] { 45.381645f, 0.0010070801f, 52.68981f };


    @Test
    public void e2eTest() throws IOException, InterruptedException, ExecutionException {
        try (InputStream is = getClass().getClassLoader().getResourceAsStream("test_tiles.voxels")) {
            ExecutorService executor = TestExecutorService.get();
            // load voxels from file
            VoxelFileReader reader = new VoxelFileReader();
            VoxelFile f = reader.read(is);
            // create dynamic navmesh
            DynamicNavMesh mesh = new DynamicNavMesh(f);
            // build navmesh asynchronously using multiple threads
            CompletableFuture<Boolean> future = mesh.build(executor);
            // wait for build to complete
            future.get();
            // create new query
            NavMeshQuery query = new NavMeshQuery(mesh.navMesh());
            QueryFilter filter = new DefaultQueryFilter();
            // find path
            FindNearestPolyResult start = query.findNearestPoly(START_POS, EXTENT, filter).result;
            FindNearestPolyResult end = query.findNearestPoly(END_POS, EXTENT, filter).result;
            List<Long> path = query.findPath(start.getNearestRef(), end.getNearestRef(), start.getNearestPos(),
                    end.getNearestPos(), filter, NavMeshQuery.DT_FINDPATH_ANY_ANGLE, Float.MAX_VALUE).result;
            // check path length without any obstacles
            assertEquals(17, path.size(), 0);
            // place obstacle
            Collider colldier = new SphereCollider(SPHERE_POS, 20, SampleAreaModifications.SAMPLE_POLYAREA_TYPE_GROUND, 0.1f);
            long colliderId = mesh.addCollider(colldier);
            // update navmesh asynchronously
            future = mesh.update(executor);
            // wait for update to complete
            future.get();
            // create new query
            query = new NavMeshQuery(mesh.navMesh());
            // find path again
            start = query.findNearestPoly(START_POS, EXTENT, filter).result;
            end = query.findNearestPoly(END_POS, EXTENT, filter).result;
            path = query.findPath(start.getNearestRef(), end.getNearestRef(), start.getNearestPos(), end.getNearestPos(), filter,
                    NavMeshQuery.DT_FINDPATH_ANY_ANGLE, Float.MAX_VALUE).result;
            // check path length with obstacles
            assertEquals(22, path.size(), 0);
            // remove obstacle
            mesh.removeCollider(colliderId);
            // update navmesh asynchronously
            future = mesh.update(executor);
            // wait for update to complete
            future.get();
            // create new query
            query = new NavMeshQuery(mesh.navMesh());
            // find path one more time
            start = query.findNearestPoly(START_POS, EXTENT, filter).result;
            end = query.findNearestPoly(END_POS, EXTENT, filter).result;
            path = query.findPath(start.getNearestRef(), end.getNearestRef(), start.getNearestPos(), end.getNearestPos(), filter,
                    NavMeshQuery.DT_FINDPATH_ANY_ANGLE, Float.MAX_VALUE).result;
            // path length should be back to the initial value
            assertEquals(17, path.size(), 0);
        }
    }
}
