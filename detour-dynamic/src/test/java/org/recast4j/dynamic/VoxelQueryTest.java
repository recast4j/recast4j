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

package org.recast4j.dynamic;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.io.IOException;
import java.io.InputStream;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.function.BiFunction;

import org.assertj.core.data.Offset;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.recast4j.dynamic.io.VoxelFile;
import org.recast4j.dynamic.io.VoxelFileReader;
import org.recast4j.recast.Heightfield;

@ExtendWith(MockitoExtension.class)
public class VoxelQueryTest {

    private static final int TILE_WIDTH = 100;
    private static final int TILE_DEPTH = 90;
    private static final float[] ORIGIN = new float[] { 50, 10, 40 };
    @Mock
    private BiFunction<Integer, Integer, Optional<Heightfield>> hfProvider;

    @Test
    public void shouldTraverseTiles() {
        // Given
        when(hfProvider.apply(anyInt(), anyInt())).thenReturn(Optional.empty());
        VoxelQuery query = new VoxelQuery(ORIGIN, TILE_WIDTH, TILE_DEPTH, hfProvider);
        float[] start = { 120, 10, 365 };
        float[] end = { 320, 10, 57 };
        // When
        query.raycast(start, end);
        // Then
        ArgumentCaptor<Integer> captorX = ArgumentCaptor.forClass(Integer.class);
        ArgumentCaptor<Integer> captorZ = ArgumentCaptor.forClass(Integer.class);
        verify(hfProvider, times(6)).apply(captorX.capture(), captorZ.capture());
        assertThat(captorX.getAllValues()).containsExactly(0, 1, 1, 1, 2, 2);
        assertThat(captorZ.getAllValues()).containsExactly(3, 3, 2, 1, 1, 0);
    }

    @Test
    public void shouldHandleRaycastWithoutObstacles() throws Exception {
        DynamicNavMesh mesh = createDynaMesh();
        VoxelQuery query = mesh.voxelQuery();
        float[] start = { 7.4f, 0.5f, -64.8f };
        float[] end = { 31.2f, 0.5f, -75.3f };
        Optional<Float> hit = query.raycast(start, end);
        assertThat(hit).isEmpty();
    }

    @Test
    public void shouldHandleRaycastWithObstacles() throws Exception {
        DynamicNavMesh mesh = createDynaMesh();
        VoxelQuery query = mesh.voxelQuery();
        float[] start = { 32.3f, 0.5f, 47.9f };
        float[] end = { -31.2f, 0.5f, -29.8f };
        Optional<Float> hit = query.raycast(start, end);
        assertThat(hit).isPresent();
        assertThat(hit.get()).isEqualTo(0.5263836f, Offset.strictOffset(1e-7f));
    }

    private DynamicNavMesh createDynaMesh() throws IOException, InterruptedException, ExecutionException {
        DynamicNavMesh mesh;
        try (InputStream is = getClass().getClassLoader().getResourceAsStream("test_tiles.voxels")) {
            ExecutorService executor = TestExecutorService.get();
            // load voxels from file
            VoxelFileReader reader = new VoxelFileReader();
            VoxelFile f = reader.read(is);
            // create dynamic navmesh
            mesh = new DynamicNavMesh(f);
            // build navmesh asynchronously using multiple threads
            CompletableFuture<Boolean> future = mesh.build(executor);
            // wait for build to complete
            future.get();
        }
        return mesh;
    }
}
