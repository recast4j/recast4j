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

package org.recast4j.dynamic.io;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.io.InputStream;

import org.junit.Test;

public class VoxelFileReaderTest {

    @Test
    public void shouldReadSingleTileFile() throws IOException {
        try (InputStream is = getClass().getClassLoader().getResourceAsStream("test.voxels")) {
            VoxelFileReader reader = new VoxelFileReader();
            VoxelFile f = reader.read(is);
            assertFalse(f.useTiles);
            assertArrayEquals(new float[] { -100.0f, 0f, -100f, 100f, 5f, 100f }, f.bounds, 0f);
            assertEquals(0.25f, f.cellSize, 0f);
            assertEquals(0.5f, f.walkableRadius, 0f);
            assertEquals(2f, f.walkableHeight, 0f);
            assertEquals(0.5f, f.walkableClimb, 0f);
            assertEquals(20f, f.maxEdgeLen, 0f);
            assertEquals(2f, f.maxSimplificationError, 0f);
            assertEquals(2f, f.minRegionArea, 0f);
            assertEquals(1, f.tiles.size());
            assertEquals(0.001f, f.tiles.get(0).cellHeight, 0f);
            assertEquals(810, f.tiles.get(0).width);
            assertEquals(810, f.tiles.get(0).depth);
            assertArrayEquals(new float[] { -101.25f, 0f, -101.25f }, f.tiles.get(0).boundsMin, 0f);
            assertArrayEquals(new float[] { 101.25f, 1048.5751f, 101.25f }, f.tiles.get(0).boundsMax, 0f);

        }
    }

    @Test
    public void shouldReadMultiTileFile() throws IOException {
        try (InputStream is = getClass().getClassLoader().getResourceAsStream("test_tiles.voxels")) {
            VoxelFileReader reader = new VoxelFileReader();
            VoxelFile f = reader.read(is);
            assertTrue(f.useTiles);
            assertArrayEquals(new float[] { -100.0f, 0f, -100f, 100f, 5f, 100f }, f.bounds, 0f);
            assertEquals(0.25f, f.cellSize, 0f);
            assertEquals(0.5f, f.walkableRadius, 0f);
            assertEquals(2f, f.walkableHeight, 0f);
            assertEquals(0.5f, f.walkableClimb, 0f);
            assertEquals(20f, f.maxEdgeLen, 0f);
            assertEquals(2f, f.maxSimplificationError, 0f);
            assertEquals(2f, f.minRegionArea, 0f);
            assertEquals(100, f.tiles.size());
            assertEquals(0.001f, f.tiles.get(0).cellHeight, 0f);
            assertEquals(90, f.tiles.get(0).width);
            assertEquals(90, f.tiles.get(0).depth);
            assertArrayEquals(new float[] { -101.25f, 0f, -101.25f }, f.tiles.get(0).boundsMin, 0f);
            assertArrayEquals(new float[] { -78.75f, 1048.5751f, -78.75f }, f.tiles.get(0).boundsMax, 0f);

        }
    }
}
