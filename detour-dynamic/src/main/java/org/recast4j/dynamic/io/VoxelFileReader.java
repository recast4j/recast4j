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

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.recast4j.detour.io.IOUtils;

public class VoxelFileReader {

    public VoxelFile read(InputStream stream) throws IOException {
        ByteBuffer buf = IOUtils.toByteBuffer(stream);
        VoxelFile file = new VoxelFile();
        int magic = buf.getInt();
        if (magic != VoxelFile.MAGIC) {
            magic = IOUtils.swapEndianness(magic);
            if (magic != VoxelFile.MAGIC) {
                throw new IOException("Invalid magic");
            }
            buf.order(buf.order() == ByteOrder.BIG_ENDIAN ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN);
        }
        file.version = buf.getInt();
        boolean isExportedFromAstar = (file.version & VoxelFile.VERSION_EXPORTER_MASK) == 0;
        file.walkableRadius = buf.getFloat();
        file.walkableHeight = buf.getFloat();
        file.walkableClimb = buf.getFloat();
        file.walkableSlopeAngle = buf.getFloat();
        file.cellSize = buf.getFloat();
        file.maxSimplificationError = buf.getFloat();
        file.maxEdgeLen = buf.getFloat();
        file.minRegionArea = (int) buf.getFloat();
        if (!isExportedFromAstar) {
            file.regionMergeArea = buf.getInt();
            file.vertsPerPoly = buf.getInt();
            file.buildMeshDetail = buf.get() != 0;
            file.detailSampleDistance = buf.getFloat();
            file.detailSampleMaxError = buf.getFloat();
        } else {
            file.regionMergeArea = 6 * file.minRegionArea;
            file.vertsPerPoly = 6;
            file.buildMeshDetail = true;
            file.detailSampleDistance = file.maxEdgeLen * 0.5f;
            file.detailSampleMaxError = file.maxSimplificationError * 0.8f;
        }
        file.useTiles = buf.get() != 0;
        file.tileSizeX = buf.getInt();
        file.tileSizeZ = buf.getInt();
        file.rotation[0] = buf.getFloat();
        file.rotation[1] = buf.getFloat();
        file.rotation[2] = buf.getFloat();
        file.bounds[0] = buf.getFloat();
        file.bounds[1] = buf.getFloat();
        file.bounds[2] = buf.getFloat();
        file.bounds[3] = buf.getFloat();
        file.bounds[4] = buf.getFloat();
        file.bounds[5] = buf.getFloat();
        if (isExportedFromAstar) {
            // bounds are saved as center + size
            file.bounds[0] -= 0.5f * file.bounds[3];
            file.bounds[1] -= 0.5f * file.bounds[4];
            file.bounds[2] -= 0.5f * file.bounds[5];
            file.bounds[3] += file.bounds[0];
            file.bounds[4] += file.bounds[1];
            file.bounds[5] += file.bounds[2];
        }
        file.tileCount = buf.getInt();
        for (int tile = 0; tile < file.tileCount; tile++) {
            int tileX = buf.getInt();
            int tileZ = buf.getInt();
            int width = buf.getInt();
            int depth = buf.getInt();
            int borderSize = buf.getInt();
            float[] boundsMin = new float[3];
            boundsMin[0] = buf.getFloat();
            boundsMin[1] = buf.getFloat();
            boundsMin[2] = buf.getFloat();
            float[] boundsMax = new float[3];
            boundsMax[0] = buf.getFloat();
            boundsMax[1] = buf.getFloat();
            boundsMax[2] = buf.getFloat();
            if (isExportedFromAstar) {
                // bounds are local
                boundsMin[0] += file.bounds[0];
                boundsMin[1] += file.bounds[1];
                boundsMin[2] += file.bounds[2];
                boundsMax[0] += file.bounds[0];
                boundsMax[1] += file.bounds[1];
                boundsMax[2] += file.bounds[2];
            }
            float cellSize = buf.getFloat();
            float cellHeight = buf.getFloat();
            int voxelSize = buf.getInt();
            int position = buf.position();
            byte[] bytes = new byte[voxelSize];
            buf.get(bytes);
            ByteBuffer data = ByteBuffer.wrap(bytes);
            data.order(buf.order());
            file.addTile(new VoxelTile(tileX, tileZ, width, depth, boundsMin, boundsMax, cellSize, cellHeight, borderSize, data));
            buf.position(position + voxelSize);
        }
        return file;
    }

}
