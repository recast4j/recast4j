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
import java.io.OutputStream;
import java.nio.ByteOrder;

import org.recast4j.detour.io.DetourWriter;

public class VoxelFileWriter extends DetourWriter {

    private final LZ4VoxelTileCompressor compressor = new LZ4VoxelTileCompressor();

    public void write(OutputStream stream, VoxelFile f, boolean compression) throws IOException {
        write(stream, f, VoxelFile.PREFERRED_BYTE_ORDER, compression);
    }

    public void write(OutputStream stream, VoxelFile f, ByteOrder byteOrder, boolean compression) throws IOException {
        write(stream, VoxelFile.MAGIC, byteOrder);
        write(stream, VoxelFile.VERSION_EXPORTER_RECAST4J | (compression ? VoxelFile.VERSION_COMPRESSION_LZ4 : 0), byteOrder);
        write(stream, f.walkableRadius, byteOrder);
        write(stream, f.walkableHeight, byteOrder);
        write(stream, f.walkableClimb, byteOrder);
        write(stream, f.walkableSlopeAngle, byteOrder);
        write(stream, f.cellSize, byteOrder);
        write(stream, f.maxSimplificationError, byteOrder);
        write(stream, f.maxEdgeLen, byteOrder);
        write(stream, f.minRegionArea, byteOrder);
        write(stream, f.regionMergeArea, byteOrder);
        write(stream, f.vertsPerPoly, byteOrder);
        write(stream, f.buildMeshDetail);
        write(stream, f.detailSampleDistance, byteOrder);
        write(stream, f.detailSampleMaxError, byteOrder);
        write(stream, f.useTiles);
        write(stream, f.tileSizeX, byteOrder);
        write(stream, f.tileSizeZ, byteOrder);
        write(stream, f.rotation[0], byteOrder);
        write(stream, f.rotation[1], byteOrder);
        write(stream, f.rotation[2], byteOrder);
        write(stream, f.bounds[0], byteOrder);
        write(stream, f.bounds[1], byteOrder);
        write(stream, f.bounds[2], byteOrder);
        write(stream, f.bounds[3], byteOrder);
        write(stream, f.bounds[4], byteOrder);
        write(stream, f.bounds[5], byteOrder);
        write(stream, f.tiles.size(), byteOrder);
        for (VoxelTile t : f.tiles) {
            writeTile(stream, t, byteOrder, compression);
        }
    }

    public void writeTile(OutputStream stream, VoxelTile tile, ByteOrder byteOrder, boolean compression) throws IOException {
        write(stream, tile.tileX, byteOrder);
        write(stream, tile.tileZ, byteOrder);
        write(stream, tile.width, byteOrder);
        write(stream, tile.depth, byteOrder);
        write(stream, tile.borderSize, byteOrder);
        write(stream, tile.boundsMin[0], byteOrder);
        write(stream, tile.boundsMin[1], byteOrder);
        write(stream, tile.boundsMin[2], byteOrder);
        write(stream, tile.boundsMax[0], byteOrder);
        write(stream, tile.boundsMax[1], byteOrder);
        write(stream, tile.boundsMax[2], byteOrder);
        write(stream, tile.cellSize, byteOrder);
        write(stream, tile.cellHeight, byteOrder);
        byte[] bytes = tile.spanData;
        if (compression) {
            bytes = compressor.compress(bytes);
        }
        write(stream, bytes.length, byteOrder);
        stream.write(bytes);
    }

}
