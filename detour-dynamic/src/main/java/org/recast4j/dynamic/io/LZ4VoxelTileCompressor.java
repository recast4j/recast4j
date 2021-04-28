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

import java.nio.ByteOrder;

import net.jpountz.lz4.LZ4Factory;

public class LZ4VoxelTileCompressor {

    byte[] decompress(byte[] data) {
        int originalSize = ByteUtils.getIntBE(data, 0);
        return LZ4Factory.fastestInstance().fastDecompressor().decompress(data, 4, originalSize);
    }

    byte[] compress(byte[] data) {
        byte[] compressed = LZ4Factory.fastestInstance().highCompressor().compress(data);
        byte[] result = new byte[4 + compressed.length];
        ByteUtils.putInt(data.length, result, 0, ByteOrder.BIG_ENDIAN);
        System.arraycopy(compressed, 0, result, 4, compressed.length);
        return result;
    }

}
