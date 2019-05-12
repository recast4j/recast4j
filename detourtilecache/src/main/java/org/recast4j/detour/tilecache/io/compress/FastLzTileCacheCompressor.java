/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.detour.tilecache.io.compress;

import java.util.Arrays;

import org.recast4j.detour.tilecache.TileCacheCompressor;

public class FastLzTileCacheCompressor implements TileCacheCompressor {

    @Override
    public byte[] decompress(byte[] buf, int offset, int len, int outputlen) {
        byte[] output = new byte[outputlen];
        FastLz.decompress(buf, offset, len, output, 0, outputlen);
        return output;
    }

    @Override
    public byte[] compress(byte[] buf) {
        byte[] output = new byte[FastLz.calculateOutputBufferLength(buf.length)];
        int len = FastLz.compress(buf, 0, buf.length, output, 0, output.length);
        return Arrays.copyOf(output, len);
    }

}
