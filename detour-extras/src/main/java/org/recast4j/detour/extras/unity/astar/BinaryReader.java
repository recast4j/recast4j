/*
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
package org.recast4j.detour.extras.unity.astar;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import org.recast4j.detour.io.IOUtils;

abstract class BinaryReader {

    protected ByteBuffer toByteBuffer(ZipFile file, String filename) throws IOException {
        ZipEntry graphReferences = file.getEntry(filename);
        ByteBuffer buffer = IOUtils.toByteBuffer(file.getInputStream(graphReferences));
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        return buffer;
    }

}
