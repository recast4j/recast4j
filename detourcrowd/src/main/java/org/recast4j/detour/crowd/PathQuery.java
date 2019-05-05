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
package org.recast4j.detour.crowd;

import java.util.List;

import org.recast4j.detour.QueryFilter;
import org.recast4j.detour.Status;

public class PathQuery {
    long ref;
    /// Path find start and end location.
    float[] startPos = new float[3], endPos = new float[3];
    long startRef, endRef;
    /// Result.
    List<Long> path;
    /// State.
    Status status;
    int keepAlive;
    QueryFilter filter; /// < TODO: This is potentially dangerous!
}
