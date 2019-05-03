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
package org.recast4j.detour;

public enum Status {

    FAILURE, SUCCSESS, IN_PROGRESS, PARTIAL_RESULT, FAILURE_INVALID_PARAM;

    public boolean isFailed() {
        return this == FAILURE || this == FAILURE_INVALID_PARAM;
    }

    public boolean isInProgress() {
        return this == IN_PROGRESS;
    }

    public boolean isSuccess() {
        return this == Status.SUCCSESS || this == Status.PARTIAL_RESULT;
    }

    public boolean isPartial() {
        return this == Status.PARTIAL_RESULT;
    }
}
