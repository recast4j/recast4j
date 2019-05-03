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

public class Result<T> {

    public final T result;
    public final Status status;
    public final String message;

    private Result(T result, Status status, String message) {
        this.result = result;
        this.status = status;
        this.message = message;
    }

    public static <T> Result<T> success(T result) {
        return new Result<>(result, Status.SUCCSESS, null);
    }

    public static <T> Result<T> failure() {
        return new Result<>(null, Status.FAILURE, null);
    }

    public static <T> Result<T> invalidParam() {
        return new Result<>(null, Status.FAILURE_INVALID_PARAM, null);
    }

    public static <T> Result<T> failure(String message) {
        return new Result<>(null, Status.FAILURE, message);
    }

    public static <T> Result<T> invalidParam(String message) {
        return new Result<>(null, Status.FAILURE_INVALID_PARAM, message);
    }

    public static <T> Result<T> failure(T result) {
        return new Result<>(result, Status.FAILURE, null);
    }

    public static <T> Result<T> partial(T result) {
        return new Result<>(null, Status.PARTIAL_RESULT, null);
    }

    public static <T> Result<T> of(Status status, String message) {
        return new Result<>(null, status, message);
    }

    public static <T> Result<T> of(Status status, T result) {
        return new Result<>(result, status, null);
    }

    public boolean failed() {
        return status.isFailed();
    }

    public boolean succeeded() {
        return status.isSuccess();
    }

}
