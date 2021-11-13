package com.team2910.lib.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}