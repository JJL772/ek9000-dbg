
#pragma once

typedef struct {
    const char* s;
    int e;
} enum_str_map_t;

int enum_from_str(enum_str_map_t* map, int n, const char* v, int* err);

#define ARRAYSIZE(a) (sizeof(a) / sizeof(a[0]))