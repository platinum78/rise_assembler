#include <iostream>
#include <cstdio>
using namespace std;

typedef struct Task_
{
    int type;
    double configuration[6];
    struct Task_ *next;
} Task;
