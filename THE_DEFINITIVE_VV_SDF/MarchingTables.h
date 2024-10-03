#pragma once

class MarchingTables
{
public:
    /// <summary>
    /// Holy Marching Cubes Edge Table
    /// </summary>
    const static int edge_table[256];

    /// <summary>
    /// Holy Marching Cubes Triangle Table
    /// </summary>
    const static int tri_table[256][16];
};