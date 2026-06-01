#ifndef OUTPUT_H
#define OUTPUT_H

#include "ds.h"

#include <fstream>
#include <stdexcept>

namespace skew {

/*==================================================
 * DFS Output
 *=================================================*/

inline void printTreeDFS(
    int nodeId,
    const DesignDB& db,
    std::ofstream& fout,
    int level
)
{
    const TreeNode& node =
        db.tree[nodeId];

    //----------------------------------
    // Root
    //----------------------------------

    if (node.isRoot())
    {
        fout
            << "Root: "
            << node.name
            << "\n";
    }
    else
    {
        for (int i = 0; i < level; ++i)
        {
            fout << "\t";
        }

        fout
            << "["
            << level
            << "] "
            << node.name
            << " ("
            << node.instType
            << ")";

        if (node.isSink)
        {
            fout << " (SINK)";
        }

        fout << "\n";
    }

    //----------------------------------
    // Children
    //----------------------------------

    for (size_t i = 0;
         i < node.children.size();
         ++i)
    {
        printTreeDFS(
            node.children[i],
            db,
            fout,
            level + 1
        );
    }
}

/*==================================================
 * Write Clock Tree
 *=================================================*/

inline void writeClockTree(
    const std::string& filename,
    const DesignDB& db
)
{
    std::ofstream fout(
        filename.c_str()
    );

    if (!fout)
    {
        throw std::runtime_error(
            "Cannot open output file: "
            + filename
        );
    }

    if (!db.validTreeNode(db.rootId))
    {
        throw std::runtime_error(
            "Invalid root node."
        );
    }

    printTreeDFS(
        db.rootId,
        db,
        fout,
        0
    );

    fout.close();
}

} // namespace skew

#endif