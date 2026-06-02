#include "ds.h"
#include "ps.h"
#include "output.h"
#include "SPFA.h"

#include <iostream>
#include <string>

using namespace skew;
using namespace std;

int main(int argc,char* argv[])
//int main()
{
    if(argc != 3)
        return 1;

    string testcaseDir = argv[1];
    string outputFile  = argv[2];
    /*string testcaseDir = "testcase0";
    string outputFile = "testcase0/modified_clk_tree.txt";//要改成structure*/

    DesignDB db;

    //----------------------------------
    // Step 1 Clock Tree & Timing Path Model
    //----------------------------------

    parser::buildDesignDB(
        testcaseDir,
        db
    );

    //----------------------------------
    // Step 2 Critical-Path SPFA Target Scheduling
    //----------------------------------
    SPFA::spfa(
        db
    );

    //----------------------------------
    // Step 3 Bounded Bottom-Up DP
    //----------------------------------


    //----------------------------------
    // Step 4 Root Top-K
    //----------------------------------


    
    //----------------------------------
    // Step 5 Output
    //----------------------------------

    writeClockTree(
        outputFile,
        db
    );

    return 0;
}