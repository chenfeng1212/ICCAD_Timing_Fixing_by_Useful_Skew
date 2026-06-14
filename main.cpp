#include "ds.h"
#include "ps.h"
#include "SPFA.h"
#include "clk_tree_dp.h"
#include "Evaluator.h"

#include <iostream>
#include <string>

using namespace skew;
using namespace std;

int main(int argc,char* argv[])
{
    if(argc != 3)
        return 1;

    string testcaseDir = argv[1];
    string outputFile  = argv[2];

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
    double lambda = 0.005;
    int topK = 100;
    double bucketPrecision = 0.0001;
    int bucketKeep = 20;

    ClkTreeDP dp(
        db,
        lambda,
        topK,
        bucketPrecision,
        bucketKeep,
        testcaseDir
    );

    auto rootCandidates = dp.RunDP(db.rootId);

    //----------------------------------
    // Step 4 Root Top-K + Output
    //----------------------------------

    Evaluator evaluator(db);

    evaluator.runTopKEvaluationAndOutput(
        rootCandidates,
        outputFile
    );

    return 0;
}