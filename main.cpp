#include "ds.h"
#include "ps.h"
#include "SPFA.h"
#include "clk_tree_dp.h"
#include "Evaluator.h"
#include "Beam_Search.h"

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

    
    // clk_tree_dp
    /*
    for (int i = 0; i < 4; i++) {
        cout << i+1 << endl;
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
    }*/
   
    
    //original slack information
    double old_tns_ss = 0.0;
    double old_wns_ss = 0.0;
    double old_tns_ff = 0.0;
    double old_wns_ff = 0.0;
    for (auto& path : db.paths){
        if (path.setupSlack < 0.0) {
            old_tns_ss += path.setupSlack;
            if (old_wns_ss > path.setupSlack) {
                old_wns_ss = path.setupSlack;
            }
        }
        if (path.holdSlack < 0.0) {
            old_tns_ff += path.holdSlack;
            if (old_wns_ff > path.holdSlack) {
                old_wns_ff = path.holdSlack;
            }
        }
    }


    //beam_search
    for (int i = 0; i < 4; i++) {
        cout << "\nRound " << i+1 << "\n";
        ClkTreeBS bs(db, db.rootId);
        auto rootCandidates = bs.RunBS();

        Evaluator evaluator(db);

        evaluator.runTopKEvaluationAndOutput(
            rootCandidates,
            outputFile
        );
        if(true) {
            db.computeClockArrival();
            db.computeAllSlacks();

            double tns_ss = 0.0;
            double wns_ss = 0.0;
            double tns_ff = 0.0;
            double wns_ff = 0.0;
            for (auto& path : db.paths){
                if (path.setupSlack < 0.0) {
                    tns_ss += path.setupSlack;
                    if (wns_ss > path.setupSlack) {
                        wns_ss = path.setupSlack;
                    }
                }
                if (path.holdSlack < 0.0) {
                    tns_ff += path.holdSlack;
                    if (wns_ff > path.holdSlack) {
                        wns_ff = path.holdSlack;
                    }
                }
            }
            cout << "wns_ss = " << wns_ss << ", tns_ss = " << tns_ss << endl;
            cout << "wns_ff = " << wns_ff << ", tns_ff = " << tns_ff << endl;
            cout << "old_wns_ss = " << old_wns_ss << ", old_tns_ss = " << old_tns_ss << endl;
            cout << "old_wns_ff = " << old_wns_ff << ", old_tns_ff = " << old_tns_ff << endl;
            int worser= 0;
            if (wns_ss < old_wns_ss)
                worser++;
            if (wns_ff < old_wns_ff)
                worser++;
            if (tns_ss < old_tns_ss)
                worser++;
            if (tns_ff < old_tns_ff)
                worser++;
            old_tns_ss = tns_ss;
            old_wns_ss = wns_ss;
            old_tns_ff = tns_ff;
            old_wns_ff = wns_ff;
        }
        //if (worser >= 3)
            //break;
    }
    
    return 0;
}