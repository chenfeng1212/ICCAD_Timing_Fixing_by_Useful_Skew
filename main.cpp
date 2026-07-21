#include "ds.h"
#include "ps.h"
#include "Beam_Search.h"
#include "output.h"
//#include "SPFA.h"
//#include "clk_tree_dp.h"
//#include "Evaluator.h"

#include <iostream>
#include <string>

using namespace skew;
using namespace std;

int main(int argc,char* argv[])
{
    Timer timer("Total RunTime");

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
    
    //original slack information
    double old_tns_ss = 0.0;
    double old_wns_ss = 0.0;
    double old_tns_ff = 0.0;
    double old_wns_ff = 0.0;
    int old_ss_count = 0;
    int old_ff_count = 0;
    for (auto& path : db.paths){
        if (path.setupSlack < 0.0) {
            old_tns_ss += path.setupSlack;
            old_ss_count++;
            if (old_wns_ss > path.setupSlack) {
                old_wns_ss = path.setupSlack;
            }
        }
        if (path.holdSlack < 0.0) {
            old_tns_ff += path.holdSlack;
            old_ff_count++;
            if (old_wns_ff > path.holdSlack) {
                old_wns_ff = path.holdSlack;
            }
        }
    }

    //beam_search
    ClkTreeBS bs(db, db.rootId);
    db.subtreeFFs.resize(db.tree.size(), {});

    db.buildFFtoPaths();
    db.InitializeSubtreeFFs(db.rootId);

    //修critical path時的理想結果
    db.critical_ss_threshold = -0.05;//將ss的negative slack修到這個程度
    db.critical_ff_threshold = -0.01;//將ff的negative slack修到這個程度
    db.critical_limit = 0.02;//修復ss/ff時另一個corner能接受的惡化極限

    bool cleanMode = false;//剩下的都是小slack，threshold可以變為>=0，limit變小
    int cycle_count = (db.tree.size() / 10); //testcase0和4在差不多1200+後開始惡化了，需要想判斷終止條件
    int critical_to_solve = 15; // 每次迭代優先修最嚴重的15條path

    for (int i = 0; i < 1200; i++) {// 原本是i < cycle_count
        cout << "\nRound " << i+1 << "\n";
        auto rootCandidates = bs.RunBS(cleanMode, critical_to_solve);

        if (i >= 500 || critical_to_solve > 15)
            cleanMode = true;
        if (rootCandidates[0].operations.size() == 0)
            break;
        /*
        Evaluator evaluator(db);

        evaluator.runTopKEvaluationAndOutput(
            rootCandidates,
            outputFile
        );*/
        if(true) {
            db.computeClockArrival();
            db.computeAllSlacks();

            double tns_ss = 0.0;
            double wns_ss = 0.0;
            double tns_ff = 0.0;
            double wns_ff = 0.0;
            int ss_count = 0;
            int ff_count = 0;
            for (auto& path : db.paths){
                if (path.setupSlack < 0.0) {
                    tns_ss += path.setupSlack;
                    ss_count++;
                    if (wns_ss > path.setupSlack) {
                        wns_ss = path.setupSlack;
                    }
                }
                if (path.holdSlack < 0.0) {
                    tns_ff += path.holdSlack;
                    ff_count++;
                    if (wns_ff > path.holdSlack) {
                        wns_ff = path.holdSlack;
                    }
                }
            }
            cout << "old_wns_ss = " << old_wns_ss << ", wns_ss = " << wns_ss << endl;
            cout << "old_tns_ss = " << old_tns_ss << ", tns_ss = " << tns_ss << endl;
            cout << "old_wns_ff = " << old_wns_ff << ", wns_ff = " << wns_ff << endl;
            cout << "old_tns_ff = " << old_tns_ff << ", tns_ff = " << tns_ff << endl;
            cout << "old_ss_nvp = " << old_ss_count << ", ss_nvp = " << ss_count << endl;
            cout << "old_ff_nvp = " << old_ff_count << ", ff_nvp = " << ff_count << endl;

            /*int worser= 0;
            if (wns_ss < old_wns_ss)
                worser++;
            if (wns_ff < old_wns_ff)
                worser++;
            if (tns_ss < old_tns_ss)
                worser++;
            if (tns_ff < old_tns_ff)
                worser++;*/
            old_tns_ss = tns_ss;
            old_wns_ss = wns_ss;
            old_tns_ff = tns_ff;
            old_wns_ff = wns_ff;
            old_ss_count = ss_count;
            old_ff_count = ff_count;
        }
        
        //還沒試過時間檢測
        //if (timer.elapsedSec() >= 590.0){
        //    cout << "TIME LIMIT REACHED" << endl;
        //    break;
        //}
    }
    
    writeClockTree(outputFile, db);
    
    return 0;
}