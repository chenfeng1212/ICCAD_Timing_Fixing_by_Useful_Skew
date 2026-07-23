#include "ds.h"
#include "ps.h"
#include "Beam_Search.h"
#include "output.h"
//#include "SPFA.h"
//#include "clk_tree_dp.h"
//#include "Evaluator.h"

#include <iostream>
#include <string>
#include <algorithm>
#include <utility>

using namespace skew;
using namespace std;

namespace {
// 依官方公式計算單一狀態的 Score：每個指標用「相對原始值的改善比例」正規化
// (0 = 沒改善, 1 = 完全修好)，再跨 5 組權重劇本取平均。值越大越好。
// 與 Evaluator::selectRobustWinner 使用同一組劇本，確保停滯判斷跟最終選解一致。
double computeScore(
    double tns_ss, double wns_ss, double tns_ff, double wns_ff, double area,
    double o_tns_ss, double o_wns_ss, double o_tns_ff, double o_wns_ff, double o_area)
{
    double t_tns_ss = (o_tns_ss < 0.0) ? (1.0 - tns_ss / o_tns_ss) : 0.0;
    double t_tns_ff = (o_tns_ff < 0.0) ? (1.0 - tns_ff / o_tns_ff) : 0.0;
    double t_wns_ss = (o_wns_ss < 0.0) ? (1.0 - wns_ss / o_wns_ss) : 0.0;
    double t_wns_ff = (o_wns_ff < 0.0) ? (1.0 - wns_ff / o_wns_ff) : 0.0;
    double t_area   = (o_area  > 0.0) ? (1.0 - area   / o_area)   : 0.0;

    static const double scen[5][3] = { // {w_tns, w_wns, w_area}
        {0.60, 0.35, 0.05},
        {0.35, 0.60, 0.05},
        {0.50, 0.40, 0.10},
        {0.45, 0.45, 0.10},
        {0.55, 0.40, 0.05},
    };
    double sum = 0.0;
    for (int s = 0; s < 5; ++s) {
        sum += scen[s][0] * (t_tns_ss + t_tns_ff)
             + scen[s][1] * (t_wns_ss + t_wns_ff)
             + scen[s][2] * (t_area);
    }
    return sum / 5.0;
}
} // namespace

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
    
#ifdef DEBUG
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
#endif

    // Score 計算所需的原始基準（有副作用依賴，不可包進 #ifdef DEBUG）
    double orig_tns_ss = 0.0, orig_wns_ss = 0.0;
    double orig_tns_ff = 0.0, orig_wns_ff = 0.0;
    int    orig_nvp = 0;   // 原始違規路徑數（setup + hold）
    for (const auto& path : db.paths) {
        if (path.setupSlack < 0.0) {
            orig_tns_ss += path.setupSlack;
            orig_wns_ss = std::min(orig_wns_ss, path.setupSlack);
            orig_nvp++;
        }
        if (path.holdSlack < 0.0) {
            orig_tns_ff += path.holdSlack;
            orig_wns_ff = std::min(orig_wns_ff, path.holdSlack);
            orig_nvp++;
        }
    }
    double orig_area = db.computeTotalBufferArea();

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

    // ---- 追蹤最佳解 + 動態偵測停滯 ----
    // 停滯採「雙指標」：只有 Score 與 NVP 同時連續 STALL_LIMIT 輪都沒進步才算停滯，
    // 避免 area 權重雜訊在 timing/NVP 仍在改善時把還能收斂的測資誤殺。
    const int    STALL_LIMIT    = 150;   // 雙指標連續未進步的容忍輪數（可調；先試 150）
    const double TIME_LIMIT_SEC = 570.0; // 硬時間上限，預留 output 時間（官方 600s）
    double bestScore = 0.0;       // 初始狀態 cur==orig → score 0，作為保底基準
    auto   bestTree  = db.tree;   // 保底：最差就輸出初始 clock tree
    int    bestNVP   = orig_nvp;  // 追蹤到過的最低違規路徑數（第二層停滯指標）
    int    noImprove = 0;         // Score 與 NVP 同時未進步的連續輪數

    for (int i = 0; i < 1200; i++) {// 原本是i < cycle_count
#ifdef DEBUG
        cout << "\nRound " << i+1 << "\n";
#endif
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

        // 每輪套用完 operations 後，必須重算 clock arrival 與 slack，
        // 讓 db.paths 的 slack 成為下一輪 UpdateViolationList 的正確 baseline。
        // 這兩行有副作用、後續邏輯依賴，絕對不能包進 #ifdef DEBUG。
        db.computeClockArrival();
        db.computeAllSlacks();

        // ---- 計算本輪 Score / NVP、追蹤最佳解、偵測停滯 ----
        double cur_tns_ss = 0.0, cur_wns_ss = 0.0;
        double cur_tns_ff = 0.0, cur_wns_ff = 0.0;
        int    cur_nvp = 0;
        for (const auto& path : db.paths) {
            if (path.setupSlack < 0.0) {
                cur_tns_ss += path.setupSlack;
                cur_wns_ss = std::min(cur_wns_ss, path.setupSlack);
                cur_nvp++;
            }
            if (path.holdSlack < 0.0) {
                cur_tns_ff += path.holdSlack;
                cur_wns_ff = std::min(cur_wns_ff, path.holdSlack);
                cur_nvp++;
            }
        }
        double score = computeScore(
            cur_tns_ss, cur_wns_ss, cur_tns_ff, cur_wns_ff, db.computeTotalBufferArea(),
            orig_tns_ss, orig_wns_ss, orig_tns_ff, orig_wns_ff, orig_area);

        // 最佳解仍以 Score 為準（與 Evaluator 選解一致）：收斂時 timing 項飽和、
        // 主導分數，最高分那輪即是收斂那輪，故快照 max-Score 的 tree 是正確的。
        bool scoreImproved = (score > bestScore);
        if (scoreImproved) {
            bestScore = score;
            bestTree  = db.tree;   // 快照目前最佳的 clock tree
        }

        // 第二層指標：只要 NVP 還在下降就不算停滯，避免 area 雜訊誤殺仍在收斂的測資
        bool nvpImproved = (cur_nvp < bestNVP);
        if (nvpImproved)
            bestNVP = cur_nvp;

        // Score 與 NVP「同時」都沒進步，才累計停滯輪數
        if (scoreImproved || nvpImproved)
            noImprove = 0;
        else
            noImprove++;

        if (noImprove >= STALL_LIMIT) {
            if (cleanMode) {
                break;             // 已在 cleanMode 仍雙指標停滯 → 提早結束，用最佳解保底
            }
            cleanMode = true;      // 尚未進 cleanMode → 切換後再給一次機會
            noImprove = 0;
        }

        // 時間安全網：拉高 STALL_LIMIT + NVP 雙指標後大測資可能跑更久，
        // 用硬上限確保不超時；break 時一樣輸出最佳解，不損失結果。
        if (timer.elapsedSec() >= TIME_LIMIT_SEC) {
#ifdef DEBUG
            cout << "TIME LIMIT REACHED at " << timer.elapsedSec() << "s\n";
#endif
            break;
        }

#ifdef DEBUG
        {
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
#endif
    }
    
    // 輸出追蹤到的最佳解，而非最後一輪（可能已惡化）的狀態
    db.tree = std::move(bestTree);
    writeClockTree(outputFile, db);

    return 0;
}