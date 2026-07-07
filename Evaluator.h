#include "ds.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

// 定義權重劇本 (對齊官方評分公式的 α / β / γ)
//   w_tns  : TNS 改善權重 (setup + hold 兩個 corner 一起)
//   w_wns  : WNS 改善權重 (setup + hold 兩個 corner 一起)
//   w_area : 面積縮減權重 (官方明示佔比低)
struct WeightScenario {
    double w_tns;
    double w_wns;
    double w_area;
};

class Evaluator {
public:
    Evaluator(skew::DesignDB& main_db) : db(main_db) {}

    // 主進入點
    void runTopKEvaluationAndOutput(const std::vector<skew::DPState>& root_candidates, const std::string& outputFile) {
        // Timer timer("Evaluate");
        // cout << "into Evaluate" << endl;
                
        /*size_t pos = outputFile.find_last_of('/');
        string testcase = outputFile.substr(0, pos + 1);
        std::ofstream fout(testcase + "candidate_score.txt");
        if (!fout.is_open()){
            cerr << "cannout open output file\n";
            exit(1);
        }*/
        
        if (root_candidates.empty()) {
            std::cerr << "Error: No candidates available at Root!" << std::endl;
            return;
        }
        //else cout << "candidates size: " << root_candidates.size() << endl;

        int best_candidate_idx = 0;

        // 如果有多個候選人，啟動沙盒評分與多情境投票
        if (root_candidates.size() > 1) {
            std::vector<skew::ScoreSummary> candidate_scores;
            // 紀錄原始成績 (做為分母)
            double original_tns_ss = 0.0, original_wns_ss = 0.0;
            double original_tns_ff = 0.0, original_wns_ff = 0.0;
            calculateOriginalMetrics(original_tns_ss, original_wns_ss, original_tns_ff, original_wns_ff);
            double original_area = db.originalArea;

            // output original information
            /*fout << "Original: \n" 
                 << "Area = " << original_area << "\n";
            fout << "Setup\n";
            fout << "TNS = " << original_tns_ss << "\n";
            fout << "WNS = " << original_wns_ss << "\n\n";
            fout << "Hold\n";
            fout << "TNS = " << original_tns_ff << "\n";
            fout << "WNS = " << original_wns_ff << "\n\n";*/

            // 1. 沙盒模擬 (Sandbox Evaluation)
            int candidate = 0;
            for (const auto& state : root_candidates) {
                skew::DesignDB sandbox_db = db; // 深拷貝
                // 套用 operations
                applyOperationsToDB(sandbox_db, state.operations);
                candidate ++;
                // 重新計算 Clock Arrival 與 Slacks
                sandbox_db.computeClockArrival();
                sandbox_db.computeAllSlacks();
                
                // 結算成績
                skew::ScoreSummary summary = calculateSummary(sandbox_db);
                candidate_scores.push_back(summary);
            }
            
            // 2. 多情境投票
            best_candidate_idx = selectRobustWinner(
                candidate_scores,
                original_tns_ss, original_wns_ss,
                original_tns_ff, original_wns_ff,
                original_area
            );
            // output best candidate's information
            /*dumpCandidateScore(candidate_scores, root_candidates, best_candidate_idx,
                original_tns_ss, original_wns_ss, original_tns_ff, original_wns_ff, original_area, fout
            );*/

#ifdef BENCH
            // ---- 報告用量測: 同一組候選解上，比較「舊選解」與「新選解」各自的結果 ----
            benchDump(root_candidates, candidate_scores, best_candidate_idx,
                      original_wns_ss, original_tns_ss,
                      original_wns_ff, original_tns_ff, original_area);
#endif
        }
#ifdef BENCH
        else {
            std::cerr << "BENCH_SINGLE," << root_candidates.size() << "\n";
        }
#endif

        //cout << "choose candidate: " << best_candidate_idx << "\n";
        // 3. 正式套用最佳解到 Main DB
        applyOperationsToDB(db, root_candidates[best_candidate_idx].operations);
        
        db.computeClockArrival();
        db.computeAllSlacks();
        
        // 4. 輸出最終 Tree 結構
        // cout << "into output" << endl;
        exportModifiedClockTree(outputFile);
    }

private:
    skew::DesignDB& db;

    // 將 DPState 記錄的 operations 實際修改到 Tree 上
    void applyOperationsToDB(skew::DesignDB& target_db, const std::vector<skew::Operation>& ops) {
        int newBufferCount = 0;
        int resizecount = 0;
        for (const auto& op : ops) {
            if (op.type == skew::OperationType::RESIZE_BUFFER) {
                // Resize: 更換 cellId 與 instType
                target_db.tree[op.nodeId].cellId = op.newCellId;
                target_db.tree[op.nodeId].instType = target_db.getCell(op.newCellId).name;
                resizecount += 1;
            } 
            else if (op.type == skew::OperationType::INSERT_BUFFER) {
                // Insert: 新增 Node
                // add new_buf_name
                string newBufferName = "NEW_BUF_" + to_string(newBufferCount);
                int new_node_id = target_db.addTreeNode(
                    newBufferName, 
                    skew::NodeType::BUFFER, 
                    target_db.getCell(op.newCellId).name, 
                    target_db.tree[op.insertChildId].level, // 調整 Level
                    op.newCellId
                );
                //cout << "newbuffername = " << newBufferName << " ";
                newBufferCount++;
                
                // 將 New Buffer 標記起來
                target_db.tree[new_node_id].isNewBuffer = true;

                // 重新綁定 Parent 與 Child
                replaceChildInTree(target_db, op.insertParentId, op.insertChildId, new_node_id);
                target_db.setParent(op.insertChildId, new_node_id);
                target_db.tree[op.insertChildId].level; // 更新下游 Level
                updateSubtreeLevel(
                    target_db,
                    op.insertChildId,
                    1
                );
            }

        }
        //cout << "resize(" << resizecount << "), insert(" << newBufferCount << ")\n";
    }

    // add update level function
    void updateSubtreeLevel(DesignDB& db, int nodeId, int delta){
        db.tree[nodeId].level += delta;

        for(int child : db.tree[nodeId].children){
            updateSubtreeLevel(db, child, delta);
        }
    }

    // 將 Child 從 Parent 的陣列中替換為 New Node
    void replaceChildInTree(skew::DesignDB& target_db, int parentId, int oldChildId, int newChildId) {
        auto& children = target_db.tree[parentId].children;
        for (auto& child : children) {
            if (child == oldChildId) {
                child = newChildId;
                break;
            }
        }
        target_db.tree[newChildId].parent = parentId;
    }

    // 計算初始指標
    void calculateOriginalMetrics(double& tns_ss, double& wns_ss, double& tns_ff, double& wns_ff) {
        tns_ss = 0.0; wns_ss = 0.0;
        tns_ff = 0.0; wns_ff = 0.0;
        for (const auto& path : db.paths) {
            if (path.setupSlack < 0) {
                tns_ss += path.setupSlack;
                wns_ss = std::min(wns_ss, path.setupSlack);
            }
            if (path.holdSlack < 0) {
                tns_ff += path.holdSlack;
                wns_ff = std::min(wns_ff, path.holdSlack);
            }
        }
    }

    // 結算沙盒成績
    skew::ScoreSummary calculateSummary(const skew::DesignDB& target_db) {
        skew::ScoreSummary summary;
        summary.tnsSS = 0.0; summary.wnsSS = 0.0;
        summary.tnsFF = 0.0; summary.wnsFF = 0.0;

        for (const auto& path : target_db.paths) {
            if (path.setupSlack < 0) {
                summary.tnsSS += path.setupSlack;
                summary.wnsSS = std::min(summary.wnsSS, path.setupSlack);
            }
            if (path.holdSlack < 0) {
                summary.tnsFF += path.holdSlack;
                summary.wnsFF = std::min(summary.wnsFF, path.holdSlack);
            }
        }
        summary.totalArea = target_db.computeTotalBufferArea();
        return summary;
    }

    // 單一權重劇本下，計算某候選人的正規化分數
    //   每一項都用「相對原始值的改善比例」正規化 (0 = 沒改善, 1 = 完全修好)，
    //   讓 TNS / WNS / Area 三個量級不同的指標可以放在同一把尺上加權比較。
    //   註: 當原始值為 0 (該 corner 本來就沒違例) 時，該項對所有候選人皆為常數 0，
    //       不影響同一劇本內的排名，可安全略過。
    double scoreOneScenario(
        const skew::ScoreSummary& sc, const WeightScenario& w,
        double orig_tns_ss, double orig_wns_ss,
        double orig_tns_ff, double orig_wns_ff,
        double orig_area) const
    {
        double t_tns_ss = (orig_tns_ss < 0) ? (1.0 - sc.tnsSS / orig_tns_ss) : 0.0;
        double t_tns_ff = (orig_tns_ff < 0) ? (1.0 - sc.tnsFF / orig_tns_ff) : 0.0;
        double t_wns_ss = (orig_wns_ss < 0) ? (1.0 - sc.wnsSS / orig_wns_ss) : 0.0;
        double t_wns_ff = (orig_wns_ff < 0) ? (1.0 - sc.wnsFF / orig_wns_ff) : 0.0;
        double t_area   = (orig_area  > 0) ? (1.0 - sc.totalArea / orig_area) : 0.0;

        // 對齊官方公式: α·(TNS_SS+TNS_FF) + β·(WNS_SS+WNS_FF) + γ·Area
        return w.w_tns  * (t_tns_ss + t_tns_ff)
             + w.w_wns  * (t_wns_ss + t_wns_ff)
             + w.w_area * (t_area);
    }

    // 多情境投票機制 (Borda 計分: 每個劇本內前三名分別得 3 / 2 / 1 分，累加取總分最高者)
    int selectRobustWinner(
        const std::vector<skew::ScoreSummary>& scores,
        double orig_tns_ss, double orig_wns_ss,
        double orig_tns_ff, double orig_wns_ff,
        double orig_area)
    {
        // 隱藏權重劇本: 官方側重 slack、面積佔比低，因此所有劇本 w_area 皆壓在 0.05~0.10，
        // 並在不確定 α:β 真實比例的前提下，涵蓋 TNS 主導 / WNS 主導 / 兩者均衡多種切分。
        std::vector<WeightScenario> scenarios = {
            // {w_tns, w_wns, w_area}
            {0.60, 0.35, 0.05}, // TNS 主導
            {0.35, 0.60, 0.05}, // WNS 主導
            {0.50, 0.40, 0.10}, // 均衡略偏 TNS
            {0.45, 0.45, 0.10}, // TNS / WNS 對半
            {0.55, 0.40, 0.05}, // slack 極重、面積極輕
        };

        // 前三名得分表
        const double RANK_POINTS[3] = {3.0, 2.0, 1.0};

        std::vector<double> borda_points(scores.size(), 0.0);
        std::vector<double> score_sum(scores.size(), 0.0); // 連續分數總和，作為 Borda 平手時的細分依據

        for (const auto& w : scenarios) {
            // 算出此劇本下每個候選人的分數
            std::vector<std::pair<double, int>> ranked; // (score, candidateIdx)
            ranked.reserve(scores.size());
            for (size_t i = 0; i < scores.size(); ++i) {
                double s = scoreOneScenario(
                    scores[i], w,
                    orig_tns_ss, orig_wns_ss,
                    orig_tns_ff, orig_wns_ff,
                    orig_area
                );
                ranked.push_back({s, static_cast<int>(i)});
                score_sum[i] += s;
            }

            // 由高到低排序，前三名發 3 / 2 / 1 分
            std::sort(ranked.begin(), ranked.end(),
                      [](const std::pair<double,int>& a, const std::pair<double,int>& b) {
                          return a.first > b.first;
                      });
            for (int r = 0; r < 3 && r < static_cast<int>(ranked.size()); ++r) {
                borda_points[ranked[r].second] += RANK_POINTS[r];
            }
        }

        // 取 Borda 總分最高者；平手時以連續分數總和細分
        int final_winner_index = 0;
        for (size_t i = 1; i < borda_points.size(); ++i) {
            if (borda_points[i] > borda_points[final_winner_index] ||
                (borda_points[i] == borda_points[final_winner_index] &&
                 score_sum[i]    >  score_sum[final_winner_index])) {
                final_winner_index = static_cast<int>(i);
            }
        }
        return final_winner_index;
    }

#ifdef BENCH
    // ============================================================
    //  以下僅在 -DBENCH 編譯時存在，用來產生報告用的「舊 vs 新」對照數據，
    //  不影響正式輸出，也不更動其他模組。
    // ============================================================

    // 舊版選解邏輯: 只看 TNS + Area、贏者全拿 1 票
    int selectRobustWinner_OLD(const std::vector<skew::ScoreSummary>& scores,
                               double ots, double otf, double oa) {
        const double S[3][3] = {{0.33,0.33,0.34},{0.45,0.45,0.10},{0.60,0.20,0.20}};
        std::vector<int> votes(scores.size(), 0);
        for (const auto& w : S) {
            double best = -skew::INF; int wi = -1;
            for (size_t i = 0; i < scores.size(); ++i) {
                const auto& sc = scores[i];
                double tss = (ots < 0) ? (1.0 - sc.tnsSS/ots) : 1.0;
                double tff = (otf < 0) ? (1.0 - sc.tnsFF/otf) : 1.0;
                double ta  = (oa  > 0) ? (1.0 - sc.totalArea/oa) : 0.0;
                double cur = w[0]*tss + w[1]*tff + w[2]*ta;
                if (cur > best) { best = cur; wi = static_cast<int>(i); }
            }
            if (wi != -1) votes[wi]++;
        }
        int fi = 0, mv = -1;
        for (size_t i = 0; i < votes.size(); ++i) if (votes[i] > mv) { mv = votes[i]; fi = static_cast<int>(i); }
        return fi;
    }

    // 重算某候選解的完整指標 (含違例路徑數 NVP)
    void benchMetrics(const std::vector<skew::Operation>& ops,
                      double& swns, double& stns, int& snvp,
                      double& hwns, double& htns, int& hnvp, double& area) {
        skew::DesignDB sb = db;
        applyOperationsToDB(sb, ops);
        sb.computeClockArrival();
        sb.computeAllSlacks();
        swns = stns = hwns = htns = 0.0; snvp = hnvp = 0;
        for (const auto& p : sb.paths) {
            if (p.setupSlack < 0) { stns += p.setupSlack; swns = std::min(swns, p.setupSlack); snvp++; }
            if (p.holdSlack  < 0) { htns += p.holdSlack;  hwns = std::min(hwns, p.holdSlack);  hnvp++; }
        }
        area = sb.computeTotalBufferArea();
    }

    void benchDump(const std::vector<skew::DPState>& cands,
                   const std::vector<skew::ScoreSummary>& scores, int newIdx,
                   double owns_ss, double otns_ss, double owns_ff, double otns_ff, double oarea) {
        int oldIdx = selectRobustWinner_OLD(scores, otns_ss, otns_ff, oarea);
        double a[7], b[7]; int an[2], bn[2];
        benchMetrics(cands[oldIdx].operations, a[0],a[1],an[0],a[2],a[3],an[1],a[4]);
        benchMetrics(cands[newIdx].operations, b[0],b[1],bn[0],b[2],b[3],bn[1],b[4]);
        std::cerr << "ORIG," << owns_ss << "," << otns_ss << "," << owns_ff << "," << otns_ff << "," << oarea << "\n";
        std::cerr << "BENCH,ncand=" << cands.size() << ",oldIdx=" << oldIdx << ",newIdx=" << newIdx << "\n";
        std::cerr << "OLD," << a[0] << "," << a[1] << "," << an[0] << "," << a[2] << "," << a[3] << "," << an[1] << "," << a[4] << "\n";
        std::cerr << "NEW," << b[0] << "," << b[1] << "," << bn[0] << "," << b[2] << "," << b[3] << "," << bn[1] << "," << b[4] << "\n";
    }
#endif

    // recursive output Clock Tree
    void exportModifiedClockTree(const std::string& filename) {
        std::ofstream out(filename);
        if (!out.is_open()){
            cerr << "cannout open output file\n";
            exit(1);
        }
        if (!out.is_open()) {
            std::cerr << "Error: Cannot open output file: " << filename << std::endl;
            return;
        }

        out << "Root: " << db.tree[db.rootId].name << "\n";
        for (int childId : db.tree[db.rootId].children) {
            printNode(out, childId);
        }
        out.close();
    }

    // DFS print
    void printNode(std::ofstream& out, int nodeId) {
        const auto& node = db.tree[nodeId];
        // add tab
        for (int i = 1; i <= node.level; i++)
            out << "\t";
        out << "[" << node.level << "] " << node.name << " (" << node.instType << ")";
        if (node.isSink) out << " (SINK)";
        out << "\n";
        
        for (int childId : node.children) {
            printNode(out, childId);
        }
    }

    // debug
    void dumpOperations(
        std::ostream& os,
        const std::vector<Operation>& ops)
    {
        for(const auto& op : ops)
        {
            const auto& node = db.tree[op.nodeId];
            os << node.name << " ";
            if(op.type ==
                OperationType::NONE)
            {
                os << "NoOp\n";
            }

            else if(op.type ==
                OperationType::RESIZE_BUFFER)
            {
                os
                << "Resize ";

                os
                << db.tree[op.nodeId].name;

                os
                << " ";

                os
                << db.getCell(
                    op.oldCellId
                ).name;

                os
                << " -> ";

                os
                << db.getCell(
                    op.newCellId
                ).name;

                os << "\n";
            }

            else if(op.type ==
                OperationType::INSERT_BUFFER)
            {
                os
                << "Insert ";

                os
                << op.newBufferName;

                os
                << " between ";

                os
                << db.tree[
                    op.insertParentId
                ].name;

                os
                << " and ";

                os
                << db.tree[
                    op.insertChildId
                ].name;

                os
                << "\n";
            }
        }
    }

    void dumpCandidateScore(
        const std::vector<ScoreSummary>& scores,
        const std::vector<DPState>& candidates,
        const int best_idx,
        const double& original_tns_ss,
        const double& original_wns_ss,
        const double& original_tns_ff,
        const double& original_wns_ff,
        const double& original_area,
        std::ofstream& fout
    )
    {
            const auto& s = scores[best_idx];

            fout
                << "====================================\n";

            fout
                << "Candidate "
                << best_idx
                << "\n";

            fout
                << "AreaDelta = "
                << candidates[best_idx].areaDelta
                << "\n\n";

            fout
                << "Setup\n";

            fout
                << "TNS = "
                << s.tnsSS
                << "\n";

            fout
                << "WNS = "
                << s.wnsSS
                << "\n\n";

            fout
                << "Hold\n";

            fout
                << "TNS = "
                << s.tnsFF
                << "\n";

            fout
                << "WNS = "
                << s.wnsFF
                << "\n";

            /*fout
                << "Operations\n";

            dumpOperations(
                    fout,
                    candidates[i].operations
            );*/
            double ss_tns = (1 - s.tnsSS/original_tns_ss);
            double ss_wns = (1 - s.wnsSS/original_wns_ss);
            fout
                << "ss_score = (tns)"
                << ss_tns
                << ", (wns)"
                << ss_wns
                << "\n-> "
                << (ss_tns + ss_wns)
                << "\n";
            double ff_tns = (1 - s.tnsFF/original_tns_ff);
            double ff_wns = (1 - s.wnsFF/original_wns_ff);
            fout
                << "ff_score = (tns)"
                << ff_tns
                << ", (wns)"
                << ff_wns
                << "\n-> "
                << (ff_tns + ff_wns)
                << "\n";
            
            fout
                << "area_score = "
                << (1 - s.totalArea/original_area)
                << "\n";
            
            fout << "\n";
    }
};