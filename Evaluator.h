#include "ds.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

// 定義權重劇本
struct WeightScenario {
    double alpha;
    double beta;
    double gamma;
};

class Evaluator {
public:
    Evaluator(skew::DesignDB& main_db) : db(main_db) {}

    // 主進入點
    void runTopKEvaluationAndOutput(const std::vector<skew::DPState>& root_candidates, const std::string& out_file) {
        if (root_candidates.empty()) {
            std::cerr << "Error: No candidates available at Root!" << std::endl;
            return;
        }

        int best_candidate_idx = 0;

        // 如果有多個候選人，啟動沙盒評分與多情境投票
        if (root_candidates.size() > 1) {
            std::vector<skew::ScoreSummary> candidate_scores;
            
            // 紀錄原始成績 (做為分母)
            double original_tns_ss = 0.0, original_wns_ss = 0.0;
            double original_tns_ff = 0.0, original_wns_ff = 0.0;
            calculateOriginalMetrics(original_tns_ss, original_wns_ss, original_tns_ff, original_wns_ff);
            double original_area = db.originalArea;

            // 1. 沙盒模擬 (Sandbox Evaluation)
            for (const auto& state : root_candidates) {
                skew::DesignDB sandbox_db = db; // 深拷貝
                
                // 套用 operations
                applyOperationsToDB(sandbox_db, state.operations);
                
                // 重新計算 Clock Arrival 與 Slacks
                sandbox_db.computeClockArrival();
                sandbox_db.computeAllSlacks();
                
                // 結算成績
                skew::ScoreSummary summary = calculateSummary(sandbox_db);
                candidate_scores.push_back(summary);
            }

            // 2. 多情境投票
            best_candidate_idx = selectRobustWinner(
                candidate_scores, original_tns_ss, original_tns_ff, original_area
            );
        }

        // 3. 正式套用最佳解到 Main DB
        applyOperationsToDB(db, root_candidates[best_candidate_idx].operations);
        db.computeClockArrival();
        db.computeAllSlacks();
        
        // 4. 輸出最終 Tree 結構
        exportModifiedClockTree(out_file);
    }

private:
    skew::DesignDB& db;

    // 將 DPState 記錄的 operations 實際修改到 Tree 上
    void applyOperationsToDB(skew::DesignDB& target_db, const std::vector<skew::Operation>& ops) {
        for (const auto& op : ops) {
            if (op.type == skew::OperationType::RESIZE_BUFFER) {
                // Resize: 更換 cellId 與 instType
                target_db.tree[op.nodeId].cellId = op.newCellId;
                target_db.tree[op.nodeId].instType = target_db.getCell(op.newCellId).name;
            } 
            else if (op.type == skew::OperationType::INSERT_BUFFER) {
                // Insert: 新增 Node
                int new_node_id = target_db.addTreeNode(
                    op.newBufferName, 
                    skew::NodeType::BUFFER, 
                    target_db.getCell(op.newCellId).name, 
                    target_db.tree[op.insertParentId].level + 1, // 調整 Level
                    op.newCellId
                );
                
                // 將 New Buffer 標記起來
                target_db.tree[new_node_id].isNewBuffer = true;

                // 重新綁定 Parent 與 Child
                replaceChildInTree(target_db, op.insertParentId, op.insertChildId, new_node_id);
                target_db.setParent(op.insertChildId, new_node_id);
                target_db.tree[op.insertChildId].level += 1; // 更新下游 Level
            }
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
        target_db.setParent(newChildId, parentId);
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

    // 多情境投票機制
    int selectRobustWinner(
        const std::vector<skew::ScoreSummary>& scores, 
        double orig_tns_ss, double orig_tns_ff, double orig_area) 
    {
        // 建立多種可能的隱藏權重劇本
        std::vector<WeightScenario> scenarios = {
            {0.33, 0.33, 0.34}, // 平均
            {0.45, 0.45, 0.10}, // Timing 優先
            {0.60, 0.20, 0.20}, // SS 優先
            {0.20, 0.60, 0.20}, // FF 優先
            {0.20, 0.20, 0.60}  // 面積優先
        };

        std::vector<int> vote_counts(scores.size(), 0);

        for (const auto& scenario : scenarios) {
            double best_score = -skew::INF;
            int winner_index = -1;

            for (size_t i = 0; i < scores.size(); ++i) {
                const auto& sc = scores[i];
                
                double term_ss = (orig_tns_ss < 0) ? (1.0 - (sc.tnsSS / orig_tns_ss)) : 1.0;
                double term_ff = (orig_tns_ff < 0) ? (1.0 - (sc.tnsFF / orig_tns_ff)) : 1.0;
                double term_area = (orig_area > 0) ? (1.0 - (sc.totalArea / orig_area)) : 0.0;

                double current_score = (scenario.alpha * term_ss) + 
                                       (scenario.beta * term_ff) + 
                                       (scenario.gamma * term_area);

                if (current_score > best_score) {
                    best_score = current_score;
                    winner_index = i;
                }
            }
            if (winner_index != -1) vote_counts[winner_index]++;
        }

        // find the highest votes
        int final_winner_index = 0;
        int max_votes = -1;
        for (size_t i = 0; i < vote_counts.size(); ++i) {
            if (vote_counts[i] > max_votes) {
                max_votes = vote_counts[i];
                final_winner_index = i;
            }
        }
        return final_winner_index;
    }

    // recursive output Clock Tree
    void exportModifiedClockTree(const std::string& filename) {
        std::ofstream out(filename);
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
        out << "[" << node.level << "] " << node.name << " (" << node.instType << ")";
        if (node.isSink) out << " (SINK)";
        out << "\n";
        
        for (int childId : node.children) {
            printNode(out, childId);
        }
    }
};