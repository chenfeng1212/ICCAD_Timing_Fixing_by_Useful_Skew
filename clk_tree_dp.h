#ifndef CLK_TREE_DP_H
#define CLK_TREE_DP_H

#include "ds.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>

using namespace std;
using namespace skew;

namespace skew{
    class ClkTreeDP{
    public:
        using DPStates = vector<DPState>;

        ClkTreeDP(
            DesignDB& db,
            double lambda,
            int topK,
            double bucketPrecision,
            int bucketKeep,
            const string& testcase
        )
        :m_db(db),
        m_nodes(db.tree),
        m_cellLibs(db.libs),
        m_ffInfos(db.ffs),
        m_lambda(lambda),
        m_topK(topK),
        m_bucketPrecision(bucketPrecision),
        m_bucketKeep(bucketKeep)
        {
            //m_DebugExpand.open(testcase + "/expand_debug.txt");
            //m_DebugPrune.open(testcase + "/prune_debug.txt");
            //m_check.open(testcase + "/check.txt");
        }

        DPStates RunDP(const int rootId){
            //Timer timer("DP");
            //cout << "into RunDP" << endl;
            DPStates states = DP(rootId);
            if (states.empty()){
                throw runtime_error("No solution");
            }
            return states;
        }

    private:
        //DesignDB
        DesignDB& m_db;
        const vector<TreeNode>& m_nodes;
        const vector<CellLib>& m_cellLibs;
        const vector<FFInfo>& m_ffInfos;
        double m_lambda;
        int m_topK;
        double m_bucketPrecision;
        int m_bucketKeep;

        //debug
        std::ofstream m_check;
        std::ofstream m_DebugExpand;
        std::ofstream m_DebugPrune;

    /*
        void Debug_NoOp(const TreeNode& u, DPState& newState){
            m_DebugExpand.open("testcase0/expand_debug.txt");
            m_DebugExpand
                << "Node "
                << u.name
                << "\n";

            m_DebugExpand
                << "Operation : NoOp\n";

            m_DebugExpand
                << "SS Delay = "
                << newState.ssDelayDelta
                << "\n";

            m_DebugExpand
                << "FF Delay = "
                << newState.ffDelayDelta
                << "\n";

            m_DebugExpand
                << "Gain = "
                << newState.estimatedGain
                << "\n\n";
        }
        
        void Debug_Resize(const TreeNode& u, DPState& newState, const CellLib& oldCell, const CellLib& newCell,
            double deltaSS, double deltaFF, double deltaArea)
            {
            m_DebugExpand
                << "Node "
                << u.name
                << "\n";

            m_DebugExpand
                << "Resize "
                << oldCell.name
                << " -> "
                << newCell.name
                << "\n";

            m_DebugExpand
                << "DeltaSS = "
                << deltaSS
                << "\n";

            m_DebugExpand
                << "DeltaFF = "
                << deltaFF
                << "\n";

            m_DebugExpand
                << "DeltaArea = "
                << deltaArea
                << "\n";

            m_DebugExpand
                << "Gain = "
                << newState.estimatedGain
                << "\n\n";
        }

        void DumpState(
            std::ostream& os,
            const DPState& state
        )
        {
            os << "Gain = "
            << state.estimatedGain
            << "\n";

            os << "SS Delay Delta = "
            << state.ssDelayDelta
            << "\n";

            os << "FF Delay Delta = "
            << state.ffDelayDelta
            << "\n";

            os << "Area Delta = "
            << state.areaDelta
            << "\n";

            os << "Operations:\n";

            for(size_t i=0;i<state.operations.size();i++)
            {
                const auto& op =
                    state.operations[i];

                os << "[" << i << "] ";

                if(op.type ==
                OperationType::NONE)
                {
                    os << "NoOp";
                }
                else if(op.type ==
                        OperationType::RESIZE_BUFFER)
                {
                    os << "Resize";

                    os << " node="
                    << m_nodes[op.nodeId].name;

                    os << " "
                    << m_db.getCell(op.oldCellId).name;

                    os << " -> "
                    << m_db.getCell(op.newCellId).name;
                }
                else if(op.type ==
                        OperationType::INSERT_BUFFER)
                {
                    os << "Insert";

                    os << " parent="
                    << m_nodes[op.insertParentId].name;

                    os << " buffer="
                    << op.newBufferName;
                }

                os << "\n";
            }

            os << "\n";
        }
        
    */
        // debug end

        //----------------------------------
        // Main DP
        //----------------------------------

        DPStates DP(const int nodeId){
            const TreeNode& u = m_nodes[nodeId];
            string name = u.name;
            //m_DebugPrune << "====================================\n";
            //m_DebugPrune << "Node: " << name << endl;
            // if u is FF, return leaf state
            if (u.isFF()){
                return InitializeLeaf(name);
            }
            
            vector<DPStates>childStates = {};
            //get DP(child)
            for (auto &child : u.children){
                childStates.push_back(DP(child));
            }

            //merge
            //m_DebugPrune << "into get child DP, size = " << childStates.size() << endl;
            //m_DebugPrune << "into get Merge, ";
            DPStates mergedStates = MergeChildren(childStates);
            //m_DebugPrune << "size = " << mergedStates.size() << endl;

            // avoid memory out of range
            sort(mergedStates.begin(), mergedStates.end(), [](const DPState& a, const DPState& b){
                return a.estimatedGain > b.estimatedGain;
            });
            if (mergedStates.size() > 200)
                mergedStates.resize(200);

            // return if u is rootId (cannot do any expand operations)
            if (u.id == m_db.rootId){
                return Prune(mergedStates, m_topK, m_bucketPrecision, m_bucketKeep);
            }
            //m_check << "After merged: " << name << ": " << mergedStates.size() << endl;

            DPStates expanded;
            //m_DebugPrune << "into get expand, ";

            for (const auto& state : mergedStates){
                if (u.isBuffer()){
                    auto noOpStates = ExpandNoOp(state, u, m_lambda);

                    expanded.insert(expanded.end(), noOpStates.begin(), noOpStates.end());
                    
                    auto resizeStates = ExpandResize(state, u, m_lambda);

                    expanded.insert(expanded.end(), resizeStates.begin(), resizeStates.end());
                }

                // insert buffer between node u and parent of u
                auto insertStates = ExpandInsert(state, u, m_lambda);

                expanded.insert(expanded.end(), insertStates.begin(), insertStates.end());

                // use sort will keep better candidates states (prune didn't work well)
                // expanded = Prune(expanded, 500, m_bucketPrecision, m_bucketKeep);
                sort(expanded.begin(), expanded.end(), [](const DPState& a, const DPState& b){
                    return a.estimatedGain > b.estimatedGain;
                });
                if (expanded.size() > 200)
                    expanded.resize(200);
            }

            //m_DebugPrune << "size = " << expanded.size() << endl;
            return Prune(expanded, m_topK, m_bucketPrecision, m_bucketKeep);
        }
    

        //----------------------------------
        // Leaf
        //----------------------------------

        DPStates InitializeLeaf(const string& name){
            DPState state;
            int ffId = m_db.getFFId(name);
            const FFInfo& ff = m_db.ffs[ffId];
            state.ssDelayDelta = 0.0;
            state.ffDelayDelta = 0.0;
            state.areaDelta = 0.0;

            state.sssumTargetShift = ff.targetShiftSS;
            state.ffsumTargetShift = ff.targetShiftFF;
            state.ffCount = 1;

            state.estimatedGain = 0.0;

            return {state};
        }

        //----------------------------------
        // Merge
        //----------------------------------

        DPState Merge(const DPState& A, const DPState& B){
            DPState C;
            C.ffCount = A.ffCount + B.ffCount;
            C.sssumTargetShift = A.sssumTargetShift + B.sssumTargetShift;
            C.ffsumTargetShift = A.ffsumTargetShift + B.ffsumTargetShift;
            C.ssDelayDelta = (A.ssDelayDelta * A.ffCount + B.ssDelayDelta * B.ffCount) / C.ffCount; // use weighted average
            C.ffDelayDelta = (A.ffDelayDelta * A.ffCount + B.ffDelayDelta * B.ffCount) / C.ffCount; // use weighted average
            C.areaDelta = A.areaDelta + B.areaDelta;
            C.operations.reserve(A.operations.size() + B.operations.size());
            C.operations.insert(C.operations.end(), A.operations.begin(), A.operations.end());
            C.operations.insert(C.operations.end(), B.operations.begin(), B.operations.end());

            return C;
        }

        DPStates MergeChildren(const vector<DPStates>& childStates){
            if (childStates.empty()){
                return {};
            }

            DPStates result = childStates[0];

            for (size_t i = 1; i < childStates.size(); i++){
                DPStates temp;
                for (const auto& s1 : result){
                    for (const auto& s2 : childStates[i]){
                        temp.push_back(Merge(s1, s2));
                    }
                }

                // sort after each time merge to avoid memory out of range
                sort(temp.begin(), temp.end(), [](const DPState& a, const DPState& b){
                    return a.estimatedGain > b.estimatedGain;
                });
                if (temp.size() > 100)
                    temp.resize(100);
                result.swap(temp);
            }
            return result;
        }

        //----------------------------------
        // Expansion
        //----------------------------------

        DPStates ExpandNoOp(const DPState& state, const TreeNode& u, double lambda){
            DPState newState = state;

            newState.operations.push_back(Operation::NoOp(u.id));
            newState.estimatedGain = Evaluate(newState, lambda);
            return {newState};
        }

        DPStates ExpandResize(const DPState& state, const TreeNode& u, double lambda){
            DPStates candidates;
            string name = u.name;
            int cellId = u.cellId;
            const CellLib& oldCell = m_db.getCell(cellId);

            int fanout = (int)u.children.size();
            double oldSS = oldCell.getDelaySS(fanout);
            double oldFF = oldCell.getDelayFF(fanout);
            double oldArea = oldCell.area;
            for (const CellLib& newCell: m_cellLibs){
                if (newCell.name == oldCell.name){
                    continue;
                }
                if (!newCell.isLegalFanout(fanout)){
                    continue;
                }

                double deltaSS = newCell.getDelaySS(fanout) - oldSS;
                double deltaFF = newCell.getDelayFF(fanout) - oldFF;
                double deltaArea = newCell.area - oldArea;
                DPState newState = state;
                newState.ssDelayDelta += deltaSS;
                newState.ffDelayDelta += deltaFF;
                newState.areaDelta += deltaArea;

                int oldCellId = m_db.getCellId(oldCell.name);
                int newCellId = m_db.getCellId(newCell.name);

                newState.operations.push_back(Operation::Resize(u.id, oldCellId, newCellId, deltaArea, deltaSS, deltaFF));
                newState.estimatedGain = Evaluate(newState, lambda);
                candidates.push_back(move(newState));
            }
            
            return candidates;
        }

        DPStates ExpandInsert(const DPState& state, const TreeNode& u, double lambda){
            //add node between node and parent of u
            DPStates candidates;
            
            int parent_Id = u.parent;
            if (parent_Id == INVALID_ID || u.id == m_db.rootId){
                return candidates;
            }
            int u_Id = u.id;
            int fanout = (int)u.children.size();

            for (const CellLib& buffer : m_cellLibs){
                string newCellName = buffer.name;
                int newCellId = m_db.getCellId(newCellName);
                double deltaSS = buffer.getDelaySS(1);
                double deltaFF = buffer.getDelayFF(1);
                double deltaArea = buffer.area;

                DPState newState = state;
                newState.ssDelayDelta += deltaSS;
                newState.ffDelayDelta += deltaFF;
                newState.areaDelta += deltaArea;

                //Insert(int parentId, int childId, int newCellId, double areaDelta, double ssDelta, double ffDelta)
                newState.operations.push_back(Operation::Insert(parent_Id, u_Id, newCellId, deltaArea, deltaSS, deltaFF));
                newState.estimatedGain = Evaluate(newState, lambda);

                candidates.push_back(move(newState));
            }
            return candidates;
        }

        //----------------------------------
        // Evaluation
        //----------------------------------

        double Evaluate(const DPState& state, double lambda) {
            if(state.ffCount == 0){
                return -1e18;
            }

            double meanTargetSS = state.sssumTargetShift / state.ffCount;
            double meanTargetFF = state.ffsumTargetShift / state.ffCount;

            // ssDelay >> ffDelay -> use weighted method so that the effect of ffDelay can be considered too
            double error = abs(meanTargetSS - state.ssDelayDelta) + 3 * abs(meanTargetFF - state.ffDelayDelta); // ss > ff = area
            double gain = -(error + lambda * state.areaDelta);
            
            return gain;
        }

        //----------------------------------
        // Pruning
        //----------------------------------

        bool Dominates(const DPState& A, const DPState& B){ // A better than B, don't keep B
            double errorA = -A.estimatedGain;
            double errorB = -B.estimatedGain;
            
            double A_SS = abs(A.sssumTargetShift / A.ffCount - A.ssDelayDelta);
            double B_SS = abs(B.sssumTargetShift / B.ffCount - B.ssDelayDelta);

            return (errorA < errorB && A_SS < B_SS);
            //return (errorA < errorB)或是return (errorA < errorB && (A_SS < B_SS || A_FF < B_FF))會只返回一個error最低的解（areadelta高，ss的tns會被優化到，但是其餘幾乎都惡化）
        }
        
        DPStates ParetoPrune(const DPStates& states){
            DPStates result;
            for (size_t i = 0; i < states.size(); i++){
                bool dominated = false;
                for (size_t j = 0; j < states.size(); j++){
                    if (i == j){
                        continue;
                    }
                    
                    if (Dominates(states[j], states[i])){
                        dominated = true;
                        break;
                    }
                }
                if (!dominated){
                    result.push_back(states[i]);
                }
            }
            sort(result.begin(), result.end(), [](const DPState& a, const DPState& b){
                return a.estimatedGain > b.estimatedGain;
            });
            // size can set larger to keep more candidates, but testcase4 will out of range
            if (result.size() > 100)
                result.resize(100);

            return result;
        }

        DPStates BucketPrune(const DPStates& states, double precision, int keepPerBucket){
            unordered_map<long long, DPStates> buckets;
            for (const auto& s : states){
                long long key = (long long)round(s.ssDelayDelta / precision);
                buckets[key].push_back(s);
            }

            DPStates result;

            for (auto& bucket : buckets){
                DPStates& vec = bucket.second;

                sort(vec.begin(), vec.end(),
                    [](const DPState& a, const DPState& b){
                        return a.estimatedGain > b.estimatedGain;
                    });

                int cnt = min(keepPerBucket, (int)vec.size());

                for (int i = 0; i < cnt; i++){
                    result.push_back(vec[i]);
                }
            }
            return result;

            
        }

        DPStates Prune(DPStates states, int topK, double precision, int bucketKeep){
            //int before = states.size();
            states = ParetoPrune(states);
            //int afterPareto = states.size();
            states = BucketPrune(states, precision, bucketKeep);
            //int afterBucket = states.size();

            sort(states.begin(), states.end(), [](const DPState& a, const DPState& b){
                return a.estimatedGain > b.estimatedGain;
            });

            if ((int)states.size() > topK){
                states.resize(topK);
            }

            //m_DebugPrune << "Before: " << before << "\n";
            //m_DebugPrune << "After Pareto: " << afterPareto << "\n";
            //m_DebugPrune << "After Bucket: " << afterBucket << "\n";
            //m_DebugPrune << "After TopK: " << states.size() << "\n";
            return states;
        }
    };
}

#endif