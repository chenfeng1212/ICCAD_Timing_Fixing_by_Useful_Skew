#ifndef BEAM_SEARCH_H
#define BEAM_SEARCH_H

#include "ds.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>

using namespace std;
using namespace skew;

enum class RepairSide{
    LAUNCH,
    CAPTURE
};

struct PathViolation{
    int pathId = INVALID_ID;

    double setupSlack = 0.0;
    double holdSlack = 0.0;

    double setupWeight = 0.0;
    double holdWeight = 0.0;

    double totalWeight = 0.0;
};

struct CandidateOperation{
    Operation operation;
    double SetupGain = 0.0;
    double HoldGain = 0.0;
    double score = 0.0;
};

struct BufferCandidate{
    int nodeId = INVALID_ID;

    double maxSetupSlack = 0.0;
    double maxHoldSlack = 0.0;
    double setupWeight = 0.0;
    double holdWeight = 0.0;
    double totalWeight = 0.0;

    int setupViolationCount = 0;
    int holdViolationCount = 0;

    int worst_path_count = 0;
};

namespace skew{
    class ClkTreeBS{
    public:
        using DPStates = vector<DPState>;
        ClkTreeBS(
            DesignDB& db,
            int rootId
        )
        :db(db),
        nodes(db.tree),
        cellLibs(db.libs),
        ffInfos(db.ffs),
        rootId(rootId)
        {}

        DPStates RunBS(){
            cout << "Nodes = " << nodes.size() << ", FFs = " << ffInfos.size() << endl;
            //check_violation.open("check_violation.txt");

            //output_candop.open("Candidates_Operations.txt");
            //output_delta.open("delta.txt");
            DPState preprocess = Preprocess();
            
            cout << "Time 0" << endl;
            DPStates states = BS(preprocess);

            //int cycle_count = (nodes.size() - ffInfos.size()) / 100;
            for (int i = 0; i < 10; i++){
                cout << "Time " << i+1 << endl;
                check_violation << "\n" << i+1 << endl;
                states = BS(states[0]);
                if (states[0].operations.size() == 0)
                    break;
            }
            //output_operations(states[0]);
            return states;
        }
    private:
        //DesignDB
        DesignDB& db;
        const vector<TreeNode>& nodes;
        const vector<CellLib>& cellLibs;
        const vector<FFInfo>& ffInfos;
        int rootId;

        unordered_map<int, BufferCandidate> bufferCandidates;

        //SolutionState solution;

        //debug
        ofstream check_violation;
        ofstream output_delta;
        ofstream output_ffpaths;
        ofstream output_candop;

        void output_LCA(const vector<PathViolation>& violations){
            ofstream check_LCA;
            check_LCA.open("check_LCA.txt");
            for (auto& v : violations){
                int pathId = v.pathId;
                auto& path = db.paths[pathId];

                check_LCA << pathId << ": "
                    << db.ffs[path.launchFF].name << " to " << db.ffs[path.captureFF].name << "\n";
                check_LCA << "setup: " << path.setupSlack 
                    << ", hold: " << path.holdSlack
                    << ", total : " << v.totalWeight << "\n";

                check_LCA << "LCA = " << nodes[path.LCAnode].name 
                    << "\nCapture Buffers = ";
                for (auto& c : path.captureBuffers)
                    check_LCA << nodes[c].name << ", ";
                check_LCA << "\nLaunch Buffers = ";
                for (auto& l : path.launchBuffers)
                    check_LCA << nodes[l].name << ", ";
                check_LCA << "\n\n";
            }
        }

        void output_violation(const vector<PathViolation>& violations){
            double setupTNS = 0.0;
            double holdTNS = 0.0;
            double setupWNS = 0.0;
            double holdWNS = 0.0;
            int SSvpCount = 0;
            int FFvpCount = 0;
            int i = 0;
            for (auto& v : violations){
                double setup = v.setupSlack;
                double hold = v.holdSlack;
                if (i < 20) {
                check_violation << i+1 << "(" << v.pathId << "): "
                    << "setup: " << setup 
                    << ", hold: " << hold
                    << ", total : " << v.totalWeight << "\n";
                    i++;
                }
                
                if (v.setupSlack < 0.0){
                    setupTNS += setup;
                    SSvpCount ++;
                    if (setupWNS > setup)
                        setupWNS = setup;
                }

                if (v.holdSlack < 0.0){
                    FFvpCount ++;
                    holdTNS += hold;
                    if (holdWNS > hold)
                        holdWNS = hold;
                }
            }
            check_violation << "\nSetupTNS = " << setupTNS << ", SetupWNS = " << setupWNS << ", Count = " << SSvpCount << "\n";
            check_violation << "\nHoldTNS = " << holdTNS << ", HoldWNS = " << holdWNS <<  ", Count = " << FFvpCount << "\n";
            check_violation << endl;
        }

        void output_candidates(const vector<BufferCandidate*>& rankedCandidates)
        {
            ofstream check_buffer;
            check_buffer.open("check_buffer_candidate.txt");

            //int topK = min((int)rankedCandidates.size(), 20);
            int topK = rankedCandidates.size();

            for(int i = 0; i < topK; i++)
            {
                const auto* candidate = rankedCandidates[i];

                int nodeId = candidate->nodeId;

                check_buffer 
                    << "============================\n";
                check_buffer
                    << "Rank : " << i + 1 << "\n";

                check_buffer
                    << "Node ID : "
                    << nodeId
                    << ", Level = "
                    << nodes[nodeId].level
                    << "\n";

                check_buffer
                    << "Node Name : "
                    << nodes[nodeId].name << "\n";

                check_buffer
                    << "Setup Weight : "
                    << candidate->setupWeight << "\n";

                check_buffer
                    << "Hold Weight : "
                    << candidate->holdWeight << "\n";

                check_buffer
                    << "Total Weight : "
                    << candidate->totalWeight << "\n";

                check_buffer
                    << "Setup Violation Count : "
                    << candidate->setupViolationCount << "\n";

                check_buffer
                    << "Hold Violation Count : "
                    << candidate->holdViolationCount << "\n";

                check_buffer << "\n";
            }
        }

        void output_operations(DPState& state){
            ofstream output_op;
            output_op.open("output_operations.txt");
            int count = 1;

            int resize_count = 0;
            int insert_count = 0;
            for (auto& op : state.operations){
                output_op << count << " (" << nodes[op.nodeId].name << ") : ";
                if (op.type == OperationType::RESIZE_BUFFER){
                    output_op << cellLibs[op.oldCellId].name << "->" << cellLibs[op.newCellId].name;
                    output_op << "(SS) " << op.ssDelayDelta << ", (FF) " << op.ffDelayDelta << ", (area) " << op.areaDelta << endl;
                    resize_count++;
                }
                else if (op.type == OperationType::INSERT_BUFFER){
                    output_op << " add " << cellLibs[op.newCellId].name;
                    output_op << "(SS) " << op.ssDelayDelta << ", (FF) " << op.ffDelayDelta << ", (area) " << op.areaDelta << endl;
                    insert_count++;
                }
                count++;
            }
            cout << "Resize = " << resize_count << ", Insert = " << insert_count << endl;
        }

        void output_FFpaths(){
            output_ffpaths.open("FFtoPaths.txt");
            for (int ffId = 0; ffId < db.FFtoPaths.size(); ffId++) {
                output_ffpaths << ffInfos[ffId].name << ": ";
                auto& paths = db.FFtoPaths[ffId];
                for (auto& path : paths) {
                    auto& p = db.paths[path.pathId];
                    if (path.pathRole == PathRole::CAPTURE){
                        output_ffpaths << "(CAPTURE)" << ffInfos[p.launchFF].name << "->" << ffInfos[p.captureFF].name << endl;
                    }
                    else if (path.pathRole == PathRole::LAUNCH){
                        output_ffpaths << "(LAUNCH)" << ffInfos[p.launchFF].name << "->" << ffInfos[p.captureFF].name << endl;
                    }
                }
            }
        }

        void output_candidateoperations(const vector<CandidateOperation>& candOP) {
            int count = 1;
            for (auto& cand : candOP){
                auto& op = cand.operation;
                output_candop << count << " ";
                output_candop << nodes[op.nodeId].name << ": ";
                if (op.type == OperationType::RESIZE_BUFFER){
                    output_candop << "Resize "
                        << cellLibs[op.oldCellId].name << " -> "
                        << cellLibs[op.newCellId].name;
                }
                else if (op.type == OperationType::INSERT_BUFFER){
                    output_candop << "Insert "
                        << cellLibs[op.newCellId].name;
                }
                output_candop << " (score = " << cand.score << ")\n";
                count++;
            }
        }

        DPStates BS(DPState& state){
            auto violations = UpdateViolationList(state);
            //output_violation(violations); // 輸出前20個critical paths
            CriticalRepair(state, violations);
            //update sorted violation paths
            cout << "Repaired_critical, ";
            violations = UpdateViolationList(state);// 將前10個critical paths進行優化
            //output_violation(violations); // 輸出解決原本最critical的10個paths後的前20個critical paths（基本上會變成tns惡化，但後面會慢慢修好）
            
            //sort hotspot candidate buffer
            auto rankedCandidates = rankBufferCandidates(state, violations);
            cout << "rankedCandidates = " << rankedCandidates.size() << endl;
            //output_candidates(rankedCandidates);

            //candidate expand operation -> save to solution states
            auto CandidateOperations = GenerateCandidateOperations(state, rankedCandidates);
            cout << "CandidateOperations = " << CandidateOperations.size() << endl;
            //output_candidateoperations(CandidateOperations);

            ApplyBestOperations(state, CandidateOperations, bufferCandidates);//將candidateOperation擇優放進state
            cout << "Operations: " << state.operations.size() << "\n\n";

            return {state};
        }

        // Pre-process
        DPState Preprocess(){
            db.subtreeFFs.resize(nodes.size(), {});
            buildFFtoPaths();
            output_FFpaths();
            collectSubtreeFF(rootId);
            DPState state = initializeNodeState();
            return state;
        }

        void buildFFtoPaths(){
            db.FFtoPaths.resize(ffInfos.size(), {});

            for(const auto& path : db.paths)
            {
                db.FFtoPaths[path.launchFF].push_back(
                {
                    path.id,
                    PathRole::LAUNCH
                });

                db.FFtoPaths[path.captureFF].push_back(
                {
                    path.id,
                    PathRole::CAPTURE
                });
            }
        }

        void collectSubtreeFF(const int nodeId){
            db.subtreeFFs[nodeId].clear();
            const TreeNode& node = db.tree[nodeId];

            if(node.type == NodeType::FF)
            {
                db.subtreeFFs[nodeId].push_back(node.ffId);
                return;
            }

            for(int childId : node.children)
            {
                collectSubtreeFF(childId);

                db.subtreeFFs[nodeId].insert(
                    db.subtreeFFs[nodeId].end(),
                    db.subtreeFFs[childId].begin(),
                    db.subtreeFFs[childId].end()
                );
            }

            sort(db.subtreeFFs[nodeId].begin(), db.subtreeFFs[nodeId].end());
        }

        DPState initializeNodeState(){
            DPState state;
            state.operations.clear();

            int pathSize = db.paths.size();

            state.pathSlackDeltaSS.assign(pathSize, 0.0);
            state.pathSlackDeltaFF.assign(pathSize, 0.0);

            int nodeSize = nodes.size();
            state.nodeOperationState.resize(nodeSize, {NodeOperationState(false, OperationType::NONE)});
            //for (auto& s : state.nodeOperationState){
            //    s = NodeOperationState(false, OperationType::NONE);
            //}

            return state;
        }
            
        //------------------------------------------
        //         Repair Critical Path
        //------------------------------------------
        //修好最嚴重的10個violation paths（方法：在FF上插入多個連續的buffer）
        //目前是都只挑選最大delay的buffer
        void CriticalRepair(DPState& state, const vector<PathViolation>& violations) {
            int topK = min(10, (int)violations.size());
            for (int i = 0; i < topK; i++) {
                auto& v = violations[i];
                if (v.setupSlack < 0.0 && v.setupWeight >= v.holdWeight) {
                    RepairCriticalSetupPath(state, v);
                }
                else if (violations[i].holdSlack < 0.0) {
                    RepairCriticalHoldPath(state, v);
                }
            }
        }
        
        void RepairCriticalSetupPath(DPState& state, const PathViolation& v) {
            auto& path = db.paths[v.pathId];
            int nodeId = ffInfos[path.captureFF].treeNodeId;
            int parentId = nodes[nodeId].parent;
            
            if (parentId == INVALID_ID)
                return;
            if (state.nodeOperationState[nodeId].modified)
                return;
            
            int insert_buffer = getMaxSSDelay();
            double ssDelay = cellLibs[insert_buffer].getDelaySS(1);
            double ffDelay = cellLibs[insert_buffer].getDelayFF(1);
            
            int insertCount = 0;
            double setup = path.setupSlack;
            double hold = path.holdSlack;

            const double targetSlack = -0.10; // 修復setup的目標
            const double holdLimit = 0.05; // 修復setup可以接受hold惡化的極限

            while (setup < targetSlack) { //用while方便每次insert檢查hold的惡化程度
                double newSetup = setup + ssDelay;
                double newHold = hold - ffDelay;

                if (newHold < holdLimit)
                    break;
                
                setup = newSetup;
                hold = newHold;

                insertCount++;

                if (insertCount >= 5)
                    break;
            }
            if (insertCount == 0)
                return;
            
            double ssDelta = ssDelay * insertCount;
            double ffDelta = ffDelay * insertCount;
            double areaDelta = cellLibs[insert_buffer].area;

            // build operation
            Operation op = Operation::Insert(parentId, nodeId, insert_buffer, insertCount, areaDelta, ssDelta, ffDelta);
            state.operations.push_back(op);
            state.nodeOperationState[nodeId].modified = true;

            UpdatePathDelta(state, op, true);
        }

        void RepairCriticalHoldPath(DPState& state, const PathViolation& v) {
            auto& path = db.paths[v.pathId];
            int nodeId = ffInfos[path.launchFF].treeNodeId;
            int parentId = nodes[nodeId].parent;
            
            if (parentId == INVALID_ID)
                return;
            if (state.nodeOperationState[nodeId].modified)
                return;
            
            int insert_buffer = getMaxFFDelay();
            double ssDelay = cellLibs[insert_buffer].getDelaySS(1);
            double ffDelay = cellLibs[insert_buffer].getDelayFF(1);
            
            int insertCount = 0;
            double setup = path.setupSlack;
            double hold = path.holdSlack;

            const double targetSlack = -0.05; // 修復hold的目標
            const double setupLimit = 0.05; // 修復hold可以接受setup惡化的極限

            while (setup < targetSlack) { //用while方便每次insert檢查hold的惡化程度
                double newSetup = setup - ssDelay;
                double newHold = hold + ffDelay;

                if (newSetup < setupLimit)
                    break;
                
                setup = newSetup;
                hold = newHold;

                insertCount++;

                if (insertCount >= 5)
                    break;
            }
            if (insertCount == 0)
                return;
            
            double ssDelta = ssDelay * insertCount;
            double ffDelta = ffDelay * insertCount;
            double areaDelta = cellLibs[insert_buffer].area;

            // build operation
            Operation op = Operation::Insert(parentId, nodeId, insert_buffer, insertCount, areaDelta, ssDelta, ffDelta);
            state.operations.push_back(op);
            state.nodeOperationState[nodeId].modified = true;

            UpdatePathDelta(state, op, false);
        }

        int getMaxSSDelay() {
            double max_SS = 0.0;
            int max_SS_buffer = 0;
            for (int i = 0; i < cellLibs.size(); i++) {
                auto& buffer = cellLibs[i];
                if (max_SS < buffer.getDelaySS(1)) {
                    max_SS = buffer.getDelaySS(1);
                    max_SS_buffer = i;
                }
            }
            return max_SS_buffer;
        }

        int getMaxFFDelay() {
            double max_FF = 0.0;
            int max_FF_buffer = 0;
            for (int i = 0; i < cellLibs.size(); i++) {
                auto& buffer = cellLibs[i];
                if (max_FF < buffer.getDelayFF(1)) {
                    max_FF = buffer.getDelayFF(1);
                    max_FF_buffer = i;
                }
            }
            return max_FF_buffer;
        }

        void UpdatePathDelta(DPState& state, const Operation& op, bool setupRepair) {
            int ffId = nodes[op.nodeId].ffId;
            auto& paths = db.FFtoPaths[ffId];
            double ffDelay = op.ffDelayDelta;
            double ssDelay = op.ssDelayDelta;
            double count = op.insertCount;

            for (auto& p : paths) {
                int pathId = p.pathId;
                if (setupRepair) {
                    if(p.pathRole == PathRole::CAPTURE) {
                        state.pathSlackDeltaSS[pathId]
                            += ssDelay * count;

                        state.pathSlackDeltaFF[pathId]
                            -= ffDelay * count;
                    }
                }
                else {
                    if(p.pathRole == PathRole::LAUNCH) {
                        state.pathSlackDeltaSS[pathId]
                            -= ssDelay * count;

                        state.pathSlackDeltaFF[pathId]
                            += ffDelay * count;
                    }
                }
            }
        }

        // Create Violation Paths List
        vector<PathViolation> UpdateViolationList(DPState& state){
            vector<PathViolation> violations;

            double tns_ss = 0.0;
            double wns_ss = 0.0;
            double tns_ff = 0.0;
            double wns_ff = 0.0;

            for (const auto& path : db.paths){
                PathViolation v;

                v.pathId = path.id;
                
                double ss = path.setupSlack + state.pathSlackDeltaSS[path.id];
                double ff = path.holdSlack + state.pathSlackDeltaFF[path.id];

                v.setupSlack = ss;
                v.holdSlack = ff;
                
                if (ss < 0) {
                    v.setupWeight = ss * ss;
                    tns_ss += v.setupSlack;
                    if (wns_ss > v.setupSlack) {
                        wns_ss = v.setupSlack;
                    }
                }
                else
                    v.setupWeight = 0.0;

                if (ff < 0){
                    v.holdWeight = ff * ff;
                    tns_ff += v.holdSlack;
                    if (wns_ff > v.holdSlack) {
                        wns_ff = v.holdSlack;
                    }
                }
                else
                    v.holdWeight = 0.0;

                v.totalWeight = v.setupWeight + v.holdWeight;

                if (v.totalWeight > 0.0)
                    violations.push_back(v);
            }
            cout << "violation paths = " << violations.size() << endl;
            cout << "wns_ss = " << wns_ss << ", tns_ss = " << tns_ss << endl;
            cout << "wns_ff = " << wns_ff << ", tns_ff = " << tns_ff << endl;

            // sort
            sort(violations.begin(), violations.end(), []
            (const auto& a, const auto& b){
                return a.totalWeight > b.totalWeight;
            });

            int pathSize = db.paths.size();
            state.pathSlackDeltaFF.assign(pathSize, 0.0);
            state.pathSlackDeltaSS.assign(pathSize, 0.0);

            return violations;
        }

        //------------------------------------------
        //            Beam-Search（目前還只是Greedy Search，確保有優化，還沒寫好產生多組候選解的）
        //------------------------------------------

        // Calculate weight of the buffers in the violation paths
        void buildBufferCandidates ( //對violation paths經過的buffer計算權重，決定在哪些buffer上resize或insert buffer能夠盡量優化大范圍的violation paths
            const DPState& state,
            const vector<PathViolation>& violations)
        {
            bufferCandidates.clear();
            int topK = 0.4 * violations.size();
            for (int i = 0; i < topK; i++){
                const auto& violation = violations[i];
                const auto& path = db.paths[violation.pathId];

                if (violation.setupSlack < 0){
                    for (int nodeId : path.captureBuffers){
                        if (state.nodeOperationState[nodeId].modified)
                            continue;
                        updateBufferCandidate(nodeId, violation, RepairSide::CAPTURE);
                    }
                }
                if (violation.holdSlack < 0){
                    for (int nodeId : path.launchBuffers){
                        if (state.nodeOperationState[nodeId].modified)
                            continue;
                        updateBufferCandidate(nodeId, violation, RepairSide::LAUNCH);
                    }
                }
            }

            for (auto& kv : bufferCandidates){
                auto& candidate = kv.second;
                candidate.totalWeight = candidate.setupWeight + candidate.holdWeight;
            }
        }

        void updateBufferCandidate(
            int nodeId,
            const PathViolation& violation, 
            RepairSide side)
        {
            auto& candidate = bufferCandidates[nodeId];
            if(candidate.nodeId == INVALID_ID)
                candidate.nodeId = nodeId;

            if (side == RepairSide::CAPTURE) {
                candidate.setupWeight += violation.setupWeight;
                candidate.setupViolationCount ++;
                //if (candidate.maxSetupSlack > violation.setupSlack)
                //    candidate.maxSetupSlack = violation.setupSlack;
            }
            else {
                candidate.holdWeight += violation.holdWeight;
                candidate.holdViolationCount ++;
                //if (candidate.maxHoldSlack > violation.holdSlack)
                //    candidate.maxHoldSlack = violation.holdSlack;
            }
        }
    
        // Sort candidate buffers
        vector<BufferCandidate*> rankBufferCandidates(const DPState& state, const vector<PathViolation>& violations){ //將buffer candidates通過權重進行排序，權重越高，經過的violation paths越嚴重/越多
            bufferCandidates.clear();
            buildBufferCandidates(state, violations);
            //cout << "bufferCandidates.size() = " << bufferCandidates.size() << endl;
            vector<BufferCandidate*> ranked;
            ranked.reserve(bufferCandidates.size());
            for(auto& kv : bufferCandidates)
                ranked.push_back(&kv.second);

            sort(ranked.begin(), ranked.end(),
            [](const BufferCandidate* a,
                const BufferCandidate* b)
            {
                if (fabs(a->totalWeight - b->totalWeight) > 1e-5)
                    return a->totalWeight > b->totalWeight;
                return a->setupWeight > b->setupWeight;
            });

            return ranked;
        }

        vector<CandidateOperation> ResizeBuffer(const BufferCandidate& candidate, const DPState& state){ //對buffer嘗試resize操作
            vector<CandidateOperation> ops;

            int nodeId = candidate.nodeId;
            if (state.nodeOperationState[nodeId].modified) {
                return ops;
            }

            int oldCellId = nodes[nodeId].cellId;
            const auto& oldCell = cellLibs[oldCellId];

            int fanout = nodes[nodeId].children.size();
            double oriSSDelay = oldCell.getDelaySS(fanout);
            double oriFFDelay = oldCell.getDelayFF(fanout);
            // choose new cell from library
            for (int cellId = 0; cellId < cellLibs.size(); cellId++){
                if (cellId == oldCellId)
                    continue;
                auto& cell = cellLibs[cellId];

                double newSSDelay = cell.getDelaySS(fanout);
                double newFFDelay = cell.getDelayFF(fanout);

                //if (newSSDelay <= oriSSDelay && newFFDelay <= oriFFDelay)
                //    continue;
                
                CandidateOperation cand;
                double areadelta = cell.area - oldCell.area;
                double SSdelta = newSSDelay - oriSSDelay;
                double FFdelta = newFFDelay - oriFFDelay;

                if (SSdelta < 0.0 && FFdelta < 0.0)
                    continue;
                cand.operation = Operation::Resize(nodeId, oldCellId, cellId, areadelta, SSdelta, FFdelta);
                ops.push_back(cand);
            }

            return ops;
        }

        vector<CandidateOperation> InsertBuffer(const BufferCandidate& candidate, const DPState& state){ //嘗試插入buffer
            vector<CandidateOperation> ops;

            int nodeId = candidate.nodeId;
            if (nodeId == INVALID_ID)
                return ops;
            if (state.nodeOperationState[nodeId].modified){
                return ops;
            }

            const auto& curNode = nodes[nodeId];
            int parentId = curNode.parent;
            if (parentId == rootId)
                return ops;
            int fanout = nodes[nodeId].children.size();

            // choose cell from library
            for (int cellId = 0; cellId < cellLibs.size(); cellId++){
                auto& cell = cellLibs[cellId];

                double SSDelta = cell.getDelaySS(1);
                double FFDelta = cell.getDelayFF(1);
                
                CandidateOperation cand;
                double areadelta = cell.area;

                int insertCount = 1;
                cand.operation = Operation::Insert(parentId, nodeId, cellId, insertCount, areadelta, SSDelta, FFDelta);
                ops.push_back(cand);
            }

            return ops;  
        }
        
        void EstimateGain(CandidateOperation& cand, const BufferCandidate& buffer){ //粗略計算這個operation的gain值
            double ssDelay = cand.operation.ssDelayDelta;
            double ffDelay = cand.operation.ffDelayDelta;
            
            cand.SetupGain = ssDelay * buffer.setupViolationCount;
            cand.HoldGain = ffDelay * buffer.holdViolationCount;

            int nodeId = buffer.nodeId;
            //output_delta << nodes[nodeId].name << ", op : ";
            //if (cand.operation.type == OperationType::RESIZE_BUFFER)
            //    output_delta << "Resize -> " << cellLibs[cand.operation.newCellId].name << "(" << ssDelay << ", " << ffDelay << ")" << endl;
            //else if (cand.operation.type == OperationType::INSERT_BUFFER)
            //    output_delta << "Insert " << cellLibs[cand.operation.newCellId].name << "(" << ssDelay << ", " << ffDelay << ")" << endl;
            double slackGain = calculateGain(nodeId, ssDelay, ffDelay);
            //output_delta << ", slackGain = " << slackGain << ", areaDelta = " << cand.operation.areaDelta << endl;
            cand.score = slackGain - 0.0001 * cand.operation.areaDelta;
        }

        double calculateGain(const int nodeId, const double& ssdelay, const double& ffdelay) { //計算這個operation對子樹的FFs身上的path slack的影響
            auto& affectedFFs = db.subtreeFFs[nodeId];

            //double oldtotalSetupSlack = 0.0;
            //double oldtotalHoldSlack = 0.0;

            double setupGain = 0.0;
            double holdGain = 0.0;

            for (auto& ff : affectedFFs) {
                auto& paths = db.FFtoPaths[ff];

                for (auto& path : paths){
                    double newSetupSlack = 0.0;
                    double newHoldSlack = 0.0;
                    double oldSetupSlack = db.paths[path.pathId].setupSlack;
                    double oldHoldSlack = db.paths[path.pathId].holdSlack;

                    if (oldSetupSlack >= 0.0 && oldHoldSlack >= 0.0)
                        continue;
                    //oldtotalSetupSlack += oldSetupSlack;
                    //oldtotalHoldSlack += oldHoldSlack;

                    //output_delta << path.pathId << ": SS = " << oldSetupSlack << ", FF = " << oldHoldSlack << endl;
                    if (path.pathRole == PathRole::CAPTURE) {
                        if (oldSetupSlack < 0.0){
                            //if (ssdelay >= 0.0 && fabs(oldSetupSlack) > 10 * ssdelay)
                            //    newtotalSetupSlack += (ssdelay);
                            newSetupSlack = oldSetupSlack + ssdelay;
                            oldSetupSlack = oldSetupSlack;
                        }
                        if (oldHoldSlack < 0.0){
                            newHoldSlack = (oldHoldSlack - ffdelay);
                            oldHoldSlack = oldHoldSlack;
                        }
                    }
                    else if (path.pathRole == PathRole::LAUNCH) {
                        if (oldSetupSlack < 0.0){
                            newSetupSlack = (oldSetupSlack - ssdelay);
                            oldSetupSlack = oldSetupSlack;
                        }
                        if (oldHoldSlack < 0.0){
                            newHoldSlack = (oldHoldSlack + ffdelay);
                            oldHoldSlack = oldHoldSlack;
                        }
                    }
               
                    setupGain += (Penalty(oldSetupSlack) - Penalty(newSetupSlack));
                    holdGain += (Penalty(oldHoldSlack) - Penalty(newHoldSlack));
                    //output_delta << "oldSetupSlack = " << oldSetupSlack << ", newSetupSlack = " << newSetupSlack << ", ";
                    //output_delta << "oldHoldSlack = " << oldHoldSlack << ", newHoldSlack = " << newHoldSlack << endl;
                }
            }
            //setupGain = (Penalty(oldH) - oldtotalSetupSlack); // fabs(oldtotalSetupSlack);
            //holdGain = (newtotalHoldSlack - oldtotalHoldSlack) ;// fabs(oldtotalHoldSlack);
            output_delta << "setupGain = " << setupGain << ", holdGain = " << holdGain;
            
            return (setupGain + holdGain);
        }
    
        double Penalty(const double& slack) { //penalty = negative slack的平方，讓嚴重的negative slack有更大影響
            if (slack >= 0.0)
                return 0.0;
            return slack * slack;
        }

        //記錄BufferCandidates進行resize/insert的可能，並依照score進行排序
        //insert的部分是對node的children嘗試insert（對node insert的話如果path的兩個FF都在這個node的subtree就不會優化到，所以改為對children insert）
        vector<CandidateOperation> GenerateCandidateOperations(const DPState& state, const vector<BufferCandidate*>& rankedCandidates){
            vector<CandidateOperation> candidates;

            int topK = min((int)rankedCandidates.size(), (int)(0.1 * nodes.size()));
            for(int i = 0; i < topK; i++)
            {
                auto* buffer =
                    rankedCandidates[i];

                int nodeId = buffer->nodeId;
                //Resize
                auto ops_resize =
                    ResizeBuffer(*buffer, state);

                for(auto& op : ops_resize)
                {
                    EstimateGain(op, *buffer);

                    if (op.score < -0.05) 
                        continue;
                    candidates.push_back(op);
                }

                //cout << "resize: " << candidates.size() << ", insert: ";
                //Insert
                auto& parentNode = nodes[nodeId];
                for (auto& childId : parentNode.children) {

                    auto it = bufferCandidates.find(childId);
                    if (it == bufferCandidates.end())
                        continue;

                    const auto& childCandidate = it->second;
                    if (childCandidate.setupWeight == 0.0 || childCandidate.holdWeight > childCandidate.setupWeight)
                        continue;

                    auto ops_insert = 
                        InsertBuffer(childCandidate, state);

                    for(auto& op : ops_insert)
                    {
                        EstimateGain(op, childCandidate);
                        if (op.score < -0.1) 
                            continue;
                        candidates.push_back(op);
                    }
                }
                
                //cout << candidates.size() << endl;
            }

            sort(candidates.begin(), candidates.end(),
            [](const CandidateOperation& a, const CandidateOperation& b){
                return a.score > b.score;
            });

            //topK = min((int)candidates.size(), 200);
            int resize_count = 0;
            int insert_count = 0;
            for (int c = 0; c < topK; c++){
                if (candidates[c].operation.type == OperationType::RESIZE_BUFFER)
                    resize_count++;
                
                if (candidates[c].operation.type == OperationType::INSERT_BUFFER)
                    insert_count++;
            }
            cout << "Candidate Operations = "<< topK << ", resize = " << resize_count << ", insert = " << insert_count << endl;
            /*for (int i = 0; i < topK; i++){
                auto& cand = candidates[i];
                output_delta << i << ": ";
                if (cand.operation.type == OperationType::RESIZE_BUFFER)
                    output_delta << nodes[cand.operation.nodeId].name << "(RESIZE): ";
                else if (cand.operation.type == OperationType::INSERT_BUFFER)
                    output_delta << nodes[cand.operation.nodeId].name << "(INSERT): ";
                output_delta << "SetupGain = " << cand.SetupGain << ", HoldGain = " << cand.HoldGain << ", areaDelta = " << cand.operation.areaDelta << ", score = " << cand.score << endl;
            }
            output_delta << endl;*/
            return candidates;
        }

        //從上面的candidateOperation中依照score大小一一放到solution中，並記錄為modified
        void ApplyBestOperations(
            DPState& state, 
            const vector<CandidateOperation>& CandidateOperations,
            const unordered_map<int, BufferCandidate>& bufferCandidates)
        {
            const int applyOperationCount = min(((int)nodes.size()/200), 40); // 避免在一次迭代中修改太多，因為只是粗算優化結果，修改太多可能會重復優化同個部分導致惡化
            // apply的數量可以再調整，對於多個nodes的優化不明顯
            vector<uint8_t> repairCount; // 記錄這個FF在這次迭代被優化了幾次，避免一直重復被優化，導致其餘部分嚴重惡化
            repairCount.resize(ffInfos.size(), 0);

            int applied = 0;
            int resized = 0;
            int inserted = 0;
            for (const auto& cand : CandidateOperations){
                int nodeId;
                if (cand.operation.type == OperationType::INSERT_BUFFER)
                    nodeId = cand.operation.insertChildId;
                else 
                    nodeId = cand.operation.nodeId;

                if (state.nodeOperationState[nodeId].modified){
                    continue;
                }
                auto it = bufferCandidates.find(nodeId);

                if (it == bufferCandidates.end())
                    continue;

                //檢查operation是否是在修已修過的node
                if (OverlapRepaired(repairCount, nodeId)) {
                    cout << "overlap ";// 似乎沒用到
                    continue;
                }

                ApplyOperations(state, cand, it->second, repairCount);
                //cout << "apply ";
                applied++;
                if (cand.operation.type == OperationType::RESIZE_BUFFER)
                    resized++;
                if (cand.operation.type == OperationType::INSERT_BUFFER){
                    inserted++;
                }
                
                if (applied > applyOperationCount)
                    break;
            }
            cout << "Resize count = " << resized << ", Insert count = " << inserted << endl;
            //cout << "\nApply done\n";
        }

        bool OverlapRepaired(const vector<uint8_t>& repairCount, const int nodeId) {
            auto& subFFs = db.subtreeFFs[nodeId];
            int repaired = 0;
            for (auto& ff : subFFs) {
                if (repairCount[ff] >= 2)
                    repaired++;
            }
            double repairedRatio = repaired / subFFs.size();
            return (repairedRatio >= 0.7);
        }

        void ApplyOperations(
            DPState& state, 
            const CandidateOperation& cand, 
            const BufferCandidate& buffer, 
            vector<uint8_t> repairCount)
        {
            const Operation& op = cand.operation;

            int nodeId;
            //if (op.type == OperationType::INSERT_BUFFER)
            //    nodeId = op.insertChildId;
            //else if (op.type == OperationType::RESIZE_BUFFER)
                nodeId = op.nodeId;

            if (state.nodeOperationState[nodeId].modified)
                return;

            state.operations.push_back(op);
            state.nodeOperationState[nodeId].modified = true;
            state.nodeOperationState[nodeId].op_type = op.type;
            state.totalAreaDelta += op.areaDelta;

            auto& affectedFFs = db.subtreeFFs[nodeId];
            for (auto& ff : affectedFFs) {
                repairCount[ff]++;

                auto& paths = db.FFtoPaths[ff];

                for (auto& path : paths){
                    int pathId = path.pathId;
                    
                    if (path.pathRole == PathRole::CAPTURE) {
                        state.pathSlackDeltaSS[pathId] += op.ssDelayDelta;
                        state.pathSlackDeltaFF[pathId] -= op.ffDelayDelta;
                    }
                    else if (path.pathRole == PathRole::LAUNCH) {
                        state.pathSlackDeltaSS[pathId] -= op.ssDelayDelta;
                        state.pathSlackDeltaFF[pathId] += op.ffDelayDelta;
                    }
                }
            }
        }
    };
}


#endif