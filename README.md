# ICCAD_Timing_Fixing_by_Useful_Skew
## ds.h README

### Purpose

`ds.h` 定義本專案 Step 1 所需的核心資料結構，用來建立 Clock Tree Model 與 Timing Path Model。

此檔案不負責 parsing，也不負責最佳化演算法，只提供後續 parser、SPFA、Bottom-Up DP 與 scoring 使用的共用資料格式。

---

### Main Data Structures

#### CellLib

用來記錄 `buf.lib` 中每種 buffer cell 的資訊。

包含：

- cell name
- width / height / area
- SS delay table
- FF delay table
- max fanout

delay table 使用 `fanout - 1` 作為 index。

---

#### TreeNode

用來表示 clock tree 中的每個節點。

節點可能是：

- Root
- Buffer
- FF

包含：

- node name
- node type
- parent / children
- buffer cell type
- fanout
- clock arrival time
- Euler tour interval

Clock tree 本身是後續 Bottom-Up DP 的主要結構。

---

#### FFInfo

用來另外記錄所有 FF 的資訊。

由於 timing path 都是 FF-to-FF，所以將 FF 從 clock tree 中獨立建立 index，方便後續查詢。

包含：

- FF name
- 對應的 tree node id
- SS / FF clock arrival time
- target arrival time
- target shift
- connected timing paths

---

#### TimingPath

用來記錄 `SS_delay.rpt` 與 `FF_delay.rpt` 中實際出現的 FF-to-FF path。

不建立完整 FF pair graph。

包含：

- launch FF
- capture FF
- SS data delay
- FF data delay
- setup slack
- hold slack
- critical flag

---

#### ConstraintEdge

用於 Step 2 的 SPFA / Bellman-Ford constraint graph。

每條 edge 表示：

```text
x[to] <= x[from] + weight
```

## Evaluator.h README

### Purpose

`Evaluator.h` 定義本專案 Step 4 所需的核心評估與輸出模組，用來執行 Top-Down 回溯、完整時序評估以及最終電路結構輸出。

此檔案承接 Step 3 Bottom-Up DP 傳遞至 Root 節點的 Top-K 候選狀態 (`DPState`)，透過沙盒模擬與多情境投票機制，選出最具強健性 (Robust) 的解答，並產生符合大會格式的修改後 Clock Tree 檔案。

---

### Main Mechanisms & Functions

#### Sandbox Simulation (沙盒模擬)

用來對 Step 3 產生的候選 `DPState` 進行安全的獨立測試，避免直接汙染主資料庫。

包含：
- 複製 `DesignDB` 以建立獨立沙盒環境
- 執行 `DPState.operations` 中的 `RESIZE_BUFFER` 與 `INSERT_BUFFER` 操作
- 動態更新 Tree Node 的 Parent-Child 指標與階層 (Level)

---

#### Full Timing Evaluation (完整時序評估)

用來獲取精確的時序與面積數據，消除 Bottom-Up DP 過程中的局部估算誤差。

包含：
- 呼叫 `sandbox_db.computeClockArrival()` 重新計算時脈到達時間
- 呼叫 `sandbox_db.computeAllSlacks()` 重新計算所有 Timing Path 的 Setup/Hold Slack
- 彙整與計算整體的 $TNS_{SS}$, $WNS_{SS}$, $TNS_{FF}$, $WNS_{FF}$ 以及 Total Area

---

#### Scenario Voting (多情境投票機制)

用來應對大會未公開計分權重 ($\alpha, \beta, \gamma$) 的盲測挑戰，確保選出的解答不會因為過度擬合單一指標而遭到重罰。

包含：
- 定義多種權重分配劇本 (如：Timing 優先、Area 節省優先、SS/FF 優先、絕對平均等)
- 讓 Top-K 候選人在每一套劇本的公式中進行計分對決
- 統計各候選人贏得的「劇本冠軍數 (票數)」
- 選出得票數最高、各項指標最均衡的強健解 (Robust Winner)

---

#### Tree Realization & Output (電路實體化與輸出)

用來將最終勝出的解答正式套用，並產生比賽要求繳交的檔案。

包含：
- 將勝出者的 `operations` 正式 Commit 到主程式的 `DesignDB`
- 利用 DFS (深度優先搜尋) 走訪修改後的 Clock Tree
- 匯出符合比賽規定格式的 `modified_clk_tree.structure`