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

## SPFA.h README

由於把SS和FF分開比較好做，我在ds.h加了個buildConstraintGraph(ConstraintKind kind)
原本定義
hold:
x_capture - x_launch <= FF_data_delay - Thold = weight
edge:
launch -> capture

但考量到縮小 x_capture 比增加 x_launch 難的多，我將其改成
x_launch - x_capture >= -weight
edge:
capture -> launch

在固定 x_capture 的情況下，藉由增加 x_launch 來符合條件
(我沒有改原本的定義，只是在buildConstraintGraph中的from和to交換而已)

## clk_tree_dp.h README

`clk_tree_dp.h`定義本專案step 3的 Bottom-Up Dynamic Programming (DP) 。
這一步主要是對 Clock Tree 進行優化，透過調整 Clock Buffer 的配置來改善 Timing 表現。

目前支援三種操作：

NoOp：不做修改
Resize：更換 Buffer Cell
Insert：插入新的 Buffer

DP 會由 FF 葉節點往上遞迴計算，在每個節點保留較佳的候選解，最後於 Root 取得最佳修改方案。

### 演算法流程
```mermaid
graph TD
    A[FF Leaf] --> B[Merge Children]
    B --> C["Expand
    (NoOP / Resize / Insert)"]
    C --> D[Evaluate]
    D --> E[Prune]
    E --> F[Return States]
```

### DP State
每個候選解以 `DPState` 表示。

主要紀錄：

- `ssDelayDelta`：SS Corner 延遲變化量。
- `ffDelayDelta`：FF Corner 延遲變化量。
- `areaDelta`：面積變化量。
- `sssumTargetShift`：所有 FF 的 SS Target Shift 總和。
- `ffsumTargetShift`：所有 FF 的 FF Target Shift 總和。
- `ffCount`：此 State 包含的 FF 數量。
- `estimatedGain`：評分結果。
- `operations`：紀錄所有修改操作。

### Functions

#### Leaf Initialization
當節點為 FF 時建立初始狀態：

Delay Change = 0
Area Change = 0
將 FF 的 Target Shift 存入 State
ffCount = 1

作為 DP 的起始狀態。

#### Merge
將多個Child的DP結果合併。

- TargetShift：直接加總。
    `sumTargetShift = ChildA + ChildB`
- Delay：使用加權平均。
    `Delay = (DelayA × CountA + DelayB × CountB) / TotalCount`
- Area：直接累加。
    `Area = AreaA + AreaB`
- Operations：將兩個Child的操紀錄合併。
- *Merge之後會先做一次排序（通過EstimatedGain），避免候選解數量爆炸。*

#### Expand
Merge完成後，對每個State嘗試不同修改方式。

- 1. NoOP
    維持原本buffer不變。
    用途：保留原始解，作為其他操作的比較基準。

- 2. Resize
    嘗試將目前Buffer換成Library中其他合法Cell。
    更新：SS Dela, FF Delay, Area，並記錄Resize Operation。

- 3. Insert Buffer
    在節點與其Parent之間插入新的Buffer。
    從
    ```mermaid
    graph TD
    A{Parent} --> B{U}
    ```
    變成
    ```mermaid
    graph TD
    A{Parent} --> B(New Buffer)
    B --> C{U}
    ```
    更新：SS Dela, FF Delay, Area，並記錄Resize Operation。
- *三種操作之後也會先做一次排序（通過EstimatedGain），避免候選解數量爆炸。*

#### Evaluation
每個State會計算一個分數(`EstimatedGain`)。
目前使用：
```
Error = |TargetSS - DelaySS| + 3 ×|TargetFF - DelayFF|
```
再加上面積成本：
```
Gain = -(Error + λ × Area)
```
其中：SS 為主要優化目標，FF 次要考量，λ 為面積權重。
Gain 越大表示解越好。

#### Pruning
為避免State數量爆炸，使用三層Pruning。
- Pareto Pruning
    刪除明顯較差的State。
    若State A的Error更小，SS誤差更小，則視為支配(Dominate) State B。
    
- Bucket Pruning
    依據SS Delay分桶：
    ```
    Bucket = round (SSDelay / Precision)
    ```
    每個Bucket只保留前幾名解。
    參數：
    ```
    bucketPrecision
    bucketKeep
    ```
- Top-K Pruning
    最後依照Gain排序。
    只保留 `topK` 個最佳解。
    
#### Important Parameter（重要參數）
| 參數 | 功能 |
|:--|:--|
| lambda | 面積懲罰權重 |
| topK | 每個節點保留的最大State數量 |
| bucketPrecision | Bucket分桶精度 |
| bucketKeep | 每個Bucket保留數量 |
