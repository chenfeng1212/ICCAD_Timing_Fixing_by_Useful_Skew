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