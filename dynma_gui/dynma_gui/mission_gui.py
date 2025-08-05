import sys
import yaml
import subprocess
import numpy as np

try:
    import qdarkstyle
    _USE_DARK = True
except ImportError:
    _USE_DARK = False
from PyQt5.QtGui import QFont
from functools import partial
from collections import defaultdict
from dynma.msg import MissionInfo, TaskInfo, Murtap, UvPubState
from PyQt5.QtCore import Qt, QTimer, QProcess              # ★ QProcess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTableWidget, QTableWidgetItem, QMessageBox,
    QLabel, QSizePolicy, QSplitter, QPlainTextEdit          # ★ 콘솔 위젯
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from math import cos, radians
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# ---- ROS 2 ------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from dynma.msg import MissionInfo, TaskInfo, Murtap, UvPubState 
from px4_uv.msg import Action  # correct Action type

# =============================================================================
# ROS BACKEND
# =============================================================================
class RosBackend(Node):
    def __init__(self):
        super().__init__("mission_gui")

        # --- Publisher / Subscribers ---------------------------------
        self._pub = self.create_publisher(MissionInfo, "/MissionInfo", 10)

        self._sub_mission = self.create_subscription(
            MissionInfo, "/MissionInfo", self._on_mission, 10)

        self._sub_pubstate = self.create_subscription(
            UvPubState, "/PubState", self._on_pubstate, 10)

        self._sub_murtap = None 
        

        # --- 상태 저장용 ---------------------------------------------
        self.metrics       = defaultdict(lambda: {"iter": 0, "battery": 0})
        self.latest_tasks  = defaultdict(list)

    # ---------------- /MissionInfo 콜백 ------------------------------
    def _on_mission(self, msg: MissionInfo):
        """Update map with task way-points sent in MissionInfo.
        Coordinates are latitude / longitude in **degrees**."""
        if getattr(self, "gui", None) is None:
            return

        for t in msg.tasks:
            # setpoint.x = latitude(°) , setpoint.y = longitude(°)
            lon = t.actions[0].setpoint.y   # X-axis  (°)
            lat = t.actions[0].setpoint.x   # Y-axis  (°)
            self.gui.map_canvas.set_task_pos(t.task_id, lon, lat)

        self.gui.map_canvas.redraw()


    # ---------------- /PubState 콜백 ---------------------------------
    def _on_pubstate(self, msg: UvPubState):
        # 1) 메트릭 갱신
        self.metrics[msg.agent_id]["battery"] = msg.current_state.battery
        self.latest_tasks[msg.agent_id]       = list(msg.assigned_task_ids)

        # 2) 플롯 업데이트 ─ NED(m) → 위·경도(°) 변환
        if getattr(self, "gui", None):
            # 2-A. NED 원점 대비 오프셋 [m]
            east  = msg.current_state.pos.y      # +E [m]
            north = msg.current_state.pos.x      # +N [m]

            # 2-B. NED (0,0) 의 지리 좌표
            ref_lat = msg.current_state.ref_lat
            ref_lon = msg.current_state.ref_lon

            # 2-C. 간이 변환 (±몇 km 내 오차 cm 수준)
            d_lat = north / 111_320.0
            d_lon = east  / (111_320.0 * cos(radians(ref_lat)))

            lat = ref_lat + d_lat
            lon = ref_lon + d_lon

            # 0,0(초기화 미완) 메시지는 무시
            if abs(east) < 1e-3 and abs(north) < 1e-3:
                return

            # 2-D. 에이전트 위치 등록 (Lon→X, Lat→Y)
            self.gui.map_canvas.set_agent_pos(msg.agent_id, lon, lat)

            # 2-E. 할당선 갱신
            for tid in msg.assigned_task_ids:
                self.gui.map_canvas.set_assignment(tid, msg.agent_id)

            self.gui.map_canvas.redraw()



    # ---------------------------------------------------------------------
    def set_murtap_callback(self, cb):
        """
        GUI 측에서 원하는 콜백(cb)으로 /Murtap 구독을 교체한다.
        """
        if self._sub_murtap is not None:
            self.destroy_subscription(self._sub_murtap)
        self._sub_murtap = self.create_subscription(
            Murtap, "/Murtap", cb, 10)
    

    # ---------------------------------------------------------------------
    def publish_mission(self, task_rows):
        """
        Convert rows [(lat, lon, alt), …] into a MissionInfo message
        and publish it.  Coordinates are **latitude / longitude in degrees**;
        altitude is in metres AGL.
        """
        msg = MissionInfo()
        msg.mission_id   = "T1"
        msg.description  = "GUI-composed mission"

        # ── 미션 영역(BBox) -------------------------------------------------
        lats = [r[0] for r in task_rows]
        lons = [r[1] for r in task_rows]
        msg.leftup_lat,  msg.leftup_lon  = max(lats), min(lons)   # NW corner
        msg.rightdown_lat, msg.rightdown_lon = min(lats), max(lons)  # SE corner
        msg.max_height = max(r[2] for r in task_rows) + 40.0      # 여유 고도

        # ── 각 태스크(웨이포인트) -------------------------------------------
        for idx, (lat, lon, alt) in enumerate(task_rows, 1):
            t = TaskInfo()
            t.task_id               = f"WP{idx}"
            t.task_description      = f"Waypoint {idx}"
            t.target_type           = 1
            t.mission_id            = msg.mission_id
            t.required_payload_type = 0
            t.task_status           = 0
            t.task_type             = 0

            act = Action()
            act.action = 0
            #   Lat(°) → setpoint.x,  Lon(°) → setpoint.y,  Alt(m) → setpoint.z
            act.setpoint.x = float(lat)
            act.setpoint.y = float(lon)
            act.setpoint.z = float(alt)
            t.actions.append(act)

            t.reward = 1.0
            msg.tasks.append(t)

        # ── 퍼블리시 --------------------------------------------------------
        self._pub.publish(msg)
        self.get_logger().info(
            f"Mission {msg.mission_id} published with {len(msg.tasks)} tasks.")

            

# =============================================================================
# MATPLOTLIB CANVASES
# =============================================================================
class MapCanvas(FigureCanvas):
    """Agent · Task 위치 + 할당선을 보여주는 간단한 2-D 맵."""

    def __init__(self, parent=None):
        # 1) Figure 생성
        self.fig = Figure(figsize=(4, 4), tight_layout=True)

        # 2) 축을 먼저 만든다
        self.ax = self.fig.add_subplot(111)

        # 3) Canvas 초기화 (축이 이미 존재하므로 OK)
        super().__init__(self.fig)

        # 4) 축 기본 설정
        self.ax.set_aspect("equal", "box")
        self.ax.grid(True, linestyle=":", alpha=0.3)
        self.ax.set_xlabel("Lon (°)")
        self.ax.set_ylabel("Lat (°)")

        # 내부 상태
        self._agents = {}      # {agent_id: (lon, lat)}
        self._tasks  = {}      # {task_id : (lon, lat)}
        self._assign = {}      # {task_id : agent_id}


    # ---------- 데이터 갱신 API ---------- #
    def set_agent_pos(self, aid, lon, lat):
        self._agents[aid] = (lon, lat)

    def set_task_pos(self, tid, lon, lat):
        self._tasks[tid] = (lon, lat)

    def set_assignment(self, tid, aid):
        self._assign[tid] = aid

    def redraw(self):
        # --- 1) 축 초기화 ----------------------------------------
        self.ax.cla()
        self.ax.set_aspect("equal","box")
        self.ax.grid(True, linestyle=":", alpha=0.3)
        self.ax.set_xlabel("Lon (°)")
        self.ax.set_ylabel("Lat (°)")

        # --- 2) Agents 그리기 ------------------------------------
        if self._agents:
            xs, ys = zip(*self._agents.values())
            self.ax.scatter(xs, ys, c="tab:blue", s=60, label="Agents")
            for aid,(x,y) in self._agents.items():
                self.ax.text(x, y, f" {aid}", va="bottom", ha="left",
                             color="blue", fontsize=8)

        # --- 3) Tasks 그리기 -------------------------------------
        if self._tasks:
            xs, ys = zip(*self._tasks.values())
            self.ax.scatter(xs, ys, c="tab:red", marker="s",
                            s=60, label="Tasks")
            for tid,(x,y) in self._tasks.items():
                self.ax.text(x, y, f" {tid}", va="top", ha="right",
                             color="red", fontsize=8)

        # --- 4) Assignment 선 그리기 -----------------------------
        for tid, aid in self._assign.items():
            if aid in self._agents and tid in self._tasks:
                x1,y1 = self._agents[aid]
                x2,y2 = self._tasks[tid]
                self.ax.plot([x1,x2], [y1,y2],
                             linestyle="--", linewidth=1,
                             color="tab:green", alpha=0.8)

        # --- 5) 고정 축 범위 (테스트용) ---------------------------
        self.ax.set_xlim(127.2108, 127.2207)   # Lon 최소/최대
        self.ax.set_ylim(34.6084,  34.6090)   # Lat 최소/최대

        # --- 6) 1:1 비율 유지 ------------------------------------
        self.ax.set_aspect('equal', adjustable='datalim')

        # --- 7) 범례 & 그리기 ------------------------------------
        self.ax.legend(loc="upper right")
        self.draw()   # draw_idle 대신 draw()로 즉시 갱신





class StateHistoryPlot(FigureCanvas):
    """Heat-map of state vector evolution per agent."""
    def __init__(self):
        fig = Figure(figsize=(5, 2.5))
        super().__init__(fig)
        self.ax = self.figure.add_subplot(111)
        from collections import defaultdict
        self.history = defaultdict(list)   # agent_id → [state vectors]
        self._cbar = None                  # 하나만 만들기
    # ------------------------------
    def append_state(self, agent_id, vector):
        self.history[agent_id].append(vector)
        self.redraw(agent_id)

    # ------------------------------
    def redraw(self, agent_id):
        data = self.history[agent_id]
        if not data:
            self.draw(); return

        import numpy as np
        mat = np.array(data)              # shape (iters, d)

        im = self.ax.imshow(mat, aspect='auto',
                            cmap='viridis', origin='lower')

        # colorbar 한 번만 생성
        if self._cbar is None:
            self._cbar = self.figure.colorbar(im, ax=self.ax, shrink=0.8)
        else:
            self._cbar.update_normal(im)
        self.ax.set_xlabel("State index")
        self.ax.set_ylabel("Iter")
        self.ax.set_title(f"State evolution – {agent_id}")
        self.ax.grid(True, linestyle="--", alpha=.3)
        h = len(data)                         # data = history[agent_id]
        self.ax.set_ylim(-0.5, h - 0.5)       # ❷ 추가 – y축 범위 전체 노출
        self.ax.set_aspect('auto')            # ❸ 추가 – 세로 해상도 픽셀 단위

        self.figure.tight_layout()
        self.draw()

# =============================================================================
# MAIN GUI
# =============================================================================
class MissionGUI(QWidget):
    def __init__(self, ros: RosBackend):
        super().__init__()
        self.ros = ros
        self.ros.gui = self 
        self.setWindowTitle("Mission Composer & MURTAP Monitor")
        self._agents = []
        self._state_buffer = {}

        vbox = QVBoxLayout(self)
        
         # ── 빌드/런치용 프로세스 핸들 ─────────────────────────────
        self.build_proc   = None      # colcon build QProcess
        self.launch_procs = []        # [QProcess, …] 에이전트별 ros2 launch


        # ── Splitter 준비 (← 먼저 left/right 레이아웃을 만든다) ────────────────
        splitter = QSplitter(Qt.Horizontal)

        left_box  = QWidget(); left_v  = QVBoxLayout(left_box)
        right_box = QWidget(); right_v = QVBoxLayout(right_box)

        splitter.addWidget(left_box)
        splitter.addWidget(right_box)
        splitter.setStretchFactor(0, 1)
        vbox.addWidget(splitter)

        # ── **right_v** 위젯들 (순서 이동) ────────────────────────────────
        right_v.addWidget(QLabel("Current task assignment"))
        self.assign_table = QTableWidget(0, 2)
        self.assign_table.setHorizontalHeaderLabels(["Agent", "Tasks"])
        right_v.addWidget(self.assign_table)
        
        # ── 위치·할당 맵 ──────────────────────────────
        right_v.addWidget(QLabel("Agent / Task Map"))
        self.map_canvas = MapCanvas(self)
        right_v.addWidget(self.map_canvas)

        right_v.addWidget(QLabel("Live agent metrics"))
        self.metric_table = QTableWidget(0, 4)
        self.metric_table.setHorizontalHeaderLabels(
            ["Agent", "Iteration", "Battery", "WP count"])
        right_v.addWidget(self.metric_table)
        
        fit_btn = QPushButton("Fit Map")
        fit_btn.clicked.connect(self.map_canvas.redraw)
        right_v.addWidget(fit_btn)
        
        # ── State Evolution (per agent) plots ───────────────────────────
        right_v.addWidget(QLabel("State Evolution (per agent)"))
        self.state_plots = {}

        # 에이전트별로 global-state index offset을 달리 줍니다
        offsets = {
            "KARI_4": 0,   # s1,s2,s3  → global idx 1,2,3
            "KARI_5": 3,   # s4,s5,s6  → global idx 4,5,6
            "KARI_6": 6,   # s7,s8,s9  → global idx 7,8,9
        }

        for aid, off in offsets.items():
            lbl = QLabel(f"{aid} 상태 변화")
            right_v.addWidget(lbl)

            plot = AgentLinePlot(n_states=3, offset=off)
            plot.ax.set_title(aid)
            right_v.addWidget(plot)

            self.state_plots[aid] = plot


        
        # ── 터미널 로그 표시용 ──────────────────────────
        right_v.addWidget(QLabel("Build / Launch Log"))
        self.log_console = QPlainTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setMaximumBlockCount(500)     # 최근 500줄만 유지
        right_v.addWidget(self.log_console)

        # ── Task 테이블 (left_v) ─────────────────────────────
        left_v.addWidget(QLabel("Task Setpoints (double-click to edit):"))
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Lat", "Lon", "Alt"])
        left_v.addWidget(self.table)

        # ── GPS 테이블 (left_v) ──────────────────────────────
        left_v.addWidget(QLabel("Initial GPS for agents (optional):"))
        self.init_table = QTableWidget(0, 3)                # ← 먼저 만든다
        self.init_table.setHorizontalHeaderLabels(["Agent", "Lat", "Lon"])
        for aid in ["KARI_4", "KARI_5", "KARI_6", "KARI_7", "KARI_8"]:
            r = self.init_table.rowCount()
            self.init_table.insertRow(r)
            self.init_table.setItem(r, 0, QTableWidgetItem(aid))
        left_v.addWidget(self.init_table)

        # ── 버튼 4개 ─────────────────────────────────────────
        add_btn  = QPushButton("Add Row");   add_btn.clicked.connect(self.add_row)
        del_btn  = QPushButton("Delete Row");del_btn.clicked.connect(self.delete_row)
        pub_btn  = QPushButton("Publish Mission"); pub_btn.clicked.connect(self.publish_mission)
        apply_btn = QPushButton("Apply Init Pos"); apply_btn.clicked.connect(self.apply_init_pos)
        launch_btn = QPushButton("Launch Agents");   launch_btn.clicked.connect(self.launch_agents)
        # L49 직후

        # 머티리얼 QSS
        btn_qss = """
QPushButton{background:#0066CC; color:white; border:none;
            border-radius:6px; padding:6px 14px;}
QPushButton:hover  {background:#2288FF;}
QPushButton:pressed{background:#004C99;}
"""
        for b in (add_btn, del_btn, pub_btn, apply_btn, launch_btn):
            b.setStyleSheet(btn_qss)

        # 버튼 배치
        h = QHBoxLayout()
        h.addWidget(add_btn); h.addWidget(del_btn)
        h.addStretch(1);      h.addWidget(pub_btn); h.addWidget(launch_btn)
        left_v.addLayout(h)
        left_v.addWidget(apply_btn)         # Apply 버튼은 GPS 테이블 바로 아래

        # hook ROS callback
        self.ros.set_murtap_callback(self.on_murtap)

        # Spin timer
        UPDATE_MS = 1          # ← Hz 

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_assignments)
        self.timer.timeout.connect(self.refresh_metrics)
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.ros, timeout_sec=0))
        self.timer.start(UPDATE_MS)
        
        self._map_timer = QTimer(self)
        self._map_timer.timeout.connect(self.map_canvas.redraw)
        self._map_timer.start(10)  

    # ---------------------- GUI helpers --------------------------------
    def add_row(self):
        r = self.table.rowCount()
        self.table.insertRow(r)
        for c in range(3):
            self.table.setItem(r, c, QTableWidgetItem("0"))

    def delete_row(self):
        r = self.table.currentRow()
        if r >= 0:
            self.table.removeRow(r)

    def get_rows(self):
        rows = []
        for r in range(self.table.rowCount()):
            try:
                lat = float(self.table.item(r, 0).text())
                lon = float(self.table.item(r, 1).text())
                alt = float(self.table.item(r, 2).text())
                rows.append((lat, lon, alt))
            except (ValueError, AttributeError):
                pass
        return rows

    def publish_mission(self):
        rows = self.get_rows()
        if not rows:
            QMessageBox.warning(self, "No Tasks", "Please add at least one task row.")
            return
        self.ros.publish_mission(rows)
        
    def refresh_assignments(self):
        """/PubState에서 받은 할당 WP 목록을 테이블로 갱신."""
        tasks = self.ros.latest_tasks
        self.assign_table.setRowCount(len(tasks))
        for row, aid in enumerate(sorted(tasks.keys())):
            self.assign_table.setItem(row, 0, QTableWidgetItem(aid))
            txt = ", ".join(tasks[aid]) if tasks[aid] else "-"
            self.assign_table.setItem(row, 1, QTableWidgetItem(txt))
            
    def _append_log(self, text:str):
        """콘솔 창에 한 줄 추가하고 스크롤을 끝으로 이동."""
        self.log_console.appendPlainText(text.rstrip())
        sb = self.log_console.verticalScrollBar()
        sb.setValue(sb.maximum())
        
    def refresh_metrics(self):
        """
        /Murtap 와 /PubState 로부터 RosBackend 가 모아 둔
        self.ros.metrics · self.ros.latest_tasks 를 읽어
        [Agent, Iteration, Battery, WP count] 테이블을 새로 그린다.
        """
        metrics = self.ros.metrics                 # dict[agent] = {"iter": .., "battery": ..}
        tasks   = self.ros.latest_tasks            # dict[agent] = [WP1, WP2, ...]

        self.metric_table.setRowCount(len(metrics))
        for row, aid in enumerate(sorted(metrics.keys())):
            m = metrics[aid]
            wp_cnt = len(tasks.get(aid, []))
            values = [aid, m.get("iter", 0), m.get("battery", 0), wp_cnt]

            for col, val in enumerate(values):
                item = QTableWidgetItem(str(val))
                item.setFlags(Qt.ItemIsEnabled)    # 읽기 전용
                self.metric_table.setItem(row, col, item)
    # 2) ---------------------- Init-GPS 파라미터 전송 ----------------------
    def apply_init_pos(self):
        NODE_NAME = "MURTAP"
        for r in range(self.init_table.rowCount()):
            aid = self.init_table.item(r, 0).text()
            print("ROW", r, "AID", aid)                     # ← ① 루프 진입 확인

            try:
                lat_txt = self.init_table.item(r, 1).text()
                lon_txt = self.init_table.item(r, 2).text()
                print("   raw", lat_txt, lon_txt)           # ← ② 셀 값 확인
                lat = float(lat_txt)
                lon = float(lon_txt)
            except (ValueError, AttributeError) as e:
                print("   PARSE FAIL:", e)                  # ← ③ 변환 실패 여부
                continue

            print("APPLY", aid, lon, lat)                   # ← ④ 최종 좌표 출력

            for key, val in (("my_coord_lat", lat), ("my_coord_lon", lon)):
                subprocess.run(
                    ["ros2", "param", "set",
                     f"/{aid}/{NODE_NAME}", key, str(val)],
                    check=False)

            self.map_canvas.set_agent_pos(aid, lon, lat)    # lon→X, lat→Y

        self.map_canvas.redraw()                
        
    def launch_agents(self):
        # 0) 에이전트 ID 목록 추출 --------------------------------------------
        agents = [self.init_table.item(r,0).text().strip()
                  for r in range(self.init_table.rowCount())
                  if self.init_table.item(r,0)]
        if not agents:
            QMessageBox.warning(self, "No agent", "Agent list is empty.")
            return

        # 1) colcon build ------------------------------------------------------
        self._append_log(">>> colcon build 시작")
        if self.build_proc:                          # 이전 빌드가 남아있다면 kill
            self.build_proc.kill()
        self.build_proc = QProcess(self)
        self.build_proc.setProgram("/bin/bash")
        self.build_proc.setArguments(["-c",
            "cd ~/KARI-ROS2-DMA && colcon build --packages-select dynma"])
        self.build_proc.readyReadStandardOutput.connect(
            lambda: self._append_log(
                str(self.build_proc.readAllStandardOutput(), 'utf-8')))
        self.build_proc.readyReadStandardError.connect(
            lambda: self._append_log(
                str(self.build_proc.readAllStandardError(), 'utf-8')))
        # 빌드가 끝나면 에이전트 런치 -----------------------------------------
        self.build_proc.finished.connect(lambda *_: self._start_launches(agents))
        self.build_proc.start()
        
    def _start_launches(self, agents: list):
        ret = self.build_proc.exitCode()
        self._append_log(f"<<< colcon build 종료 (코드 {ret})")

        if ret != 0:
            QMessageBox.critical(
                self, "Build failed",
                "colcon build 가 실패했습니다. 로그를 확인하세요."
            )
            return

        self._append_log(">>> 각 에이전트 ros2 launch 실행")

        # ── 기존 프로세스 정리 ───────────────────────────────
        for p in self.launch_procs:
            p.kill()
        self.launch_procs.clear()

        # ── 에이전트별 ros2 launch 실행 ─────────────────────
        for aid in agents:
            try:
                idx = int(aid.split('_')[-1]) + 1         # KARI_4 → 5
            except ValueError:
                self._append_log(f"[WARN] 잘못된 에이전트 ID: {aid}")
                continue

            p = QProcess(self)
            cmd = (
                "cd ~/KARI-ROS2-DMA && "
                "source install/setup.bash && "
                f"ros2 launch dynma UAV_MURTAP_NULL.py x:={idx}"
            )
            p.setProgram("/bin/bash")
            p.setArguments(["-c", cmd])

            # 표준 출력·에러 → GUI 로그 연결
            p.readyReadStandardOutput.connect(
                lambda p=p, a=aid: self._append_log(
                    f"[{a}] " + str(p.readAllStandardOutput(), "utf-8"))
            )
            p.readyReadStandardError.connect(
                lambda p=p, a=aid: self._append_log(
                    f"[{a}][ERR] " + str(p.readAllStandardError(), "utf-8"))
            )
            p.finished.connect(
                lambda code, status, a=aid: self._append_log(
                    f"[{a}] launch 종료 (code={code})")
            )

            p.start()
            self.launch_procs.append(p)

        QMessageBox.information(
            self, "Launch",
            f"{len(self.launch_procs)} agent(s) launched."
        )

    def closeEvent(self, e):
        if self.build_proc:
            self.build_proc.kill()
        for p in self.launch_procs:
            p.kill()
        super().closeEvent(e)


    # ------------------------- /Murtap ---------------------------------
    # ------------------------- /Murtap ---------------------------------
    def on_murtap(self, msg):
        import numpy as np

        # (1) 전체 state 길이 확인 (항상 9개)
        d = len(msg.state)
        if d != 9:
            return

        aid = msg.agent_id

        # (2) 고정된 에이전트 리스트
        expected_agents = ["KARI_4", "KARI_5", "KARI_6", "KARI_7", "KARI_8"]
        # 아직 메시지 안 받은 에이전트 있으면 대기
        self._state_buffer[aid] = list(msg.state)
        if any(a not in self._state_buffer for a in expected_agents):
            return

        # (3) 한 에이전트당 state 개수
        n = d // len(expected_agents)   # = 9 // 3 = 3

        # (4) 각 에이전트별로 슬라이스해서 append
        full = np.array(self._state_buffer[aid])
        for idx, a in enumerate(expected_agents):
            # idx=0 -> 0:3, idx=1 -> 3:6, idx=2 -> 6:9
            start = idx * n
            col_vec = full[start:start+n]   # 길이=3 벡터

            # append only to its own plot
            if a == aid:
                self.state_plots[a].append_state(col_vec)



class AgentLinePlot(QWidget):
    def __init__(self, parent=None, n_states=3, offset=0):
        super().__init__(parent)
        self.n_states = n_states
        self.offset = offset        # ← global index offset
        self.history = []  # 리스트에 매 이터레이션마다의 vector를 저장

        # Matplotlib 세팅
        self.fig = Figure(figsize=(5,1.5))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Iteration")
        self.ax.set_ylabel("State")
        self.ax.set_ylim(0, 1)   # state 값 범위에 따라 조정
        self.ax.grid(True)

        layout = QVBoxLayout()
        layout.setContentsMargins(0,0,0,0)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def append_state(self, col_vec):
        import numpy as np
        self.history.append(col_vec)
        data = np.array(self.history)
        iters = np.arange(data.shape[0])

        self.ax.clear()
        # ▶ 여기서 레전드 라벨을 offset 기반으로
        for i in range(self.n_states):
            label = f"s{self.offset + i + 1}"
            self.ax.plot(iters, data[:, i], label=label)
        self.ax.set_xlabel("Iteration")
        self.ax.set_ylabel("State")
        self.ax.legend(loc="upper right", fontsize="x-small", ncol=self.n_states)
        self.ax.grid(True)
        self.canvas.draw()

# =============================================================================
# MAIN
# =============================================================================

# ---- main -------------------------------------------------------------------
# ---------- mission_gui.py 마지막 부분 ----------

def main():
    rclpy.init()
    ros_node = RosBackend()

    app = QApplication(sys.argv)
    if _USE_DARK:                           # ← 조건부 적용
        app.setStyleSheet(qdarkstyle.load_stylesheet())
    app.setFont(QFont("Noto Sans", 10))
                  # ③ 통일 폰트

    gui = MissionGUI(ros_node)
    gui.resize(1100, 650)
    gui.show()
    sys.exit(app.exec_())



if __name__ == '__main__':
    main()
