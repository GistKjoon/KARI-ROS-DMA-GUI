import sys
import yaml
import subprocess
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
    """ROS2 node managing Mission publisher and /Murtap subscriber."""
    def __init__(self):
        super().__init__("mission_gui")
        self._pub = self.create_publisher(MissionInfo, "/MissionInfo", 10)
        # Placeholder subscription; real callback set via set_murtap_callback
        self._sub = self.create_subscription(Murtap, "/Murtap", lambda _: None, 1)
        self._sub_pubstate = self.create_subscription(
            UvPubState, "/PubState", self._on_pubstate, 10)
        self.latest_tasks = defaultdict(list)   # agent_id → [WP1, WP2, …]
        
        self.metrics      = defaultdict(lambda: {"iter": 0, "battery": 0})
        self.latest_tasks = defaultdict(list)
        
        self._sub_murtap = self.create_subscription(
            Murtap, "/Murtap", self._on_murtap, 10)

        # /PubState 구독 → 배터리·WP 목록
        self._sub_pubstate = self.create_subscription(
            UvPubState, "/PubState", self._on_pubstate, 10)

    def _on_murtap(self, msg: Murtap):
        self.metrics[msg.agent_id]["iter"] = msg.iteration

    def _on_pubstate(self, msg: UvPubState):
        self.metrics[msg.agent_id]["battery"] = msg.current_state.battery
        self.latest_tasks[msg.agent_id] = list(msg.assigned_task_ids)
        
    def _on_pubstate(self, msg: UvPubState):
        self.latest_tasks[msg.agent_id] = list(msg.assigned_task_ids)

    # ---------------------------------------------------------------------
    def set_murtap_callback(self, cb):
        self.destroy_subscription(self._sub)
        self._sub = self.create_subscription(Murtap, "/Murtap", cb, 10)

    # ---------------------------------------------------------------------
    def publish_mission(self, task_rows):
        """Convert rows [(lat,lon,alt), …] into MissionInfo and publish."""
        msg = MissionInfo()
        msg.mission_id = "T1"
        msg.description = "GUI‑composed mission"
        lats = [r[0] for r in task_rows]
        lons = [r[1] for r in task_rows]
        msg.leftup_lat, msg.leftup_lon = max(lats), min(lons)
        msg.rightdown_lat, msg.rightdown_lon = min(lats), max(lons)
        msg.max_height = max(r[2] for r in task_rows) + 40.0

        for idx, (lat, lon, alt) in enumerate(task_rows, 1):
            t = TaskInfo()
            t.task_id = f"WP{idx}"
            t.task_description = f"Waypoint {idx}"
            t.target_type = 1
            t.mission_id = msg.mission_id
            t.required_payload_type = 0
            t.task_status = 0
            t.task_type = 0
            act = Action()
            act.action = 0
            act.setpoint.x, act.setpoint.y, act.setpoint.z = float(lat), float(lon), float(alt)
            t.actions.append(act)
            t.reward = 1.0
            msg.tasks.append(t)
        self._pub.publish(msg)
        self.get_logger().info(
            f"Mission {msg.mission_id} published with {len(msg.tasks)} tasks.")

# =============================================================================
# MATPLOTLIB CANVASES
# =============================================================================


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
        self.ax.cla()
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

        right_v.addWidget(QLabel("Live agent metrics"))
        self.metric_table = QTableWidget(0, 4)
        self.metric_table.setHorizontalHeaderLabels(
            ["Agent", "Iteration", "Battery", "WP count"])
        right_v.addWidget(self.metric_table)
        
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
        for aid in ["KARI_4", "KARI_6"]:
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
        UPDATE_MS = 50          # ← Hz 

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_assignments)
        self.timer.timeout.connect(self.refresh_metrics)
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.ros, timeout_sec=0))
        self.timer.start(UPDATE_MS)

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
        NODE_NAME = "MURTAP"                     # ← 소문자 확인
        for r in range(self.init_table.rowCount()):
            aid = self.init_table.item(r, 0).text()
            try:
                lat = float(self.init_table.item(r, 1).text())
                lon = float(self.init_table.item(r, 2).text())
            except (ValueError, AttributeError):
                continue
            for key, val in (("my_coord_lat", lat), ("my_coord_lon", lon)):
                subprocess.run(
                    ["ros2", "param", "set",
                     f"/{aid}/{NODE_NAME}", key, str(val)],
                    check=False
                )
                
        
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
    def on_murtap(self, msg):
        d = len(msg.state)
        if d == 0:
            return
        import numpy as np
        aid = msg.agent_id
        # Track agents
        if aid not in self._agents:
            self._agents.append(aid)
            self._agents.sort()
        N = len(self._agents)
        m = d // N if N else 0
        # Store latest state
        self._state_buffer[aid] = list(msg.state)
        # Draw when we have all agents' fresh states
        if len(self._state_buffer) < N:
            return
        # Build state matrix (tasks×agents)
        mat = np.zeros((m, N))
        for col, a in enumerate(self._agents):
            vec = np.array(self._state_buffer[a]).reshape(m, N)
            mat[:, col] = vec[:, col]
            # Append to history plot
            

        if aid == "KARI_4":
            self.state4_plot.append_state(aid, vec.flatten())
        elif aid == "KARI_6":
            self.state6_plot.append_state(aid, vec.flatten())
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

