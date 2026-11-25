#!/usr/bin/env python3
"""Nav2 单点导航实测：依次发送若干 NavigateToPose 目标并量化。

记录内容：
  - 成功与否（action result）
  - 耗时（仿真时间 + 墙上时间）
  - 终点位姿误差（AMCL 认为的位姿 / Gazebo 真值，均对齐到地图初始帧）
  - 路径长度（规划路径 / 真值实际行驶距离）
  - 恢复行为次数（action feedback / result）
  - 每个目标的真值与 AMCL 轨迹（供绘图）

用法:
    ros2 run diffbot_nav send_goals --ros-args -p out_dir:=/path/to/work
"""

import csv
import json
import math
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path

import tf2_ros


GOALS = [
    # (name, x, y, yaw, 说明)
    ("G1_near_straight", 1.3, 0.4, 0.0, "近距直线：起点到房间东北侧空地"),
    ("G2_cross_room", -2.6, 2.4, 1.5708, "跨房间：绕过 box_a，到西北角"),
    ("G3_narrow_corridor", -1.4, -2.55, -1.5708,
     "绕障 + 窄道：绕 box_b，进入 box_b 与南墙之间 0.82 m 走廊"),
]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def path_length(path):
    total = 0.0
    for i in range(1, len(path.poses)):
        p0 = path.poses[i - 1].pose.position
        p1 = path.poses[i].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


class NavTester(Node):
    def __init__(self):
        super().__init__("nav_tester")
        self.declare_parameter("out_dir", "/tmp/nav_test")
        self.declare_parameter("goal_timeout", 180.0)  # 墙上秒
        self.out_dir = self.get_parameter("out_dir").value
        self.goal_timeout = float(self.get_parameter("goal_timeout").value)
        os.makedirs(self.out_dir, exist_ok=True)

        self.truth = None          # (x, y, yaw) world
        self.amcl = None           # (x, y, yaw) map
        self.last_plan = None
        self.plan_len = 0.0
        self.feedback_recoveries = 0
        self.travel = 0.0
        self._prev_truth = None
        self.last_feedback_time = None

        self.create_subscription(ModelStates, "/model_states", self.on_truth, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")

    # ---------- 回调 ----------
    def on_truth(self, msg):
        try:
            i = msg.name.index("diff_chassis")
        except ValueError:
            return
        p = msg.pose[i]
        self.truth = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_plan(self, msg):
        self.last_plan = msg
        # 取本目标期间最长的一次规划路径（首帧通常是完整全局路径，
        # 之后的重规划可能只覆盖剩余路段）。
        self.plan_len = max(self.plan_len, path_length(msg))

    # ---------- 工具 ----------
    def spin_for(self, seconds, stop_cond=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if stop_cond is not None and stop_cond():
                return True
        return False

    def lookup_amcl(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, yaw_from_quat(q))

    def wait_ready(self, timeout=120.0):
        """等 /model_states、map->base_footprint TF 与 action server 就绪。"""
        def ready():
            if self.truth is None:
                return False
            a = self.lookup_amcl()
            if a is None:
                return False
            self.amcl = a
            return self.ac.server_is_ready()

        if not self.spin_for(timeout, ready):
            raise RuntimeError("Nav2 未在 %.0f s 内就绪" % timeout)
        self.get_logger().info("Nav2 就绪：truth=%s amcl=%s" % (self.truth, self.amcl))

    def world_to_map(self, pose):
        x, y, yaw = pose
        xo, yo, yo_ = self.map_offset
        c, s = math.cos(-yo_), math.sin(-yo_)
        # p_map = R(-yaw_off) * (p_world - t)
        dx, dy = x - xo, y - yo
        return (c * dx - s * dy, s * dx + c * dy, yaw - yo_)

    # ---------- 主流程 ----------
    def run(self):
        self.wait_ready()

        # 起点对齐：用建图/AMCL 初始帧把 world 真值映射到 map 坐标系，
        # 这样终点误差比较的是"同一坐标系下的物理到达"，而不是两套坐标的原始差值。
        xw, yw, yaww = self.truth
        xm, ym, yawm = self.lookup_amcl()
        yaw_off = yaww - yawm
        c, s = math.cos(yaw_off), math.sin(yaw_off)
        tx = xw - (c * xm - s * ym)
        ty = yw - (s * xm + c * ym)
        self.map_offset = (tx, ty, yaw_off)
        self.get_logger().info(
            "world->map 对齐: yaw=%.4f rad, t=(%.4f, %.4f)" %
            (yaw_off, tx, ty))

        results = []
        traj_rows = []
        for idx, (name, gx, gy, gyaw, desc) in enumerate(GOALS, start=1):
            self.get_logger().info("=== 目标 %s: (%.2f, %.2f, %.2f) %s ===" %
                                   (name, gx, gy, gyaw, desc))
            results.append(self.run_goal(idx, name, gx, gy, gyaw, desc, traj_rows))

        json_path = os.path.join(self.out_dir, "goals_metrics.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump({"map": "room_map (map_r1_final)",
                       "world_alignment": {"yaw_rad": self.map_offset[2],
                                           "tx": self.map_offset[0],
                                           "ty": self.map_offset[1]},
                       "goals": results}, f, ensure_ascii=False, indent=2)
        csv_path = os.path.join(self.out_dir, "goals_metrics.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["goal", "desc", "success", "sim_duration_s",
                        "wall_duration_s", "amcl_err_xy_m", "amcl_err_yaw_deg",
                        "truth_err_xy_m", "truth_err_yaw_deg",
                        "truth_abs_err_xy_m", "truth_abs_err_yaw_deg",
                        "planned_len_m", "actual_len_m", "recoveries"])
            for r in results:
                w.writerow([r["name"], r["desc"], r["success"],
                            "%.2f" % r["sim_duration_s"],
                            "%.2f" % r["wall_duration_s"],
                            "%.3f" % r["amcl_err_xy_m"],
                            "%.1f" % r["amcl_err_yaw_deg"],
                            "%.3f" % r["truth_err_xy_m"],
                            "%.1f" % r["truth_err_yaw_deg"],
                            "%.3f" % r["truth_abs_err_xy_m"],
                            "%.1f" % r["truth_abs_err_yaw_deg"],
                            "%.2f" % r["planned_len_m"],
                            "%.2f" % r["actual_len_m"],
                            r["recoveries"]])
        traj_path = os.path.join(self.out_dir, "trajectories.csv")
        with open(traj_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["goal", "t_sim", "truth_x", "truth_y", "truth_yaw",
                        "map_x", "map_y", "map_yaw"])
            w.writerows(traj_rows)
        self.get_logger().info("结果已写入 %s / %s / %s" %
                               (json_path, csv_path, traj_path))

        for r in results:
            self.get_logger().info(
                "[%s] success=%s sim=%.1fs truth_err=%.3fm/%.1fdeg "
                "planner=%.2fm actual=%.2fm rec=%d" %
                (r["name"], r["success"], r["sim_duration_s"],
                 r["truth_err_xy_m"], r["truth_err_yaw_deg"],
                 r["planned_len_m"], r["actual_len_m"], r["recoveries"]))
        return all(r["success"] for r in results)

    def run_goal(self, idx, name, gx, gy, gyaw, desc, traj_rows):
        self.feedback_recoveries = 0
        self.last_feedback_time = None
        self.last_plan = None
        self.plan_len = 0.0

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)

        start_sim = self.get_clock().now()
        start_wall = time.monotonic()
        self.travel = 0.0
        self._prev_truth = self.truth

        send_future = self.ac.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = send_future.result()
        if not handle.accepted:
            return self._result(name, desc, False, start_sim, start_wall,
                                gx, gy, gyaw, 0)

        result_future = handle.get_result_async()
        last_sample = time.monotonic()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now - last_sample >= 0.1:
                last_sample = now
                self.sample(idx, traj_rows)
            if time.monotonic() - start_wall > self.goal_timeout:
                self.get_logger().error("%s 超时，取消目标" % name)
                handle.cancel_goal_async()
                self.spin_for(3.0)
                break

        success = False
        sim_duration = (self.get_clock().now() - start_sim).nanoseconds / 1e9
        if result_future.done():
            wrapped = result_future.result()
            success = (wrapped.status == 4)  # STATUS_SUCCEEDED
            # Humble 的 NavigateToPose.Result 只有 std_msgs/Empty，
            # navigation_time / number_of_recoveries 都在 feedback 里。
            if self.last_feedback_time is not None:
                sim_duration = self.last_feedback_time

        self.sample(idx, traj_rows)
        return self._result(name, desc, success, start_sim, start_wall,
                            gx, gy, gyaw, self.feedback_recoveries,
                            sim_duration)

    def _result(self, name, desc, success, start_sim, start_wall, gx, gy, gyaw,
                recoveries, sim_duration=None):
        end_truth = self.truth
        amcl = self.lookup_amcl() or self.amcl
        if sim_duration is None:
            sim_duration = (self.get_clock().now() - start_sim).nanoseconds / 1e9
        wall_duration = time.monotonic() - start_wall

        amcl_err = math.hypot(amcl[0] - gx, amcl[1] - gy)
        amcl_yaw = abs(angle_diff(amcl[2], gyaw))

        truth_map = self.world_to_map(end_truth)
        truth_err = math.hypot(truth_map[0] - gx, truth_map[1] - gy)
        truth_yaw = abs(angle_diff(truth_map[2], gyaw))
        # 绝对误差：地图帧与 Gazebo 世界帧本来就对齐（建图末态误差 0.023 m/0.28°），
        # 因此可直接用世界坐标与目标比较，结果包含 AMCL 的绝对定位偏差。
        truth_abs_err = math.hypot(end_truth[0] - gx, end_truth[1] - gy)
        truth_abs_yaw = abs(angle_diff(end_truth[2], gyaw))

        return {
            "name": name,
            "desc": desc,
            "success": bool(success),
            "sim_duration_s": round(sim_duration, 2),
            "wall_duration_s": round(wall_duration, 2),
            "amcl_err_xy_m": round(amcl_err, 3),
            "amcl_err_yaw_deg": round(math.degrees(amcl_yaw), 1),
            "truth_err_xy_m": round(truth_err, 3),
            "truth_err_yaw_deg": round(math.degrees(truth_yaw), 1),
            "truth_abs_err_xy_m": round(truth_abs_err, 3),
            "truth_abs_err_yaw_deg": round(math.degrees(truth_abs_yaw), 1),
            "planned_len_m": round(self.plan_len, 2),
            "actual_len_m": round(self.travel, 2),
            "recoveries": int(recoveries),
        }

    def on_feedback(self, msg):
        fb = msg.feedback
        self.feedback_recoveries = max(self.feedback_recoveries,
                                       int(fb.number_of_recoveries))
        self.last_feedback_time = (fb.navigation_time.sec +
                                   fb.navigation_time.nanosec / 1e9)

    def sample(self, idx, traj_rows):
        if self.truth is None:
            return
        a = self.lookup_amcl()
        if a is None:
            return
        if self._prev_truth is not None:
            self.travel += math.hypot(self.truth[0] - self._prev_truth[0],
                                      self.truth[1] - self._prev_truth[1])
        self._prev_truth = self.truth
        t = self.world_to_map(self.truth)
        traj_rows.append([
            idx,
            round(self.get_clock().now().nanoseconds / 1e9, 3),
            round(t[0], 4), round(t[1], 4), round(t[2], 4),
            round(a[0], 4), round(a[1], 4), round(a[2], 4),
        ])


def main():
    rclpy.init()
    node = NavTester()
    try:
        node.run()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error("测试中断: %s" % exc)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
