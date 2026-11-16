#!/usr/bin/env python3
"""
ECEN 433 - route GUI
====================

Live city map with a side panel for sending routes. Runs on the laptop (it
needs a display), in the robot's namespace:

    dts devel run -R DUCKIEBOT_NAME -L route-gui -X

Pick where the robot is (on the road From -> To) and the goals to visit in
order, then Send. That publishes route_planner_node/init and /goal. The map
shows your pending picks (green), then whatever the planner sends back on
route_planner_node/route (orange), with intersection_node/route_progress
greying out the legs already driven. If a parking stall is picked, Send also
publishes it on ~park (UInt16, latched) for the parking node. The stalls are
the ones listed in config/parking_lot.yaml.
"""

import os
import sys
import tkinter as tk
from tkinter import ttk

import rospkg
import rospy
import yaml
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from std_msgs.msg import Empty, UInt16, UInt16MultiArray

PKG = rospkg.RosPack().get_path("intersection_control")
sys.path.append(os.path.join(PKG, "src"))
from duckietown_map import CITY, TURN_NAMES  # noqa: E402

with open(os.path.join(PKG, "config", "parking_lot.yaml")) as f:
    STALLS = ["none"] + [str(i) for i in range(len(yaml.safe_load(f)["stalls"]))]

NODES = [str(n) for n in sorted(CITY.intersections)]
PENDING = "#27ae60"


class RouteGui:
    def __init__(self, root):
        rospy.init_node("route_gui_node", disable_signals=True)
        self.root = root
        self.route = None           # [from, to, ..., goal] from the planner
        self.turns = None
        self.goals = []             # goals of that route, for the rings
        self.progress = None        # legs finished, from intersection_node
        self.done = False           # last turn done
        self.sent = None            # (init, goals) last sent, so it isn't drawn as pending
        self.stall = None           # parking stall sent with the route
        self.artists = []
        self.dirty = True

        self.pub_init = rospy.Publisher("route_planner_node/init", UInt16MultiArray,
                                        queue_size=1, latch=True)
        self.pub_goal = rospy.Publisher("route_planner_node/goal", UInt16MultiArray,
                                        queue_size=1, latch=True)
        self.pub_park = rospy.Publisher("~park", UInt16, queue_size=1, latch=True)
        rospy.Subscriber("route_planner_node/route", UInt16MultiArray, self.route_cb, queue_size=1)
        rospy.Subscriber("route_planner_node/turns", UInt16MultiArray, self.turns_cb, queue_size=1)
        rospy.Subscriber("route_planner_node/goal", UInt16MultiArray, self.goal_cb, queue_size=1)
        rospy.Subscriber("intersection_node/route_progress", UInt16, self.progress_cb,
                         queue_size=10)
        rospy.Subscriber("intersection_node/route_done", Empty, self.done_cb, queue_size=1)

        root.title(f"Route planner: {rospy.get_namespace().strip('/') or 'duckiebot'}")
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        fig = Figure(figsize=(7, 7))
        self.ax = fig.add_subplot()
        CITY.draw_base(self.ax)
        fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(fig, master=root)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        panel = ttk.Frame(root, padding=12)
        panel.grid(row=0, column=1, sticky="ns")

        start = ttk.LabelFrame(panel, text="Robot is on the road", padding=8)
        start.grid(row=0, column=0, sticky="ew")
        ttk.Label(start, text="From").grid(row=0, column=0, sticky="w")
        self.from_box = ttk.Combobox(start, values=NODES, width=5, state="readonly")
        self.from_box.grid(row=0, column=1, padx=4, pady=2)
        self.from_box.bind("<<ComboboxSelected>>", self.from_changed)
        ttk.Label(start, text="To").grid(row=1, column=0, sticky="w")
        self.to_box = ttk.Combobox(start, values=[], width=5, state="readonly")
        self.to_box.grid(row=1, column=1, padx=4, pady=2)
        self.to_box.bind("<<ComboboxSelected>>", lambda e: self.changed())

        goals = ttk.LabelFrame(panel, text="Goals, in order", padding=8)
        goals.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        self.goal_box = ttk.Combobox(goals, values=NODES, width=5, state="readonly")
        self.goal_box.grid(row=0, column=0, padx=(0, 4), pady=2)
        ttk.Button(goals, text="Add", command=self.add_goal).grid(row=0, column=1, sticky="ew")
        self.goal_list = tk.Listbox(goals, height=6, width=14)
        self.goal_list.grid(row=1, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Button(goals, text="Remove", command=self.remove_goal).grid(row=2, column=0,
                                                                        sticky="ew")
        ttk.Button(goals, text="Clear", command=self.clear_goals).grid(row=2, column=1,
                                                                       sticky="ew")

        park = ttk.LabelFrame(panel, text="Then park", padding=8)
        park.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(park, text="Stall").grid(row=0, column=0, sticky="w")
        self.stall_box = ttk.Combobox(park, values=STALLS, width=5, state="readonly")
        self.stall_box.set(STALLS[0])
        self.stall_box.grid(row=0, column=1, padx=4, pady=2)

        ttk.Button(panel, text="Send", command=self.send).grid(row=3, column=0, sticky="ew",
                                                               pady=10)
        self.message = ttk.Label(panel, text="", foreground="#c0392b", wraplength=220)
        self.message.grid(row=4, column=0, sticky="ew")

        status = ttk.LabelFrame(panel, text="Planned route", padding=8)
        status.grid(row=5, column=0, sticky="ew", pady=(10, 0))
        self.status = ttk.Label(status, text="", justify="left", wraplength=220)
        self.status.grid(row=0, column=0, sticky="w")

        root.after(100, self.poll)

    # --- ROS callbacks: store and flag, the Tk thread draws -----------------
    def route_cb(self, msg):
        self.route = list(msg.data)
        self.dirty = True

    def turns_cb(self, msg):
        self.turns = [TURN_NAMES[c] for c in msg.data]
        self.dirty = True

    def goal_cb(self, msg):
        self.goals = list(msg.data)
        self.dirty = True

    def progress_cb(self, msg):
        self.progress = msg.data
        if msg.data == 0:
            self.done = False
        self.dirty = True

    def done_cb(self, msg):
        self.done = True
        self.dirty = True

    # --- panel ---------------------------------------------------------------
    def selection(self):
        init = [int(b.get()) for b in (self.from_box, self.to_box) if b.get()]
        return init, [int(g) for g in self.goal_list.get(0, tk.END)]

    def changed(self):
        self.message.config(text="")
        self.dirty = True

    def from_changed(self, event):
        options = [str(n) for n in CITY.neighbors(int(self.from_box.get()))]
        self.to_box.config(values=options)
        if self.to_box.get() not in options:
            self.to_box.set("")
        self.changed()

    def add_goal(self):
        if self.goal_box.get():
            self.goal_list.insert(tk.END, self.goal_box.get())
            self.changed()

    def remove_goal(self):
        for i in reversed(self.goal_list.curselection()):
            self.goal_list.delete(i)
        self.changed()

    def clear_goals(self):
        self.goal_list.delete(0, tk.END)
        self.changed()

    def send(self):
        init, goals = self.selection()
        if len(init) != 2:
            self.message.config(text="Pick the road the robot is on (From and To).")
            return
        if not goals:
            self.message.config(text="Add at least one goal.")
            return
        self.pub_init.publish(UInt16MultiArray(data=init))
        self.pub_goal.publish(UInt16MultiArray(data=goals))
        self.stall = None if self.stall_box.get() == "none" else int(self.stall_box.get())
        if self.stall is not None:
            self.pub_park.publish(UInt16(data=self.stall))
        self.sent = (init, goals)
        self.message.config(text="")
        rospy.loginfo("route_gui_node: sent init %s, goals %s, park in %s",
                      init, goals, "no stall" if self.stall is None else f"stall {self.stall}")
        self.dirty = True

    # --- drawing -------------------------------------------------------------
    def status_text(self):
        if not self.route:
            return "Nothing planned yet."
        lines = [" → ".join(map(str, self.route))]
        if self.turns is not None:
            lines.append("Turns: " + (", ".join(self.turns) or "none"))
        legs = [CITY.road_between(a, b)["length_m"] for a, b in zip(self.route[1:], self.route[2:])]
        if legs:
            lines.append(f"Legs: {', '.join(f'{d:.2f}' for d in legs)} m")
            lines.append(f"Total: {sum(legs):.2f} m after {self.route[1]}")
        if self.stall is not None:
            lines.append(f"Then park in stall {self.stall}")
        n = len(self.route) - 1
        p = self.progress
        if p is None:
            lines.append("\nWaiting for intersection_node.")
        elif p >= n:
            lines.append(f"\nStopped at {self.route[-1]}'s stop line.")
        elif self.done:
            lines.append("\nLast turn done: parking takes over.")
        else:
            lines.append(f"\nLeg {p + 1}/{n}: {self.route[p]} → {self.route[p + 1]}")
        return "\n".join(lines)

    def redraw(self):
        for a in self.artists:
            a.remove()
        self.artists = []
        if self.route:
            self.artists += CITY.draw_route(self.ax, self.route, self.progress, self.goals)
        init, goals = self.selection()
        if (init, goals) != self.sent:
            if len(init) == 2:
                self.artists += CITY.draw_leg(self.ax, *init, PENDING, lw=4, z=5, arrow=True)
            self.artists += CITY.draw_goals(self.ax, goals, PENDING, linestyle="--")
        self.status.config(text=self.status_text())
        self.canvas.draw_idle()

    def poll(self):
        if rospy.is_shutdown():
            self.root.destroy()
            return
        if self.dirty:
            self.dirty = False
            self.redraw()
        self.root.after(100, self.poll)

    def close(self):
        rospy.signal_shutdown("window closed")
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    RouteGui(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.signal_shutdown("ctrl-c")
