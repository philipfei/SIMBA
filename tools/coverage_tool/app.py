"""Coverage Path Planner - desktop app.

Run:  python app.py  [optional/path/to/map.yaml]

1. Open a ROS map (.yaml next to its .pgm).
2. 'Set start' and click where the robot starts.
3. Optional: 'Draw area', left-click the corners, right-click to close.
   Without an area the whole reachable map is covered.
4. Adjust settings, press 'Plan path', then 'Export Nav2 waypoints'.
"""
import os
import queue
import sys
import threading
import traceback
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from coverage import export, pipeline, render
from coverage.grid import load_ros_map
from coverage.settings import default_settings, load_settings, save_settings

# (section, key, label, kind, options); choice options are (value, shown text)
FIELDS = [
    ('Robot', None, None, 'header', None),
    ('geometry', 'robot_radius_m', 'Robot radius (m)', float, None),
    ('geometry', 'coverage_disk_radius_m', 'Coverage radius (m)', float, None),
    ('motion', 'linear_m_s', 'Speed (m/s)', float, None),
    ('motion', 'angular_rad_s', 'Turn rate (rad/s)', float, None),
    ('Path', None, None, 'header', None),
    ('strategy', 'mode', 'Pattern', 'choice', [('auto', 'Auto (cheapest)'), ('regions', 'Lanes / mixed regions'),
                                                ('spiral', 'Spiral only')]),
    ('strategy', 'wall_loop_first', 'Follow the outer walls first', bool, None),
    ('spiral', 'overlap_pct', 'Overlap (%)', 'slider', (0, 50)),
    ('spiral', 'obstacle_mode', 'Spiral obstacles', 'choice', [('around', 'Loop around them'), ('outer', 'Outer walls only')]),
    ('spiral', 'wall_side', 'Wall on robot\'s', 'choice', [('right', 'Right (CCW)'), ('left', 'Left (CW)')]),
    ('planning', 'max_segment_length_m', 'Max goal spacing (m)', float, None),
    ('Cost function', None, None, 'header', None),
    ('cost', 'stop_penalty_s', 'Stop per sharp turn (s)', float, None),
    ('cost', 'smooth_turn_deg', 'No stop for bends up to (deg)', float, None),
    ('cost', 'bend_s_per_rad', 'Gentle bend cost (s/rad)', float, None),
    ('cost', 'reverse_penalty_s', 'Reversal / hairpin (s)', float, None),
    ('cost', 'overlap_weight', 'Double coverage weight', float, None),
    ('cost', 'missed_weight', 'Missed area weight', float, None),
    ('cost', 'edge_band_m', 'Edge band along walls (m)', float, None),
    ('cost', 'edge_missed_weight', 'Missed edge weight', float, None),
    ('Optimiser', None, None, 'header', None),
    ('optimizer', 'search_time_s', 'Time per candidate (s)', float, None),
    ('optimizer', 'polish_time_s', 'Extra time for best (s)', float, None),
]

VIEWS = ['Plan', 'Robot path & coverage', 'Boustrophedon cells', 'Regions used by the plan']

HELP = ("Every candidate path (spirals, straight lanes, mixed regions, rings-or-lanes per region) is "
        "scored in seconds: drive time + bends (gentle bends cost a little, sharp corners stop + rotate) "
        "+ hairpin penalty + weight x floor swept twice (brush over walls is free) + weight x missed "
        "floor. Missed floor within the edge band of a wall uses the (lower) edge weight, so lines can "
        "stay straight past bumpy walls; set it equal to the missed-area weight to forbid that. "
        "The cheapest path wins. Loop/lane spacing = 2 x coverage radius x (1 - overlap).")


class App:
    def __init__(self, root, map_path=None):
        self.root = root
        root.title('Coverage Path Planner')
        root.geometry('1600x900')
        self.grid = None
        self.map_path = None
        self.start = None
        self.polygon = []
        self.polygon_closed = False
        self.result = None
        self.mode = None
        self.settings = default_settings()
        self.queue = queue.Queue()
        self.vars = {}
        self._build()
        if map_path:
            self.open_map(map_path)
        self.root.after(100, self._poll)

    # ------------------------------------------------------------------ layout
    def _build(self):
        bar = ttk.Frame(self.root, padding=(6, 6, 6, 0))
        bar.pack(side=tk.TOP, fill=tk.X)
        self.buttons = {}
        for name, cmd in [('Open map...', self.open_map), ('Set start', lambda: self.set_mode('start')),
                          ('Draw area', lambda: self.set_mode('area')), ('Clear area', self.clear_area),
                          ('Plan path', self.run_plan), ('Export Nav2 waypoints...', self.export),
                          ('Save image...', self.save_image)]:
            b = ttk.Button(bar, text=name, command=cmd)
            b.pack(side=tk.LEFT, padx=2)
            self.buttons[name] = b
        self.view = tk.StringVar(value=VIEWS[0])
        self._cbar = None
        view_box = ttk.Combobox(bar, textvariable=self.view, values=VIEWS, state='readonly', width=24)
        view_box.pack(side=tk.RIGHT, padx=(0, 6))
        view_box.bind('<<ComboboxSelected>>', lambda e: self.redraw())
        ttk.Label(bar, text='View:').pack(side=tk.RIGHT)
        self.show_cov = tk.BooleanVar(value=True)
        self.show_conn = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text='Show coverage', variable=self.show_cov, command=self.redraw).pack(side=tk.RIGHT)
        ttk.Checkbutton(bar, text='Show connectors', variable=self.show_conn, command=self.redraw).pack(side=tk.RIGHT, padx=6)

        body = ttk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True)
        side = ttk.Frame(body, padding=8, width=300)
        side.pack(side=tk.LEFT, fill=tk.Y)
        row = 0
        for section, key, label, kind, opts in FIELDS:
            if kind == 'header':
                ttk.Label(side, text=section, font=('TkDefaultFont', 10, 'bold')).grid(
                    row=row, column=0, columnspan=2, sticky='w', pady=(8 if row else 0, 2))
            elif kind is bool:
                v = tk.BooleanVar(value=self.settings[section][key])
                ttk.Checkbutton(side, text=label, variable=v).grid(row=row, column=0, columnspan=2, sticky='w')
                self.vars[(section, key)] = (v, kind)
            elif kind == 'choice':
                ttk.Label(side, text=label).grid(row=row, column=0, sticky='w')
                shown = dict(opts)
                v = tk.StringVar(value=shown[self.settings[section][key]])
                ttk.Combobox(side, textvariable=v, values=[t for _, t in opts], state='readonly',
                             width=16).grid(row=row, column=1, sticky='e')
                self.vars[(section, key)] = (v, ('choice', opts))
            elif kind == 'slider':
                ttk.Label(side, text=label).grid(row=row, column=0, sticky='w')
                box = ttk.Frame(side)
                box.grid(row=row, column=1, sticky='e')
                v = tk.IntVar(value=int(round(self.settings[section][key])))
                ttk.Scale(box, from_=opts[0], to=opts[1], variable=v, length=110,
                          command=lambda _=None, v=v: (v.set(int(round(float(v.get())))), self.update_spacing())
                          ).pack(side=tk.LEFT)
                ttk.Label(box, textvariable=v, width=3, anchor='e').pack(side=tk.LEFT)
                self.vars[(section, key)] = (v, 'slider')
                row += 1
                self.spacing_text = tk.StringVar()
                ttk.Label(side, textvariable=self.spacing_text, foreground='#1b5fb4').grid(
                    row=row, column=0, columnspan=2, sticky='w')
            else:
                ttk.Label(side, text=label).grid(row=row, column=0, sticky='w')
                v = tk.StringVar(value=str(self.settings[section][key]))
                ttk.Entry(side, textvariable=v, width=10).grid(row=row, column=1, sticky='e')
                self.vars[(section, key)] = (v, kind)
                if key == 'coverage_disk_radius_m':
                    v.trace_add('write', lambda *_: self.update_spacing())
            row += 1
        self.update_spacing()
        sb = ttk.Frame(side)
        sb.grid(row=row, column=0, columnspan=2, pady=6, sticky='w')
        ttk.Button(sb, text='Save settings...', command=self.save_settings).pack(side=tk.LEFT)
        ttk.Button(sb, text='Load settings...', command=self.load_settings).pack(side=tk.LEFT, padx=4)
        row += 1
        ttk.Label(side, text=HELP, wraplength=280, foreground='#555').grid(row=row, column=0, columnspan=2, sticky='w')
        row += 1
        results = ttk.Frame(body, padding=8)
        results.pack(side=tk.RIGHT, fill=tk.Y)
        ttk.Label(results, text='Result', font=('TkDefaultFont', 10, 'bold')).pack(anchor='w', pady=(0, 2))
        self.stats = tk.Text(results, width=40, font=('TkFixedFont', 9), relief='flat', background='#f4f4f4')
        self.stats.pack(fill=tk.Y, expand=True)
        self.stats.insert('1.0', 'No plan yet.')
        self.stats.configure(state='disabled')

        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.toolbar = NavigationToolbar2Tk(self.canvas, right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect('button_press_event', self.on_click)
        self.status = tk.StringVar(value='Open a ROS map (.yaml) to begin.')
        ttk.Label(self.root, textvariable=self.status, anchor='w', padding=(8, 3), relief='sunken').pack(side=tk.BOTTOM, fill=tk.X)
        self.redraw()

    # ---------------------------------------------------------------- settings
    def read_settings(self):
        s = default_settings()
        for (section, key), (v, kind) in self.vars.items():
            raw = v.get()
            if isinstance(kind, tuple):  # choice: shown text -> value
                s[section][key] = {t: val for val, t in kind[1]}[raw]
                continue
            if kind == 'slider':
                s[section][key] = float(raw)
                continue
            try:
                s[section][key] = kind(raw) if kind in (float, int) else raw
            except ValueError:
                raise ValueError(f'"{raw}" is not a valid number for {section}.{key}')
        g, p = s['geometry'], s['planning']
        if min(g['robot_radius_m'], g['coverage_disk_radius_m'], p['max_segment_length_m']) <= 0:
            raise ValueError('Radii and goal spacing must be positive.')
        return s

    def update_spacing(self):
        if not hasattr(self, 'spacing_text'):
            return
        try:
            r = float(self.vars[('geometry', 'coverage_disk_radius_m')][0].get())
            o = float(self.vars[('spiral', 'overlap_pct')][0].get())
            self.spacing_text.set(f'Loop spacing (auto): {2 * r * (1 - o / 100):.3f} m')
        except (ValueError, KeyError):
            self.spacing_text.set('Loop spacing (auto): -')

    def apply_settings(self, s):
        for (section, key), (v, kind) in self.vars.items():
            val = s[section][key]
            if isinstance(kind, tuple):
                v.set(dict(kind[1]).get(val, kind[1][0][1]))
            elif kind == 'slider':
                v.set(int(round(float(val))))
            else:
                v.set(val if kind is bool else str(val))
        self.update_spacing()

    def save_settings(self):
        try:
            s = self.read_settings()
        except ValueError as e:
            return messagebox.showerror('Settings', str(e))
        path = filedialog.asksaveasfilename(defaultextension='.json', filetypes=[('JSON', '*.json')])
        if path:
            save_settings(s, path)
            self.status.set(f'Settings saved to {path}')

    def load_settings(self):
        path = filedialog.askopenfilename(filetypes=[('JSON', '*.json')])
        if path:
            self.apply_settings(load_settings(path))
            self.status.set(f'Settings loaded from {path}')

    # --------------------------------------------------------------- map I/O
    def open_map(self, path=None):
        path = path or filedialog.askopenfilename(title='Open ROS map', filetypes=[('ROS map', '*.yaml *.yml'), ('All', '*.*')])
        if not path:
            return
        try:
            self.grid = load_ros_map(path)
        except Exception as e:
            return messagebox.showerror('Could not open map', f'{e}')
        self.map_path = path
        self.start, self.polygon, self.polygon_closed, self.result = None, [], False, None
        h, w = self.grid.shape
        self.status.set(f'{os.path.basename(path)}: {w}x{h} cells at {self.grid.resolution} m. Click "Set start".')
        self.set_mode('start')
        self.redraw(reset_view=True)

    def set_mode(self, mode):
        if self.grid is None:
            return
        self.mode = mode
        if mode == 'area':
            self.polygon, self.polygon_closed = [], False
            self.status.set('Draw area: left-click corners, right-click to close the polygon.')
            self.redraw()
        elif mode == 'start':
            self.status.set('Click on the map where the robot starts.')

    def clear_area(self):
        self.polygon, self.polygon_closed = [], False
        self.status.set('Area cleared: the whole reachable map will be covered.')
        self.redraw()

    def on_click(self, ev):
        if self.grid is None or ev.inaxes != self.ax or self.toolbar.mode or ev.xdata is None:
            return
        x, y = float(ev.xdata), float(ev.ydata)
        if self.mode == 'start' and ev.button == 1:
            rc = self.grid.cell((x, y))
            if not (self.grid.valid(rc) and self.grid.free[rc]):
                self.status.set('That spot is not free space - click inside the mapped area.')
                return
            self.start = [x, y]
            self.mode = None
            self.status.set(f'Start set at ({x:.2f}, {y:.2f}). Optionally "Draw area", then "Plan path".')
        elif self.mode == 'area':
            if ev.button == 1:
                self.polygon.append([x, y])
            elif ev.button == 3:
                if len(self.polygon) >= 3:
                    self.polygon_closed = True
                    self.mode = None
                    self.status.set(f'Area set ({len(self.polygon)} corners). Press "Plan path".')
                else:
                    self.status.set('An area needs at least 3 corners.')
        self.redraw()

    # ---------------------------------------------------------------- drawing
    def redraw(self, reset_view=False):
        xl, yl = self.ax.get_xlim(), self.ax.get_ylim()
        if self._cbar is not None:
            self._cbar.remove()
            self._cbar = None
        self.ax.clear()
        if self.grid is None:
            self.ax.text(0.5, 0.5, 'Open a ROS map (.yaml)', ha='center', va='center', transform=self.ax.transAxes, color='#777')
            self.ax.set_axis_off()
        else:
            render.draw_map(self.ax, self.grid)
            view = self.view.get()
            if self.result is not None and view == 'Robot path & coverage':
                self._cbar = render.draw_robot_path(self.ax, self.grid, self.result)
            elif self.result is not None and view == 'Boustrophedon cells':
                render.draw_cells(self.ax, self.grid, self.result['cells'])
            elif self.result is not None and view == 'Regions used by the plan':
                if self.result.get('plan_cells') is not None:
                    render.draw_cells(self.ax, self.grid, self.result['plan_cells'])
                else:
                    self.ax.set_title('The chosen plan did not split the room into regions', fontsize=9)
            elif self.result is not None:
                render.draw_result(self.ax, self.grid, self.result, self.show_cov.get(), self.show_conn.get())
            elif self.start:
                self.ax.plot(*self.start, 'o', color=render.COLORS['start'], ms=8, mec='white', zorder=6)
            render.draw_polygon(self.ax, self.polygon, self.polygon_closed)
            if not reset_view:
                self.ax.set_xlim(xl)
                self.ax.set_ylim(yl)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def show_stats(self, st, candidates=()):
        lines = [
            f"Chosen: {st['pattern']}",
            f"Cost            {st['cost_total_s']:.0f} s",
            f"  drive         {st['cost_drive_s']:.0f} s",
            f"  turning       {st['cost_turn_s']:.0f} s ({st['sharp_turns']} stops, {st['cost_bend_s']:.0f} s bends)",
            f"  hairpins      {st['cost_reverse_s']:.0f} s ({st['hairpins']})",
            f"  double cover  {st['cost_overlap_s']:.0f} s ({st['overlap_m2']:.1f} m2)",
            f"  missed        {st['cost_missed_s']:.0f} s ({st['missed_m2']:.2f} m2, {st['edge_missed_m2']:.2f} at edges)",
            f"Est. time       {st['estimated_time_s'] / 60:.1f} min",
            f"Coverage        {st['coverage_of_reachable_pct']:.1f} % of reachable"
            + (f"  ({st['uncovered_interior_cells']} open-floor cells left!)" if st['uncovered_interior_cells'] else ''),
            f"                {st['coverage_pct']:.1f} % of area",
            f"Path length     {st['total_length_m']:.1f} m ({st['connector_length_m']:.1f} transit)",
            f"Spacing         {st['path_spacing_m']:.3f} m",
            f"Nav2 goals      {st['goals']}",
            f"Planned in      {st['planning_time_s']:.1f} s",
            "",
            "Candidates (cost, s):",
        ] + [f"  {c['total_s']:>6.0f}  {c['name']}" for c in candidates]
        self.stats.configure(state='normal')
        self.stats.delete('1.0', tk.END)
        self.stats.insert('1.0', '\n'.join(lines))
        self.stats.configure(state='disabled')

    # --------------------------------------------------------------- planning
    def run_plan(self):
        if self.grid is None:
            return messagebox.showinfo('Plan', 'Open a map first.')
        if self.start is None:
            self.set_mode('start')
            return messagebox.showinfo('Plan', 'Click "Set start" and pick the robot start position first.')
        if self.polygon and not self.polygon_closed:
            return messagebox.showinfo('Plan', 'Right-click to close the area polygon first (or "Clear area").')
        try:
            self.settings = self.read_settings()
        except ValueError as e:
            return messagebox.showerror('Settings', str(e))
        self.buttons['Plan path'].state(['disabled'])
        self.mode = None
        polygon = [list(p) for p in self.polygon] if self.polygon_closed else None
        args = (self.grid, list(self.start), polygon, self.settings)

        def work():
            try:
                res = pipeline.plan(*args, progress=lambda m: self.queue.put(('status', m)))
                self.queue.put(('done', res))
            except Exception as e:
                traceback.print_exc()
                self.queue.put(('error', e))
        threading.Thread(target=work, daemon=True).start()
        self.status.set('Planning...')

    def _poll(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == 'status':
                    self.status.set(payload)
                elif kind == 'done':
                    self.result = payload
                    self.buttons['Plan path'].state(['!disabled'])
                    self.show_stats(payload['stats'], payload.get('candidates', []))
                    st = payload['stats']
                    self.status.set(f"Plan ready ({st['pattern']}): {st['coverage_of_reachable_pct']:.1f}% of reachable area, {st['goals']} goals, "
                                    f"~{st['estimated_time_s'] / 60:.1f} min. Export when happy.")
                    self.redraw()
                elif kind == 'error':
                    self.buttons['Plan path'].state(['!disabled'])
                    self.status.set('Planning failed.')
                    msg = str(payload)
                    messagebox.showerror('Planning failed', msg)
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    # ----------------------------------------------------------------- export
    def export(self):
        if self.result is None:
            return messagebox.showinfo('Export', 'Plan a path first.')
        base = os.path.splitext(os.path.basename(self.map_path))[0] + '_coverage.yaml'
        path = filedialog.asksaveasfilename(initialfile=base, defaultextension='.yaml',
                                            filetypes=[('YAML', '*.yaml'), ('JSON', '*.json')])
        if path:
            n = export.save(self.result, path, self.map_path, self.settings)
            self.status.set(f'Exported {n} Nav2 poses to {path}')

    def save_image(self):
        if self.grid is None:
            return
        path = filedialog.asksaveasfilename(defaultextension='.png', filetypes=[('PNG', '*.png')])
        if path:
            self.fig.savefig(path, dpi=150, bbox_inches='tight')
            self.status.set(f'Image saved to {path}')


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use('vista' if sys.platform == 'win32' else 'clam')
    except tk.TclError:
        pass
    App(root, sys.argv[1] if len(sys.argv) > 1 else None)
    root.mainloop()


if __name__ == '__main__':
    main()
