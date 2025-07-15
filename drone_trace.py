import json
import os
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.path import Path
from PIL import Image, ImageTk

class AgroDronePlannerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("АгроДрон-Маршрут MVP")
        self.root.geometry("1200x800")

        self.state = {
            "map_path": None,
            "scale": 1.0,
            "polygons": [],
            "base_point": None,
            "drone_params": {
                "tank_volume": 20.0,
                "spray_width": 6.0,
                "consumption_rate": 0.1,
                "work_speed": 5.0,
                "transport_speed": 10.0
            },
            "route": []
        }
        self.mode = 'idle'
        self.current_polygon_points = []
        self.dragging_vertex_info = None

        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.controls_frame = tk.Frame(main_frame, width=300)
        self.controls_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        self.controls_frame.pack_propagate(False)

        canvas_frame = tk.Frame(main_frame)
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas.mpl_connect('button_press_event', self._on_canvas_click)
        self.canvas.mpl_connect('motion_notify_event', self._on_canvas_drag)
        self.canvas.mpl_connect('button_release_event', self._on_canvas_release)

        self._create_control_widgets()

    def _create_control_widgets(self):
        file_frame = tk.LabelFrame(self.controls_frame, text="Файл")
        file_frame.pack(fill=tk.X, padx=5, pady=5)
        tk.Button(file_frame, text="Загрузить карту", command=self._load_map).pack(fill=tk.X)
        tk.Button(file_frame, text="Сохранить проект", command=self._save_project).pack(fill=tk.X)
        tk.Button(file_frame, text="Загрузить проект", command=self._load_project).pack(fill=tk.X)
        tk.Button(file_frame, text="Экспорт в PNG", command=self._export_to_png).pack(fill=tk.X)

        tools_frame = tk.LabelFrame(self.controls_frame, text="Инструменты")
        tools_frame.pack(fill=tk.X, padx=5, pady=5)
        tk.Button(tools_frame, text="Рисовать поле (ЛКМ)", command=lambda: self._set_mode('draw_polygon')).pack(fill=tk.X)
        tk.Button(tools_frame, text="Указать базу (ЛКМ)", command=lambda: self._set_mode('set_base')).pack(fill=tk.X)
        tk.Button(tools_frame, text="Редактировать (ЛКМ/ПКМ)", command=lambda: self._set_mode('edit')).pack(fill=tk.X)

        params_frame = tk.LabelFrame(self.controls_frame, text="Параметры дрона")
        params_frame.pack(fill=tk.X, padx=5, pady=5)
        self.drone_param_vars = {}
        for key, value in self.state["drone_params"].items():
            frame = tk.Frame(params_frame)
            frame.pack(fill=tk.X)
            tk.Label(frame, text=key, width=15, anchor='w').pack(side=tk.LEFT)
            var = tk.StringVar(value=str(value))
            entry = tk.Entry(frame, textvariable=var)
            entry.pack(side=tk.RIGHT, expand=True, fill=tk.X)
            self.drone_param_vars[key] = var

        calc_frame = tk.Frame(self.controls_frame)
        calc_frame.pack(fill=tk.X, padx=5, pady=15)
        tk.Button(calc_frame, text="Рассчитать маршрут", command=self._calculate_route, font=("Arial", 12, "bold"), bg="lightblue").pack(fill=tk.X, ipady=5)

    def _set_mode(self, mode):
        self.mode = mode
        self.current_polygon_points = []
        self.dragging_vertex_info = None
        self._redraw_canvas()

    def _on_canvas_click(self, event):
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata

        if self.mode == 'draw_polygon':
            if event.button == 1:
                self.current_polygon_points.append((x, y))
                if len(self.current_polygon_points) > 1 and self._distance(self.current_polygon_points[0], self.current_polygon_points[-1]) < 5 * self.state['scale']:
                    self.state['polygons'].append({'points': self.current_polygon_points[:-1], 'type': 'inclusion'})
                    self.current_polygon_points = []
                self._redraw_canvas()
            elif event.button == 2 or event.dblclick:
                if len(self.current_polygon_points) > 2:
                    self.state['polygons'].append({'points': self.current_polygon_points, 'type': 'inclusion'})
                self.current_polygon_points = []
                self._redraw_canvas()

        elif self.mode == 'set_base':
            if event.button == 1:
                self.state['base_point'] = (x, y)
                self._redraw_canvas()

        elif self.mode == 'edit':
            if event.button == 1:
                for i, poly in enumerate(self.state['polygons']):
                    for j, vertex in enumerate(poly['points']):
                        if self._distance((x, y), vertex) < 5 * self.state['scale']:
                            self.dragging_vertex_info = {'polygon_index': i, 'vertex_index': j}
                            return
            elif event.button == 3:
                for i, poly in enumerate(self.state['polygons']):
                    path = Path(poly['points'])
                    if path.contains_point((x, y)):
                        current_type = poly['type']
                        new_type = 'exclusion' if current_type == 'inclusion' else 'inclusion'
                        poly['type'] = new_type
                        self._redraw_canvas()
                        break

    def _on_canvas_drag(self, event):
        if self.dragging_vertex_info and event.inaxes == self.ax:
            x, y = event.xdata, event.ydata
            p_idx = self.dragging_vertex_info['polygon_index']
            v_idx = self.dragging_vertex_info['vertex_index']
            self.state['polygons'][p_idx]['points'][v_idx] = (x, y)
            self._redraw_canvas()

    def _on_canvas_release(self, event):
        if self.dragging_vertex_info:
            self.dragging_vertex_info = None

    def _redraw_canvas(self):
        self.ax.clear()
        self.ax.grid(True)

        if self.state.get("map_image"):
            self.ax.imshow(self.state["map_image"], extent=(0, self.state["map_image"].width, 0, self.state["map_image"].height))

        for poly in self.state['polygons']:
            points = poly['points']
            if len(points) > 1:
                p = np.array(points + [points[0]])
                style = {'linewidth': 2.5, 'edgecolor': 'green', 'facecolor': 'green', 'alpha': 0.2} if poly['type'] == 'inclusion' else \
                        {'linewidth': 1.5, 'edgecolor': 'red', 'facecolor': 'red', 'alpha': 0.3, 'hatch': '///'}
                self.ax.plot(p[:, 0], p[:, 1], color=style['edgecolor'], lw=style['linewidth'])
                if len(points) > 2:
                    self.ax.fill(p[:, 0], p[:, 1], color=style['facecolor'], alpha=style['alpha'], hatch=style.get('hatch'))

        if self.current_polygon_points:
            p = np.array(self.current_polygon_points)
            self.ax.plot(p[:, 0], p[:, 1], 'go--', lw=1.5)

        if self.state['base_point']:
            x, y = self.state['base_point']
            self.ax.plot(x, y, 'H', markersize=12, color='orange', markeredgecolor='black')

        for segment in self.state['route']:
            points = np.array(segment['points'])
            if len(points) > 0:
                style = {'color': 'blue', 'linestyle': '-', 'linewidth': 1.0} if segment['type'] == 'work' else \
                        {'color': 'orange', 'linestyle': '--', 'linewidth': 1.2}
                self.ax.plot(points[:, 0], points[:, 1], **style)

        self.ax.set_aspect('equal', adjustable='box')
        self.canvas.draw()

    def _load_map(self):
        filepath = filedialog.askopenfilename(
            title="Выберите файл карты",
            filetypes=[("Изображения", "*.png *.jpg *.jpeg"), ("Все файлы", "*.*")]
        )
        if not filepath:
            return

        try:
            scale = simpledialog.askfloat("Масштаб", "Введите масштаб (метров в пикселе):", initialvalue=1.0, minvalue=0.0001)
            if scale is None:
                return
            self.state['scale'] = scale
        except (ValueError, TypeError):
            messagebox.showerror("Ошибка", "Неверный формат масштаба.")
            return

        try:
            self.state['map_path'] = filepath
            self.state['map_image'] = Image.open(filepath)
            self._redraw_canvas()
        except Exception as e:
            messagebox.showerror("Ошибка загрузки", f"Не удалось загрузить изображение: {e}")

    def _save_project(self):
        filepath = filedialog.asksaveasfilename(
            defaultextension=".agriplan",
            filetypes=[("Проекты АгроДрона", "*.agriplan"), ("Все файлы", "*.*")]
        )
        if not filepath:
            return

        try:
            state_to_save = self.state.copy()
            state_to_save.pop('map_image', None)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(state_to_save, f, indent=4, ensure_ascii=False)
            messagebox.showinfo("Успех", f"Проект сохранен в {filepath}")
        except Exception as e:
            messagebox.showerror("Ошибка сохранения", f"Не удалось сохранить проект: {e}")

    def _load_project(self):
        filepath = filedialog.askopenfilename(
            filetypes=[("Проекты АгроДрона", "*.agriplan"), ("Все файлы", "*.*")]
        )
        if not filepath:
            return

        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                loaded_state = json.load(f)
            
            self.state.update(loaded_state)
            
            for key, var in self.drone_param_vars.items():
                var.set(str(self.state['drone_params'].get(key, '')))

            if self.state.get('map_path') and os.path.exists(self.state['map_path']):
                self.state['map_image'] = Image.open(self.state['map_path'])
            else:
                self.state['map_image'] = None
                if self.state.get('map_path'):
                    messagebox.showwarning("Внимание", f"Файл карты не найден по пути: {self.state['map_path']}")

            self._redraw_canvas()
            messagebox.showinfo("Успех", "Проект успешно загружен.")

        except Exception as e:
            messagebox.showerror("Ошибка загрузки", f"Не удалось загрузить проект: {e}")

    def _export_to_png(self):
        filepath = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG Изображение", "*.png"), ("Все файлы", "*.*")]
        )
        if not filepath:
            return

        try:
            self.fig.savefig(filepath, dpi=300)
            messagebox.showinfo("Успех", f"Изображение сохранено в {filepath}")
        except Exception as e:
            messagebox.showerror("Ошибка экспорта", f"Не удалось сохранить изображение: {e}")

    def _distance(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def _calculate_route(self):
        inclusion_polygons = [p for p in self.state['polygons'] if p['type'] == 'inclusion']
        if not inclusion_polygons:
            messagebox.showerror("Ошибка", "Необходимо определить хотя бы одно основное поле (зону включения).")
            return
        if not self.state['base_point']:
            messagebox.showerror("Ошибка", "Необходимо указать положение базы.")
            return
        try:
            for key, var in self.drone_param_vars.items():
                self.state['drone_params'][key] = float(var.get())
            if any(v <= 0 for v in self.state['drone_params'].values()):
                 raise ValueError("Параметры дрона должны быть положительными числами.")
        except (ValueError, TypeError) as e:
            messagebox.showerror("Ошибка параметров", f"Неверный формат параметров дрона: {e}")
            return

        try:
            raw_path = self._boustrophedon_path_algorithm()
            if not raw_path:
                messagebox.showinfo("Информация", "Не удалось построить базовый маршрут. Возможно, поле слишком маленькое.")
                return

            final_route = self._add_refueling_trips(raw_path)
            self.state['route'] = final_route

        except Exception as e:
            messagebox.showerror("Ошибка расчета", f"Произошла ошибка при расчете маршрута: {e}")
            return

        self._redraw_canvas()
        messagebox.showinfo("Успех", "Маршрут успешно рассчитан.")

    def _boustrophedon_path_algorithm(self):
        inclusion_polygons = [p['points'] for p in self.state['polygons'] if p['type'] == 'inclusion']
        exclusion_polygons = [p['points'] for p in self.state['polygons'] if p['type'] == 'exclusion']
        
        inclusion_paths = [Path(p) for p in inclusion_polygons]
        exclusion_paths = [Path(p) for p in exclusion_polygons]

        spray_width = self.state['drone_params']['spray_width']
        
        all_points = np.vstack([p for p in inclusion_polygons])
        min_x, min_y = np.min(all_points, axis=0)
        max_x, max_y = np.max(all_points, axis=0)

        scan_lines_y = np.arange(min_y + spray_width / 2, max_y, spray_width)
        path_segments_by_line = []

        for y in scan_lines_y:
            intersections = []
            all_polys_for_intersect = inclusion_polygons + exclusion_polygons
            for poly in all_polys_for_intersect:
                for i in range(len(poly)):
                    p1 = poly[i]
                    p2 = poly[(i + 1) % len(poly)]
                    if (p1[1] > y and p2[1] < y) or (p1[1] < y and p2[1] > y):
                        x_intersect = p1[0] + (y - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1])
                        intersections.append(x_intersect)
            
            intersections.sort()
            
            line_segments = []
            for i in range(0, len(intersections), 2):
                if i + 1 < len(intersections):
                    x1, x2 = intersections[i], intersections[i+1]
                    mid_point = ((x1 + x2) / 2, y)
                    
                    is_in_inclusion = any(path.contains_point(mid_point) for path in inclusion_paths)
                    is_in_exclusion = any(path.contains_point(mid_point) for path in exclusion_paths)
                    
                    if is_in_inclusion and not is_in_exclusion:
                        line_segments.append([(x1, y), (x2, y)])
            path_segments_by_line.append(line_segments)

        full_path = []
        for i, line_segments in enumerate(path_segments_by_line):
            if not line_segments:
                continue
            if i % 2 != 0:
                line_segments.reverse()
            
            for segment in line_segments:
                if i % 2 != 0:
                    segment.reverse()
                
                if full_path and segment:
                    full_path.append(segment[0])
                
                full_path.extend(segment)
        
        return full_path

    def _add_refueling_trips(self, raw_path):
        params = self.state['drone_params']
        tank_volume = params['tank_volume']
        consumption_per_sq_meter = params['consumption_rate']
        spray_width = params['spray_width']
        base_point = self.state['base_point']
        
        current_fuel = tank_volume
        final_route = []
        current_work_segment = []
        
        if not raw_path:
            return []
        
        final_route.append({'type': 'transport', 'points': [base_point, raw_path[0]]})
        current_work_segment.append(raw_path[0])

        for i in range(len(raw_path) - 1):
            p1 = raw_path[i]
            p2 = raw_path[i+1]
            
            segment_length = self._distance(p1, p2)
            fuel_needed_for_segment = segment_length * spray_width * consumption_per_sq_meter
            
            fuel_to_finish_and_return = fuel_needed_for_segment + (self._distance(p2, base_point) / params['transport_speed']) * (params['work_speed'] * spray_width * consumption_per_sq_meter)
            
            if current_fuel >= fuel_to_finish_and_return:
                current_work_segment.append(p2)
                current_fuel -= fuel_needed_for_segment
            else:
                if len(current_work_segment) > 1:
                    final_route.append({'type': 'work', 'points': list(current_work_segment)})
                
                final_route.append({'type': 'transport', 'points': [p1, base_point]})
                final_route.append({'type': 'transport', 'points': [base_point, p2]})
                
                current_fuel = tank_volume
                current_work_segment = [p2]

        if len(current_work_segment) > 1:
            final_route.append({'type': 'work', 'points': list(current_work_segment)})
        
        if final_route:
            last_point = final_route[-1]['points'][-1]
            final_route.append({'type': 'transport', 'points': [last_point, base_point]})
        
        return final_route

def main():
    try:
        root = tk.Tk()
        app = AgroDronePlannerApp(root)
        root.mainloop()
    except Exception as e:
        pass

if __name__ == "__main__":
    main()