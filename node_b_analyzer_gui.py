"""
node_b_analyzer_gui.py — CANbus Veri Analizörü + Gerçek Zamanlı GUI (Node B)
══════════════════════════════════════════════════════════════════════════════
Sanal CAN bus'ı dinler, hex frame'leri decode eder ve iki panelde gösterir:
  • Sol  : Renkli terminal logger (raw hex + decoded değer)
  • Sağ  : Gerçek zamanlı 4-grafik matplotlib paneli

Çalıştırma:
    1. Önce bu scripti başlat  → python node_b_analyzer_gui.py
    2. Sonra simülatörü başlat → python node_a_sender.py
"""

import time
import threading
import queue
import collections
import tkinter as tk
from tkinter import ttk
import can
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.gridspec import GridSpec
import matplotlib.dates as mdates
from datetime import datetime
import numpy as np

from can_protocol import (
    decode_frame,
    CAN_ID_RPM, CAN_ID_TEMP, CAN_ID_TORQUE,
    CAN_ID_VOLTAGE, CAN_ID_CURRENT, CAN_ID_ERROR,
)

# ──────────────────────────────────────────────────────────────────
#  Ayarlar
# ──────────────────────────────────────────────────────────────────
WINDOW_SECONDS  = 60       # Grafiklerde kaç saniyelik veri tutulur
MAX_LOG_LINES   = 200      # Terminal log'da max satır sayısı
UPDATE_MS       = 200      # GUI güncelleme aralığı (ms)
GRAPH_ALPHA     = 0.85
LINE_WIDTH      = 1.6

# ── Renkler (koyu tema) ───────────────────────────────────────────
BG_DARK    = "#0D1117"     # GitHub koyu arka plan
BG_PANEL   = "#161B22"
BG_PLOT    = "#0D1117"
FG_WHITE   = "#E6EDF3"
FG_DIM     = "#8B949E"
ACCENT     = "#58A6FF"     # Mavi accent
GREEN      = "#3FB950"
YELLOW     = "#D29922"
RED        = "#F85149"
ORANGE     = "#DB6D28"
PURPLE     = "#BC8CFF"

SIGNAL_COLORS = {
    CAN_ID_RPM:     GREEN,
    CAN_ID_TEMP:    ORANGE,
    CAN_ID_TORQUE:  PURPLE,
    CAN_ID_CURRENT: ACCENT,
    CAN_ID_VOLTAGE: YELLOW,
}

# ──────────────────────────────────────────────────────────────────
#  Veri deposu (thread-safe deques)
# ──────────────────────────────────────────────────────────────────
MAX_POINTS = 1000

class DataStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.times     = collections.deque(maxlen=MAX_POINTS)
        self.rpm       = collections.deque(maxlen=MAX_POINTS)
        self.temp      = collections.deque(maxlen=MAX_POINTS)
        self.torque    = collections.deque(maxlen=MAX_POINTS)
        self.current   = collections.deque(maxlen=MAX_POINTS)
        self.voltage   = collections.deque(maxlen=MAX_POINTS)
        self.errors    : list[tuple] = []  # (time, flag_list)

        self.latest = {
            "rpm": 0.0, "temp": 0.0, "torque": 0.0,
            "current": 0.0, "voltage": 0.0, "errors": [],
            "total_frames": 0, "error_frames": 0,
        }

    def push(self, can_id, value, t):
        with self.lock:
            self.latest["total_frames"] += 1
            now = t

            if can_id == CAN_ID_RPM:
                self.times.append(now)
                self.rpm.append(value)
                self.latest["rpm"] = value

            elif can_id == CAN_ID_TEMP:
                self.temp.append(value)
                self.latest["temp"] = value

            elif can_id == CAN_ID_TORQUE:
                self.torque.append(value)
                self.latest["torque"] = value

            elif can_id == CAN_ID_CURRENT:
                self.current.append(value)
                self.latest["current"] = value

            elif can_id == CAN_ID_VOLTAGE:
                self.voltage.append(value)
                self.latest["voltage"] = value

            elif can_id == CAN_ID_ERROR:
                self.latest["error_frames"] += 1

    def snapshot(self):
        with self.lock:
            return (
                list(self.times),
                list(self.rpm),
                list(self.temp),
                list(self.torque),
                list(self.current),
                list(self.voltage),
                dict(self.latest),
            )


# ──────────────────────────────────────────────────────────────────
#  CAN Listener Thread
# ──────────────────────────────────────────────────────────────────
def can_listener_thread(store: DataStore, log_queue: queue.Queue, stop_event: threading.Event):
    """Ayrı thread'de virtual CAN bus'ı dinler."""
    try:
        bus = can.Bus(interface='udp_multicast', channel='239.0.0.1')
    except Exception as e:
        log_queue.put(("ERROR", 0, b"", f"CAN Bus bağlanamadı: {e}", True))
        return

    log_queue.put(("SYS", 0, b"", "✓ Virtual CAN Bus bağlandı — frame bekleniyor...", False))

    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue

            t = time.time()
            frame = decode_frame(msg.arbitration_id, bytes(msg.data))

            if frame is None:
                continue

            # Veri deposuna ekle
            store.push(msg.arbitration_id, frame.value, t)

            # Log kuyruğuna ekle (blocking olmayan put)
            is_err = frame.is_error and bool(frame.error_list)
            try:
                log_queue.put_nowait((
                    "FRAME",
                    msg.arbitration_id,
                    bytes(msg.data),
                    frame.formatted_value,
                    is_err,
                ))
            except queue.Full:
                pass  # Kuyruk doluysa frame'i at, GUI yavaş kalmasın

        except can.CanError:
            continue

    bus.shutdown()


# ──────────────────────────────────────────────────────────────────
#  Ana GUI Sınıfı
# ──────────────────────────────────────────────────────────────────
class CANAnalyzerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("🏭 CANbus Endüstriyel Veri Analizörü — Node B")
        self.root.configure(bg=BG_DARK)
        self.root.geometry("1400x800")
        self.root.minsize(1100, 650)

        self.store      = DataStore()
        self.log_queue  = queue.Queue(maxsize=500)
        self.stop_event = threading.Event()

        self._build_ui()
        self._start_can_thread()
        self._schedule_updates()

    # ── UI Builder ────────────────────────────────────────────────
    def _build_ui(self):
        # Başlık Barı
        header = tk.Frame(self.root, bg=ACCENT, height=4)
        header.pack(fill="x", side="top")

        title_bar = tk.Frame(self.root, bg=BG_PANEL, pady=6)
        title_bar.pack(fill="x")

        tk.Label(
            title_bar,
            text="⚡ ENDÜSTRİYEL CANbus VERİ ANALİZÖRÜ",
            font=("Consolas", 14, "bold"),
            fg=ACCENT, bg=BG_PANEL,
        ).pack(side="left", padx=16)

        tk.Label(
            title_bar,
            text="Node B  |  Virtual CAN  |  500 kbps",
            font=("Consolas", 9),
            fg=FG_DIM, bg=BG_PANEL,
        ).pack(side="left", padx=4)

        # KPI bar
        self.kpi_frame = tk.Frame(self.root, bg=BG_DARK, pady=4)
        self.kpi_frame.pack(fill="x", padx=10)
        self._build_kpi_bar()

        # İçerik (sol log + sağ grafik)
        content = tk.Frame(self.root, bg=BG_DARK)
        content.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        content.columnconfigure(0, weight=2)
        content.columnconfigure(1, weight=5)
        content.rowconfigure(0, weight=1)

        self._build_log_panel(content)
        self._build_graph_panel(content)

    def _build_kpi_bar(self):
        """Anlık değer kartları."""
        kpi_defs = [
            ("🔄 RPM",      "rpm_var",     GREEN,  "rpm"),
            ("🌡 SICAKLIK", "temp_var",    ORANGE, "°C"),
            ("⚙ TORK",      "torque_var",  PURPLE, "N·m"),
            ("〰 AKIM",     "current_var", ACCENT, "A"),
            ("⚡ GERİLİM",  "voltage_var", YELLOW, "V"),
            ("📦 FRAME",    "frame_var",   FG_DIM, "adet"),
        ]

        self.kpi_vars = {}
        for label, var_name, color, unit in kpi_defs:
            card = tk.Frame(self.kpi_frame, bg=BG_PANEL, padx=12, pady=5,
                            relief="flat", bd=0)
            card.pack(side="left", padx=4, pady=2)

            tk.Label(card, text=label, font=("Consolas", 8), fg=FG_DIM,
                     bg=BG_PANEL).pack()

            var = tk.StringVar(value="—")
            self.kpi_vars[var_name] = var
            tk.Label(card, textvariable=var, font=("Consolas", 13, "bold"),
                     fg=color, bg=BG_PANEL).pack()

            tk.Label(card, text=unit, font=("Consolas", 7), fg=FG_DIM,
                     bg=BG_PANEL).pack()

    def _build_log_panel(self, parent):
        """Sol panel: hex log terminal."""
        frame = tk.Frame(parent, bg=BG_PANEL, bd=1, relief="flat")
        frame.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        header = tk.Frame(frame, bg=BG_PANEL, pady=6)
        header.pack(fill="x")
        tk.Label(header, text="◼ RAW CAN LOG  (HEX → DECODE)",
                 font=("Consolas", 10, "bold"), fg=GREEN, bg=BG_PANEL,
                 padx=10).pack(side="left")

        # Text widget + scrollbar
        txt_frame = tk.Frame(frame, bg=BG_PANEL)
        txt_frame.pack(fill="both", expand=True)

        scrollbar = ttk.Scrollbar(txt_frame)
        scrollbar.pack(side="right", fill="y")

        self.log_text = tk.Text(
            txt_frame,
            bg=BG_DARK, fg=FG_WHITE,
            font=("Consolas", 9),
            wrap="none",
            state="disabled",
            yscrollcommand=scrollbar.set,
            selectbackground=BG_PANEL,
            insertbackground=FG_WHITE,
            bd=0, padx=8, pady=4,
        )
        self.log_text.pack(fill="both", expand=True)
        scrollbar.config(command=self.log_text.yview)

        # Renk tag'leri
        self.log_text.tag_configure("ts",       foreground="#555E6B", font=("Consolas", 8))
        self.log_text.tag_configure("id",       foreground=PURPLE)
        self.log_text.tag_configure("sep",      foreground="#30363D")
        self.log_text.tag_configure("hex",      foreground=ACCENT)
        self.log_text.tag_configure("normal",   foreground=GREEN)
        self.log_text.tag_configure("warn",     foreground=YELLOW)
        self.log_text.tag_configure("error",    foreground=RED, font=("Consolas", 9, "bold"))
        self.log_text.tag_configure("sys",      foreground=YELLOW, font=("Consolas", 9, "italic"))

    def _build_graph_panel(self, parent):
        """Sağ panel: matplotlib grafikleri."""
        frame = tk.Frame(parent, bg=BG_PANEL)
        frame.grid(row=0, column=1, sticky="nsew")

        header = tk.Frame(frame, bg=BG_PANEL, pady=6)
        header.pack(fill="x")
        tk.Label(header, text="◼ GERÇEK ZAMANLI TELEMETRİ GRAFİKLERİ",
                 font=("Consolas", 10, "bold"), fg=ACCENT, bg=BG_PANEL,
                 padx=10).pack(side="left")

        self.fig = plt.Figure(figsize=(9, 6), facecolor=BG_DARK)
        self.fig.subplots_adjust(hspace=0.55, left=0.08, right=0.97,
                                 top=0.95, bottom=0.08)

        gs = GridSpec(2, 2, figure=self.fig)
        ax_defs = [
            (gs[0, 0], "Motor Devri",    "RPM",  GREEN,  (0, 3000)),
            (gs[0, 1], "Motor Sıcaklığı","°C",   ORANGE, (20, 100)),
            (gs[1, 0], "Tork",           "N·m",  PURPLE, (0, 60)),
            (gs[1, 1], "Faz Akımı",      "A",    ACCENT, (0, 8)),
        ]

        self.axes  = []
        self.lines = []

        for gs_pos, title, unit, color, ylim in ax_defs:
            ax = self.fig.add_subplot(gs_pos)
            ax.set_facecolor(BG_PLOT)
            ax.tick_params(colors=FG_DIM, labelsize=8)
            ax.set_title(title, color=FG_WHITE, fontsize=9, fontweight="bold", pad=4)
            ax.set_ylabel(unit, color=FG_DIM, fontsize=8)
            ax.set_ylim(*ylim)
            ax.grid(True, color="#21262D", linewidth=0.7, alpha=0.8)
            for spine in ax.spines.values():
                spine.set_edgecolor("#30363D")

            line, = ax.plot([], [], color=color, linewidth=LINE_WIDTH,
                            alpha=GRAPH_ALPHA)
            fill = ax.fill_between([], [], alpha=0.12, color=color)

            self.axes.append((ax, unit, ylim, color, fill))
            self.lines.append(line)

        canvas = FigureCanvasTkAgg(self.fig, master=frame)
        canvas.get_tk_widget().pack(fill="both", expand=True, padx=4, pady=(0, 4))
        self.canvas = canvas

    # ── Thread & Güncelleme ───────────────────────────────────────
    def _start_can_thread(self):
        t = threading.Thread(
            target=can_listener_thread,
            args=(self.store, self.log_queue, self.stop_event),
            daemon=True,
        )
        t.start()

    def _schedule_updates(self):
        self._update_log()
        self._update_graphs()
        self._update_kpis()

    def _update_log(self):
        """Kuyruktan log mesajlarını Text widget'a yazdır."""
        try:
            self.log_text.configure(state="normal")
            count = 0

            while count < 30:
                try:
                    kind, can_id, data, value, is_err = self.log_queue.get_nowait()
                except queue.Empty:
                    break

                ts = time.strftime("%H:%M:%S")

                if kind == "SYS":
                    self.log_text.insert("end", f"[{ts}] {value}\n", "sys")
                elif kind == "ERROR":
                    self.log_text.insert("end", f"[{ts}] !! {value}\n", "error")
                else:
                    hex_str = " ".join(f"{b:02X}" for b in data)
                    signal  = self._id_to_signal(can_id)
                    try:
                        is_hot = (can_id == CAN_ID_TEMP and float(value.split()[0]) > 78)
                    except Exception:
                        is_hot = False
                    val_tag = "error" if is_err else ("warn" if is_hot else "normal")

                    self.log_text.insert("end", f"[{ts}] ", "ts")
                    self.log_text.insert("end", f"0x{can_id:03X}", "id")
                    self.log_text.insert("end", " | ", "sep")
                    self.log_text.insert("end", f"{hex_str:<23}", "hex")
                    self.log_text.insert("end", " | ", "sep")
                    self.log_text.insert("end", f"{signal:<16} -> ", "ts")
                    self.log_text.insert("end", f"{value}\n", val_tag)

                count += 1

            # Satır sınırı
            try:
                lines = int(self.log_text.index("end-1c").split(".")[0])
                if lines > MAX_LOG_LINES:
                    self.log_text.delete("1.0", f"{lines - MAX_LOG_LINES}.0")
            except Exception:
                pass

            self.log_text.see("end")
            self.log_text.configure(state="disabled")
        except Exception:
            pass  # Hata olsa bile aşağıdaki after() her zaman çalışır
        finally:
            self.root.after(UPDATE_MS, self._update_log)

    def _update_graphs(self):
        """Grafikleri güncelle."""
        try:
            times, rpm, temp, torque, curr, volt, latest = self.store.snapshot()

            if len(times) < 2:
                return

            t_arr = np.array(times)
            now   = time.time()
            mask  = t_arr >= (now - WINDOW_SECONDS)

            if mask.sum() < 2:
                return

            t_rel = t_arr[mask] - t_arr[mask][0]
            series = [
                np.array(rpm)[mask],
                np.array(temp)[mask],
                np.array(torque)[mask],
                np.array(curr)[mask],
            ]

            # Snapshot al, iterasyon sırasında self.axes'i değiştirme
            axes_snapshot = list(self.axes)
            new_axes = []

            for i, (line, ax_info) in enumerate(zip(self.lines, axes_snapshot)):
                ax, unit, ylim, color, old_fill = ax_info
                s = series[i]

                if len(s) == 0:
                    new_axes.append(ax_info)
                    continue

                line.set_data(t_rel, s)
                ax.set_xlim(0, max(t_rel[-1], 10))

                # Dinamik Y aralığı — gerçek veriyi tam göster
                data_min = float(s.min())
                data_max = float(s.max())
                margin = max((data_max - data_min) * 0.10, 0.5)
                lo = max(0.0, data_min - margin)
                hi = data_max + margin
                ax.set_ylim(lo, hi)

                # Fill alanı: eski'yi kaldır, yeni çiz
                try:
                    old_fill.remove()
                except Exception:
                    pass
                new_fill = ax.fill_between(t_rel, s, alpha=0.12, color=color)
                new_axes.append((ax, unit, ylim, color, new_fill))

            # Güncellenmiş axes listesini tek seferde ata
            self.axes = new_axes
            self.canvas.draw_idle()

        except Exception:
            pass  # Hata olsa bile after() her zaman çalışır
        finally:
            self.root.after(UPDATE_MS, self._update_graphs)

    def _update_kpis(self):
        """Üst KPI kartlarını güncelle."""
        try:
            _, _, _, _, _, _, latest = self.store.snapshot()
            self.kpi_vars["rpm_var"].set(f"{latest['rpm']:.0f}")
            self.kpi_vars["temp_var"].set(f"{latest['temp']:.1f}")
            self.kpi_vars["torque_var"].set(f"{latest['torque']:.2f}")
            self.kpi_vars["current_var"].set(f"{latest['current']:.2f}")
            self.kpi_vars["voltage_var"].set(f"{latest['voltage']:.1f}")
            self.kpi_vars["frame_var"].set(f"{latest['total_frames']}")
        except Exception:
            pass
        finally:
            self.root.after(500, self._update_kpis)

    def _id_to_signal(self, can_id: int) -> str:
        return {
            CAN_ID_RPM:     "Motor RPM",
            CAN_ID_TEMP:    "Sıcaklık",
            CAN_ID_TORQUE:  "Tork",
            CAN_ID_VOLTAGE: "DC Gerilim",
            CAN_ID_CURRENT: "Faz Akımı",
            CAN_ID_ERROR:   "⚠ HATA",
        }.get(can_id, f"ID:0x{can_id:03X}")

    def on_close(self):
        self.stop_event.set()
        self.root.after(300, self.root.destroy)


# ──────────────────────────────────────────────────────────────────
#  Giriş noktası
# ──────────────────────────────────────────────────────────────────
def main():
    root = tk.Tk()

    # TTK koyu tema
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TScrollbar",
                    background=BG_PANEL,
                    troughcolor=BG_DARK,
                    arrowcolor=FG_DIM)

    app = CANAnalyzerApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
