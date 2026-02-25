"""
node_a_sender.py — Motor Sürücü Simülatörü (Node A)
═══════════════════════════════════════════════════════
Sanal CAN bus üzerinden gerçekçi motor telemetrisi yayınlar.
Her 100ms'de bir paket gönderilir.

Çalıştırma:
    python node_a_sender.py
"""

import time
import math
import random
import can
from rich.console import Console
from rich.panel import Panel
from rich.text import Text
from rich.live import Live
from rich.table import Table
from rich.style import Style
import threading

from can_protocol import (
    CAN_ID_RPM, CAN_ID_TEMP, CAN_ID_TORQUE,
    CAN_ID_VOLTAGE, CAN_ID_CURRENT, CAN_ID_ERROR,
    encode_rpm, encode_temp, encode_torque,
    encode_voltage, encode_current, encode_error,
    ERROR_OVERHEAT, ERROR_OVERCURRENT, ERROR_UNDERVOLTAGE,
)

console = Console()

# ── Simülasyon Parametreleri ──────────────────────────────────────
SEND_INTERVAL   = 0.10   # saniye (100 ms)
SIM_SPEED       = 1.0    # Simülasyon hızı çarpanı
ERROR_PROB      = 0.04   # Hata frame'i yayınlama olasılığı

# ── Renk Paleti ───────────────────────────────────────────────────
COLOR_NORMAL    = "bright_green"
COLOR_WARN      = "yellow"
COLOR_ERROR     = "bright_red"
COLOR_HEX       = "cyan"
COLOR_ID        = "magenta"
COLOR_TIMESTAMP = "bright_black"


class MotorSimulator:
    """Gerçekçi motor davranışı simüle eden sınıf."""

    def __init__(self):
        self.t = 0.0
        self.rpm_base = 1500.0
        self.load = 0.5

    def step(self, dt: float):
        self.t += dt

    def rpm(self) -> float:
        """Sinüs bazlı değişken RPM: 800–2500 arası."""
        wave = math.sin(self.t * 0.3) * 0.5 + math.sin(self.t * 0.07) * 0.3
        base_rpm = self.rpm_base + wave * 850
        noise = random.gauss(0, 15)
        return max(0, base_rpm + noise)

    def temperature(self, current_rpm: float) -> float:
        """RPM'e bağlı sıcaklık artışı + soğuma simülasyonu."""
        load_factor = current_rpm / 2500.0
        ambient = 25.0
        heat_rise = load_factor * 55.0
        oscillation = math.sin(self.t * 0.05) * 3.0
        noise = random.gauss(0, 0.5)
        return ambient + heat_rise + oscillation + noise

    def torque(self, current_rpm: float) -> float:
        """Güç sabiti ile tork (N·m)."""
        power_kw = 7.5  # Sabit 7.5 kW motor
        if current_rpm < 1.0:
            return 0.0
        torque = (power_kw * 1000 * 60) / (2 * math.pi * current_rpm)
        noise = random.gauss(0, 0.3)
        return max(0, torque + noise)

    def voltage(self) -> float:
        """DC Bus gerilimi: 540–560 V arası."""
        return 550.0 + math.sin(self.t * 0.15) * 8.0 + random.gauss(0, 1.0)

    def current(self, torque_nm: float) -> float:
        """Motor akımı (tork / moment sabitinden)."""
        kt = 2.1  # Motor moment sabiti
        return max(0, torque_nm / kt + random.gauss(0, 0.05))

    def error_flags(self, temp_c: float, current_a: float, voltage_v: float) -> int:
        """Gerçek koşullara göre hata bayraklarını oluştur."""
        flags = 0
        if temp_c > 80.0:       flags |= ERROR_OVERHEAT
        if current_a > 5.0:     flags |= ERROR_OVERCURRENT
        if voltage_v < 520.0:   flags |= ERROR_UNDERVOLTAGE
        # Rastgele geçici hata:
        if random.random() < ERROR_PROB:
            flags |= random.choice([ERROR_OVERCURRENT, ERROR_UNDERVOLTAGE])
        return flags


def format_hex_log(can_id: int, data: bytes, signal: str, value: str, has_error: bool) -> Text:
    """Terminal'de tek satır log formatı oluşturur."""
    ts = time.strftime("%H:%M:%S")
    hex_data = " ".join(f"{b:02X}" for b in data)

    color = COLOR_ERROR if has_error else COLOR_NORMAL

    line = Text()
    line.append(f"[{ts}] ", style=COLOR_TIMESTAMP)
    line.append(f"ID:0x{can_id:03X} ", style=COLOR_ID)
    line.append("│ ", style="bright_black")
    line.append(f"{hex_data:<23}", style=COLOR_HEX)
    line.append(" │ ", style="bright_black")
    line.append(f"{signal:<18}", style="white")
    line.append(" → ", style="bright_black")
    line.append(value, style=color)
    return line


def make_status_table(rpm, temp, torq, volt, curr, err_flags):
    """Anlık değerleri gösteren özet tablo."""
    table = Table(
        title="⚡ Motor Telemetri Özeti",
        border_style="bright_blue",
        title_style="bold bright_cyan",
        min_width=55,
    )
    table.add_column("Parametre", style="bold white", width=20)
    table.add_column("Değer",     style="bright_green", justify="right", width=15)
    table.add_column("Durum",     style="yellow",       width=12)

    def status(val, low, high):
        if val < low or val > high:
            return "[red]⚠ ALARM[/red]"
        return "[green]✓ NORMAL[/green]"

    table.add_row("🔄 Devir",       f"{rpm:.0f} RPM",  status(rpm,  200, 2800))
    table.add_row("🌡  Sıcaklık",   f"{temp:.1f} °C",  status(temp, 20,  80))
    table.add_row("⚙  Tork",        f"{torq:.2f} N·m", status(torq, 0,   50))
    table.add_row("⚡ DC Gerilim",  f"{volt:.1f} V",   status(volt, 500, 600))
    table.add_row("〰  Faz Akımı",  f"{curr:.2f} A",   status(curr, 0,   6))

    err_str = "YOK" if err_flags == 0 else f"FLAGS=0x{err_flags:02X}"
    err_sty = "green" if err_flags == 0 else "red bold"
    table.add_row("🚨 Hata",        f"[{err_sty}]{err_str}[/{err_sty}]", "")
    return table


def main():
    console.print(Panel(
        "[bold bright_cyan]🏭 ENDÜSTRİYEL CANbus SİMÜLATÖR[/bold bright_cyan]\n"
        "[bright_black]Node A — Motor Sürücü Emülatörü[/bright_black]\n"
        "[yellow]Virtual CAN Bus üzerinden yayın başlıyor...[/yellow]",
        border_style="bright_cyan",
        width=65,
    ))

    time.sleep(0.5)

    try:
        bus = can.Bus(interface='udp_multicast', channel='239.0.0.1')
    except Exception as e:
        console.print(f"[red]CAN Bus başlatılamadı: {e}[/red]")
        return

    sim = MotorSimulator()
    frame_count = 0
    log_lines: list[Text] = []
    MAX_LOG = 25

    console.print("[bright_green]✓ Virtual CAN Bus bağlantısı kuruldu.[/bright_green]")
    console.print("[bright_black]─" * 65 + "[/bright_black]")
    console.print("[bold white]Akan CAN Frame'leri:[/bold white]\n")

    try:
        while True:
            loop_start = time.perf_counter()
            sim.step(SEND_INTERVAL)

            # ── Anlık değerleri hesapla ──────────────────────────
            rpm    = sim.rpm()
            temp   = sim.temperature(rpm)
            torq   = sim.torque(rpm)
            volt   = sim.voltage()
            curr   = sim.current(torq)
            flags  = sim.error_flags(temp, curr, volt)

            # ── Frame'leri gönder ────────────────────────────────
            frames_to_send = [
                (CAN_ID_RPM,     encode_rpm(rpm),        f"{rpm:.0f} RPM",     False),
                (CAN_ID_TEMP,    encode_temp(temp),       f"{temp:.1f} °C",     temp > 78),
                (CAN_ID_TORQUE,  encode_torque(torq),     f"{torq:.2f} N·m",    False),
                (CAN_ID_VOLTAGE, encode_voltage(volt),    f"{volt:.1f} V",      volt < 520),
                (CAN_ID_CURRENT, encode_current(curr),    f"{curr:.2f} A",      curr > 5.5),
            ]

            for can_id, data, val_str, is_warn in frames_to_send:
                msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
                try:
                    bus.send(msg)
                except can.CanError as e:
                    console.print(f"[red]Gönderme hatası: {e}[/red]")

                signal_name = {
                    CAN_ID_RPM: "Motor RPM", CAN_ID_TEMP: "Sıcaklık",
                    CAN_ID_TORQUE: "Tork", CAN_ID_VOLTAGE: "DC Gerilim",
                    CAN_ID_CURRENT: "Faz Akımı",
                }.get(can_id, "?")

                line = format_hex_log(can_id, data, signal_name, val_str, is_warn)
                log_lines.append(line)
                console.print(line)

            # ── Hata frame'i ─────────────────────────────────────
            if flags != 0:
                err_data = encode_error(flags)
                err_msg  = can.Message(arbitration_id=CAN_ID_ERROR, data=err_data,
                                       is_extended_id=False)
                try:
                    bus.send(err_msg)
                except can.CanError:
                    pass

                err_line = format_hex_log(
                    CAN_ID_ERROR, err_data, "HATA BAYRAĞ",
                    f"FLAGS=0x{flags:02X}", True
                )
                console.print(err_line)
                log_lines.append(err_line)

            frame_count += 1

            # ── Her 20 frame'de özet tablo ────────────────────────
            if frame_count % 20 == 0:
                console.rule(f"[bright_black]Frame #{frame_count * 5}[/bright_black]")
                console.print(make_status_table(rpm, temp, torq, volt, curr, flags))
                console.print()

            # ── 100 ms döngüsünü koru ─────────────────────────────
            elapsed = time.perf_counter() - loop_start
            sleep_t = max(0, SEND_INTERVAL - elapsed)
            time.sleep(sleep_t)

    except KeyboardInterrupt:
        console.print("\n[yellow]⏹  Simülatör durduruldu.[/yellow]")
    finally:
        bus.shutdown()
        console.print(f"[bright_black]Toplam {frame_count * 5} frame gönderildi.[/bright_black]")


if __name__ == "__main__":
    main()
