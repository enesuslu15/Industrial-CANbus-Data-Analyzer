"""
node_c_ml_anomaly_detector.py — Proje 8: ML Tabanlı Anomali Tespiti
════════════════════════════════════════════════════════════════════════
CAN bus'ı dinleyerek motor verilerini toplar, eğitilmiş bir Makine 
Öğrenmesi modeli (Isolation Forest) ile anomali tespiti yapar.
Anomali tespit ederse CAN bus'a yeni bir "CAN_ID_ML_ANOMALY" frame'i atar.

Kullanım:
  1. Önce modeli eğitmek için (15 sn boyunca sağlıklı veri toplar):
     python node_c_ml_anomaly_detector.py --train

  2. Gerçek zamanlı anomali tespiti için (Eğitim bittikten sonra):
     python node_c_ml_anomaly_detector.py
"""

import time
import argparse
import pandas as pd
import numpy as np
import pickle
import can
import threading
import warnings
from sklearn.ensemble import IsolationForest
from rich.console import Console
from rich.panel import Panel

# sklearn warning'leri gizlemek için
warnings.filterwarnings("ignore")

from can_protocol import (
    decode_frame,
    CAN_ID_RPM, CAN_ID_TEMP, CAN_ID_TORQUE,
    CAN_ID_VOLTAGE, CAN_ID_CURRENT, CAN_ID_ML_ANOMALY,
    encode_ml_anomaly
)

console = Console()
MODEL_FILE = "anomaly_model.pkl"

class MLNode:
    def __init__(self):
        self.latest_data = {
            "rpm": 1500.0,
            "temp": 40.0,
            "torque": 10.0,
            "voltage": 550.0,
            "current": 2.0
        }
        self.lock = threading.Lock()
        self.running = True

    def _can_listener(self):
        try:
            bus = can.Bus(interface='udp_multicast', channel='239.0.0.1')
        except Exception as e:
            console.print(f"[red]CAN Bus bağlanamadı: {e}[/red]")
            return

        while self.running:
            try:
                msg = bus.recv(timeout=0.1)
                if msg is None:
                    continue

                frame = decode_frame(msg.arbitration_id, bytes(msg.data))
                if frame is None:
                    continue

                with self.lock:
                    if frame.can_id == CAN_ID_RPM:
                        self.latest_data["rpm"] = frame.value
                    elif frame.can_id == CAN_ID_TEMP:
                        self.latest_data["temp"] = frame.value
                    elif frame.can_id == CAN_ID_TORQUE:
                        self.latest_data["torque"] = frame.value
                    elif frame.can_id == CAN_ID_VOLTAGE:
                        self.latest_data["voltage"] = frame.value
                    elif frame.can_id == CAN_ID_CURRENT:
                        self.latest_data["current"] = frame.value

            except can.CanError:
                pass
        
        bus.shutdown()

    def train_model(self, duration=15):
        console.print(Panel("[bright_yellow]EĞİTİM MODU BAŞLIYOR...[/bright_yellow]\nLütfen Node A simülatörünün çalıştığından emin olun.\nSistem sağlıklı verileri topluyor..."))
        
        # Dinleme thread'ini başlat
        t = threading.Thread(target=self._can_listener, daemon=True)
        t.start()

        data_rows = []
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                time.sleep(0.1) # 100ms'de bir örnek al
                with self.lock:
                    row = self.latest_data.copy()
                data_rows.append(row)
                
                elapsed = int(time.time() - start_time)
                console.print(f"[cyan]Veri toplanıyor... {elapsed}/{duration} saniye[/cyan]", end="\r")
        except KeyboardInterrupt:
            pass

        self.running = False
        t.join()

        console.print("\n[green]Veri toplama tamamlandı. Model eğitiliyor...[/green]")
        df = pd.DataFrame(data_rows)
        
        # Isolation Forest Modeli
        # contamination=0.02 demek, "verinin en uç %2'lik kısmını anomali olarak değerlendir"
        model = IsolationForest(n_estimators=100, contamination=0.02, random_state=42)
        model.fit(df)

        with open(MODEL_FILE, "wb") as f:
            pickle.dump(model, f)

        console.print(f"[bold bright_green]✓ Model eğitildi ve {MODEL_FILE} olarak kaydedildi![/bold bright_green]")
        console.print("[white]Kullanılan özellikler: RPM, Temp, Torque, Voltage, Current[/white]")

    def run_inference(self):
        try:
            with open(MODEL_FILE, "rb") as f:
                model = pickle.load(f)
            console.print(f"[green]✓ {MODEL_FILE} başarıyla yüklendi.[/green]")
        except FileNotFoundError:
            console.print("[red]Model bulunamadı! Önce '--train' parametresi ile çalıştırın.[/red]")
            return

        console.print(Panel("[bright_cyan]GERÇEK ZAMANLI ANOMALİ TESPİTİ BAŞLADI (Node C)[/bright_cyan]\nMotor verileri analiz ediliyor..."))

        # Hem okuma hem yazma için bus
        try:
            bus = can.Bus(interface='udp_multicast', channel='239.0.0.1')
        except Exception as e:
            console.print(f"[red]CAN Bus bağlanamadı: {e}[/red]")
            return

        # Dinleme thread'i (verileri güncellemek için)
        t = threading.Thread(target=self._can_listener, daemon=True)
        t.start()

        anomaly_count = 0
        try:
            while True:
                time.sleep(0.1)
                
                with self.lock:
                    current_features = [
                        self.latest_data["rpm"],
                        self.latest_data["temp"],
                        self.latest_data["torque"],
                        self.latest_data["voltage"],
                        self.latest_data["current"]
                    ]

                # Scikit-Learn: Normal -> 1, Anomali -> -1
                df_features = pd.DataFrame([current_features], columns=["rpm", "temp", "torque", "voltage", "current"])
                prediction = model.predict(df_features)[0]
                
                is_anomaly = (prediction == -1)

                if is_anomaly:
                    anomaly_count += 1
                    console.print(f"[bold red]🚨 ANOMALİ TESPİT EDİLDİ![/bold red] ({anomaly_count}) Veri: {current_features}")
                    
                    # CAN Bus'a Anomali Alarmı Gönder
                    payload = encode_ml_anomaly(True)
                    msg = can.Message(arbitration_id=CAN_ID_ML_ANOMALY, data=payload, is_extended_id=False)
                    try:
                        bus.send(msg)
                    except can.CanError:
                        pass
                else:
                    # Heartbeat olarak normal yayınla
                    payload = encode_ml_anomaly(False)
                    msg = can.Message(arbitration_id=CAN_ID_ML_ANOMALY, data=payload, is_extended_id=False)
                    try:
                        bus.send(msg)
                    except can.CanError:
                        pass
                    
        except KeyboardInterrupt:
            console.print("\n[yellow]⏹  Anomali tespiti durduruldu.[/yellow]")
        finally:
            self.running = False
            t.join()
            bus.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CANbus ML Anomaly Detector")
    parser.add_argument("--train", action="store_true", help="15 saniyelik sağlıklı veri ile modeli eğit")
    args = parser.parse_args()

    node = MLNode()
    if args.train:
        node.train_model(duration=15)
    else:
        node.run_inference()

