# TOGAY Model Uydu Yer İstasyonu Arayüzü

<p>
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python" />
  <img src="https://img.shields.io/badge/PyQt5-41CD52?style=for-the-badge&logo=qt&logoColor=white" alt="PyQt5" />
  <img src="https://img.shields.io/badge/pyqtgraph-1F2A44?style=for-the-badge" alt="pyqtgraph" />
  <img src="https://img.shields.io/badge/VTK-1F6FEB?style=for-the-badge" alt="VTK" />
  <img src="https://img.shields.io/badge/Folium-77B829?style=for-the-badge&logo=leaflet&logoColor=white" alt="Folium" />
  <img src="https://img.shields.io/badge/pySerial-20232A?style=for-the-badge" alt="pySerial" />
</p>

## 🇬🇧 Overview

Ground station software written for a model satellite competition by team TOGAY. It receives telemetry over serial, plots it live, shows the satellite's attitude on a 3D model, marks GPS position on a map and logs every packet to CSV.

**Quick start:** `pip install -r requirements.txt && python main.py`

## 🇹🇷 Proje hakkında

TOGAY takımının model uydu yarışması için geliştirdiği yer istasyonu arayüzü. Seri porttan gelen telemetri paketlerini ayrıştırır, canlı grafiklerde gösterir, uydunun duruşunu 3B model üzerinde canlandırır, GPS konumunu haritada işaretler ve tüm paketleri CSV dosyasına kaydeder.

## ✨ Özellikler

- Telemetri: paket no, uydu statüsü, hata kodu, basınç, yükseklik, iniş hızı, sıcaklık, pil gerilimi, GPS, pitch/roll/yaw
- Altı canlı grafik (pyqtgraph)
- VTK ile STL modelden 3B duruş gösterimi
- Folium haritasında GPS konumu
- Hata kodu göstergeleri ve taşıyıcı ayırma komutu
- Her paketin `veri.csv` dosyasına kaydı

## ⚙️ Kurulum ve çalıştırma

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

```bash
python main.py
```

## 📁 Dosya yapısı

```text
TOGAY_ModelUyduYerIstasyonuArayuz/
├── gui.py
├── logo_rc.py
├── main.py
├── scaled_model.stl
└── veri.csv
```

## 📝 Notlar

- `gui.py` Qt Designer çıktısıdır, `logo_rc.py` arayüzdeki logo kaynaklarını içerir.
- `PyAudio` kurulumu için sistemde PortAudio gerekir (Ubuntu: `sudo apt install portaudio19-dev`).
