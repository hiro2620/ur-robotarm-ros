import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルのパス
csv_path = "force_data.csv"

# データ読み込み
df = pd.read_csv(csv_path)

fig, ax1 = plt.subplots(figsize=(10, 5))

# Force_Z（左軸）
ax1.plot(df["Timestamp"], df["Force_Z"], label="Force_Z", color="tab:blue")
ax1.axhline(y=-5, color="tab:green", linestyle="--", label="Force_Z=5")
ax1.set_xlabel("Timestamp")
ax1.set_ylabel("Force_Z [N]", color="tab:blue")
ax1.tick_params(axis='y', labelcolor="tab:blue")

# Displacement_Z（右軸）
ax2 = ax1.twinx()
ax2.plot(df["Timestamp"], df["Displacement_Z"], label="Displacement_Z", color="tab:red")
ax2.set_ylabel("Displacement_Z [a.u.]", color="tab:red")
ax2.tick_params(axis='y', labelcolor="tab:red")

plt.title("Force_Z and Displacement_Z over Time")
fig.tight_layout()
plt.show()
