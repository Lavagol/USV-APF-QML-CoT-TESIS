# plot_forces.py
# Gráfica de ‖F_Att‖ y ‖F_rep‖ usando matplotlib
# ------------------------------------------------------------------------

import pandas as pd
import matplotlib.pyplot as plt

# 1. Cargar el CSV
csv_path = "run_20250804_090442_steps.csv"  # ajusta la ruta si es necesario
df = pd.read_csv(csv_path)

# 2. Eje X: 'step' si existe; de lo contrario, índice
t = df['step'] if 'step' in df.columns else df.index

# 3. Graficar
plt.figure(figsize=(8, 4))
plt.plot(t, df['F_Att'], label='‖F_Att‖', linewidth=1.6)
plt.plot(t, df['F_rep'], label='‖F_rep‖', linewidth=1.6)
plt.xlabel('Paso de simulación')
plt.ylabel('Magnitud de la fuerza [N]')
plt.title('‖F_Att‖ y ‖F_rep‖ vs. tiempo')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
